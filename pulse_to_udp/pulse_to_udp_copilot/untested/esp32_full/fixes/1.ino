#include <Arduino.h>
#include <ESPAsyncUDP.h>
#include <WiFi.h>

#define TRANSLATION_TIMEBASE 240000000 // ESP32 clock rate

// Macros for ESP32 optimization
inline uint32_t getCycleCount() {
    uint32_t count;
    asm volatile("rsr %0,ccount" : "=a"(count));
    return count;
}

inline uint8_t fastDigitalRead(uint8_t pin) {
    return (GPIO.in >> pin) & 0x1;
}

// UDP Configuration
AsyncUDP udp;
const char* multicastIP = "239.255.0.1";
const uint16_t multicastPort = 12345;

// Pin Definitions
const uint8_t CAM_SENSOR_PIN = 4;
const uint8_t IGNITION_PIN = 5;
const uint8_t INJECTION_PIN_1 = 6;
const uint8_t INJECTION_PIN_2 = 7;
const uint8_t INJECTION_PIN_3 = 8;
const uint8_t INJECTION_PIN_4 = 9;
const uint8_t LAMBDA_PIN = A0;
const uint8_t MAP_SENSOR_PIN = A1;

// Constants
const uint8_t NUM_CYLINDERS = 4;
const uint8_t CAM_PULSES_PER_CYCLE = 4;
const uint16_t ENGINE_CYCLE_DEGREES = 720; // Two crankshaft revolutions
const uint8_t NUM_LAMBDA_SAMPLES = 8;
const uint8_t MAP_SAMPLES_PER_DEGREE = 1;
const uint16_t TOTAL_MAP_SAMPLES = (ENGINE_CYCLE_DEGREES * MAP_SAMPLES_PER_DEGREE / NUM_CYLINDERS);
const uint16_t LAMBDA_INTERVAL = (ENGINE_CYCLE_DEGREES / NUM_LAMBDA_SAMPLES);

// Watchdog timeout values (in milliseconds)
const uint32_t CAM_SIGNAL_TIMEOUT = 500;  // 500ms without cam signal indicates engine stopped
const uint32_t MINIMUM_RPM = 100;  // Minimum valid RPM
const uint32_t MAXIMUM_RPM = 15000;  // Maximum valid RPM

// Struct Definitions
#pragma pack(push, 1)
struct UnifiedPacket {
    float camRpm;
    uint8_t sequentialCycle;
    uint32_t timestamp;  // Added timestamp for packet ordering
    bool engineRunning;  // Added engine running status

    struct CylinderData {
        uint32_t ignitionStartAngle;
        uint32_t ignitionDuration;
        uint32_t injectionStartAngle;
        uint32_t injectionDuration;
        uint16_t mapSamples[TOTAL_MAP_SAMPLES];
        float lambdaSamples[NUM_LAMBDA_SAMPLES];
    } cylinder[NUM_CYLINDERS];
};
#pragma pack(pop)

// Create a non-volatile working copy and a volatile transmission copy
UnifiedPacket workingPacket;
volatile UnifiedPacket transmitPacket;

// Variables for Camshaft
volatile uint32_t lastRisingEdge = 0;  // For RPM calculation
volatile uint32_t currentRisingEdge = 0;  // For RPM calculation
volatile uint32_t camPulseStart = 0;  // Rising edge timestamp
volatile uint32_t camPulseEnd = 0;    // Falling edge timestamp
volatile uint32_t pulseWidth = 0;     // Duration between rising and falling edge
volatile uint8_t currentCylinder = 0;
volatile bool engineCycleStarted = false;
volatile uint32_t lastCamSignalTime = 0;  // For timeout detection

// Constants for pulse width detection
const uint32_t EXPECTED_SHORT_PULSE_WIDTH = TRANSLATION_TIMEBASE / 1000;  // 1ms in ESP32 cycles
const float PULSE_WIDTH_TOLERANCE = 0.3;  // ±30% tolerance

// Lambda Sampling
volatile uint32_t nextLambdaSampleTime = 0;
volatile uint8_t currentLambdaSample = 0;

// MAP Sampling
volatile uint16_t mapSampleIndex = 0;

// RPM calculation buffer
const uint8_t RPM_BUFFER_SIZE = 4;
volatile uint32_t risingEdgeTimes[RPM_BUFFER_SIZE];
volatile uint8_t rpmBufferIndex = 0;

// Error tracking
volatile uint32_t errorCount = 0;
volatile uint32_t lastErrorTime = 0;
const uint32_t ERROR_REPORT_INTERVAL = 1000;  // Report errors every 1 second

// Function declarations
void checkEngineTimeout();
float calculateAverageRPM();
void sampleLambda();
void sampleMAP();
void sendUnifiedPacket();
void processData();
void reportError(const char* errorMessage);

// Calculate average RPM from the last few measurements
float calculateAverageRPM() {
    uint32_t totalTime = 0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 1; i < RPM_BUFFER_SIZE; i++) {
        uint32_t timeDiff = risingEdgeTimes[i] - risingEdgeTimes[i-1];
        if (timeDiff > 0) {
            totalTime += timeDiff;
            validSamples++;
        }
    }
    
    if (validSamples == 0) return 0.0f;
    
    // Calculate average time between pulses
    uint32_t averageTime = totalTime / validSamples;
    
    // Convert to RPM: (60 seconds * clock rate) / (average time * pulses per revolution)
    float rpm = (60.0f * TRANSLATION_TIMEBASE) / (float)(averageTime * CAM_PULSES_PER_CYCLE);
    
    // Validate RPM range
    if (rpm < MINIMUM_RPM || rpm > MAXIMUM_RPM) {
        reportError("Invalid RPM calculated");
        return workingPacket.camRpm;  // Return last valid RPM
    }
    
    return rpm;
}

// ISR: Camshaft Sensor
void IRAM_ATTR camISR() {
    uint32_t currentTime = getCycleCount();
    lastCamSignalTime = millis();  // Update timeout tracker
    
    if (fastDigitalRead(CAM_SENSOR_PIN)) { // Rising edge
        // Store timestamp for RPM calculation
        lastRisingEdge = currentRisingEdge;
        currentRisingEdge = currentTime;
        
        // Store rising edge time in circular buffer
        risingEdgeTimes[rpmBufferIndex] = currentTime;
        rpmBufferIndex = (rpmBufferIndex + 1) % RPM_BUFFER_SIZE;
        
        camPulseStart = currentTime;
        
    } else { // Falling edge
        camPulseEnd = currentTime;
        pulseWidth = camPulseEnd - camPulseStart;
        
        // Validate pulse width
        if (pulseWidth == 0 || pulseWidth > TRANSLATION_TIMEBASE) {
            reportError("Invalid pulse width detected");
            return;
        }
        
        // Check if this is a short pulse (indicating start of engine cycle)
        bool isShortPulse = (pulseWidth > (EXPECTED_SHORT_PULSE_WIDTH * (1 - PULSE_WIDTH_TOLERANCE))) && 
                           (pulseWidth < (EXPECTED_SHORT_PULSE_WIDTH * (1 + PULSE_WIDTH_TOLERANCE)));
        
        if (isShortPulse) {
            currentCylinder = 0;  // Reset to first cylinder
            engineCycleStarted = true;
            workingPacket.sequentialCycle++;  // Increment cycle counter
        } else if (engineCycleStarted) {
            currentCylinder = (currentCylinder + 1) % NUM_CYLINDERS;
        }
        
        // Calculate and update RPM
        if (lastRisingEdge > 0) {
            float newRpm = calculateAverageRPM();
            if (newRpm > 0) {
                workingPacket.camRpm = newRpm;
            }
        }
        
        // Reset sampling states for the new cylinder
        nextLambdaSampleTime = currentTime + (pulseWidth / NUM_LAMBDA_SAMPLES);
        currentLambdaSample = 0;
        mapSampleIndex = 0;
    }
}

// ISR: Ignition timing
void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    if (!engineCycleStarted) return;  // Ignore if engine cycle hasn't started
    
    if (fastDigitalRead(IGNITION_PIN)) { // Rising edge
        workingPacket.cylinder[currentCylinder].ignitionStartAngle =
            (uint32_t)(360.0f * (float)(currentTime - camPulseStart) / (float)(pulseWidth));
    } else { // Falling edge
        workingPacket.cylinder[currentCylinder].ignitionDuration = currentTime - camPulseStart;
    }
}

// ISR: Injection timing
void IRAM_ATTR injectionISR(uint8_t injectorIndex, uint8_t injectorPin) {
    uint32_t currentTime = getCycleCount();
    if (!engineCycleStarted) return;  // Ignore if engine cycle hasn't started
    
    if (fastDigitalRead(injectorPin)) { // Rising edge
        workingPacket.cylinder[injectorIndex].injectionStartAngle =
            (uint32_t)(360.0f * (float)(currentTime - camPulseStart) / (float)(pulseWidth));
    } else { // Falling edge
        workingPacket.cylinder[injectorIndex].injectionDuration = currentTime - camPulseStart;
    }
}

// Lambda Sampling with bounds checking
void sampleLambda() {
    if (!engineCycleStarted || currentCylinder >= NUM_CYLINDERS || 
        currentLambdaSample >= NUM_LAMBDA_SAMPLES) return;
        
    uint32_t currentTime = getCycleCount();
    if (currentTime >= nextLambdaSampleTime) {
        // Read and convert analog value with bounds checking
        int analogValue = analogRead(LAMBDA_PIN);
        float voltage = (analogValue * 5.0f) / 1023.0f;
        if (voltage >= 0.0f && voltage <= 5.0f) {
            workingPacket.cylinder[currentCylinder].lambdaSamples[currentLambdaSample] = voltage;
        } else {
            reportError("Invalid lambda reading");
        }
        currentLambdaSample++;
        nextLambdaSampleTime += pulseWidth / NUM_LAMBDA_SAMPLES;
    }
}

// MAP Sampling with bounds checking
void sampleMAP() {
    if (!engineCycleStarted || currentCylinder >= NUM_CYLINDERS || 
        mapSampleIndex >= TOTAL_MAP_SAMPLES) return;
        
    uint32_t currentTime = getCycleCount();
    uint32_t sampleTime = camPulseStart + mapSampleIndex * (pulseWidth / TOTAL_MAP_SAMPLES);
    
    if (currentTime >= sampleTime) {
        int analogValue = analogRead(MAP_SENSOR_PIN);
        if (analogValue >= 0 && analogValue <= 1023) {
            workingPacket.cylinder[currentCylinder].mapSamples[mapSampleIndex] = (uint16_t)analogValue;
        } else {
            reportError("Invalid MAP reading");
        }
        mapSampleIndex++;
    }
}

// Check for engine timeout
void checkEngineTimeout() {
    uint32_t currentTime = millis();
    if (currentTime - lastCamSignalTime > CAM_SIGNAL_TIMEOUT) {
        engineCycleStarted = false;
        workingPacket.engineRunning = false;
        workingPacket.camRpm = 0;
    } else {
        workingPacket.engineRunning = true;
    }
}

// Error reporting
void reportError(const char* errorMessage) {
    uint32_t currentTime = millis();
    errorCount++;
    
    // Report errors periodically to avoid flooding
    if (currentTime - lastErrorTime > ERROR_REPORT_INTERVAL) {
        Serial.printf("Error: %s (Count: %lu)\n", errorMessage, errorCount);
        lastErrorTime = currentTime;
    }
}

// Send Unified Packet with timeout checking
void sendUnifiedPacket() {
    checkEngineTimeout();
    
    // Update timestamp
    workingPacket.timestamp = millis();
    
    // Copy working packet to transmit packet atomically
    noInterrupts();
    memcpy((void*)&transmitPacket, &workingPacket, sizeof(UnifiedPacket));
    interrupts();
    
    // Send packet
    if (WiFi.status() == WL_CONNECTED) {
        udp.writeTo((uint8_t*)&transmitPacket, sizeof(UnifiedPacket), 
                   IPAddress(239, 255, 0, 1), multicastPort);
    }
}

// Process Data
void processData() {
    sampleLambda();
    sampleMAP();

    // Send packet when MAP sampling for current cylinder is complete
    if (mapSampleIndex >= TOTAL_MAP_SAMPLES) {
        sendUnifiedPacket();
    }
}

void setup() {
    // Configure pins with pullup resistors where appropriate
    pinMode(CAM_SENSOR_PIN, INPUT_PULLUP);
    pinMode(IGNITION_PIN, INPUT_PULLUP);
    pinMode(INJECTION_PIN_1, INPUT_PULLUP);
    pinMode(INJECTION_PIN_2, INPUT_PULLUP);
    pinMode(INJECTION_PIN_3, INPUT_PULLUP);
    pinMode(INJECTION_PIN_4, INPUT_PULLUP);
    pinMode(LAMBDA_PIN, INPUT);
    pinMode(MAP_SENSOR_PIN, INPUT);

    // Initialize WiFi with timeout
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin("SSID", "PASSWORD");
    
    uint32_t wifiStartTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 30000) {
        delay(500);
    }

    // Initialize UDP multicast
    if (udp.connect(IPAddress(239, 255, 0, 1), multicastPort)) {
        udp.listen_multicast(IPAddress(239, 255, 0, 1), multicastPort);
    }

    // Initialize packet data
    memset(&workingPacket, 0, sizeof(UnifiedPacket));
    memset((void*)&transmitPacket, 0, sizeof(UnifiedPacket));

    // Attach interrupts with error checking
    if (!digitalPinToInterrupt(CAM_SENSOR_PIN)) {
        reportError("Invalid CAM_SENSOR_PIN for interrupt");
        return;
    }
    attachInterrupt(digitalPinToInterrupt(CAM_SENSOR_PIN), camISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN), ignitionISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_1), 
                   []() { injectionISR(0, INJECTION_PIN_1); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_2), 
                   []() { injectionISR(1, INJECTION_PIN_2); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_3), 
                   []() { injectionISR(2, INJECTION_PIN_3); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_4), 
                   []() { injectionISR(3, INJECTION_PIN_4); }, CHANGE);

    // Initialize serial with a higher baud rate for debugging
    Serial.begin(500000);
    Serial.println("System Initializing...");

    // Initialize timing variables
    lastCamSignalTime = millis();
    lastErrorTime = millis();

    // Initialize RPM buffer
    for (uint8_t i = 0; i < RPM_BUFFER_SIZE; i++) {
        risingEdgeTimes[i] = 0;
    }

    Serial.println("System Initialized");
}

void loop() {
    static uint32_t lastStatusTime = 0;
    const uint32_t STATUS_INTERVAL = 1000; // Status update every 1 second
    
    // Process engine data
    processData();
    
    // Periodic status check and reporting
    uint32_t currentTime = millis();
    if (currentTime - lastStatusTime >= STATUS_INTERVAL) {
        lastStatusTime = currentTime;
        
        // Check WiFi connection
        if (WiFi.status() != WL_CONNECTED) {
            reportError("WiFi connection lost");
            // Attempt to reconnect
            WiFi.reconnect();
        }
        
        // Print debug information
        if (Serial) {
            Serial.printf("Status: RPM=%.1f, Cycle=%u, Errors=%lu, Running=%d\n",
                        workingPacket.camRpm,
                        workingPacket.sequentialCycle,
                        errorCount,
                        workingPacket.engineRunning);
                        
            // Print current cylinder data
            if (engineCycleStarted && currentCylinder < NUM_CYLINDERS) {
                Serial.printf("Cylinder %u: Ign=%lu°, Inj=%lu°\n",
                            currentCylinder + 1,
                            workingPacket.cylinder[currentCylinder].ignitionStartAngle,
                            workingPacket.cylinder[currentCylinder].injectionStartAngle);
            }
        }
    }
    
    // Check for engine timeout
    checkEngineTimeout();
    
    // Small delay to prevent overwhelming the processor
    delay(1);
}

// Cleanup function (called on system reset)
void cleanup() {
    // Detach all interrupts
    detachInterrupt(digitalPinToInterrupt(CAM_SENSOR_PIN));
    detachInterrupt(digitalPinToInterrupt(IGNITION_PIN));
    detachInterrupt(digitalPinToInterrupt(INJECTION_PIN_1));
    detachInterrupt(digitalPinToInterrupt(INJECTION_PIN_2));
    detachInterrupt(digitalPinToInterrupt(INJECTION_PIN_3));
    detachInterrupt(digitalPinToInterrupt(INJECTION_PIN_4));
    
    // Close UDP connection
    udp.close();
    
    // Disable WiFi
    WiFi.disconnect(true);
    
    Serial.println("System Shutdown Complete");
}
