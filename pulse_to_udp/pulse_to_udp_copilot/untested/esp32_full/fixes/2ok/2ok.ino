#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include "driver/dedic_gpio.h"
#include "driver/gpio.h"

#include "soc/gpio_struct.h"

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
const uint8_t CAM_SENSOR_PIN  = 34;
const uint8_t IGNITION_PIN    = 35;
const uint8_t INJECTION_PIN_1 = 32;
const uint8_t INJECTION_PIN_2 = 33;
const uint8_t INJECTION_PIN_3 = 27;
const uint8_t INJECTION_PIN_4 = 2;
const uint8_t LAMBDA_PIN      = 36 ;
const uint8_t MAP_SENSOR_PIN  = 39 ;

// Constants
const uint8_t NUM_CYLINDERS = 4;
const uint8_t CAM_PULSES_PER_CYCLE = 4;
const uint16_t ENGINE_CYCLE_DEGREES = 720;
const uint8_t NUM_LAMBDA_SAMPLES = 8;
const uint8_t MAP_SAMPLES_PER_DEGREE = 1;
const uint16_t TOTAL_MAP_SAMPLES = (ENGINE_CYCLE_DEGREES * MAP_SAMPLES_PER_DEGREE / NUM_CYLINDERS);
const uint16_t LAMBDA_INTERVAL = (ENGINE_CYCLE_DEGREES / NUM_LAMBDA_SAMPLES);

// Watchdog timeout values
const uint32_t CAM_SIGNAL_TIMEOUT = 500;
const uint32_t MINIMUM_RPM = 100;
const uint32_t MAXIMUM_RPM = 15000;

// Struct Definitions
#pragma pack(push, 1)
struct UnifiedPacket {
    float camRpm;
    uint8_t sequentialCycle;
    uint32_t timestamp;
    bool engineRunning;

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
volatile uint32_t lastRisingEdge = 0;
volatile uint32_t currentRisingEdge = 0;
volatile uint32_t camPulseStart = 0;
volatile uint32_t camPulseEnd = 0;
volatile uint32_t pulseWidth = 0;
volatile uint8_t currentCylinder = 0;
volatile bool engineCycleStarted = false;
volatile uint32_t lastCamSignalTime = 0;

// Constants for pulse width detection
const uint32_t EXPECTED_SHORT_PULSE_WIDTH = TRANSLATION_TIMEBASE / 1000;
const float PULSE_WIDTH_TOLERANCE = 0.3;

// Lambda Sampling
volatile uint32_t nextLambdaSampleTime = 0;
volatile uint8_t currentLambdaSample = 0;

// MAP Sampling
volatile uint16_t mapSampleIndex = 0;

// RPM calculation buffer
const uint8_t RPM_BUFFER_SIZE = 4;
volatile uint32_t risingEdgeTimes[RPM_BUFFER_SIZE];
volatile uint8_t rpmBufferIndex = 0;

// Function declarations
void checkEngineTimeout();
float calculateAverageRPM();
void sampleLambda();
void sampleMAP();
void sendUnifiedPacket();
void processData();

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
    
    uint32_t averageTime = totalTime / validSamples;
    float rpm = (60.0f * TRANSLATION_TIMEBASE) / (float)(averageTime * CAM_PULSES_PER_CYCLE);
    
    return (rpm >= MINIMUM_RPM && rpm <= MAXIMUM_RPM) ? rpm : workingPacket.camRpm;
}

// ISR: Camshaft Sensor
void IRAM_ATTR camISR() {
    uint32_t currentTime = getCycleCount();
    lastCamSignalTime = millis();
    
    if (fastDigitalRead(CAM_SENSOR_PIN)) {
        lastRisingEdge = currentRisingEdge;
        currentRisingEdge = currentTime;
        
        risingEdgeTimes[rpmBufferIndex] = currentTime;
        rpmBufferIndex = (rpmBufferIndex + 1) % RPM_BUFFER_SIZE;
        
        camPulseStart = currentTime;
        
    } else {
        camPulseEnd = currentTime;
        pulseWidth = camPulseEnd - camPulseStart;
        
        if (pulseWidth == 0 || pulseWidth > TRANSLATION_TIMEBASE) return;
        
        bool isShortPulse = (pulseWidth > (EXPECTED_SHORT_PULSE_WIDTH * (1 - PULSE_WIDTH_TOLERANCE))) && 
                           (pulseWidth < (EXPECTED_SHORT_PULSE_WIDTH * (1 + PULSE_WIDTH_TOLERANCE)));
        
        if (isShortPulse) {
            currentCylinder = 0;
            engineCycleStarted = true;
            workingPacket.sequentialCycle++;
        } else if (engineCycleStarted) {
            currentCylinder = (currentCylinder + 1) % NUM_CYLINDERS;
        }
        
        if (lastRisingEdge > 0) {
            float newRpm = calculateAverageRPM();
            if (newRpm > 0) {
                workingPacket.camRpm = newRpm;
            }
        }
        
        nextLambdaSampleTime = currentTime + (pulseWidth / NUM_LAMBDA_SAMPLES);
        currentLambdaSample = 0;
        mapSampleIndex = 0;
    }
}

// ISR: Ignition timing
void IRAM_ATTR ignitionISR() {
    if (!engineCycleStarted) return;
    
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(IGNITION_PIN)) {
        workingPacket.cylinder[currentCylinder].ignitionStartAngle =
            (uint32_t)(360.0f * (float)(currentTime - camPulseStart) / (float)(pulseWidth));
    } else {
        workingPacket.cylinder[currentCylinder].ignitionDuration = currentTime - camPulseStart;
    }
}

// ISR: Injection timing
void IRAM_ATTR injectionISR(uint8_t injectorIndex, uint8_t injectorPin) {
    if (!engineCycleStarted) return;
    
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(injectorPin)) {
        workingPacket.cylinder[injectorIndex].injectionStartAngle =
            (uint32_t)(360.0f * (float)(currentTime - camPulseStart) / (float)(pulseWidth));
    } else {
        workingPacket.cylinder[injectorIndex].injectionDuration = currentTime - camPulseStart;
    }
}

// Lambda Sampling
void sampleLambda() {
    if (!engineCycleStarted || currentCylinder >= NUM_CYLINDERS || 
        currentLambdaSample >= NUM_LAMBDA_SAMPLES) return;
        
    uint32_t currentTime = getCycleCount();
    if (currentTime >= nextLambdaSampleTime) {
        float voltage = (analogRead(LAMBDA_PIN) * 5.0f) / 1023.0f;
        workingPacket.cylinder[currentCylinder].lambdaSamples[currentLambdaSample] = voltage;
        currentLambdaSample++;
        nextLambdaSampleTime += pulseWidth / NUM_LAMBDA_SAMPLES;
    }
}

// MAP Sampling
void sampleMAP() {
    if (!engineCycleStarted || currentCylinder >= NUM_CYLINDERS || 
        mapSampleIndex >= TOTAL_MAP_SAMPLES) return;
        
    uint32_t currentTime = getCycleCount();
    uint32_t sampleTime = camPulseStart + mapSampleIndex * (pulseWidth / TOTAL_MAP_SAMPLES);
    
    if (currentTime >= sampleTime) {
        workingPacket.cylinder[currentCylinder].mapSamples[mapSampleIndex] = analogRead(MAP_SENSOR_PIN);
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

// Send Unified Packet
void sendUnifiedPacket() {
    checkEngineTimeout();
    workingPacket.timestamp = millis();
    
    noInterrupts();
    memcpy((void*)&transmitPacket, &workingPacket, sizeof(UnifiedPacket));
    interrupts();
    
//    if (WiFi.status() == WL_CONNECTED) {
        udp.writeTo((uint8_t*)&transmitPacket, sizeof(UnifiedPacket), 
                   IPAddress(239, 255, 0, 1), multicastPort);
//    }
}

// Process Data
void processData() {
    sampleLambda();
    sampleMAP();

    if (mapSampleIndex >= TOTAL_MAP_SAMPLES) {
        sendUnifiedPacket();
    }
}

void setup() {
    pinMode(CAM_SENSOR_PIN,   INPUT_PULLUP);
    pinMode(IGNITION_PIN,     INPUT_PULLUP);
    pinMode(INJECTION_PIN_1,  INPUT_PULLUP);
    pinMode(INJECTION_PIN_2,  INPUT_PULLUP);
    pinMode(INJECTION_PIN_3,  INPUT_PULLUP);
    pinMode(INJECTION_PIN_4,  INPUT_PULLUP);
    pinMode(LAMBDA_PIN,       INPUT);
    pinMode(MAP_SENSOR_PIN,   INPUT);



    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin("SSID", "PASSWORD");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    if (udp.connect(IPAddress(239, 255, 0, 1), multicastPort)) {
//        udp.listen_multicast(IPAddress(239, 255, 0, 1), multicastPort);
    }

    memset(&workingPacket, 0, sizeof(UnifiedPacket));
    memset((void*)&transmitPacket, 0, sizeof(UnifiedPacket));

    attachInterrupt(digitalPinToInterrupt(CAM_SENSOR_PIN) , camISR      , CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN)   , ignitionISR , CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_1), 
                   []() { injectionISR(0, INJECTION_PIN_1); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_2), 
                   []() { injectionISR(1, INJECTION_PIN_2); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_3), 
                   []() { injectionISR(2, INJECTION_PIN_3); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_4), 
                   []() { injectionISR(3, INJECTION_PIN_4); }, CHANGE);

    lastCamSignalTime = millis();
    
    for (uint8_t i = 0; i < RPM_BUFFER_SIZE; i++) {
        risingEdgeTimes[i] = 0;
    }
}

void loop() {
    processData();
    checkEngineTimeout();
    yield();
//    delay(1);
}
