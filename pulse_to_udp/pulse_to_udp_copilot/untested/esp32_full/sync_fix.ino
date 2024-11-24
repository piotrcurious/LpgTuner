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
#define CAM_SENSOR_PIN 4
#define IGNITION_PIN 5
#define INJECTION_PIN_1 6
#define INJECTION_PIN_2 7
#define INJECTION_PIN_3 8
#define INJECTION_PIN_4 9
#define LAMBDA_PIN A0
#define MAP_SENSOR_PIN A1

// Constants
#define NUM_CYLINDERS 4
#define CAM_PULSES_PER_CYCLE 4
#define ENGINE_CYCLE_DEGREES 720 // Two crankshaft revolutions
#define NUM_LAMBDA_SAMPLES 8
#define MAP_SAMPLES_PER_DEGREE 1
#define TOTAL_MAP_SAMPLES (ENGINE_CYCLE_DEGREES * MAP_SAMPLES_PER_DEGREE / NUM_CYLINDERS)
#define LAMBDA_INTERVAL (ENGINE_CYCLE_DEGREES / NUM_LAMBDA_SAMPLES)

// Struct Definitions
#pragma pack(push, 1)
struct UnifiedPacket {
    float camRpm;
    uint8_t sequentialCycle;

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

volatile UnifiedPacket unifiedPacket;

// Variables for Camshaft
volatile uint32_t lastRisingEdge = 0;  // For RPM calculation
volatile uint32_t currentRisingEdge = 0;  // For RPM calculation
volatile uint32_t camPulseStart = 0;  // Rising edge timestamp
volatile uint32_t camPulseEnd = 0;    // Falling edge timestamp
volatile uint32_t pulseWidth = 0;     // Duration between rising and falling edge
volatile uint8_t currentCylinder = 0;
volatile bool engineCycleStarted = false;

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
    return (60.0f * TRANSLATION_TIMEBASE) / (float)(averageTime * CAM_PULSES_PER_CYCLE);
}

// ISR: Camshaft Sensor
void IRAM_ATTR camISR() {
    uint32_t currentTime = getCycleCount();
    
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
        
        // Check if this is a short pulse (indicating start of engine cycle)
        bool isShortPulse = (pulseWidth > (EXPECTED_SHORT_PULSE_WIDTH * (1 - PULSE_WIDTH_TOLERANCE))) && 
                           (pulseWidth < (EXPECTED_SHORT_PULSE_WIDTH * (1 + PULSE_WIDTH_TOLERANCE)));
        
        if (isShortPulse) {
            currentCylinder = 0;  // Reset to first cylinder
            engineCycleStarted = true;
            unifiedPacket.sequentialCycle++;  // Increment cycle counter
        } else if (engineCycleStarted) {
            currentCylinder = (currentCylinder + 1) % NUM_CYLINDERS;
        }
        
        // Calculate and update RPM
        if (lastRisingEdge > 0) {
            unifiedPacket.camRpm = calculateAverageRPM();
        }
        
        // Reset sampling states for the new cylinder
        nextLambdaSampleTime = currentTime + (pulseWidth / NUM_LAMBDA_SAMPLES);
        currentLambdaSample = 0;
        mapSampleIndex = 0;
    }
}

// Rest of the code remains the same, including ISRs for ignition and injection,
// sampling functions, and setup/loop functions...
