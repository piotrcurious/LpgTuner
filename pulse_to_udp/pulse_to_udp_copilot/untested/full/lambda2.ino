#include "wifi_settings.h"
#include <Arduino.h>

#ifdef ESP32
#include "esp32/rom/ets_sys.h"
#define TRANSLATION_TIMEBASE 240000000
inline uint32_t getCycleCount() {
    uint32_t count;
    asm volatile("rsr %0,ccount" : "=a"(count));
    return count;
}
inline uint8_t fastDigitalRead(uint8_t pin) {
    return (GPIO.in >> pin) & 0x1;
}
#elif defined(ESP8266)
#include <eagle_soc.h>
#define TRANSLATION_TIMEBASE 80000000
inline uint32_t getCycleCount() {
    uint32_t count;
    asm volatile("rsr %0,ccount" : "=a"(count));
    return count;
}
inline uint8_t fastDigitalRead(uint8_t pin) {
    return (GPIP(pin) ? 1 : 0);
}
#else
#define TRANSLATION_TIMEBASE 16000000
#define getCycleCount() micros()
#define fastDigitalRead digitalRead
#endif

// Pin definitions
#define CRANK_PIN 4
#define IGNITION_PIN 5
#define INJECTION_PIN 6
#define LAMBDA_PIN A0 // Lambda sensor analog pin

// Constants and Buffers
#define MAX_EVENTS 16
#define NUM_LAMBDA_SAMPLES 8

// Data structures
struct CrankData {
    uint32_t risingEdgeTime[MAX_EVENTS];
    uint32_t pulseLength[MAX_EVENTS];
    float rpm[MAX_EVENTS];
    uint8_t index;
    bool sequentialCycleStart[MAX_EVENTS];
};

struct IgnitionData {
    uint32_t dwellTime[MAX_EVENTS]; // Time from crank rising edge to ignition rising edge
    uint32_t dwellDuration[MAX_EVENTS]; // Ignition pulse length
    uint32_t ignitionAngle[MAX_EVENTS];
};

struct InjectionData {
    uint32_t startAngle[MAX_EVENTS];
    uint32_t duration[MAX_EVENTS];
};

struct LambdaData {
    float samples[NUM_LAMBDA_SAMPLES]; // 8 samples during exhaust stroke
    uint8_t sampleIndex;
    bool ready;
};

volatile CrankData crankData = {};
volatile IgnitionData ignitionData = {};
volatile InjectionData injectionData = {};
volatile LambdaData lambdaData = {};

// Flags
volatile bool crankDataReady = false;
volatile bool lambdaSamplingEnabled = false;

// Timing variables
volatile uint32_t lastCrankRisingEdge = 0;
volatile uint32_t crankPulseLength = 0;
volatile uint32_t lambdaSampleInterval = 0;
volatile uint8_t currentLambdaSample = 0;

// ISR: Crank Signal
void IRAM_ATTR crankISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = crankData.index;

    if (fastDigitalRead(CRANK_PIN)) { // Rising edge
        uint32_t timeDiff = currentTime - lastCrankRisingEdge;
        if (timeDiff > 0) {
            crankData.risingEdgeTime[idx] = currentTime;
            crankData.rpm[idx] = (60.0f * TRANSLATION_TIMEBASE) / timeDiff;
            crankData.pulseLength[idx] = timeDiff;

            // Sequential cycle detection
            crankData.sequentialCycleStart[idx] = (timeDiff < crankPulseLength / 2);

            // Setup lambda sampling
            crankPulseLength = timeDiff;
            lambdaSampleInterval = crankPulseLength / NUM_LAMBDA_SAMPLES;
            currentLambdaSample = 0;
            lambdaSamplingEnabled = true;

            lastCrankRisingEdge = currentTime;
            crankDataReady = true;

            crankData.index = (idx + 1) % MAX_EVENTS;
        }
    }
}

// ISR: Ignition Signal
void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = ignitionData.index;

    if (fastDigitalRead(IGNITION_PIN)) { // Rising edge
        ignitionData.dwellTime[idx] = currentTime - lastCrankRisingEdge;
    } else { // Falling edge
        ignitionData.dwellDuration[idx] = currentTime - ignitionData.dwellTime[idx];
        ignitionData.ignitionAngle[idx] = (360.0 * ignitionData.dwellTime[idx]) / crankPulseLength;
        ignitionData.index = (idx + 1) % MAX_EVENTS;
    }
}

// ISR: Injection Signal
void IRAM_ATTR injectionISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = injectionData.index;

    if (fastDigitalRead(INJECTION_PIN)) { // Rising edge
        injectionData.startAngle[idx] = (360.0 * (currentTime - lastCrankRisingEdge)) / crankPulseLength;
    } else { // Falling edge
        injectionData.duration[idx] = currentTime - lastCrankRisingEdge;
        injectionData.index = (idx + 1) % MAX_EVENTS;
    }
}

// Lambda Sampling
void sampleLambda() {
    if (!lambdaSamplingEnabled || currentLambdaSample >= NUM_LAMBDA_SAMPLES) {
        return;
    }

    uint32_t targetTime = lastCrankRisingEdge + (lambdaSampleInterval * currentLambdaSample);
    if (getCycleCount() >= targetTime) {
        lambdaData.samples[currentLambdaSample] = analogRead(LAMBDA_PIN) * (5.0 / 1023.0);
        currentLambdaSample++;

        if (currentLambdaSample >= NUM_LAMBDA_SAMPLES) {
            lambdaSamplingEnabled = false;
            lambdaData.ready = true;
        }
    }
}

// Processing routines
void processCrankData() {
    if (crankDataReady) {
        // Handle crankshaft RPM and sequential cycle detection
        crankDataReady = false;
    }
}

void processLambdaData() {
    if (lambdaData.ready) {
        // Handle lambda sensor data
        lambdaData.ready = false;

        Serial.println("Lambda Sensor Readings:");
        for (uint8_t i = 0; i < NUM_LAMBDA_SAMPLES; i++) {
            Serial.print("Sample ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(lambdaData.samples[i], 3);
        }
    }
}

void setup() {
    // Configure pins
    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(INJECTION_PIN, INPUT);
    pinMode(LAMBDA_PIN, INPUT);

    // Attach ISRs
    attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN), ignitionISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN), injectionISR, CHANGE);

    Serial.begin(115200);
}

void loop() {
    sampleLambda();
    processCrankData();
    processLambdaData();
    yield(); // Avoid blocking
}
