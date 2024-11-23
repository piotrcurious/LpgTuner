#include <Arduino.h>

// Platform-specific configurations
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

// Pin Definitions
#define CRANK_PIN 4
#define IGNITION_PIN 5
#define INJECTION_PIN 6
#define LAMBDA_PIN A0 // Analog input for lambda sensor

// Constants
#define MAX_EVENTS 16
#define NUM_LAMBDA_SAMPLES 8

// Data Structures
struct EventBuffer {
    uint32_t risingEdgeTime[MAX_EVENTS];
    uint32_t pulseLength[MAX_EVENTS];
    float rpm[MAX_EVENTS];
    bool sequentialCycle[MAX_EVENTS];
    uint8_t index;
    bool ready;
};

struct IgnitionData {
    uint32_t dwellStart[MAX_EVENTS];
    uint32_t dwellDuration[MAX_EVENTS];
    uint32_t angle[MAX_EVENTS];
    uint8_t index;
};

struct InjectionData {
    uint32_t startAngle[MAX_EVENTS];
    uint32_t duration[MAX_EVENTS];
    uint8_t index;
};

struct LambdaData {
    float samples[NUM_LAMBDA_SAMPLES];
    uint8_t currentSample;
    bool ready;
};

// Buffers and State Variables
volatile EventBuffer crankBuffer = {};
volatile IgnitionData ignitionBuffer = {};
volatile InjectionData injectionBuffer = {};
volatile LambdaData lambdaBuffer = {};

volatile uint32_t lastCrankEdge = 0;
volatile uint32_t crankPulseLength = 0;

volatile uint32_t lambdaSampleInterval = 0;
volatile uint32_t nextLambdaSampleTime = 0;

// ISR: Crank Signal
void IRAM_ATTR crankISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = crankBuffer.index;

    if (fastDigitalRead(CRANK_PIN)) { // Rising edge
        uint32_t pulseLength = currentTime - lastCrankEdge;
        if (pulseLength > 0) {
            crankBuffer.risingEdgeTime[idx] = currentTime;
            crankBuffer.pulseLength[idx] = pulseLength;
            crankBuffer.rpm[idx] = (60.0f * TRANSLATION_TIMEBASE) / pulseLength;
            crankBuffer.sequentialCycle[idx] = (pulseLength < crankPulseLength / 2);

            lastCrankEdge = currentTime;
            crankPulseLength = pulseLength;

            // Setup lambda sampling
            lambdaSampleInterval = crankPulseLength / NUM_LAMBDA_SAMPLES;
            nextLambdaSampleTime = lastCrankEdge + lambdaSampleInterval;
            lambdaBuffer.currentSample = 0;

            crankBuffer.index = (idx + 1) % MAX_EVENTS;
            crankBuffer.ready = true;
        }
    }
}

// ISR: Ignition Signal
void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = ignitionBuffer.index;

    if (fastDigitalRead(IGNITION_PIN)) { // Rising edge
        ignitionBuffer.dwellStart[idx] = currentTime - lastCrankEdge;
    } else { // Falling edge
        ignitionBuffer.dwellDuration[idx] = currentTime - ignitionBuffer.dwellStart[idx];
        ignitionBuffer.angle[idx] = (360.0 * ignitionBuffer.dwellStart[idx]) / crankPulseLength;
        ignitionBuffer.index = (idx + 1) % MAX_EVENTS;
    }
}

// ISR: Injection Signal
void IRAM_ATTR injectionISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = injectionBuffer.index;

    if (fastDigitalRead(INJECTION_PIN)) { // Rising edge
        injectionBuffer.startAngle[idx] = (360.0 * (currentTime - lastCrankEdge)) / crankPulseLength;
    } else { // Falling edge
        injectionBuffer.duration[idx] = currentTime - lastCrankEdge;
        injectionBuffer.index = (idx + 1) % MAX_EVENTS;
    }
}

// Lambda Sampling (Triggered in Main Loop)
void sampleLambda() {
    uint32_t currentTime = getCycleCount();
    if (lambdaBuffer.currentSample < NUM_LAMBDA_SAMPLES && currentTime >= nextLambdaSampleTime) {
        lambdaBuffer.samples[lambdaBuffer.currentSample] = analogRead(LAMBDA_PIN) * (5.0 / 1023.0);
        lambdaBuffer.currentSample++;
        nextLambdaSampleTime += lambdaSampleInterval;

        if (lambdaBuffer.currentSample >= NUM_LAMBDA_SAMPLES) {
            lambdaBuffer.ready = true;
        }
    }
}

// Processing Routines
void processCrankData() {
    if (crankBuffer.ready) {
        // Process RPM, sequential detection, etc.
        crankBuffer.ready = false;
    }
}

void processLambdaData() {
    if (lambdaBuffer.ready) {
        // Process Lambda sensor data
        lambdaBuffer.ready = false;

        Serial.println("Lambda Sensor Readings:");
        for (uint8_t i = 0; i < NUM_LAMBDA_SAMPLES; i++) {
            Serial.print("Sample ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(lambdaBuffer.samples[i], 3);
        }
    }
}

void setup() {
    // Configure Pins
    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(INJECTION_PIN, INPUT);
    pinMode(LAMBDA_PIN, INPUT);

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN), ignitionISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN), injectionISR, CHANGE);

    Serial.begin(115200);
    Serial.println("System Initialized");
}

void loop() {
    sampleLambda();
    processCrankData();
    processLambdaData();
    yield(); // Avoid blocking
}
