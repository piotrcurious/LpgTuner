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
    return GPIO.in >> pin & 0x1;
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

// Signal pin definitions
#define CRANK_PIN 4
#define IGNITION_PIN 5
#define INJECTION_PIN 6
#define LAMBDA_PIN A0 // Lambda sensor analog pin

// Buffer and constants
#define MAX_EVENTS 16
#define NUM_LAMBDA_SAMPLES 8 // 8 samples per exhaust stroke

// Data structures
struct CrankData {
    uint32_t risingEdgeTime[MAX_EVENTS];
    uint32_t pulseLength[MAX_EVENTS];
    float rpm[MAX_EVENTS];
    uint8_t index;
    bool sequentialCycleStart[MAX_EVENTS];
};

struct LambdaData {
    float samples[NUM_LAMBDA_SAMPLES]; // 8 samples per exhaust stroke
    uint8_t sampleIndex;
    bool ready;
};

volatile CrankData crankData = {};
volatile LambdaData lambdaData = {};

// Flags
volatile bool crankDataReady = false;
volatile bool lambdaSamplingEnabled = false;

// Last known times
volatile uint32_t lastCrankRisingEdge = 0;
volatile uint32_t lastCrankFallingEdge = 0;

// Variables for lambda sampling
volatile uint32_t crankInterval = 0;
volatile uint32_t lambdaSampleInterval = 0;
volatile uint8_t currentLambdaSample = 0;

// ISR Handlers
void IRAM_ATTR crankISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = crankData.index;

    if (fastDigitalRead(CRANK_PIN)) {
        // Rising edge: calculate RPM and record time
        uint32_t timeDiff = currentTime - lastCrankRisingEdge;
        if (timeDiff > 0) {
            crankData.risingEdgeTime[idx] = currentTime;
            crankData.rpm[idx] = (60.0f * TRANSLATION_TIMEBASE) / timeDiff;

            // Setup lambda sampling intervals
            crankInterval = timeDiff;
            lambdaSampleInterval = crankInterval / NUM_LAMBDA_SAMPLES;
            currentLambdaSample = 0;
            lambdaSamplingEnabled = true;

            lastCrankRisingEdge = currentTime;
            crankDataReady = true;
        }
    }
}

// Sampling routine
void sampleLambda() {
    if (!lambdaSamplingEnabled || currentLambdaSample >= NUM_LAMBDA_SAMPLES) {
        return;
    }

    uint32_t targetTime = lastCrankRisingEdge + (lambdaSampleInterval * currentLambdaSample);
    if (getCycleCount() >= targetTime) {
        lambdaData.samples[currentLambdaSample] = analogRead(LAMBDA_PIN) * (5.0 / 1023.0); // Convert ADC to voltage
        currentLambdaSample++;

        // Check if all samples are collected
        if (currentLambdaSample >= NUM_LAMBDA_SAMPLES) {
            lambdaSamplingEnabled = false;
            lambdaData.sampleIndex = currentLambdaSample;
            lambdaData.ready = true;
        }
    }
}

// Processing routines
void processCrankData() {
    if (crankDataReady) {
        // Handle crank RPM and sequential cycle detection
        crankDataReady = false;
    }
}

void processLambdaData() {
    if (lambdaData.ready) {
        // Handle lambda sensor data processing
        lambdaData.ready = false;

        // Example: Print lambda values
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
    // Configure signal pins
    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(INJECTION_PIN, INPUT);
    pinMode(LAMBDA_PIN, INPUT);

    // Attach ISRs
    attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankISR, CHANGE);

    // Initialize serial for debugging
    Serial.begin(115200);
}

void loop() {
    sampleLambda();       // Perform lambda sampling
    processCrankData();   // Process crankshaft data
    processLambdaData();  // Process lambda sensor data
    yield();              // Avoid blocking other tasks
}
