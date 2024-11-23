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

// Buffer and constants
#define MAX_EVENTS 16

// Data structures for crank, ignition, and injection
struct CrankData {
    uint32_t risingEdgeTime[MAX_EVENTS];
    uint32_t pulseLength[MAX_EVENTS]; // Rising-to-falling edge intervals
    float rpm[MAX_EVENTS];
    uint8_t index;
    bool sequentialCycleStart[MAX_EVENTS]; // Flag for half-length pulse detection
};

struct IgnitionData {
    uint32_t risingEdgeDwell[MAX_EVENTS];
    uint32_t fallingEdgeAngle[MAX_EVENTS];
    uint8_t index;
};

struct InjectionData {
    uint32_t risingEdgeAngle[MAX_EVENTS];
    uint32_t pulseLength[MAX_EVENTS];
    uint8_t index;
};

volatile CrankData crankData = {};
volatile IgnitionData ignitionData = {};
volatile InjectionData injectionData = {};

// Flags
volatile bool crankDataReady = false;
volatile bool ignitionDataReady = false;
volatile bool injectionDataReady = false;

// Last known times and pulse length
volatile uint32_t lastCrankRisingEdge = 0;
volatile uint32_t lastCrankFallingEdge = 0;
volatile uint32_t lastIgnitionRisingEdge = 0;
volatile uint32_t lastInjectionRisingEdge = 0;

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
            lastCrankRisingEdge = currentTime;
            crankDataReady = true;
        }
    } else {
        // Falling edge: record pulse length and detect half-length pulse
        uint32_t pulseLength = currentTime - lastCrankRisingEdge;
        crankData.pulseLength[idx] = pulseLength;

        // Check if this is a half-length pulse
        if (idx > 0 && pulseLength < (crankData.pulseLength[(idx - 1) % MAX_EVENTS] / 2)) {
            crankData.sequentialCycleStart[idx] = true; // Flag for sequential cycle start
        } else {
            crankData.sequentialCycleStart[idx] = false;
        }

        crankData.index = (idx + 1) % MAX_EVENTS;
        lastCrankFallingEdge = currentTime;
    }
}

void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = ignitionData.index;

    if (fastDigitalRead(IGNITION_PIN)) {
        // Rising edge: record dwell time
        ignitionData.risingEdgeDwell[idx] = currentTime - lastCrankRisingEdge;
        lastIgnitionRisingEdge = currentTime;
    } else {
        // Falling edge: record ignition angle
        ignitionData.fallingEdgeAngle[idx] = currentTime - lastCrankRisingEdge;
        ignitionData.index = (idx + 1) % MAX_EVENTS;
        ignitionDataReady = true;
    }
}

void IRAM_ATTR injectionISR() {
    uint32_t currentTime = getCycleCount();
    uint8_t idx = injectionData.index;

    if (fastDigitalRead(INJECTION_PIN)) {
        // Rising edge: record injection angle
        injectionData.risingEdgeAngle[idx] = currentTime - lastCrankRisingEdge;
        lastInjectionRisingEdge = currentTime;
    } else {
        // Falling edge: record pulse length
        injectionData.pulseLength[idx] = currentTime - lastInjectionRisingEdge;
        injectionData.index = (idx + 1) % MAX_EVENTS;
        injectionDataReady = true;
    }
}

// Processing routines
void processCrankData() {
    if (crankDataReady) {
        // Handle crank RPM and sequential cycle detection
        crankDataReady = false;
    }
}

void processIgnitionData() {
    if (ignitionDataReady) {
        // Handle ignition dwell/angle data processing
        ignitionDataReady = false;
    }
}

void processInjectionData() {
    if (injectionDataReady) {
        // Handle injection angle/pulse length data processing
        injectionDataReady = false;
    }
}

void setup() {
    // Configure signal pins
    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(INJECTION_PIN, INPUT);

    // Attach ISRs
    attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN), ignitionISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN), injectionISR, CHANGE);
}

void loop() {
    processCrankData();
    processIgnitionData();
    processInjectionData();
    yield(); // Avoid blocking other tasks
}
