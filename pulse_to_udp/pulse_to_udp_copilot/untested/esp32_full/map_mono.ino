#include <Arduino.h>
#include <ESPAsyncUDP.h>
#include <WiFi.h>

// ESP32-Specific Configurations
#include "esp32/rom/ets_sys.h"
#define TRANSLATION_TIMEBASE 240000000 // ESP32 clock rate

inline uint32_t getCycleCount() {
    uint32_t count;
    asm volatile("rsr %0,ccount" : "=a"(count));
    return count;
}

inline uint8_t fastDigitalRead(uint8_t pin) {
    return (GPIO.in >> pin) & 0x1;
}

// AsyncUDP Configuration
AsyncUDP udp;
const char* multicastIP = "239.255.0.1";
const uint16_t multicastPort = 12345;

// Pin Definitions
#define CRANK_PIN 4
#define IGNITION_PIN 5
#define INJECTION_PIN 6
#define LAMBDA_PIN A0
#define MAP_SENSOR_PIN A1

// Constants
#define NUM_LAMBDA_SAMPLES 8
#define ENGINE_CYCLE_DEGREES 720 // Two revolutions of crankshaft
#define MAP_SAMPLES_PER_DEGREE 1
#define TOTAL_MAP_SAMPLES (ENGINE_CYCLE_DEGREES * MAP_SAMPLES_PER_DEGREE)

// Unified Data Structure
#pragma pack(push, 1)
struct UnifiedPacket {
    // Crank Data
    uint32_t crankTimestamp;
    float crankRpm;
    uint8_t sequentialCycle;

    // Ignition Data
    uint32_t ignitionDwellStart;
    uint32_t ignitionDwellDuration;
    uint32_t ignitionAngle;

    // Injection Data
    uint32_t injectionStartAngle;
    uint32_t injectionDuration;

    // Lambda Data
    float lambdaSamples[NUM_LAMBDA_SAMPLES];

    // MAP Data
    uint16_t mapSamples[TOTAL_MAP_SAMPLES];
};
#pragma pack(pop)

volatile UnifiedPacket unifiedPacket;

// Timing Variables
volatile uint32_t lastCrankEdge = 0;
volatile uint32_t crankPulseLength = 0;
volatile uint32_t crankStartTime = 0;

// Lambda Sampling
volatile uint32_t lambdaSampleInterval = 0;
volatile uint32_t nextLambdaSampleTime = 0;
volatile uint8_t currentLambdaSample = 0;

// MAP Sampling
volatile uint16_t currentMapSampleIndex = 0;
volatile uint32_t mapSampleInterval = 0;

// ISR: Crank Signal
void IRAM_ATTR crankISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(CRANK_PIN)) { // Rising edge
        uint32_t pulseLength = currentTime - lastCrankEdge;

        if (pulseLength > 0) {
            unifiedPacket.crankTimestamp = currentTime;
            unifiedPacket.crankRpm = (60.0f * TRANSLATION_TIMEBASE) / pulseLength;
            unifiedPacket.sequentialCycle = (pulseLength < crankPulseLength / 2);

            lastCrankEdge = currentTime;
            crankPulseLength = pulseLength;

            // Initialize MAP sampling
            mapSampleInterval = crankPulseLength / ENGINE_CYCLE_DEGREES;
            currentMapSampleIndex = 0;
            crankStartTime = currentTime;

            // Setup lambda sampling for this cycle
            lambdaSampleInterval = crankPulseLength / NUM_LAMBDA_SAMPLES;
            nextLambdaSampleTime = lastCrankEdge + lambdaSampleInterval;
            currentLambdaSample = 0;
        }
    }
}

// ISR: Ignition Signal
void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(IGNITION_PIN)) { // Rising edge
        unifiedPacket.ignitionDwellStart = currentTime - lastCrankEdge;
    } else { // Falling edge
        unifiedPacket.ignitionDwellDuration = currentTime - unifiedPacket.ignitionDwellStart;
        unifiedPacket.ignitionAngle = (360.0 * unifiedPacket.ignitionDwellStart) / crankPulseLength;
    }
}

// ISR: Injection Signal
void IRAM_ATTR injectionISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(INJECTION_PIN)) { // Rising edge
        unifiedPacket.injectionStartAngle = (360.0 * (currentTime - lastCrankEdge)) / crankPulseLength;
    } else { // Falling edge
        unifiedPacket.injectionDuration = currentTime - lastCrankEdge;
    }
}

// Lambda Sampling
void sampleLambda() {
    uint32_t currentTime = getCycleCount();
    if (currentLambdaSample < NUM_LAMBDA_SAMPLES && currentTime >= nextLambdaSampleTime) {
        unifiedPacket.lambdaSamples[currentLambdaSample] = analogRead(LAMBDA_PIN) * (5.0 / 1023.0);
        currentLambdaSample++;
        nextLambdaSampleTime += lambdaSampleInterval;
    }
}

// MAP Sampling (Non-Blocking)
void sampleMAP() {
    uint32_t currentTime = getCycleCount();
    if (currentMapSampleIndex < TOTAL_MAP_SAMPLES &&
        (currentTime - crankStartTime) >= (mapSampleInterval * currentMapSampleIndex)) {
        unifiedPacket.mapSamples[currentMapSampleIndex] = analogRead(MAP_SENSOR_PIN);
        currentMapSampleIndex++;
    }
}

// Send Unified UDP Packet
void sendUnifiedPacket() {
    udp.writeTo((uint8_t*)&unifiedPacket, sizeof(UnifiedPacket), IPAddress(239, 255, 0, 1), multicastPort);
}

// Processing Routine
void processData() {
    sampleLambda();
    sampleMAP();

    // Send packet when MAP sampling is complete
    if (currentMapSampleIndex >= TOTAL_MAP_SAMPLES) {
        sendUnifiedPacket();
    }
}

void setup() {
    // Configure Pins
    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(INJECTION_PIN, INPUT);
    pinMode(LAMBDA_PIN, INPUT);
    pinMode(MAP_SENSOR_PIN, INPUT);

    // WiFi Initialization
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin("SSID", "PASSWORD");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    // Multicast Setup
    if (udp.connect(IPAddress(239, 255, 0, 1), multicastPort)) {
        udp.listen_multicast(IPAddress(239, 255, 0, 1), multicastPort);
    }

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN), ignitionISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN), injectionISR, CHANGE);

    Serial.begin(115200);
    Serial.println("System Initialized");
}

void loop() {
    processData();
    delay(1); // Frequent data processing
}
