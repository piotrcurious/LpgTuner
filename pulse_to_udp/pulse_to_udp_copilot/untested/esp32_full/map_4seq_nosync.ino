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
#define CRANK_PIN 4
#define IGNITION_PIN 5
#define INJECTION_PIN_1 6
#define INJECTION_PIN_2 7
#define INJECTION_PIN_3 8
#define INJECTION_PIN_4 9
#define LAMBDA_PIN A0
#define MAP_SENSOR_PIN A1

// Constants
#define NUM_CYLINDERS 4
#define ENGINE_CYCLE_DEGREES 720 // Two crankshaft revolutions
#define NUM_LAMBDA_SAMPLES 8
#define MAP_SAMPLES_PER_DEGREE 1
#define TOTAL_MAP_SAMPLES (ENGINE_CYCLE_DEGREES * MAP_SAMPLES_PER_DEGREE / NUM_CYLINDERS)
#define LAMBDA_INTERVAL (ENGINE_CYCLE_DEGREES / NUM_LAMBDA_SAMPLES)

// Struct Definitions
#pragma pack(push, 1)
struct UnifiedPacket {
    float crankRpm;
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

// Variables for Crankshaft
volatile uint32_t lastCrankEdge = 0;
volatile uint32_t crankPulseLength = 0;

// Cylinder Tracking
volatile uint8_t currentCylinder = 0; // Tracks cylinder in sequence (0-3)

// Lambda Sampling
volatile uint32_t nextLambdaSampleTime = 0;
volatile uint8_t currentLambdaSample = 0;

// MAP Sampling
volatile uint16_t mapSampleIndex = 0;

// ISR: Crankshaft
void IRAM_ATTR crankISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(CRANK_PIN)) { // Rising edge
        uint32_t pulseLength = currentTime - lastCrankEdge;

        if (pulseLength > 0) {
            crankPulseLength = pulseLength;
            lastCrankEdge = currentTime;
            unifiedPacket.crankRpm = (60.0f * TRANSLATION_TIMEBASE) / crankPulseLength;

            // Cylinder and sampling resets
            currentCylinder = (currentCylinder + 1) % NUM_CYLINDERS;
            nextLambdaSampleTime = lastCrankEdge + (crankPulseLength / NUM_LAMBDA_SAMPLES);
            currentLambdaSample = 0;
            mapSampleIndex = 0;
        }
    }
}

// ISR: Ignition
void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(IGNITION_PIN)) { // Rising edge
        unifiedPacket.cylinder[currentCylinder].ignitionStartAngle =
            (360.0 * (currentTime - lastCrankEdge)) / crankPulseLength;
    } else { // Falling edge
        unifiedPacket.cylinder[currentCylinder].ignitionDuration =
            currentTime - lastCrankEdge;
    }
}

// ISR: Injection for Cylinder
void IRAM_ATTR injectionISR(uint8_t injectorIndex, uint8_t injectorPin) {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(injectorPin)) { // Rising edge
        unifiedPacket.cylinder[injectorIndex].injectionStartAngle =
            (360.0 * (currentTime - lastCrankEdge)) / crankPulseLength;
    } else { // Falling edge
        unifiedPacket.cylinder[injectorIndex].injectionDuration =
            currentTime - lastCrankEdge;
    }
}

// Lambda Sampling
void sampleLambda() {
    uint32_t currentTime = getCycleCount();
    if (currentLambdaSample < NUM_LAMBDA_SAMPLES && currentTime >= nextLambdaSampleTime) {
        unifiedPacket.cylinder[currentCylinder].lambdaSamples[currentLambdaSample] =
            analogRead(LAMBDA_PIN) * (5.0 / 1023.0);
        currentLambdaSample++;
        nextLambdaSampleTime += crankPulseLength / NUM_LAMBDA_SAMPLES;
    }
}

// MAP Sampling
void sampleMAP() {
    uint32_t currentTime = getCycleCount();
    uint32_t sampleTime = lastCrankEdge + mapSampleIndex * (crankPulseLength / TOTAL_MAP_SAMPLES);
    if (mapSampleIndex < TOTAL_MAP_SAMPLES && currentTime >= sampleTime) {
        unifiedPacket.cylinder[currentCylinder].mapSamples[mapSampleIndex] =
            analogRead(MAP_SENSOR_PIN);
        mapSampleIndex++;
    }
}

// Send Unified Packet
void sendUnifiedPacket() {
    udp.writeTo((uint8_t*)&unifiedPacket, sizeof(UnifiedPacket), IPAddress(239, 255, 0, 1), multicastPort);
}

// Process Data
void processData() {
    sampleLambda();
    sampleMAP();

    // Send packet when MAP sampling for all cylinders is complete
    if (currentCylinder == NUM_CYLINDERS - 1 && mapSampleIndex >= TOTAL_MAP_SAMPLES) {
        sendUnifiedPacket();
    }
}

void setup() {
    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    pinMode(INJECTION_PIN_1, INPUT);
    pinMode(INJECTION_PIN_2, INPUT);
    pinMode(INJECTION_PIN_3, INPUT);
    pinMode(INJECTION_PIN_4, INPUT);
    pinMode(LAMBDA_PIN, INPUT);
    pinMode(MAP_SENSOR_PIN, INPUT);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin("SSID", "PASSWORD");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    if (udp.connect(IPAddress(239, 255, 0, 1), multicastPort)) {
        udp.listen_multicast(IPAddress(239, 255, 0, 1), multicastPort);
    }

    attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IGNITION_PIN), ignitionISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_1), []() { injectionISR(0, INJECTION_PIN_1); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_2), []() { injectionISR(1, INJECTION_PIN_2); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_3), []() { injectionISR(2, INJECTION_PIN_3); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INJECTION_PIN_4), []() { injectionISR(3, INJECTION_PIN_4); }, CHANGE);

    Serial.begin(115200);
    Serial.println("System Initialized");
}

void loop() {
    processData();
    delay(1); // Frequent updates
}
