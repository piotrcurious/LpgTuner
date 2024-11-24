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
volatile uint32_t lastCamEdge = 0;
volatile uint32_t camPulseStart = 0;
volatile uint32_t camPulseLength = 0;
volatile uint8_t currentCylinder = 0;
volatile bool isShortPulseDetected = false;
const float shortPulseToleranceFactor = 0.65; // Half-length tolerance range

// Lambda Sampling
volatile uint32_t nextLambdaSampleTime = 0;
volatile uint8_t currentLambdaSample = 0;

// MAP Sampling
volatile uint16_t mapSampleIndex = 0;

// ISR: Camshaft Sensor
void IRAM_ATTR camISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(CAM_SENSOR_PIN)) { // Rising edge
        camPulseStart = currentTime; // Record the start of the pulse
    } else { // Falling edge
        camPulseLength = currentTime - camPulseStart;

        // Detect short pulse based on a tolerance window
        isShortPulseDetected = (camPulseLength <= (shortPulseToleranceFactor * camPulseLength));

        // Calculate RPM from cam pulses (4 pulses per cam revolution)
        unifiedPacket.camRpm = (60.0f * TRANSLATION_TIMEBASE) / (camPulseLength * CAM_PULSES_PER_CYCLE);

        // Synchronize to cylinder 1 on the short pulse
        if (isShortPulseDetected) {
            currentCylinder = 0; // Reset to cylinder 1
        } else {
            currentCylinder = (currentCylinder + 1) % NUM_CYLINDERS;
        }

        // Reset sampling states for the new cylinder
        nextLambdaSampleTime = camPulseStart + (camPulseLength / NUM_LAMBDA_SAMPLES);
        currentLambdaSample = 0;
        mapSampleIndex = 0;
    }
}

// ISR: Ignition
void IRAM_ATTR ignitionISR() {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(IGNITION_PIN)) { // Rising edge
        unifiedPacket.cylinder[currentCylinder].ignitionStartAngle =
            (360.0 * (currentTime - lastCamEdge)) / camPulseLength;
    } else { // Falling edge
        unifiedPacket.cylinder[currentCylinder].ignitionDuration =
            currentTime - lastCamEdge;
    }
}

// ISR: Injection for Cylinder
void IRAM_ATTR injectionISR(uint8_t injectorIndex, uint8_t injectorPin) {
    uint32_t currentTime = getCycleCount();
    if (fastDigitalRead(injectorPin)) { // Rising edge
        unifiedPacket.cylinder[injectorIndex].injectionStartAngle =
            (360.0 * (currentTime - camPulseStart)) / camPulseLength;
    } else { // Falling edge
        unifiedPacket.cylinder[injectorIndex].injectionDuration =
            currentTime - camPulseStart;
    }
}

// Lambda Sampling
void sampleLambda() {
    uint32_t currentTime = getCycleCount();
    if (currentLambdaSample < NUM_LAMBDA_SAMPLES && currentTime >= nextLambdaSampleTime) {
        unifiedPacket.cylinder[currentCylinder].lambdaSamples[currentLambdaSample] =
            analogRead(LAMBDA_PIN) * (5.0 / 1023.0);
        currentLambdaSample++;
        nextLambdaSampleTime += camPulseLength / NUM_LAMBDA_SAMPLES;
    }
}

// MAP Sampling
void sampleMAP() {
    uint32_t currentTime = getCycleCount();
    uint32_t sampleTime = camPulseStart + mapSampleIndex * (camPulseLength / TOTAL_MAP_SAMPLES);
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
    pinMode(CAM_SENSOR_PIN, INPUT);
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

    attachInterrupt(digitalPinToInterrupt(CAM_SENSOR_PIN), camISR, CHANGE);
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
