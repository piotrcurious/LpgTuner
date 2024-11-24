#include <Arduino.h>
#include <ESPAsyncUDP.h>
#include <WiFi.h>

#define TRANSLATION_TIMEBASE 240000000 // ESP32 clock rate in Hz

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
#define LAMBDA_PIN A0
#define MAP_SENSOR_PIN A1

// Constants
#define NUM_CYLINDERS 4
#define CAM_PULSES_PER_REV 4 // Camshaft pulses per engine cycle
#define SHORT_PULSE_THRESHOLD 0.65 // Percentage of a normal pulse length
#define NUM_LAMBDA_SAMPLES 8
#define ENGINE_CYCLE_DEGREES 720 // Two crankshaft revolutions
#define LAMBDA_INTERVAL (ENGINE_CYCLE_DEGREES / NUM_LAMBDA_SAMPLES)

// Struct Definitions
#pragma pack(push, 1)
struct UnifiedPacket {
    float rpm;
    uint32_t camPulseLengths[CAM_PULSES_PER_REV]; // Cam pulse lengths
    bool isCycleStart; // True if this pulse marks the start of a new engine cycle
    float lambdaSamples[NUM_LAMBDA_SAMPLES];
};
#pragma pack(pop)

volatile UnifiedPacket unifiedPacket;

// Variables for RPM and Camshaft
volatile uint32_t lastRisingEdge = 0;
volatile uint32_t camPulseStart = 0;
volatile uint32_t camPulseLength = 0;
volatile uint32_t risingEdgeInterval = 0;
volatile bool isShortPulseDetected = false;
volatile uint8_t camPulseIndex = 0;

// Lambda Sampling
volatile uint32_t nextLambdaSampleTime = 0;
volatile uint8_t currentLambdaSample = 0;

// ISR: Camshaft Sensor
void IRAM_ATTR camISR() {
    uint32_t currentTime = getCycleCount();

    if (fastDigitalRead(CAM_SENSOR_PIN)) { // Rising edge
        if (lastRisingEdge != 0) {
            risingEdgeInterval = currentTime - lastRisingEdge;
            unifiedPacket.rpm = (60.0f * TRANSLATION_TIMEBASE) / (risingEdgeInterval * CAM_PULSES_PER_REV);
        }

        lastRisingEdge = currentTime;
        camPulseStart = currentTime;
    } else { // Falling edge
        camPulseLength = currentTime - camPulseStart;
        unifiedPacket.camPulseLengths[camPulseIndex] = camPulseLength;

        // Detect short pulse to mark engine cycle start
        isShortPulseDetected = (camPulseLength <= (SHORT_PULSE_THRESHOLD * risingEdgeInterval));
        unifiedPacket.isCycleStart = isShortPulseDetected;

        // Update pulse index
        camPulseIndex = (camPulseIndex + 1) % CAM_PULSES_PER_REV;

        // Reset lambda sampling for the new cycle
        if (isShortPulseDetected) {
            nextLambdaSampleTime = camPulseStart + (risingEdgeInterval / NUM_LAMBDA_SAMPLES);
            currentLambdaSample = 0;
        }
    }
}

// Lambda Sampling
void sampleLambda() {
    uint32_t currentTime = getCycleCount();
    if (currentLambdaSample < NUM_LAMBDA_SAMPLES && currentTime >= nextLambdaSampleTime) {
        unifiedPacket.lambdaSamples[currentLambdaSample] = analogRead(LAMBDA_PIN) * (5.0 / 1023.0); // Convert to voltage
        currentLambdaSample++;
        nextLambdaSampleTime += risingEdgeInterval / NUM_LAMBDA_SAMPLES;
    }
}

// Send Unified Packet
void sendUnifiedPacket() {
    udp.writeTo((uint8_t*)&unifiedPacket, sizeof(UnifiedPacket), IPAddress(239, 255, 0, 1), multicastPort);
}

// Process Data
void processData() {
    sampleLambda();

    // Send data after lambda sampling is complete
    if (currentLambdaSample >= NUM_LAMBDA_SAMPLES) {
        sendUnifiedPacket();
    }
}

void setup() {
    pinMode(CAM_SENSOR_PIN, INPUT);
    pinMode(LAMBDA_PIN, INPUT);

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

    Serial.begin(115200);
    Serial.println("System Initialized");
}

void loop() {
    processData();
    delay(1); // Frequent updates
}
