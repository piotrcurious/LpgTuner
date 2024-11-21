#include "wifi_settings.h"
#include <ESPAsyncUDP.h>

// Pin and signal settings
constexpr uint8_t PULSE_PIN = 4;
constexpr bool PULSE_LOGIC_POLARITY = HIGH;

// Buffer and calculation settings
constexpr uint8_t MAX_SIZE = 10; // Increased buffer size for robustness
constexpr uint32_t TRANSLATION_TIMEBASE = 240000000; // Timebase for RPM calculation

// Struct to store processed pulse data
#pragma pack(push, 1)
struct PulseData {
    float rpm[MAX_SIZE];         // RPM values
    uint32_t length[MAX_SIZE];   // Pulse lengths
    uint8_t index;               // Packet index
};
#pragma pack(pop)

// Global variables
PulseData pulseData = {};
AsyncUDP udp;

volatile uint32_t lastPulseTime = 0;
volatile uint32_t risingIntervalBuffer[MAX_SIZE] = {};
volatile uint32_t pulseLengthBuffer[MAX_SIZE] = {};
volatile uint8_t writeIndex = 0;
volatile uint8_t processIndex = 0;
volatile bool newDataAvailable = false; // Indicates new data needs processing
volatile bool captureComplete = false;  // Indicates buffer is full and ready to send

// Function declarations
void initializeWiFi();
void configureInterrupts();
void sendPacket();
void processData();
void IRAM_ATTR handlePulse();
uint32_t IRAM_ATTR getCycleCount();
void debugPrint(const char* message);

// Setup function
void setup() {
    Serial.begin(115200);

    pinMode(PULSE_PIN, INPUT);
    initializeWiFi();
    configureInterrupts();
}

// Loop function
void loop() {
    if (newDataAvailable) {
        processData();
    }

    if (captureComplete) {
        sendPacket();
        captureComplete = false; // Reset flag after sending
    }

    yield(); // Allow background tasks
}

// Interrupt Service Routine (ISR)
void IRAM_ATTR handlePulse() {
    uint32_t currentTime = getCycleCount();
    bool isRisingEdge = (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY);

    // Record rising edge intervals
    if (isRisingEdge) {
        uint8_t nextIndex = (writeIndex + 1) % MAX_SIZE;
        if (nextIndex != processIndex) { // Check for buffer overflow
            risingIntervalBuffer[writeIndex] = currentTime - lastPulseTime;
            lastPulseTime = currentTime;
            newDataAvailable = true;
        } else {
            // If buffer overflows, ignore new data
        }
    } else {
        // Record falling edge pulse length
        pulseLengthBuffer[writeIndex] = currentTime - lastPulseTime;
        writeIndex = (writeIndex + 1) % MAX_SIZE;

        if (writeIndex == processIndex) {
            captureComplete = true; // Buffer is full
        }
    }
}

// Process raw data incrementally outside ISR
void processData() {
    while (processIndex != writeIndex) {
        noInterrupts(); // Protect shared data access
        uint32_t interval = risingIntervalBuffer[processIndex];
        uint32_t length = pulseLengthBuffer[processIndex];
        interrupts();

        if (interval > 0) {
            pulseData.rpm[processIndex] = static_cast<float>(60 * TRANSLATION_TIMEBASE) / interval;
        } else {
            pulseData.rpm[processIndex] = 0; // Handle invalid intervals gracefully
        }

        pulseData.length[processIndex] = length;
        processIndex = (processIndex + 1) % MAX_SIZE;

        if (processIndex == writeIndex) {
            newDataAvailable = false; // Reset flag when processing is complete
        }
    }
}

// Send the processed data via UDP
void sendPacket() {
    if (udp.connect(multicastIP, multicastPort)) {
        udp.write(reinterpret_cast<uint8_t*>(&pulseData), sizeof(pulseData));
        Serial.println("Data sent successfully.");
        pulseData.index++;
        udp.close(); // Close connection if needed
    } else {
        Serial.println("UDP connection failed.");
    }
}

// Initialize WiFi
void initializeWiFi() {
#ifdef AP_mode_on
    WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
    Serial.println("AP Mode initialized.");
#else
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
    }
    Serial.println("\nSTA Mode connected.");
#endif
    esp_wifi_set_ps(WIFI_PS_NONE); // Disable WiFi power saving
}

// Configure interrupts
void configureInterrupts() {
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);
    Serial.println("Interrupts configured.");
}

// Helper function to get the current ESP32 cycle count
uint32_t IRAM_ATTR getCycleCount() {
    uint32_t cycleCount;
    asm volatile("esync; rsr %0,ccount" : "=a"(cycleCount));
    return cycleCount;
}

// Debug print helper
void debugPrint(const char* message) {
    Serial.println(message);
}
