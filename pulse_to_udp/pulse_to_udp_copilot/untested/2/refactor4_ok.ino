#include "wifi_settings.h"
#include <ESPAsyncUDP.h>

// Pin and signal settings
constexpr uint8_t PULSE_PIN = 4;
constexpr bool PULSE_LOGIC_POLARITY = HIGH;

// Buffer and calculation settings
constexpr uint8_t MAX_SIZE = 2;
constexpr uint32_t TRANSLATION_TIMEBASE = 240000000;

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
volatile uint8_t bufferIndex = 0;          // Tracks the position for capturing data
uint8_t processedIndex = 0;                // Tracks the position for processing data
volatile bool newDataAvailable = false;    // Indicates new data is available for processing
volatile bool captureComplete = false;     // Indicates all data has been processed for a packet
bool processingInProgress = false;         // Prevents re-entering the processing routine

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
    if (newDataAvailable && !processingInProgress) {
        processingInProgress = true;
        processData(); // Process new data outside ISR
        processingInProgress = false;
    }

    if (captureComplete) {
        sendPacket();
        captureComplete = false; // Reset flag after sending
    }

    yield(); // Allow background tasks
}

// Interrupt Service Routine (ISR) to handle pulse signals
void IRAM_ATTR handlePulse() {
    uint32_t currentTime = getCycleCount();
    bool isRisingEdge = (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY);

    if (isRisingEdge) {
        if (bufferIndex < MAX_SIZE) {
            // Record interval since the last rising edge
            risingIntervalBuffer[bufferIndex] = currentTime - lastPulseTime;
            lastPulseTime = currentTime;

            newDataAvailable = true; // Signal new data is ready for processing
        }
    } else {
        if (bufferIndex < MAX_SIZE) {
            // Record pulse length
            pulseLengthBuffer[bufferIndex] = currentTime - lastPulseTime;
            bufferIndex++;

            // If buffer is full, indicate all data is captured
            if (bufferIndex == MAX_SIZE) {
                newDataAvailable = true; // Ensure final data is processed
            }
        }
    }
}

// Process raw data incrementally outside ISR
void processData() {
    while (processedIndex < bufferIndex) {
        // Calculate RPM for the current entry
        pulseData.rpm[processedIndex] = static_cast<float>(60 * TRANSLATION_TIMEBASE) / risingIntervalBuffer[processedIndex];
        // Copy pulse length
        pulseData.length[processedIndex] = pulseLengthBuffer[processedIndex];

        processedIndex++;

        // If all data has been processed, enable packet sending
        if (processedIndex == MAX_SIZE) {
            captureComplete = true;
        }
    }
}

// Send the processed data via UDP
void sendPacket() {
    if (udp.connect(multicastIP, multicastPort)) {
        udp.write(reinterpret_cast<uint8_t*>(&pulseData), sizeof(pulseData));
        udp.close();

        Serial.println("Data sent successfully.");
        pulseData.index++;

        // Reset buffer and processing state
        bufferIndex = 0;
        processedIndex = 0;
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
