#include "wifi_settings.h" // Include WiFi settings for AP/STA mode
#include <ESPAsyncUDP.h>  // Use AsyncUDP for UDP communication

// Pulse signal settings
constexpr uint8_t PULSE_PIN = 4;
constexpr bool PULSE_LOGIC_POLARITY = HIGH;

// Buffer settings
constexpr uint8_t MAX_SIZE = 2;

// Timebase for RPM calculation
constexpr uint32_t TRANSLATION_TIMEBASE = 240000000;

// Debug mode (set to true for debug prints)
constexpr bool DEBUG_MODE = false;

// Struct to store pulse data
#pragma pack(push, 1)
struct PulseData {
    float rpm[MAX_SIZE];         // Array to store RPM values
    uint32_t length[MAX_SIZE];   // Array to store pulse length values
    uint8_t index;               // Packet index
};
#pragma pack(pop)

// Global variables
PulseData pulseData = {};
AsyncUDP udp;
volatile uint32_t lastPulseTime = 0;
volatile uint32_t lastPulseLength = 0;
volatile bool dataReady = false;
uint8_t arrayIndex = 0;

// Function declarations
void initializeWiFi();
void configureInterrupts();
void sendPacket();
void IRAM_ATTR handlePulse();
uint32_t IRAM_ATTR getCycleCount();
void debugPrint(const char* message);

// Setup function
void setup() {
    // Initialize debug serial if enabled
    if (DEBUG_MODE) {
        Serial.begin(115200);
        debugPrint("Initializing...");
    }

    // Initialize the pulse pin as input
    pinMode(PULSE_PIN, INPUT);

    // Initialize WiFi and UDP
    initializeWiFi();

    // Configure pulse interrupts
    configureInterrupts();
}

// Loop function
void loop() {
    if (dataReady) {
        sendPacket();
    } else {
        yield(); // Allow background tasks
    }
}

// Function to handle the pulse interrupt
void IRAM_ATTR handlePulse() {
    uint32_t currentTime = getCycleCount();
    bool isRisingEdge = (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY);

    if (isRisingEdge) {
        uint32_t timeDiff = currentTime - lastPulseTime;
        lastPulseTime = currentTime;

        // Compute RPM and store in the buffer
        if (timeDiff > 0) { // Avoid division by zero
            pulseData.rpm[arrayIndex] = static_cast<float>(60 * TRANSLATION_TIMEBASE) / timeDiff;
        } else {
            pulseData.rpm[arrayIndex] = 0; // Handle erroneous cases gracefully
        }
    } else {
        // Calculate pulse length on falling edge
        uint32_t pulseLength = currentTime - lastPulseTime;
        pulseData.length[arrayIndex] = pulseLength;

        // Update buffer index
        if (++arrayIndex >= MAX_SIZE) {
            dataReady = true;
            arrayIndex = 0; // Reset index for next batch
        }
    }
}

// Function to send the pulse data via UDP
void sendPacket() {
    if (udp.connect(multicastIP, multicastPort)) {
        udp.write(reinterpret_cast<uint8_t*>(&pulseData), sizeof(pulseData));
        udp.close();

        if (DEBUG_MODE) {
            debugPrint("Data sent successfully.");
        }

        // Reset the buffer and flags
        pulseData.index++;
        dataReady = false;
    } else if (DEBUG_MODE) {
        debugPrint("UDP connection failed.");
    }
}

// Function to initialize WiFi
void initializeWiFi() {
#ifdef AP_mode_on
    WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
    if (DEBUG_MODE) {
        debugPrint("AP Mode initialized.");
    }
#else
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        if (DEBUG_MODE) {
            Serial.print(".");
        }
    }
    if (DEBUG_MODE) {
        debugPrint("\nSTA Mode connected.");
    }
#endif
    esp_wifi_set_ps(WIFI_PS_NONE); // Disable WiFi power saving
}

// Function to configure interrupts
void configureInterrupts() {
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);
    if (DEBUG_MODE) {
        debugPrint("Interrupts configured.");
    }
}

// Helper function to get the current ESP32 cycle count
uint32_t IRAM_ATTR getCycleCount() {
    uint32_t cycleCount;
    asm volatile("esync; rsr %0,ccount" : "=a"(cycleCount));
    return cycleCount;
}

// Helper function for debug prints
void debugPrint(const char* message) {
    if (DEBUG_MODE) {
        Serial.println(message);
    }
}
