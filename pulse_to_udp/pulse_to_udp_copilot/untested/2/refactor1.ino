//chatGPT 
#include "wifi_settings.h" // Include WiFi settings for AP/STA mode
#include <ESPAsyncUDP.h>  // Use AsyncUDP for UDP communication

// Pulse signal settings
#define PULSE_PIN 4
#define PULSE_LOGIC_POLARITY HIGH

// Buffer settings
#define MAX_SIZE 2

// Timebase for RPM calculation
#define TRANSLATION_TIMEBASE 240000000

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
bool dataReady = false;
uint8_t arrayIndex = 0;

// Function declarations
void handlePulse();
void initializeWiFi();
void sendPacket();
void configureInterrupts();
uint32_t getCycleCount();

// Setup function
void setup() {
    // Initialize the serial monitor (optional)
    // Serial.begin(115200);

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

    // Detect rising edge
    if (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY) {
        uint32_t timeDiff = currentTime - lastPulseTime;
        lastPulseTime = currentTime;

        // Compute RPM and store in the buffer
        pulseData.rpm[arrayIndex] = static_cast<double>(60 * TRANSLATION_TIMEBASE) / timeDiff;

    } else { // Detect falling edge
        uint32_t pulseLength = currentTime - lastPulseTime;
        pulseData.length[arrayIndex] = pulseLength;

        // Update buffer index
        if (arrayIndex < (MAX_SIZE - 1)) {
            arrayIndex++;
        } else {
            dataReady = true;
        }
    }
}

// Function to send the pulse data via UDP
void sendPacket() {
    udp.connect(multicastIP, multicastPort);
    udp.write(reinterpret_cast<uint8_t*>(&pulseData), sizeof(pulseData));
    udp.close();

    // Reset the buffer and flags
    arrayIndex = 0;
    dataReady = false;
    pulseData.index++;
}

// Function to initialize WiFi
void initializeWiFi() {
#ifdef AP_mode_on
    WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
#else
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
    }
#endif
    esp_wifi_set_ps(WIFI_PS_NONE); // Disable WiFi power saving
}

// Function to configure interrupts
void configureInterrupts() {
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulse, CHANGE);
}

// Helper function to get the current ESP32 cycle count
uint32_t IRAM_ATTR getCycleCount() {
    uint32_t cycleCount;
    asm volatile("esync; rsr %0,ccount" : "=a"(cycleCount));
    return cycleCount;
}
