#include "wifi_settings.h"
#include <ESPAsyncUDP.h>

// Pin and signal settings
constexpr uint8_t PULSE_PIN = 4;
constexpr bool PULSE_LOGIC_POLARITY = HIGH;

// Platform-specific timebase for RPM calculation
#if defined(ESP32)
constexpr uint32_t TRANSLATION_TIMEBASE = 240000000; // ESP32 clock cycles per second
#elif defined(ESP8266)
constexpr uint32_t TRANSLATION_TIMEBASE = 80000000;  // ESP8266 clock cycles per second
#else
constexpr uint32_t TRANSLATION_TIMEBASE = 1000000;   // Arduino micros() timebase
#endif

// Buffer and calculation settings
constexpr uint8_t MAX_SIZE = 10;

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
void IRAM_ATTR handlePulseESP32();
void ICACHE_RAM_ATTR handlePulseESP8266();
void handlePulseArduino();
uint32_t IRAM_ATTR getCycleCountESP32();
uint32_t ICACHE_RAM_ATTR getCycleCountESP8266();
bool ICACHE_RAM_ATTR digitalReadFastESP8266(uint8_t pin);
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

// Platform-specific ISR implementation
#ifdef ESP32
void IRAM_ATTR handlePulseESP32() {
    uint32_t currentTime;
    asm volatile("esync; rsr %0,ccount" : "=a"(currentTime)); // Get the current cycle count

    uint32_t gpioState = digitalRead(PULSE_PIN); // Read GPIO state
    bool isRisingEdge = (gpioState == PULSE_LOGIC_POLARITY);

    uint8_t nextIndex = (writeIndex + 1) % MAX_SIZE;

    if (nextIndex == processIndex) {
        // Buffer is full, ignore new data
        return;
    }

    if (isRisingEdge) {
        risingIntervalBuffer[writeIndex] = currentTime - lastPulseTime;
        lastPulseTime = currentTime;
        newDataAvailable = true;
    } else {
        pulseLengthBuffer[writeIndex] = currentTime - lastPulseTime;
        writeIndex = nextIndex;

        if (writeIndex == processIndex) {
            captureComplete = true; // Buffer is full
        }
    }
}
#elif defined(ESP8266)
void ICACHE_RAM_ATTR handlePulseESP8266() {
    uint32_t currentTime = getCycleCountESP8266(); // Get current cycle count
    bool gpioState = digitalReadFastESP8266(PULSE_PIN); // Fast digital read
    bool isRisingEdge = (gpioState == PULSE_LOGIC_POLARITY);

    uint8_t nextIndex = (writeIndex + 1) % MAX_SIZE;

    if (nextIndex == processIndex) {
        // Buffer is full, ignore new data
        return;
    }

    if (isRisingEdge) {
        risingIntervalBuffer[writeIndex] = currentTime - lastPulseTime;
        lastPulseTime = currentTime;
        newDataAvailable = true;
    } else {
        pulseLengthBuffer[writeIndex] = currentTime - lastPulseTime;
        writeIndex = nextIndex;

        if (writeIndex == processIndex) {
            captureComplete = true; // Buffer is full
        }
    }
}
#else
void handlePulseArduino() {
    uint32_t currentTime = micros(); // Use micros() for Arduino boards

    bool isRisingEdge = (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY);
    uint8_t nextIndex = (writeIndex + 1) % MAX_SIZE;

    if (nextIndex == processIndex) {
        // Buffer is full, ignore new data
        return;
    }

    if (isRisingEdge) {
        risingIntervalBuffer[writeIndex] = currentTime - lastPulseTime;
        lastPulseTime = currentTime;
        newDataAvailable = true;
    } else {
        pulseLengthBuffer[writeIndex] = currentTime - lastPulseTime;
        writeIndex = nextIndex;

        if (writeIndex == processIndex) {
            captureComplete = true; // Buffer is full
        }
    }
}
#endif

// Platform-specific cycle count functions
#ifdef ESP32
uint32_t IRAM_ATTR getCycleCountESP32() {
    uint32_t cycleCount;
    asm volatile("rsr %0, ccount" : "=a"(cycleCount));
    return cycleCount;
}
#elif defined(ESP8266)
uint32_t ICACHE_RAM_ATTR getCycleCountESP8266() {
    uint32_t cycleCount;
    asm volatile("rsr %0, ccount" : "=a"(cycleCount));
    return cycleCount;
}

bool ICACHE_RAM_ATTR digitalReadFastESP8266(uint8_t pin) {
    uint32_t gpioIn;
    asm volatile("movi %0, 0x60000318\n" // GPIO_IN register address
                 "l32i %0, %0, 0\n"
                 : "=r"(gpioIn));
    return (gpioIn & (1 << pin)) != 0;
}
#endif

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
#ifdef ESP32
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulseESP32, CHANGE);
#elif defined(ESP8266)
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulseESP8266, CHANGE);
#else
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handlePulseArduino, CHANGE);
#endif
    Serial.println("Interrupts configured.");
}

// Debug print helper
void debugPrint(const char* message) {
    Serial.println(message);
}
