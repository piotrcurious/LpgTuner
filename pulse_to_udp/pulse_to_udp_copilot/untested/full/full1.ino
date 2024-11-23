#include "wifi_settings.h"
#include <ESPAsyncUDP.h>

// Pin and signal settings
constexpr uint8_t CRANK_PIN = 4;
constexpr uint8_t IGNITION_PIN = 5;
constexpr bool CRANK_LOGIC_POLARITY = HIGH;
constexpr bool IGNITION_LOGIC_POLARITY = HIGH;

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
    float crankRpm[MAX_SIZE];     // Crank RPM values
    uint32_t crankLength[MAX_SIZE]; // Crank pulse lengths
    float ignitionAngle[MAX_SIZE]; // Ignition timing angle
    uint32_t ignitionLength[MAX_SIZE]; // Ignition pulse lengths
    uint8_t index;                // Packet index
};
#pragma pack(pop)

// Global variables
PulseData pulseData = {};
AsyncUDP udp;

// Crank sensor variables
volatile uint32_t lastCrankTime = 0;
volatile uint32_t crankIntervalBuffer[MAX_SIZE] = {};
volatile uint32_t crankLengthBuffer[MAX_SIZE] = {};
volatile uint8_t crankWriteIndex = 0;
volatile uint8_t crankProcessIndex = 0;
volatile bool newCrankDataAvailable = false;

// Ignition sensor variables
volatile uint32_t lastIgnitionTime = 0;
volatile uint32_t ignitionIntervalBuffer[MAX_SIZE] = {};
volatile uint32_t ignitionLengthBuffer[MAX_SIZE] = {};
volatile uint8_t ignitionWriteIndex = 0;
volatile uint8_t ignitionProcessIndex = 0;
volatile bool newIgnitionDataAvailable = false;

// Function declarations
void initializeWiFi();
void configureInterrupts();
void sendPacket();
void processCrankData();
void processIgnitionData();
void IRAM_ATTR handleCrankPulseESP32();
void IRAM_ATTR handleIgnitionPulseESP32();
void ICACHE_RAM_ATTR handleCrankPulseESP8266();
void ICACHE_RAM_ATTR handleIgnitionPulseESP8266();
uint32_t IRAM_ATTR getCycleCountESP32();
uint32_t ICACHE_RAM_ATTR getCycleCountESP8266();
bool ICACHE_RAM_ATTR digitalReadFastESP8266(uint8_t pin);
void debugPrint(const char* message);

// Setup function
void setup() {
    Serial.begin(115200);

    pinMode(CRANK_PIN, INPUT);
    pinMode(IGNITION_PIN, INPUT);
    initializeWiFi();
    configureInterrupts();
}

// Loop function
void loop() {
    if (newCrankDataAvailable) {
        processCrankData();
    }

    if (newIgnitionDataAvailable) {
        processIgnitionData();
    }

    if (crankProcessIndex == ignitionProcessIndex) {
        sendPacket();
    }

    yield(); // Allow background tasks
}

// Platform-specific ISR implementation for crank pulses
#ifdef ESP32
void IRAM_ATTR handleCrankPulseESP32() {
    uint32_t currentTime;
    asm volatile("esync; rsr %0,ccount" : "=a"(currentTime)); // Get the current cycle count

    uint32_t gpioState = digitalRead(CRANK_PIN);
    bool isRisingEdge = (gpioState == CRANK_LOGIC_POLARITY);

    uint8_t nextIndex = (crankWriteIndex + 1) % MAX_SIZE;

    if (nextIndex == crankProcessIndex) {
        return; // Buffer is full, ignore new data
    }

    if (isRisingEdge) {
        crankIntervalBuffer[crankWriteIndex] = currentTime - lastCrankTime;
        lastCrankTime = currentTime;
        newCrankDataAvailable = true;
    } else {
        crankLengthBuffer[crankWriteIndex] = currentTime - lastCrankTime;
        crankWriteIndex = nextIndex;
    }
}
#elif defined(ESP8266)
void ICACHE_RAM_ATTR handleCrankPulseESP8266() {
    uint32_t currentTime = getCycleCountESP8266(); // Get current cycle count
    bool gpioState = digitalReadFastESP8266(CRANK_PIN); // Fast digital read

    uint8_t nextIndex = (crankWriteIndex + 1) % MAX_SIZE;

    if (nextIndex == crankProcessIndex) {
        return; // Buffer is full, ignore new data
    }

    if (gpioState == CRANK_LOGIC_POLARITY) {
        crankIntervalBuffer[crankWriteIndex] = currentTime - lastCrankTime;
        lastCrankTime = currentTime;
        newCrankDataAvailable = true;
    } else {
        crankLengthBuffer[crankWriteIndex] = currentTime - lastCrankTime;
        crankWriteIndex = nextIndex;
    }
}
#endif

// Platform-specific ISR implementation for ignition pulses
#ifdef ESP32
void IRAM_ATTR handleIgnitionPulseESP32() {
    uint32_t currentTime;
    asm volatile("esync; rsr %0,ccount" : "=a"(currentTime)); // Get the current cycle count

    uint32_t gpioState = digitalRead(IGNITION_PIN);
    bool isRisingEdge = (gpioState == IGNITION_LOGIC_POLARITY);

    uint8_t nextIndex = (ignitionWriteIndex + 1) % MAX_SIZE;

    if (nextIndex == ignitionProcessIndex) {
        return; // Buffer is full, ignore new data
    }

    if (isRisingEdge) {
        ignitionIntervalBuffer[ignitionWriteIndex] = currentTime - lastIgnitionTime;
        lastIgnitionTime = currentTime;
        newIgnitionDataAvailable = true;
    } else {
        ignitionLengthBuffer[ignitionWriteIndex] = currentTime - lastIgnitionTime;
        ignitionWriteIndex = nextIndex;
    }
}
#elif defined(ESP8266)
void ICACHE_RAM_ATTR handleIgnitionPulseESP8266() {
    uint32_t currentTime = getCycleCountESP8266(); // Get current cycle count
    bool gpioState = digitalReadFastESP8266(IGNITION_PIN); // Fast digital read

    uint8_t nextIndex = (ignitionWriteIndex + 1) % MAX_SIZE;

    if (nextIndex == ignitionProcessIndex) {
        return; // Buffer is full, ignore new data
    }

    if (gpioState == IGNITION_LOGIC_POLARITY) {
        ignitionIntervalBuffer[ignitionWriteIndex] = currentTime - lastIgnitionTime;
        lastIgnitionTime = currentTime;
        newIgnitionDataAvailable = true;
    } else {
        ignitionLengthBuffer[ignitionWriteIndex] = currentTime - lastIgnitionTime;
        ignitionWriteIndex = nextIndex;
    }
}
#endif

// Data processing functions
void processCrankData() {
    while (crankProcessIndex != crankWriteIndex) {
        noInterrupts();
        uint32_t interval = crankIntervalBuffer[crankProcessIndex];
        uint32_t length = crankLengthBuffer[crankProcessIndex];
        interrupts();

        if (interval > 0) {
            pulseData.crankRpm[crankProcessIndex] = static_cast<float>(60 * TRANSLATION_TIMEBASE) / interval;
        } else {
            pulseData.crankRpm[crankProcessIndex] = 0; // Handle invalid intervals gracefully
        }

        pulseData.crankLength[crankProcessIndex] = length;
        crankProcessIndex = (crankProcessIndex + 1) % MAX_SIZE;

        if (crankProcessIndex == crankWriteIndex) {
            newCrankDataAvailable = false; // Reset flag when processing is complete
        }
    }
}

void processIgnitionData() {
    while (ignitionProcessIndex != ignitionWriteIndex) {
        noInterrupts();
        uint32_t interval = ignitionIntervalBuffer[ignitionProcessIndex];
        uint32_t length = ignitionLengthBuffer[ignitionProcessIndex];
        interrupts();

        if (interval > 0) {
            pulseData.ignitionAngle[ignitionProcessIndex] = static_cast<float>(interval * 360.0 / TRANSLATION_TIMEBASE);
        } else {
            pulseData.ignitionAngle[ignitionProcessIndex] = 0; // Handle invalid intervals gracefully
        }

        pulseData.ignitionLength[ignitionProcessIndex] = length;
        ignitionProcessIndex = (ignitionProcessIndex + 1) % MAX_SIZE;

        if (ignitionProcessIndex == ignitionWriteIndex) {
            newIgnitionDataAvailable = false; // Reset flag when processing is complete
        }
    }
}

// Send processed data via UDP
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
