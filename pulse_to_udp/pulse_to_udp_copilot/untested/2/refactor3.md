To fix the issue and ensure RPM is calculated in real-time while keeping the interrupt handler lightweight, we'll introduce an intermediate buffer for storing rising-edge intervals, as well as flags to manage the state of data capture, processing, and readiness for sending.

Here's the updated code:


---

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
volatile uint32_t risingIntervals[MAX_SIZE] = {};
volatile uint32_t pulseLengths[MAX_SIZE] = {};
volatile uint8_t pulseCount = 0;
volatile bool captureComplete = false;    // Flag indicating capture completion
volatile bool processingRequired = false; // Flag indicating processing is needed
volatile bool processingComplete = false; // Flag indicating processing is done

// Function declarations
void initializeWiFi();
void configureInterrupts();
void sendPacket();
void IRAM_ATTR handlePulse();
uint32_t IRAM_ATTR getCycleCount();
void processPulseData();
void calculateRPM();
void debugPrint(const char* message);

// Setup function
void setup() {
    if (DEBUG_MODE) {
        Serial.begin(115200);
        debugPrint("Initializing...");
    }

    pinMode(PULSE_PIN, INPUT);
    initializeWiFi();
    configureInterrupts();
}

// Loop function
void loop() {
    if (processingRequired) {
        processPulseData();
    }
    if (captureComplete && processingComplete) {
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
        // Calculate the interval between rising edges
        uint32_t risingInterval = currentTime - lastPulseTime;
        lastPulseTime = currentTime;

        if (pulseCount < MAX_SIZE) {
            risingIntervals[pulseCount] = risingInterval;
        }

    } else {
        // Calculate pulse length on falling edge
        uint32_t pulseLength = currentTime - lastPulseTime;

        if (pulseCount < MAX_SIZE) {
            pulseLengths[pulseCount] = pulseLength;
            pulseCount++;
            if (pulseCount == MAX_SIZE) {
                captureComplete = true;
                processingRequired = true; // Mark processing needed
            }
        }
    }
}

// Function to process pulse data
void processPulseData() {
    if (!processingRequired) return;

    calculateRPM();
    processingRequired = false;
    processingComplete = true; // Mark processing as complete
}

// Function to calculate RPM from rising-edge intervals
void calculateRPM() {
    for (uint8_t i = 0; i < MAX_SIZE; ++i) {
        if (risingIntervals[i] > 0) {
            pulseData.rpm[i] = static_cast<float>(60 * TRANSLATION_TIMEBASE) / risingIntervals[i];
        } else {
            pulseData.rpm[i] = 0;
        }
        pulseData.length[i] = pulseLengths[i];
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

        pulseData.index++;
        captureComplete = false;    // Reset capture flag
        processingComplete = false; // Reset processing flag
        pulseCount = 0;             // Reset pulse count
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


---

Key Updates

1. Intermediate Array:

Added risingIntervals to store distances between rising edges.

Pulse lengths and intervals are captured independently in handlePulse.



2. Flags:

captureComplete: Indicates when the buffer is full.

processingRequired: Signals that processing is needed.

processingComplete: Indicates processing is finished.



3. State Logic:

The main loop now processes data only when processingRequired is set.

Packet sending occurs only after both captureComplete and processingComplete are true.



4. Processing Separation:

RPM calculations are moved to processPulseData, triggered by processingRequired.

Sending happens only after processing.



5. Efficient Interrupt:

Minimal calculations are performed in the interrupt handler to handle short pulses efficiently.




Benefits

Real-Time Processing: Data is processed as soon as it’s captured, reducing delays.

Resilient State Management: Flags ensure correct sequencing of capture, processing, and sending.

Enhanced Debugging: Debug mode provides insights into system states.


Let me know if further refinements are needed!

