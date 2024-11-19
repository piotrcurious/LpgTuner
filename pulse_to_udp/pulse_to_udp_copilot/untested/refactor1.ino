// by Claude 

#ifndef PULSE_MONITOR_H
#define PULSE_MONITOR_H

#include <Arduino.h>
#include <AsyncUDP.h>
#include "wifi_settings.h"
#include <esp_wifi.h>

// Configuration constants
constexpr uint8_t PULSE_PIN = 4;
constexpr bool PULSE_LOGIC_POLARITY = HIGH;
constexpr size_t MAX_BUFFER_SIZE = 2;
constexpr uint32_t TRANSLATION_TIMEBASE = 240000000; // ESP32 clock frequency
constexpr uint32_t RPM_CONVERSION_FACTOR = 60 * TRANSLATION_TIMEBASE;

// Error checking constants
constexpr uint32_t MIN_PULSE_LENGTH = 100;        // Minimum valid pulse length in cycles
constexpr uint32_t MAX_PULSE_LENGTH = 12000000;   // Maximum valid pulse length in cycles
constexpr float MIN_VALID_RPM = 60.0;             // Minimum valid RPM
constexpr float MAX_VALID_RPM = 20000.0;          // Maximum valid RPM

// Packed data structure for network transmission
#pragma pack(push, 1)
struct PulseData {
    float rpm[MAX_BUFFER_SIZE];
    uint32_t pulseLength[MAX_BUFFER_SIZE];
    uint8_t index;
    uint8_t errorFlags;  // New field for error reporting
};
#pragma pack(pop)

class PulseMonitor {
public:
    PulseMonitor() : lastPulseTime(0), arrayIndex(0), dataReady(false) {
        data.index = 0;
        data.errorFlags = 0;
    }

    void begin() {
        // Initialize GPIO
        pinMode(PULSE_PIN, INPUT);
        
        // Configure WiFi based on mode
        setupWiFi();
        
        // Disable WiFi power saving for better responsiveness
        esp_wifi_set_ps(WIFI_PS_NONE);
        
        // Setup UDP
        setupUDP();
        
        // Attach interrupt handler
        attachInterrupt(digitalPinToInterrupt(PULSE_PIN), std::bind(&PulseMonitor::handlePulse, this), CHANGE);
    }

    void update() {
        if (dataReady) {
            sendPacket();
        }
    }

private:
    PulseData data;
    AsyncUDP udp;
    volatile uint32_t lastPulseTime;
    volatile uint8_t arrayIndex;
    volatile bool dataReady;
    
    void IRAM_ATTR handlePulse() {
        uint32_t currentTime;
        asm volatile("esync; rsr %0,ccount":"=a"(currentTime));
        
        if (digitalRead(PULSE_PIN) == PULSE_LOGIC_POLARITY) {
            // Rising edge
            handleRisingEdge(currentTime);
        } else {
            // Falling edge
            handleFallingEdge(currentTime);
        }
    }
    
    void IRAM_ATTR handleRisingEdge(uint32_t currentTime) {
        uint32_t timeDiff = currentTime - lastPulseTime;
        lastPulseTime = currentTime;
        
        // Calculate RPM with bounds checking
        if (timeDiff > 0 && timeDiff < MAX_PULSE_LENGTH) {
            float rpm = static_cast<double>(RPM_CONVERSION_FACTOR) / timeDiff;
            
            if (rpm >= MIN_VALID_RPM && rpm <= MAX_VALID_RPM) {
                data.rpm[arrayIndex] = rpm;
            } else {
                data.errorFlags |= 0x01;  // Set error flag for invalid RPM
                data.rpm[arrayIndex] = 0;
            }
        }
    }
    
    void IRAM_ATTR handleFallingEdge(uint32_t currentTime) {
        uint32_t pulseLength = currentTime - lastPulseTime;
        
        if (pulseLength >= MIN_PULSE_LENGTH && pulseLength <= MAX_PULSE_LENGTH) {
            data.pulseLength[arrayIndex] = pulseLength;
            
            if (arrayIndex < (MAX_BUFFER_SIZE - 1)) {
                arrayIndex++;
            } else {
                dataReady = true;
            }
        } else {
            data.errorFlags |= 0x02;  // Set error flag for invalid pulse length
        }
    }
    
    void setupWiFi() {
        #ifdef AP_MODE
            WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
        #else
            WiFi.begin(ssid, password);
            while (WiFi.status() != WL_CONNECTED) {
                delay(100);
            }
        #endif
    }
    
    void setupUDP() {
        if (udp.connect(multicastIP, multicastPort)) {
            // Optional: Set up any UDP callbacks here
        }
    }
    
    void sendPacket() {
        if (udp.connected()) {
            udp.write(reinterpret_cast<uint8_t*>(&data), sizeof(data));
            
            // Reset for next batch
            arrayIndex = 0;
            dataReady = false;
            data.index++;
            data.errorFlags = 0;
        }
    }
};

#endif // PULSE_MONITOR_H
