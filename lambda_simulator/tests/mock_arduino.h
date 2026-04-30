#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <map>
#include <vector>
#include <cstdint>

// Mock ESP32 DAC types and constants
typedef enum {
    DAC_CHANNEL_1 = 0,
    DAC_CHANNEL_2,
    DAC_CHANNEL_MAX,
} dac_channel_t;

// Mock Serial
class MockSerial {
public:
    void begin(int baud) {}
    template<typename T>
    void print(T val) { std::cout << val; }
    void print(float f, int p = 2) {
        std::cout << std::fixed << std::setprecision(p) << f;
    }
    template<typename T>
    void println(T val) { std::cout << val << std::endl; }
    void println(float f, int p = 2) {
        std::cout << std::fixed << std::setprecision(p) << f << std::endl;
    }
    void println() { std::cout << std::endl; }
};

extern MockSerial Serial;

// Mock Arduino/ESP32 functions
void delay(unsigned long ms);
unsigned long millis();
void analogReadResolution(int res);
int analogRead(int pin);

// DAC functions
void dac_output_enable(dac_channel_t channel);
void dac_output_voltage(dac_channel_t channel, uint8_t voltage);

// Arduino macros/functions
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define expf(x) std::exp(x)

// Global state for mocks
struct MockState {
    std::map<int, int> analogValues;
    std::map<dac_channel_t, uint8_t> dacValues;
    std::map<dac_channel_t, bool> dacEnabled;
    int adcResolution = 12;
};

extern MockState g_mockState;

#endif // MOCK_ARDUINO_H
