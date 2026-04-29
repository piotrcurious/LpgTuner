#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <iomanip>
#include <cmath>

// Mock Serial
class MockSerial {
public:
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(int n) { std::cout << n; }
    void print(float f, int p = 2) {
        std::cout << std::fixed << std::setprecision(p) << f;
    }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(int n) { std::cout << n << std::endl; }
    void println(float f, int p = 2) {
        std::cout << std::fixed << std::setprecision(p) << f << std::endl;
    }
    void println() { std::cout << std::endl; }
};

extern MockSerial Serial;

// Mock Arduino functions
void delay(unsigned long ms);
unsigned long millis();

#endif // MOCK_ARDUINO_H
