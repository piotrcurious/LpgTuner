#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdint.h>

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5

// Mock Serial
class MockSerial {
public:
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(std::string s) { std::cout << s; }
    void print(int n) { std::cout << n; }
    void print(double f, int p = 2) {
        std::cout << std::fixed << std::setprecision(p) << f;
    }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(std::string s) { std::cout << s << std::endl; }
    void println(int n) { std::cout << n << std::endl; }
    void println(double f, int p = 2) {
        std::cout << std::fixed << std::setprecision(p) << f << std::endl;
    }
    void println() { std::cout << std::endl; }
};

extern MockSerial Serial;

// Mock Arduino functions
void delay(unsigned long ms);
unsigned long millis();
int analogRead(int pin);
void mock_arduino_init();
void set_analog_read(int pin, int value);
void set_millis(unsigned long ms);

#endif // MOCK_ARDUINO_H
