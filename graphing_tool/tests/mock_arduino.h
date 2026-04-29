#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <cstdio>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x01
#define OUTPUT 0x02
#define INPUT_PULLUP 0x04

#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03

#define IRAM_ATTR

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

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

    template<typename... Args>
    void printf(const char* format, Args... args) {
        std::printf(format, args...);
    }
};

extern MockSerial Serial;

// Mock Arduino functions
void delay(unsigned long ms);
unsigned long millis();
unsigned long micros();

void pinMode(int pin, int mode);
int digitalRead(int pin);
void analogWrite(int pin, int value);
int analogRead(int pin);

typedef void (*voidFuncPtr)(void);
void attachInterrupt(int interrupt, voidFuncPtr callback, int mode);
int digitalPinToInterrupt(int pin);

void noInterrupts();
void interrupts();

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
void portENTER_CRITICAL(portMUX_TYPE* mux);
void portEXIT_CRITICAL(portMUX_TYPE* mux);
void portENTER_CRITICAL_ISR(portMUX_TYPE* mux);
void portEXIT_CRITICAL_ISR(portMUX_TYPE* mux);

long map(long x, long in_min, long in_max, long out_min, long out_max);

template<class T, class L, class H>
auto constrain(T x, L low, H high) -> decltype(x) {
    return (x < low) ? low : ((x > high) ? high : x);
}

#endif // MOCK_ARDUINO_H
