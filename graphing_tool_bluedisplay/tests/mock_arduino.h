#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdint.h>
#include <string>

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define FALLING 4
#define CHANGE 5

#define HIGH 1
#define LOW 0

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

// Mock BlueDisplay types
typedef uint16_t color16_t;
#define TEXT_SIZE_11 11
#define TEXT_SIZE_09 9

class BDButton {
public:
    void init(int x, int y, int w, int h, color16_t color, const char* text, int size, int flags, int value, void (*handler)(BDButton*, int16_t)) {}
    void drawButton() {}
};

class BlueDisplay {
public:
    void drawPixel(uint16_t x, uint16_t y, color16_t color);
    uint16_t drawText(uint16_t x, uint16_t y, const char* text, uint16_t size, color16_t color, color16_t bgColor);
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color16_t color) {}
    void clearDisplay(color16_t color) {}
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, color16_t color) {}
    void drawRectRel(uint16_t x, uint16_t y, int16_t w, int16_t h, color16_t color, uint16_t stroke = 1) {}
    void initCommunication(MockSerial* s, void (*initFunc)(), void (*guiFunc)(), void (*reFunc)() = NULL) {}
    void checkAndHandleEvents() {}
    int getDisplayWidth() { return 800; }
    int getDisplayHeight() { return 480; }
};

extern BlueDisplay BlueDisplay1;

// Helper macros
#define arduino_constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define arduino_map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

// Mock Arduino functions
void delay(unsigned long ms);
unsigned long millis();
unsigned long micros();
int analogRead(int pin);
int digitalRead(int pin);
void pinMode(int pin, int mode);
void attachInterrupt(int pin, void (*func)(), int mode);
int digitalPinToInterrupt(int pin);
void analogWrite(int pin, int value);

void mock_arduino_init();
void set_analog_read(int pin, int value);
void set_digital_read(int pin, int value);
void set_millis(unsigned long ms);
void set_micros(unsigned long us);

#endif // MOCK_ARDUINO_H
