#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <cstdint>

typedef std::string String;

#define PI 3.14159265358979323846
#define FALLING 0

unsigned long millis();
void delay(unsigned long ms);
int analogRead(int pin);
void noInterrupts();
void interrupts();

class MAX6675 {
public:
    MAX6675(int pin) {}
    void begin() {}
    void read() {}
    float getTemperature() { return 500.0; }
    void setSPIspeed(int speed) {}
};

class TFT_eSPI {
public:
    void init() {}
    void setRotation(int r) {}
    void fillScreen(int color) {}
    void setTextSize(int s) {}
    void setTextColor(int c) {}
    void setTextColor(int c, int b) {}
    void setCursor(int x, int y) {}
    void print(const char* s) {}
    void print(String s) {}
    void print(float f, int d = 2) {}
    void print(int i) {}
    void fillRect(int x, int y, int w, int h, int c) {}
    void drawRect(int x, int y, int w, int h, int c) {}
    void drawPixel(int x, int y, int c) {}
    void drawCircle(int x, int y, int r, int c) {}
    void drawLine(int x1, int y1, int x2, int y2, int c) {}
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return 0; }
};

#define BLACK 0
#define WHITE 1
#define RED 2
#define GREEN 3
#define BLUE 4
#define CYAN 5
#define MAGENTA 6
#define YELLOW 7
#define ORANGE 8
#define DARKGREY 9

#endif
