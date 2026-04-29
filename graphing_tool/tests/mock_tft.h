#ifndef MOCK_TFT_H
#define MOCK_TFT_H

#include "mock_arduino.h"

#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_CYAN        0x07FF
#define TFT_MAGENTA     0xF81F
#define TFT_YELLOW      0xFFE0
#define TFT_ORANGE      0xFDA0
#define TFT_DARKGREY    0x7BEF
#define TFT_NAVY        0x000F

class TFT_eSPI {
public:
    void init() {}
    void setRotation(uint8_t r) {}
    void fillScreen(uint16_t color) {}
    void setTextColor(uint16_t color, uint16_t bg = 0) {}
    void drawRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color) {}
    void drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color) {}
    void drawString(const char* string, int32_t x, int32_t y, uint8_t font = 1) {}
    void setTextSize(uint8_t s) {}
    void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color) {}
    void setCursor(int16_t x, int16_t y, uint8_t font = 1) {}
    void fillCircle(int32_t x, int32_t y, int32_t r, uint16_t color) {}
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

    template<typename... Args>
    void printf(const char* format, Args... args) {}

    void println(const char* s) {}
    void print(const char* s) {}

    // Additional methods if needed
};

#endif // MOCK_TFT_H
