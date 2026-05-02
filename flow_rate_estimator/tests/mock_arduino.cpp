#include "mock_arduino.h"
#include <map>

MockSerial Serial;

static unsigned long _millis = 0;
static std::map<int, int> _analogValues;

void delay(unsigned long ms) {
    _millis += ms;
}

unsigned long millis() {
    return _millis;
}

int analogRead(int pin) {
    return _analogValues[pin];
}

void mock_arduino_init() {
    _millis = 0;
    _analogValues.clear();
}

void set_analog_read(int pin, int value) {
    _analogValues[pin] = value;
}

void set_millis(unsigned long ms) {
    _millis = ms;
}
