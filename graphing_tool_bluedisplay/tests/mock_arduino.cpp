#include "mock_arduino.h"
#include <map>

MockSerial Serial;
BlueDisplay BlueDisplay1;

static unsigned long _millis = 0;
static unsigned long _micros = 0;
static std::map<int, int> _analogValues;
static std::map<int, int> _digitalValues;

void delay(unsigned long ms) {
    _millis += ms;
    _micros += ms * 1000;
}

unsigned long millis() {
    return _millis;
}

unsigned long micros() {
    return _micros;
}

int analogRead(int pin) {
    return _analogValues[pin];
}

int digitalRead(int pin) {
    return _digitalValues[pin];
}

void pinMode(int pin, int mode) {}
void attachInterrupt(int pin, void (*func)(), int mode) {}
int digitalPinToInterrupt(int pin) { return pin; }
void analogWrite(int pin, int value) {}

void mock_arduino_init() {
    _millis = 0;
    _micros = 0;
    _analogValues.clear();
    _digitalValues.clear();
}

void set_analog_read(int pin, int value) {
    _analogValues[pin] = value;
}

void set_digital_read(int pin, int value) {
    _digitalValues[pin] = value;
}

void set_millis(unsigned long ms) {
    _millis = ms;
    _micros = ms * 1000;
}

void set_micros(unsigned long us) {
    _micros = us;
    _millis = us / 1000;
}
