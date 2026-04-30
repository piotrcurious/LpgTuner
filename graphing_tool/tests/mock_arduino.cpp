#include "mock_arduino.h"
#include <map>

MockSerial Serial;

static unsigned long _mock_millis = 0;
static unsigned long _mock_micros = 0;

void delay(unsigned long ms) {
    _mock_millis += ms;
    _mock_micros += ms * 1000;
}

unsigned long millis() {
    return _mock_millis;
}

unsigned long micros() {
    return _mock_micros;
}

void advance_micros(unsigned long us) {
    _mock_micros += us;
    _mock_millis = _mock_micros / 1000;
}

static std::map<int, int> pinModes;
static std::map<int, int> analogValues;
static std::map<int, int> digitalValues;
static std::map<int, voidFuncPtr> interrupts_map;

void pinMode(int pin, int mode) {
    pinModes[pin] = mode;
}

int digitalRead(int pin) {
    return digitalValues[pin];
}

void setDigitalRead(int pin, int value) {
    digitalValues[pin] = value;
}

void analogWrite(int pin, int value) {
    // Just mock
}

int analogRead(int pin) {
    return analogValues[pin];
}

void setAnalogRead(int pin, int value) {
    analogValues[pin] = value;
}

void attachInterrupt(int interrupt, voidFuncPtr callback, int mode) {
    interrupts_map[interrupt] = callback;
}

int digitalPinToInterrupt(int pin) {
    return pin;
}

void triggerInterrupt(int interrupt) {
    if (interrupts_map.count(interrupt)) {
        interrupts_map[interrupt]();
    }
}

void noInterrupts() {}
void interrupts() {}

void portENTER_CRITICAL(portMUX_TYPE* mux) {}
void portEXIT_CRITICAL(portMUX_TYPE* mux) {}
void portENTER_CRITICAL_ISR(portMUX_TYPE* mux) {}
void portEXIT_CRITICAL_ISR(portMUX_TYPE* mux) {}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
