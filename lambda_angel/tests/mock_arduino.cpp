#include "mock_arduino.h"

static unsigned long _millis = 0;

unsigned long millis() {
    return _millis;
}

void delay(unsigned long ms) {
    _millis += ms;
}

int analogRead(int pin) {
    return 512;
}

void noInterrupts() {}
void interrupts() {}

#include <stdarg.h>
#include <stdio.h>

class SerialMock {
public:
    void begin(int baud) {}
    void print(const char* s) { printf("%s", s); }
    void print(float f, int d = 2) { printf("%.*f", d, f); }
    void print(int i) { printf("%d", i); }
    void println(const char* s) { printf("%s\n", s); }
    void println(float f, int d = 2) { printf("%.*f\n", d, f); }
    void println(int i) { printf("%d\n", i); }
};

SerialMock Serial;
