#include "mock_arduino.h"

MockSerial Serial;

static unsigned long _mock_millis = 0;

void delay(unsigned long ms) {
    _mock_millis += ms;
}

unsigned long millis() {
    return _mock_millis;
}
