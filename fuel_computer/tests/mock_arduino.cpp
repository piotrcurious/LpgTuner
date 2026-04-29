#include "mock_arduino.h"

MockSerial Serial;

void delay(unsigned long ms) {
    // No-op in tests
}
