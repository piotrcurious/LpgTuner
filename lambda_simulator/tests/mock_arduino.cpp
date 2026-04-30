#include "mock_arduino.h"

MockSerial Serial;
MockState g_mockState;

static unsigned long _mock_millis = 0;

void delay(unsigned long ms) {
    _mock_millis += ms;
}

unsigned long millis() {
    return _mock_millis;
}

void analogReadResolution(int res) {
    g_mockState.adcResolution = res;
}

int analogRead(int pin) {
    return g_mockState.analogValues[pin];
}

void dac_output_enable(dac_channel_t channel) {
    g_mockState.dacEnabled[channel] = true;
}

void dac_output_voltage(dac_channel_t channel, uint8_t voltage) {
    g_mockState.dacValues[channel] = voltage;
}
