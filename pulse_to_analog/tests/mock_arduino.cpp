#include "mock_arduino.h"

uint32_t mock_ccount = 0;
uint32_t mock_gpio_in = 0;
uint8_t mock_dac_val[2] = {0, 0};

void dac_output_enable(dac_channel_t channel) {}
void dac_output_voltage(dac_channel_t channel, uint8_t voltage) {
    mock_dac_val[channel] = voltage;
}
void pinMode(int pin, pinMode_t mode) {}
void attachInterrupt(int interrupt, void (*userFunc)(void), int mode) {}
int digitalPinToInterrupt(int pin) { return pin; }
uint32_t getCpuFrequencyMhz() { return 240; }
uint32_t REG_READ(uint32_t reg) {
    if (reg == GPIO_IN_REG) return mock_gpio_in;
    return 0;
}
