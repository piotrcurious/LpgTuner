#include <assert.h>
#include <iostream>
#include "include/mock_arduino.h"

#define UNIT_TEST

// Mock the Arduino.h include and other macros
#define Arduino_h
#define DRIVER_DAC_H

// Include the ASM version for testing (it uses the same logic)
#include "../pulse_to_analog_ultra_asm.ino"

// Test helpers
void trigger_pulse(uint32_t start_cycles, uint32_t duration_cycles) {
    mock_ccount = start_cycles;
    mock_gpio_in = (1 << PULSE_INPUT_PIN);
    measurePulse();

    mock_ccount = start_cycles + duration_cycles;
    mock_gpio_in = 0;
    measurePulse();
}

int main() {
    std::cout << "Running Pulse to Analog Ultra ASM tests..." << std::endl;

    setup();

    // Test Case 1: 0ms pulse
    trigger_pulse(1000, 0);
    loop();
    std::cout << "Test 1 (0ms): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    assert(mock_dac_val[DAC_CHANNEL_1] == 0);
    assert(mock_dac_val[DAC_CHANNEL_2] == 0);

    // Test Case 2: 10ms pulse
    trigger_pulse(2000, 2400000);
    loop();
    std::cout << "Test 2 (10ms): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    assert(mock_dac_val[DAC_CHANNEL_2] == 127);
    assert(mock_dac_val[DAC_CHANNEL_1] == 255);

    // Test Case 3: 20ms pulse
    trigger_pulse(3000, 4800000);
    loop();
    std::cout << "Test 3 (20ms): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    assert(mock_dac_val[DAC_CHANNEL_1] == 255);
    assert(mock_dac_val[DAC_CHANNEL_2] == 255);

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
