#include <assert.h>
#include <iostream>
#include "include/mock_arduino.h"

#define UNIT_TEST

// Mock the Arduino.h include and other macros
#define Arduino_h
#define DRIVER_DAC_H

// We need to include the .ino content but without the setup/loop naming conflict if we want multiple tests
// Or just include it and call setup/loop
#include "../pulse_to_analog_ultra.ino"

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
    std::cout << "Running Pulse to Analog Ultra tests..." << std::endl;

    setup();

    // Test Case 1: 0ms pulse (MIN_PULSE_WIDTH)
    trigger_pulse(1000, 0);
    loop();
    std::cout << "Test 1 (0ms): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    assert(mock_dac_val[DAC_CHANNEL_1] == 0);
    assert(mock_dac_val[DAC_CHANNEL_2] == 0);

    // Test Case 2: 10ms pulse (half range)
    // 10ms * 240MHz = 2,400,000 cycles
    trigger_pulse(2000, 2400000);
    loop();
    std::cout << "Test 2 (10ms): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    // 0.5 * 65535 = 32767.5 -> 32767 (0x7FFF)
    // MSB = 0x7F = 127, LSB = 0xFF = 255
    assert(mock_dac_val[DAC_CHANNEL_2] == 127);
    assert(mock_dac_val[DAC_CHANNEL_1] == 255);

    // Test Case 3: 20ms pulse (MAX_PULSE_WIDTH)
    // 20ms * 240MHz = 4,800,000 cycles
    trigger_pulse(3000, 4800000);
    loop();
    std::cout << "Test 3 (20ms): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    assert(mock_dac_val[DAC_CHANNEL_1] == 255);
    assert(mock_dac_val[DAC_CHANNEL_2] == 255);

    // Test Case 4: Overflow handling (CCOUNT wrap)
    // Start at near UINT32_MAX
    uint32_t start = 0xFFFFFF00;
    uint32_t duration = 2400000; // 10ms
    trigger_pulse(start, duration);
    loop();
    std::cout << "Test 4 (Overflow): MSB=" << (int)mock_dac_val[DAC_CHANNEL_2] << ", LSB=" << (int)mock_dac_val[DAC_CHANNEL_1] << std::endl;
    assert(mock_dac_val[DAC_CHANNEL_2] == 127);
    assert(mock_dac_val[DAC_CHANNEL_1] == 255);

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
