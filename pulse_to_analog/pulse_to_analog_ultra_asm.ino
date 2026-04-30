/*
  Pulse-to-16-Bit-Analog Converter for ESP32 - Ultra ASM Version

  This version uses full inline assembly for the pulse measurement ISR
  to minimize latency and jitter to the absolute minimum possible on Xtensa.
*/

#include <Arduino.h>
#include <driver/dac.h>

//================================================================================
// --- CONFIGURATION ---
//================================================================================
const int PULSE_INPUT_PIN = 2;
const int LSB_DAC_PIN = 25;
const int MSB_DAC_PIN = 26;

const unsigned long MIN_PULSE_WIDTH_US = 0;
const unsigned long MAX_PULSE_WIDTH_US = 20000;

//================================================================================

volatile uint32_t pulseStartCycles = 0;
volatile uint32_t pulseWidthCycles = 0;
volatile bool newPulseAvailable = false;

uint32_t minCycles;
uint32_t maxCycles;
uint32_t rangeCycles;

/**
 * ISR: Ultra-optimized using inline ASM
 */
void IRAM_ATTR measurePulse() {
#ifndef UNIT_TEST
    uint32_t now;
    uint32_t gpio_val;
    uint32_t pin_mask = (1 << PULSE_INPUT_PIN);

    // Read cycle count and GPIO in one block to keep them close
    asm volatile (
        "rsr %0, ccount \n"
        "l32i %1, %2, 0 \n"
        : "=a"(now), "=a"(gpio_val)
        : "a"(GPIO_IN_REG)
    );

    if (gpio_val & pin_mask) {
        pulseStartCycles = now;
    } else {
        pulseWidthCycles = now - pulseStartCycles;
        newPulseAvailable = true;
    }
#else
    uint32_t now;
    now = mock_ccount;
    if (REG_READ(GPIO_IN_REG) & (1 << PULSE_INPUT_PIN)) {
        pulseStartCycles = now;
    } else {
        pulseWidthCycles = now - pulseStartCycles;
        newPulseAvailable = true;
    }
#endif
}

void setup() {
    uint32_t cpuFreq = getCpuFrequencyMhz();
    minCycles = MIN_PULSE_WIDTH_US * cpuFreq;
    maxCycles = MAX_PULSE_WIDTH_US * cpuFreq;
    rangeCycles = maxCycles - minCycles;

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
    pinMode(PULSE_INPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), measurePulse, CHANGE);
}

void loop() {
    if (newPulseAvailable) {
        uint32_t capturedWidth = pulseWidthCycles;
        newPulseAvailable = false;

        if (capturedWidth < minCycles) capturedWidth = minCycles;
        if (capturedWidth > maxCycles) capturedWidth = maxCycles;

        uint64_t val = (uint64_t)(capturedWidth - minCycles) * 65535;
        uint16_t dacValue16bit = (uint16_t)(val / rangeCycles);

        dac_output_voltage(DAC_CHANNEL_1, dacValue16bit & 0xFF);
        dac_output_voltage(DAC_CHANNEL_2, dacValue16bit >> 8);
    }
}
