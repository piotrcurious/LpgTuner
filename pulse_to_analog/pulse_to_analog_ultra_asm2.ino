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
const int PULSE_INPUT_PIN = 34;   // safer than GPIO2; input-only, no boot strap issue
const int LSB_DAC_PIN     = 25;
const int MSB_DAC_PIN     = 26;

const unsigned long MIN_PULSE_WIDTH_US = 0;
const unsigned long MAX_PULSE_WIDTH_US = 20000;

//================================================================================

volatile uint32_t pulseStartCycles  = 0;
volatile uint32_t pulseWidthCycles  = 0;
volatile bool     pulseArmed        = false;   // fixes startup / first-falling-edge issue
volatile bool     newPulseAvailable = false;

// Protects shared ISR/main-thread state
portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

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
    const uint32_t pin_mask = (1UL << PULSE_INPUT_PIN);

    // Read cycle count and GPIO in one block to keep them close
    asm volatile (
        "rsr %0, ccount \n"
        "l32i %1, %2, 0 \n"
        : "=a"(now), "=a"(gpio_val)
        : "a"(GPIO_IN_REG)
    );

    const bool levelHigh = (gpio_val & pin_mask) != 0;

    if (levelHigh) {
        // Rising edge: arm measurement and remember start
        portENTER_CRITICAL_ISR(&pulseMux);
        pulseStartCycles = now;
        pulseArmed = true;
        portEXIT_CRITICAL_ISR(&pulseMux);
    } else {
        // Falling edge: only valid after a rising edge has armed the capture
        portENTER_CRITICAL_ISR(&pulseMux);
        if (pulseArmed) {
            pulseWidthCycles = now - pulseStartCycles;
            newPulseAvailable = true;
        }
        portEXIT_CRITICAL_ISR(&pulseMux);
    }
#else
    uint32_t now = mock_ccount;
    if (REG_READ(GPIO_IN_REG) & (1UL << PULSE_INPUT_PIN)) {
        portENTER_CRITICAL_ISR(&pulseMux);
        pulseStartCycles = now;
        pulseArmed = true;
        portEXIT_CRITICAL_ISR(&pulseMux);
    } else {
        portENTER_CRITICAL_ISR(&pulseMux);
        if (pulseArmed) {
            pulseWidthCycles = now - pulseStartCycles;
            newPulseAvailable = true;
        }
        portEXIT_CRITICAL_ISR(&pulseMux);
    }
#endif
}

void setup() {
    uint32_t cpuFreq = getCpuFrequencyMhz();
    minCycles  = MIN_PULSE_WIDTH_US * cpuFreq;
    maxCycles  = MAX_PULSE_WIDTH_US * cpuFreq;
    rangeCycles = maxCycles - minCycles;

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);

    pinMode(PULSE_INPUT_PIN, INPUT);  // external driver expected
    attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), measurePulse, CHANGE);
}

void loop() {
    uint32_t capturedWidth = 0;
    bool haveSample = false;

    // Atomic handoff from ISR to loop
    portENTER_CRITICAL(&pulseMux);
    if (newPulseAvailable) {
        capturedWidth = pulseWidthCycles;
        newPulseAvailable = false;
        haveSample = true;
    }
    portEXIT_CRITICAL(&pulseMux);

    if (haveSample) {
        if (capturedWidth < minCycles) capturedWidth = minCycles;
        if (capturedWidth > maxCycles) capturedWidth = maxCycles;

        uint64_t val = (uint64_t)(capturedWidth - minCycles) * 65535ULL;
        uint16_t dacValue16bit = (uint16_t)(val / rangeCycles);

        dac_output_voltage(DAC_CHANNEL_1, dacValue16bit & 0xFF);
        dac_output_voltage(DAC_CHANNEL_2, dacValue16bit >> 8);
    }
}
