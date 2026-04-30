/*
  Pulse-to-16-Bit-Analog Converter for ESP32 - Ultra Version

  This version uses the ESP32's internal cycle counter (CCOUNT) via inline assembly
  for sub-microsecond timing precision. It also utilizes direct register access
  for fast GPIO reading within the Interrupt Service Routine (ISR).

  Timing:
  - At 240MHz, 1 cycle = 4.16 nanoseconds.
  - micros() has a resolution of 1us. CCOUNT is ~240x more precise.
*/

#include <Arduino.h>
#include <driver/dac.h>

//================================================================================
// --- CONFIGURATION ---
//================================================================================
// Use GPIO 2 for input. Must be 0-31 for the fast REG_READ(GPIO_IN_REG) optimization.
const int PULSE_INPUT_PIN = 2;
const int LSB_DAC_PIN = 25; // DAC Channel 1 (GPIO 25)
const int MSB_DAC_PIN = 26; // DAC Channel 2 (GPIO 26)

const unsigned long MIN_PULSE_WIDTH_US = 0;
const unsigned long MAX_PULSE_WIDTH_US = 20000; // 20 ms

//================================================================================

// Volatiles for ISR communication
volatile uint32_t pulseStartCycles = 0;
volatile uint32_t pulseWidthCycles = 0;
volatile bool newPulseAvailable = false;

// Pre-calculated limits in cycles
uint32_t minCycles;
uint32_t maxCycles;
uint32_t rangeCycles;

#ifndef UNIT_TEST
#define READ_CCOUNT(now) asm volatile("rsr %0, ccount" : "=a"(now))
#else
#define READ_CCOUNT(now) now = mock_ccount
#endif

/**
 * ISR: Highly optimized pulse measurement
 * Uses IRAM_ATTR to ensure the code is in RAM for speed.
 */
void IRAM_ATTR measurePulse() {
    uint32_t now;
    // Read Xtensa Cycle Count register
    READ_CCOUNT(now);

    // Read GPIO input register directly (fastest way)
    // GPIO_IN_REG handles pins 0-31
    if (REG_READ(GPIO_IN_REG) & (1 << PULSE_INPUT_PIN)) {
        // Rising edge
        pulseStartCycles = now;
    } else {
        // Falling edge
        pulseWidthCycles = now - pulseStartCycles;
        newPulseAvailable = true;
    }
}

void setup() {
    // Initial baud rate for debugging if needed
    // Serial.begin(115200);

    // Get CPU frequency to convert US to cycles
    uint32_t cpuFreq = getCpuFrequencyMhz();
    minCycles = MIN_PULSE_WIDTH_US * cpuFreq;
    maxCycles = MAX_PULSE_WIDTH_US * cpuFreq;
    rangeCycles = maxCycles - minCycles;

    // Enable DACs
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);

    // Configure Input Pin
    pinMode(PULSE_INPUT_PIN, INPUT);

    // Attach Interrupt
    attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), measurePulse, CHANGE);
}

void loop() {
    if (newPulseAvailable) {
        // Capture a local copy and reset flag
        uint32_t capturedWidth = pulseWidthCycles;
        newPulseAvailable = false;

        // Constrain
        if (capturedWidth < minCycles) capturedWidth = minCycles;
        if (capturedWidth > maxCycles) capturedWidth = maxCycles;

        // Map to 16-bit value (0-65535)
        // Using 64-bit intermediate to prevent overflow:
        // (cycles * 65535) can exceed 32-bit (at 240MHz, 20ms is 4.8M cycles. 4.8M * 64K = 314G > 4G)
        uint64_t val = (uint64_t)(capturedWidth - minCycles) * 65535;
        uint16_t dacValue16bit = (uint16_t)(val / rangeCycles);

        // Split into two 8-bit values
        uint8_t lsb = dacValue16bit & 0xFF;
        uint8_t msb = (dacValue16bit >> 8) & 0xFF;

        // Update DACs
        dac_output_voltage(DAC_CHANNEL_1, lsb);
        dac_output_voltage(DAC_CHANNEL_2, msb);
    }
}
