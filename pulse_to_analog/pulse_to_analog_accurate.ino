/*
  Pulse-to-16-Bit-Analog Converter for ESP32 - High Accuracy Version

  This sketch measures the duration of an incoming digital pulse and converts it
  into a high-resolution, 16-bit analog voltage.

  This "accurate" version uses floating-point mathematics for the conversion
  logic. This ensures the highest possible precision when mapping the pulse
  duration to the 16-bit output range, avoiding potential integer overflow
  or precision loss that can occur with the standard map() function on large ranges.

  The trade-off is a potential decrease in performance compared to a purely
  integer-based approach, but it is ideal for applications where accuracy is
  more critical than raw speed.
*/

#include <driver/dac.h> // ESP32-specific DAC functions

//================================================================================
// --- CONFIGURATION ---
//================================================================================
const int PULSE_INPUT_PIN = 2;
const int LSB_DAC_PIN = 25; // DAC Channel 1 (GPIO 25)
const int MSB_DAC_PIN = 26; // DAC Channel 2 (GPIO 26)
const unsigned long MIN_PULSE_WIDTH_US = 0;
const unsigned long MAX_PULSE_WIDTH_US = 20000; // 20 ms
// #define ENABLE_SERIAL_DEBUG
//================================================================================

volatile unsigned long pulseStartTime_us = 0;
volatile unsigned long pulseWidth_us = 0;
volatile bool newPulseAvailable = false;

void IRAM_ATTR measurePulse() {
  if (digitalRead(PULSE_INPUT_PIN) == HIGH) {
    pulseStartTime_us = micros();
  } else {
    pulseWidth_us = micros() - pulseStartTime_us;
    newPulseAvailable = true;
  }
}

void setup() {
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
    Serial.println("Pulse-to-Analog Converter (Accurate Version) Initialized.");
  #endif

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);

  pinMode(PULSE_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), measurePulse, CHANGE);
}

void loop() {
  if (newPulseAvailable) {
    newPulseAvailable = false;
    unsigned long currentPulseWidth = pulseWidth_us;
    currentPulseWidth = constrain(currentPulseWidth, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

    // --- High-Accuracy 16-Bit Mapping ---
    // Using floating-point math for the most accurate scaling.
    double mappedValue = (double)(currentPulseWidth - MIN_PULSE_WIDTH_US) * 65535.0 / (double)(MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);
    uint16_t dacValue16bit = (uint16_t)mappedValue;

    uint8_t lsbValue = dacValue16bit & 0xFF;
    uint8_t msbValue = (dacValue16bit >> 8) & 0xFF;

    dac_output_voltage(DAC_CHANNEL_1, lsbValue);
    dac_output_voltage(DAC_CHANNEL_2, msbValue);

    #ifdef ENABLE_SERIAL_DEBUG
      Serial.print("Pulse: "); Serial.print(currentPulseWidth); Serial.print(" us");
      Serial.print(" -> 16-bit: "); Serial.print(dacValue16bit);
      Serial.print(" -> MSB: "); Serial.print(msbValue);
      Serial.print(" -> LSB: "); Serial.println(lsbValue);
    #endif
  }
}
