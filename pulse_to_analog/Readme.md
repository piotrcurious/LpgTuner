# Pulse-to-16-Bit-Analog Converter

This project provides two alternative ESP32 sketches for converting a digital pulse width into a high-resolution, 16-bit analog signal. Both versions achieve this by combining the ESP32's two 8-bit DACs with an external resistor network.

The two versions offer a trade-off between execution speed and conversion accuracy.

---

## Versions

### 1. `pulse_to_analog_fast.ino` (Fast Version)

This sketch uses integer-based mathematics for the core pulse-to-voltage conversion.

*   **Pros:** Higher performance, lower resource usage.
*   **Cons:** May have minor precision loss when mapping very large or non-standard pulse width ranges.
*   **Best for:** Time-critical applications where raw speed is a priority.
*   **Documentation:** See `pulse_to_analog_fast.readme.md` for details.

### 2. `pulse_to_analog_accurate.ino` (Accurate Version)

This sketch is located in the `esp32_accurate/` subdirectory and uses floating-point mathematics for the conversion.

*   **Pros:** Highest possible precision, ensuring the most accurate mapping from the pulse width to the 16-bit output.
*   **Cons:** More computationally intensive, which may result in slightly higher latency.
*   **Best for:** Applications where accuracy is the primary concern.
*   **Documentation:** See `esp32_accurate/pulse_to_analog_accurate.readme.md` for details.

---

## Core Hardware Requirement (for both versions)

Neither version will work correctly without an external resistor network to combine the DAC outputs. Please refer to the circuit diagrams in the individual `.readme.md` files for each version. The fundamental principle is to create a voltage summer where the MSB DAC has 256 times more influence than the LSB DAC.
