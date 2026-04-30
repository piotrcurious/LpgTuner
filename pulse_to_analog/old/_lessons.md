# Pulse-to-Analog Archive

This directory contains older versions of the pulse-to-analog converter.

## History
- `pulse_to_analog_fast.ino`: Initial integer-based version. Good speed, standard resolution.
- `pulse_to_analog_accurate.ino`: Initial floating-point version. High precision but slower.
- `esp32_simple2.ino`: A simplified variant of the fast version.

These have been superseded by the `ultra` versions in the parent directory, which use the ESP32 internal cycle counter for sub-microsecond precision.
