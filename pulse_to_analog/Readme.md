# Pulse-to-16-Bit-Analog Converter (Ultra Edition)

This project converts a digital pulse width into a high-resolution, 16-bit analog signal using an ESP32.

## New High-Precision Versions

The "Ultra" versions leverage the ESP32's internal cycle counter (CCOUNT) and direct register access to achieve sub-microsecond precision (~4ns at 240MHz).

### 1. `pulse_to_analog_ultra.ino`
Uses C++ with cycle-count macros for high-resolution timing and direct GPIO register reads.

### 2. `pulse_to_analog_ultra_asm.ino`
Uses inline assembly for the Interrupt Service Routine (ISR) to achieve the absolute minimum possible latency and jitter.

## Testing
A host-based C++ unit test suite is provided in `tests/`.
To run the tests:
```bash
cd tests
make run
```

## Archive
Older versions (fast, accurate, simple) have been moved to the `old/` directory.

## Hardware Requirement
Requires an external resistor network to combine the two 8-bit DACs (GPIO 25 and 26) into a 16-bit signal.
