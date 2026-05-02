# Injector Pulse Width Measurer

This project provides a precise method for measuring the pulse width of an engine's fuel injector using interrupts on an Arduino.

## Features
- High-precision timing via `micros()`.
- Interrupt-driven (supports rising and falling edges).
- Exponential Moving Average (EMA) filtering for stable results.
- Serial output for real-time monitoring.

## Pinout
- `D3`: Injector signal (Interrupt capable pin).

## Implementation
The code uses an Interrupt Service Routine (ISR) to capture the start and end times of each injection pulse. It then calculates the duration and applies a smoothing filter to reduce jitter from electronic noise or variations in engine cycle.

## Testing
The project includes a C++ unit test suite.
To run the tests:
```bash
cd tests
make run
```
