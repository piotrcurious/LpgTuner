# Simple Lambda Probe Monitor

This project implements a simple narrowband lambda probe monitor for Arduino. It estimates the Air-Fuel Ratio (AFR) based on the voltage output of the O2 sensor and calculates the oscillation frequency and duty cycle of the sensor signal.

## Features
- Narrowband O2 sensor voltage to AFR mapping.
- Moving average filtering for stable readings.
- Oscillation frequency and duty cycle measurement.
- Serial output for debugging and monitoring.

## Pinout
- `A0`: Lambda probe signal (0-1V range).
- `A1`: Injector signal (optional, for future expansion).

## How it works
Narrowband O2 sensors oscillate around the stoichiometric point (14.7 for gasoline). This code detects when the AFR crosses 14.7 and uses these crossings to calculate the frequency and duty cycle of the engine's closed-loop fuel control.

## Testing
The project includes a C++ unit test suite using a mock Arduino environment.
To run the tests:
```bash
cd tests
make run
```
