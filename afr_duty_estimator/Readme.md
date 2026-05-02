# AFR and Duty Estimator (OLED)

This project estimates the Air-Fuel Ratio (AFR) and injector duty cycle, visualizing the results on an OLED display.

## Features
- Narrowband O2 sensor voltage monitoring.
- Injector duty cycle calculation from pulse widths.
- Real-time rolling graph of AFR on a 128x64 OLED display.
- Compatible with SH1107 OLED controllers.

## Pinout
- `A0`: Oxygen sensor signal.
- `D2`: Injector signal (active low).
- `SDA/SCL`: OLED display communication.

## Implementation
The code monitors transitions in the O2 sensor and injector signals using `micros()` for high-precision timing. It calculates the period and high-time of each signal to derive duty cycles and estimated AFR.

## Testing
The project includes a C++ unit test suite.
To run the tests:
```bash
cd tests
make run
```
