# AFR and Correction Estimator

This project analyzes lambda probe and injector signals to estimate Air-Fuel Ratio (AFR) and its deviation from stoichiometry. It also calculates the correlation between the lambda signal and injector duty cycle fluctuations.

## Features
- Buffer-based data analysis (100 samples per calculation).
- State-machine based peak and valley detection with **hysteresis** for noise immunity.
- Robust AFR estimation using a narrowband model with baseline fallbacks.
- Injector signal statistics (mean, standard deviation, fluctuation rate).
- **Cross-correlation** with variable lag (Pearson coefficient) to determine transport delay.
- **EMA filtering** on results for stable readings.

## Pinout
- `A0`: Lambda probe signal.
- `A1`: Injector signal.

## How it works
The code samples the signals at 100Hz and stores them in a buffer. Once the buffer is full, it performs statistical analysis to find correlations between the O2 sensor's feedback and the injector's response, which is useful for tuning and diagnosing fuel systems.

## Testing
The project includes a C++ unit test suite using a mock Arduino environment.
To run the tests:
```bash
cd tests
make run
```
