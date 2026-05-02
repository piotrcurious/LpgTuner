# Injector Kalman Filter

This project implements a Kalman filter to estimate the state of an engine's fuel injection system. It processes multiple sensor inputs to provide filtered estimates of pulse width, Air-Fuel Ratio (AFR), fuel flow rate, and engine torque.

## Features
- 4-state Kalman Filter: `[Pulse Width, AFR, Fuel Flow, Engine Torque]`.
- Fuses data from Throttle Position, MAP, Lambda, and RPM sensors.
- Real-time prediction and update cycles.
- RPM calculation via high-resolution cam sensor timing.

## State Estimation
The filter uses a prediction model based on process noise and an update model that compares measurements against predicted states. This provides a robust way to filter out sensor noise and provide a coherent view of the engine's operation.

## Testing
The project includes a C++ unit test suite to verify the convergence of the Kalman filter.
To run the tests:
```bash
cd tests
make run
```
