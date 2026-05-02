# LPG Flow Rate Calculator

This project implements a self-calibrating linearization function and predictor for LPG (Liquefied Petroleum Gas) flow rate in automotive applications. It uses physical laws (Ideal Gas Law, Bernoulli's Equation) combined with empirical correction tables to provide accurate estimations.

## Features
- Gas density calculation based on pressure and temperature.
- Flow rate estimation through a nozzle.
- Predictor for gas pressure based on manifold pressure.
- Dynamic correction tables for non-linear system responses.
- Smoothing via history arrays.

## Physics Model
- **Ideal Gas Law**: Used to estimate density.
- **Bernoulli's Equation**: Used to calculate instantaneous flow velocity through the injector nozzle.
- **Interpolation**: Used for mapping manifold vacuum to pressure correction factors.

## Testing
The project includes a C++ unit test suite verifying the physics calculations.
To run the tests:
```bash
cd tests
make run
```
