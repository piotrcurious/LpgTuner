# LPG Flow Rate Estimator

This project provides an alternative LPG flow rate estimation using the Hagen-Poiseuille model for laminar flow.

## Features
- Volume linearization based on lookup tables.
- Pressure prediction using manifold vacuum.
- Flow rate estimation using Hagen-Poiseuille equation.
- Unit tested physics model.

## Physics Model
The core of this estimator is the Hagen-Poiseuille equation, which models the pressure drop in a fluid flowing through a cylindrical pipe. This is applied here to the injector nozzle to estimate gas flow rate under varying pressure and temperature conditions.

## Testing
The project includes a C++ unit test suite.
To run the tests:
```bash
cd tests
make run
```
