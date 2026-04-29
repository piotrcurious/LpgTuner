# Split Bank Lambda Controller

This project implements a fuel injection offset controller for an ESP32, designed to balance Air-Fuel Ratios (AFR) across two cylinder subgroups.

## Overview
The system assumes a 4-cylinder engine with two exhaust manifolds, each equipped with a narrowband lambda probe.
- **Subgroup 1 Target:** Slightly lean (~0.45V)
- **Subgroup 2 Target:** Slightly rich (~0.65V)

The controller uses **Sliding Mode Control (SMC)** to handle the highly non-linear, switching behavior of narrowband lambda sensors. SMC provides robustness against system disturbances and non-linearities that traditional PID controllers often struggle with.

## Hardware Configuration (ESP32)
- `GPIO 35`: Analog input from Lambda Sensor 1
- `GPIO 36`: Analog input from Lambda Sensor 2
- `GPIO 25 (DAC1)`: Injection offset output for Subgroup 1
- `GPIO 26 (DAC2)`: Injection offset output for Subgroup 2
- `GPIO 27`: Emulation Probe 1 Condition (Drift Direction)
- `GPIO 14`: Emulation Probe 1 Enable
- `GPIO 12`: Emulation Probe 2 Condition (Saturation Direction)
- `GPIO 13`: Emulation Probe 2 Enable

## Logic & Safety
- **SMC with Boundary Layer:** Reduces high-frequency chattering by using a linear saturation region around the switching point.
- **Drift Emulation:** If the fuel offsets between banks differ significantly, the controller signals a "Drift" condition to the primary ECU.
- **Limit Emulation:** If the controller reaches its physical DAC output limits, it signals a "Saturation" condition.
- **High-Impedance Inactive State:** Emulation pins are kept as high-impedance inputs when not actively signaling an error, preventing interference with the ECU's normal operation.

## Testing & Simulation
The project includes a comprehensive C++ based physics simulation environment in the `sim/` directory. It models engine dynamics, transport delays, and sensor characteristics.

### Running Tests
To build and run the automated test suite:
```bash
python3 split_bank_lambda/sim/run_tests.py
```
This will run multiple scenarios (Balanced, Imbalance, Saturation, Transient) and report pass/fail results.

### Visualization
To generate a plot of the most recent simulation run (Transient scenario):
```bash
python3 split_bank_lambda/sim/plot_results.py
```
(Requires `matplotlib` and `csv` modules).

## Directory Structure
- `split_bank_lambda.ino`: Main Arduino sketch.
- `ControllerLogic.h/cpp`: Core control algorithm (shared between hardware and simulation).
- `sim/`: Simulation environment and test scripts.
- `old/`: Original proof-of-concept code.
