# Lambda Sensor Simulators with Combustion Model

This sub-project contains two advanced Arduino sketches for the ESP32. Each sketch simulates oxygen sensor outputs by first modeling the physics of an internal combustion engine. They are powerful tools for testing, developing, and calibrating engine management systems in a controlled, repeatable environment without a running engine.

The core principle is to calculate the Air-Fuel Ratio (AFR) based on configurable engine parameters and dynamic inputs (controlled by potentiometers), and then generate the appropriate sensor voltage signal(s).

---

## 1. `simulator_simple.ino` - Narrowband Simulator

This sketch focuses on simulating a single **narrowband** O2 sensor.

*   **Functionality:** It runs a basic combustion model and generates the classic sharp voltage switch of a narrowband sensor: ~0.1V for lean mixtures (Lambda >= 1.0) and ~0.9V for rich mixtures (Lambda < 1.0).
*   **Use Case:** Ideal for testing standard ECUs or fuel controllers that rely on a simple rich/lean signal for feedback.
*   **Controls:**
    *   Pot 1 (`GPIO 32`): Controls Injector Pulse Width.
    *   Pot 2 (`GPIO 33`): Controls Manifold Absolute Pressure.
*   **Output:**
    *   Narrowband Signal: `GPIO 25`

---

## 2. `simulator_wideband.ino` - Wideband & Narrowband Simulator

This sketch uses a more advanced combustion model and provides **dual outputs** for both wideband and narrowband sensors.

*   **Functionality:** The physics model includes factors for Volumetric Efficiency and Combustion Efficiency for a more realistic AFR calculation. It then generates two separate signals based on this AFR.
*   **Use Case:** Perfect for testing advanced systems like standalone ECUs, AFR gauges, or data loggers that use a precise wideband signal, while still providing the standard narrowband signal for other components.
*   **Controls:**
    *   Pot 1 (`GPIO 32`): Controls Injector Pulse Width.
    *   Pot 2 (`GPIO 33`): Controls Manifold Absolute Pressure.
*   **Outputs:**
    *   Narrowband Signal: `GPIO 25`
    *   Wideband Signal: `GPIO 26`

---

## Configuration and Usage

For both simulators:
1.  **Set Parameters:** Before uploading, open the `.ino` file and configure the engine and environmental parameters at the top of the sketch to match your desired test scenario.
2.  **Connect Hardware:** Connect two 10k potentiometers to the specified GPIOs for dynamic control.
3.  **Monitor Output:** Connect the relevant DAC output pin(s) to the device you are testing. You can also view detailed debug information via the Arduino Serial Monitor at 115200 baud.

### Important Note on Wideband Voltage

The ESP32's built-in DAC has a maximum output of **3.3V**. The wideband simulator will correctly calculate a signal for a full 0-5V range, but the actual voltage on the output pin will be capped at 3.3V. If your testing requires a true 0-5V signal, an external op-amp circuit will be needed to boost the DAC output.
