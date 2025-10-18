# Fuel Computer

This project is an Arduino sketch that calculates vehicle fuel consumption. It is designed to run on a microcontroller (like an Arduino or ESP32) connected to various engine sensors. The code calculates fuel usage in two common metrics: **Liters per Hour (L/h)** and **Liters per 100 Kilometers (L/100km)**.

## Functionality

The `fuel_computer.ino` sketch provides two main functions:

1.  `calculateFuelConsumption_LitersPerHour`: This function calculates the instantaneous fuel consumption rate, which is useful for understanding fuel usage while idling or at a constant load.
2.  `calculateFuelEconomy_LitersPer100km`: This function calculates the standard fuel economy metric, which tells you how much fuel the vehicle uses to travel 100 kilometers.

## How It Works

The calculation is based on three key pieces of information from the engine:

*   **Engine RPM (Revolutions Per Minute):** The rotational speed of the engine.
*   **Injector Pulse Width (in milliseconds):** The amount of time a fuel injector is open and spraying fuel into a cylinder.
*   **Vehicle Speed (in km/h):** The speed of the vehicle.

The core logic assumes a standard 4-stroke engine, where each cylinder performs an injection once every two engine revolutions. By knowing how long the injector is open and how often it opens, we can calculate the total volume of fuel being consumed.

### Formulas Used

*   **Injections per Minute:** `RPM / 2`
*   **Total Fuel per Hour (L/h):** `(Injections per Minute * Injection Pulse Width * Injector Flow Rate * 60)`
*   **Fuel Economy (L/100km):** `(Liters per Hour / Vehicle Speed) * 100`

## Configuration

To get accurate readings, you **must** configure the script for your specific vehicle.

Inside `fuel_computer.ino`, find the following constant:

```cpp
const float INJECTOR_FLOW_RATE_L_PER_MIN = 0.5; // Liters per minute
```

You need to change the `0.5` value to the flow rate of your vehicle's fuel injectors. This information can usually be found in your vehicle's service manual or on the injector manufacturer's website. The value should be in **Liters per Minute**.

## Hardware & Connection

This code is designed to be run on a microcontroller as part of a larger vehicle monitoring system. You will need to have sensors to provide the required inputs:

*   An RPM sensor (e.g., a connection to the vehicle's tachometer signal or a sensor on the crankshaft).
*   A method to measure the injector pulse width (e.g., by tapping into the signal wire of a fuel injector).
*   A speed sensor (e.g., from the vehicle's VSS wire).

The `setup()` and `loop()` functions in the sketch provide an example of how to call the calculation functions and print the results to the Serial monitor for debugging. In a real-world application, you would replace the example static values with live data from your sensors.
