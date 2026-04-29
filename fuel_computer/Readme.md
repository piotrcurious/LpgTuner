# Trip & Fuel Computer

This project is an advanced Arduino-based Trip Computer that calculates and tracks vehicle fuel consumption and performance metrics. It is designed for high precision and long-term stability, making it suitable for real-world automotive monitoring.

## Features

-   **Instantaneous Metrics:**
    -   Fuel Consumption Rate (Liters per Hour - L/h)
    -   Fuel Economy (Liters per 100 Kilometers - L/100km)
-   **Trip Statistics (Cumulative):**
    -   Total Distance Traveled (km)
    -   Total Fuel Consumed (L)
    -   Average Trip Speed (km/h)
    -   Average Trip Economy (L/100km)
-   **High Precision:** Uses the **Kahan Summation Algorithm** to minimize floating-point rounding errors during long-term accumulation of fuel and distance.
-   **Configurable:** Supports custom injector flow rates and engine cylinder counts.

## How It Works

The computer uses three primary inputs:
1.  **Engine RPM:** To determine how many injection events occur.
2.  **Injector Pulse Width (ms):** To calculate the volume of fuel per injection.
3.  **Vehicle Speed (km/h):** To calculate distance and distance-based economy.

### Mathematical Model

The logic assumes a standard **4-stroke engine**, where each cylinder performs one injection every two engine revolutions.

-   `Injections per Minute = (RPM / 2) * Number of Cylinders`
-   `L/h = (Injections per Minute * Pulse Width * Flow Rate * 60) / 60,000`
-   `L/100km = (L/h / Speed) * 100`

## Configuration

Edit `fuel_computer.ino` to match your vehicle's specifications:

```cpp
const float INJECTOR_FLOW_RATE_L_PER_MIN = 0.5; // Calibrate to your injectors
const int NUM_CYLINDERS = 4;                   // Set your cylinder count
```

## Testing

The project includes a comprehensive C++ test suite using a mock Arduino environment.

### Running Tests

```bash
cd fuel_computer/tests
make run
```

The test suite verifies:
-   Instantaneous calculation accuracy.
-   Numerical stability of Kahan summation over 1,000,000+ iterations.
-   Correct trip statistics accumulation.
-   Handling of edge cases (e.g., engine off, vehicle stopped).

## Hardware & Connections

To use this in a vehicle, you need sensors for:
-   **RPM:** Tachometer signal or crankshaft sensor.
-   **Pulse Width:** Tapped from an injector signal wire.
-   **Speed:** Vehicle Speed Sensor (VSS) or GPS module.

The `loop()` function demonstrates how to process this data. In a real application, you would replace the example values with live sensor readings.
