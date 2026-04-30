# Lambda Angel

Lambda Angel is a diagnostic and monitoring tool for engine exhaust gas parameters, specifically designed for use with Lambda (AFR) sensors and EGT (Exhaust Gas Temperature) sensors. It provides real-time visualization of engine data on OLED or TFT displays.

## Features

- **Real-time Monitoring**: High-speed acquisition and display of Lambda and Temperature data.
- **Engine Mapping**: Logs data into RPM vs Load (MAP) heat maps for engine tuning.
- **Multiple Layouts**: Toggle between dashboard views and historical engine maps.
- **High Performance**: Optimized implementations for TFT_eSPI and LovyanGFX.
- **Shared Logic**: Centralized core calculations for consistency and testability.

## Project Structure

- `lambda_angel_oled/`: Version for 128x64 SSD1306 OLED displays.
- `lambda_angel_tft/`: Version for 320x240 TFT displays using TFT_eSPI.
- `lambda_angel_tft_map3/`: Advanced TFT version with RPM/MAP-indexed heat maps.
- `lambda_angel_lovyangfx/`: Port of the advanced version to the LovyanGFX library for improved performance.
- `tests/`: C++ unit testing framework for core logic.

## Shared Logic (`CommonLogic.h`)

All project versions share a common header that implements:
- Physical unit conversions (e.g., MAP voltage to kPa).
- Engine state binning (RPM and Load indexing).
- Low-pass filtering for sensor stability.
- Dynamic color mapping for Temperature and Lambda visualization.
- Recursive averaging for engine heat maps.

## Hardware Requirements

- **Microcontroller**: ESP32 (recommended) or ESP8266.
- **Display**: 128x64 SSD1306 OLED or 320x240 ILI9341/ST7789 TFT.
- **EGT Sensor**: MAX6675 thermocouple interface.
- **Lambda Sensor**: Narrowband or Wideband (via analog controller output).
- **MAP Sensor**: Standard 0-5V analog manifold pressure sensor.
- **RPM Input**: Pulse signal (optically isolated recommended).

## Pinout (ESP32 Example)

| Component | Pin |
|-----------|-----|
| TFT CS | 15 |
| TFT DC | 2 |
| TFT RST | 4 |
| MAX6675 CS | 5 |
| Lambda Input | 34 |
| MAP Input | 35 |
| RPM Input | 32 |

## Testing

Core logic is verified using a C++ unit test suite. To run the tests:
```bash
cd lambda_angel/tests
make run
```
