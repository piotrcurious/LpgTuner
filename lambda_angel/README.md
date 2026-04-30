# Lambda Angel

Lambda Angel is a diagnostic and monitoring tool for engine exhaust gas parameters, specifically designed for use with Lambda (AFR) sensors and EGT (Exhaust Gas Temperature) sensors. It provides real-time visualization of engine data on OLED or TFT displays.

## Features

- **Real-time Monitoring**: High-speed acquisition and display of Lambda and Temperature data.
- **Engine Mapping**: (In advanced versions) Logs data into RPM vs Load (MAP) heat maps for engine tuning.
- **Multiple Layouts**: Toggle between dashboard views, historical graphs, and heat maps.
- **Hardware Support**: Designed for ESP32 (TFT versions (TFT_eSPI and LovyanGFX)) and ESP8266/ESP32 (OLED versions).

## Project Structure

- `lambda_angel.ino`: Original version for SSD1306 OLED displays.
- `lambda_angel_tft.ino`: Ported version for 320x240 TFT displays using TFT_eSPI.
- `lambda_angel_tft_map.ino`: TFT version with basic data logging.
- `lambda_angel_tft_map3.ino`: Most advanced version featuring RPM/MAP-indexed heat maps for Temperature and Lambda.
- `lambda_angel_lovyangfx.ino`: Port of the advanced version to the LovyanGFX library for improved performance.

## Hardware Requirements

- **Microcontroller**: ESP32 (recommended) or ESP8266.
- **Display**: 128x64 SSD1306 OLED or 320x240 ILI9341/ST7789 TFT.
- **EGT Sensor**: MAX6675 thermocouple interface.
- **Lambda Sensor**: Narrowband or Wideband (via analog controller output).
- **MAP Sensor**: Standard 0-5V analog manifold pressure sensor.
- **RPM Input**: Pulse signal from ECU or ignition coil (optically isolated recommended).

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

## How it Works

The system uses a flexible "Widget" architecture. Each display layout is defined as an array of widgets, where each widget is mapped to a data source (Temp, Lambda, RPM, etc.) and a display mode (Decimal, Bar, Graph, Gauge, or Heat Map).

The Heat Map mode is particularly useful for tuning, as it automatically averages sensor readings into bins based on the current engine state (RPM and Load), allowing the tuner to see lean/rich spots or high-temp areas across the entire operating range.

## Development Status

Current versions include both `TFT_eSPI` and `LovyanGFX` implementations. Future improvements include porting to `LovyanGFX` for enhanced performance and DMA-based rendering.
