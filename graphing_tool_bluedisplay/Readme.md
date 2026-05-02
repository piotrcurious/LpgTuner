# Graphing Tool (BlueDisplay)

This is a simple map plotting tool to aid in engine tuning. It plots injection times on an RPM/Load map, with the color of the dot varying depending on the injection time.

## Features
- Real-time visualization of injector pulse width.
- RPM calculation from cam sensor pulses.
- Load estimation from MAP sensor.
- Color-coded map (Blue for short pulses, Red for long pulses).
- Compatible with the BlueDisplay Android app.

## Pinout
- `A0`: Throttle position sensor.
- `A1`: MAP sensor.
- `A2`: Lambda probe (optional visualization).
- `D2`: Cam sensor (Interrupt).
- `D3`: Injector signal (Interrupt).
- `D9`: Injector timing analog gauge (PWM).

## Dependencies
- [BlueDisplay Library](https://github.com/ArminJo/Arduino-BlueDisplay)

## Testing
The project includes a C++ unit test suite for the core logic.
To run the tests:
```bash
cd tests
make run
```
