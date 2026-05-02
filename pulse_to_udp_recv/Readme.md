# UDP Pulse Data Receiver

This project is an ESP32-based UDP receiver that listens for multicast packets containing batches of engine pulse data (RPM and pulse length).

## Features
- Joins a multicast group (`224.0.1.187:5683`) to receive data.
- Decodes a packed struct containing up to 100 pulse samples.
- Outputs received data to the Serial monitor for debugging.

## Data Structure
The receiver expects a binary packet in the following format:
```cpp
struct PulseData {
  uint32_t rpm[100];
  uint32_t length[100];
  uint8_t count;
};
```

## Setup
1. Update the `ssid` and `password` variables in `pulse_to_udp_recv.ino` with your WiFi credentials.
2. Flash the code to an ESP32.
3. Ensure the transmitter is on the same network and sending to the multicast address.

## Testing
Manual verification can be performed by sending a mock UDP packet from a PC using a tool like `socat` or a custom Python script.
