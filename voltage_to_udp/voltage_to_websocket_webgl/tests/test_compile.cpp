#include "mock_esp32.h"
#include <iostream>

// Provide instances for externs
SerialMock Serial;
ESPMock ESP;
WiFiMock WiFi;

#include "../wifi_settings.h"
#include "../frontend.h"

// The .ino content
#include "../voltage_to_websocket_webgl.ino"

int main() {
    std::cout << "Starting Mock Compilation Test..." << std::endl;
    setup();
    loop();
    std::cout << "Mock Compilation Test Passed!" << std::endl;
    return 0;
}
