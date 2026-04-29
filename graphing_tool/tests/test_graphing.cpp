#include "mock_arduino.h"
#include "mock_tft.h"
#include <cassert>
#include <iostream>

// Re-declare mock functions for testing
void setAnalogRead(int pin, int value);
void setDigitalRead(int pin, int value);
void triggerInterrupt(int interrupt);
void advance_micros(unsigned long us);

// Include the .ino file - we need to trick it a bit
#define setup arduino_setup
#define loop arduino_loop
#include "../graphing_esp32_tft_more.ino"
#undef setup
#undef loop

void test_rpm_calculation() {
    std::cout << "Testing RPM calculation..." << std::endl;

    // Reset state
    lastCamTime = 0;
    camPeriod = 1000000;
    newCamData = false;
    rpm = 0;

    // Simulate first pulse
    advance_micros(1000000);
    camISR();
    assert(lastCamTime == 1000000);
    assert(newCamData == false); // first pulse doesn't give period

    // Simulate second pulse 20ms later (3000 RPM)
    advance_micros(20000); // 20ms = 20000us
    camISR();
    assert(newCamData == true);
    assert(camPeriod == 20000);

    readSensors();
    assert(rpm == 3000.0);

    std::cout << "RPM calculation test passed!" << std::endl;
}

void test_pulse_width_measurement() {
    std::cout << "Testing pulse width measurement..." << std::endl;

    // Reset state
    injectorOnTime = 0;
    injectorOffTime = 0;
    measuredPulseWidth = 0;
    newPulseData = false;
    pulseWidth = 0;

    // Simulate injector ON
    setDigitalRead(injectorPin, HIGH);
    advance_micros(500000); // Start at 500ms
    unsigned long startTime = micros();
    injectorISR();

    // Simulate injector OFF after 2ms
    advance_micros(2000);
    setDigitalRead(injectorPin, LOW);
    injectorISR();

    assert(newPulseData == true);
    assert(measuredPulseWidth == 2000);

    measureInjectorPulseWidth();
    assert(pulseWidth == 2.0);

    std::cout << "Pulse width measurement test passed!" << std::endl;
}

void test_sensor_mapping() {
    std::cout << "Testing sensor mapping..." << std::endl;

    // Mock analog reads
    setAnalogRead(throttlePin, 2048); // ~50%
    setAnalogRead(mapPin, 1024);     // ~25%
    setAnalogRead(lambdaPin, 2048);   // ~1.0

    readSensors();

    // throttlePos = map(2048, 0, 4095, 0, 100) = 50
    assert(throttlePos >= 49 && throttlePos <= 51);

    // mapPressure = map(1024, 0, 4095, 0, 100) = 25
    assert(mapPressure >= 24 && mapPressure <= 26);

    // lambdaValue = map(2048, 0, 4095, 50, 150) / 100.0 = 1.0
    assert(lambdaValue >= 0.99 && lambdaValue <= 1.01);

    std::cout << "Sensor mapping test passed!" << std::endl;
}

void test_mode_switching() {
    std::cout << "Testing mode switching..." << std::endl;

    currentMode = MODE_SCATTER_MAP;
    buttonPressed = false;
    lastButtonPress = 0;

    // Simulate button press
    advance_micros(1000000); // 1s later
    buttonISR();
    assert(buttonPressed == true);

    // In loop, it should switch
    // Note: switchVisualizationMode has a delay(1000).
    // In our mock, delay(1000) advances _mock_millis by 1000.

    switchVisualizationMode();
    assert(currentMode == MODE_HEAT_MAP);

    std::cout << "Mode switching test passed!" << std::endl;
}

int main() {
    arduino_setup();

    test_rpm_calculation();
    test_pulse_width_measurement();
    test_sensor_mapping();
    test_mode_switching();

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
