#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "../graphing_tool_bluedisplay.ino"

// Global flags to track library calls
bool drawPixelCalled = false;
bool drawTextCalled = false;

// Override BlueDisplay methods for testing
void BlueDisplay::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    drawPixelCalled = true;
}

uint16_t BlueDisplay::drawText(uint16_t x, uint16_t y, const char* text, uint16_t size, uint16_t color, uint16_t bgColor) {
    drawTextCalled = true;
    return 0;
}

void test_sensor_readings() {
    std::cout << "Testing sensor readings..." << std::endl;
    mock_arduino_init();

    set_analog_read(throttlePin, 512); // ~50%
    set_analog_read(mapPin, 1023); // 100%
    set_analog_read(lambdaPin, 512);

    // Simulate some cam pulses (6000 RPM = 100 Hz = 10ms period)
    set_micros(100000);
    camISR();
    set_micros(110000);
    camISR();

    // Simulate injector pulse (2ms) - Active LOW
    set_micros(110000);
    set_digital_read(injectorPin, LOW); // ON
    injectorISR();
    set_micros(112000);
    set_digital_read(injectorPin, HIGH); // OFF
    injectorISR();

    readSensors();

    std::cout << "RPM: " << rpm << std::endl;
    std::cout << "PulseWidth: " << pulseWidthMs << " ms" << std::endl;
    std::cout << "Throttle: " << throttle << " %" << std::endl;

    assert(std::abs(rpm - 6000) < 1);
    assert(std::abs(pulseWidthMs - 2.0) < 0.1);
    assert(std::abs(throttle - 50.0) < 1.0);
}

void test_timeouts() {
    std::cout << "Testing timeouts..." << std::endl;
    mock_arduino_init();

    // Set initial state
    set_micros(100000);
    camISR();
    set_micros(110000);
    camISR();

    set_digital_read(injectorPin, LOW);
    injectorISR();
    set_micros(112000);
    set_digital_read(injectorPin, HIGH);
    injectorISR();

    readSensors();
    assert(rpm > 0);
    assert(pulseWidthMs > 0);

    // Wait for timeout (0.6s)
    set_micros(712000);
    readSensors();

    std::cout << "RPM after timeout: " << rpm << std::endl;
    std::cout << "PulseWidth after timeout: " << pulseWidthMs << std::endl;

    assert(rpm == 0);
    assert(pulseWidthMs == 0);
}

void test_throttling() {
    std::cout << "Testing throttling..." << std::endl;
    mock_arduino_init();
    lastDrawTime = 0;
    lastTextUpdateTime = 0;
    lastDrawnRpm = -1;
    lastDrawnLoad = -1;

    // Set engine state
    rpm = 3000;
    engineLoad = 50;
    pulseWidthMs = 5.0;
    lambda = 1.0;

    // First call should trigger draw
    drawPixelCalled = false;
    drawTextCalled = false;
    set_millis(200); // Set to 200ms to satisfy drawCurrentPulseWidth throttling
    drawChart();
    drawCurrentPulseWidth();
    assert(drawPixelCalled == true);
    assert(drawTextCalled == true);

    // Immediate second call should NOT trigger draw
    drawPixelCalled = false;
    drawTextCalled = false;
    set_millis(210);
    drawChart();
    drawCurrentPulseWidth();
    assert(drawPixelCalled == false);
    assert(drawTextCalled == false);

    // Significant change in RPM should trigger drawChart even before 50ms
    rpm = 3100;
    drawPixelCalled = false;
    set_millis(30);
    drawChart();
    assert(drawPixelCalled == true);

    // After 200ms from the first draw (at 200ms), i.e., at 401ms, drawText should trigger again
    drawTextCalled = false;
    set_millis(401);
    drawCurrentPulseWidth();
    assert(drawTextCalled == true);
}

void test_debounce() {
    std::cout << "Testing ISR debounce..." << std::endl;
    mock_arduino_init();
    lastCamTime = 0;
    lastValidCamTime = 0;
    camPeriod = 0;

    set_micros(100000);
    camISR();
    assert(lastCamTime == 100000);
    assert(lastValidCamTime == 100000);

    // This pulse is too fast (1ms < 4ms)
    set_micros(101000);
    camISR();
    assert(lastCamTime == 100000);
    assert(lastValidCamTime == 100000);
    assert(camPeriod == 0);

    // This pulse is okay (5ms > 4ms)
    set_micros(105000);
    camISR();
    assert(lastCamTime == 105000);
    assert(lastValidCamTime == 105000);
    assert(camPeriod == 5000);
}

int main() {
    test_sensor_readings();
    test_timeouts();
    test_throttling();
    test_debounce();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
