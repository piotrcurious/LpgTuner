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

    // Simulate second pulse 20ms later (3000 RPM if PPR=1)
    advance_micros(20000); // 20ms = 20000us
    camISR();
    assert(newCamData == true);
    assert(camPeriod == 20000);

    readSensors();
    assert(rpm == (3000.0 / PULSES_PER_REV));

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

void test_heatmap_update() {
    std::cout << "Testing Heat Map update..." << std::endl;

    clearHeatMap();
    rpm = 1000; // Col 2 (1000/500)
    throttlePos = 100;
    mapPressure = 44.45; // Row 4 (44.45 / 11.11 = 4.0009)
    pulseWidth = 5.0;
    lambdaValue = 1.0;

    drawHeatMap();

    int col = constrain((int)(rpm / 500), 0, HEATMAP_COLS - 1);
    float load = (mapPressure * throttlePos) / 100.0;
    int row = constrain((int)(load / 11.11), 0, HEATMAP_ROWS - 1);
    std::cout << "RPM: " << rpm << " -> Col: " << col << std::endl;
    std::cout << "Load: " << load << " -> Row: " << row << std::endl;

    assert(heatMap[col][row].count == 1);
    assert(heatMap[col][row].avgPulseWidth == 5.0f);

    // Add another point in same cell
    pulseWidth = 7.0;
    drawHeatMap();
    assert(heatMap[col][row].count == 2);
    // EMA: 5.0 * 0.9 + 7.0 * 0.1 = 4.5 + 0.7 = 5.2
    assert(std::abs(heatMap[col][row].avgPulseWidth - 5.2f) < 0.001f);

    std::cout << "Heat Map update test passed!" << std::endl;
}

void test_color_functions() {
    std::cout << "Testing color functions..." << std::endl;

    // 0ms should be Blue
    uint16_t c0 = getColorForPulseWidth(0);
    // Blue in RGB565: R=0, G=0, B=31 -> 0x001F
    assert(c0 == 0x001F);

    // 15ms should be Red (new full scale)
    uint16_t c15 = getColorForPulseWidth(15);
    // Red in RGB565: R=31, G=0, B=0 -> 0xF800
    assert(c15 == 0xF800);

    // Lambda tests
    uint16_t cStoich = getColorForAFR(1.0);
    assert(cStoich == TFT_GREEN);

    uint16_t cRich = getColorForAFR(0.8);
    assert(cRich == TFT_RED);

    uint16_t cLean = getColorForAFR(1.3);
    assert(cLean == TFT_BLUE);

    std::cout << "Color functions test passed!" << std::endl;
}

void test_density_map() {
    std::cout << "Testing Density Map..." << std::endl;

    clearHeatMap();
    rpm = 500; // Col 1
    throttlePos = 100;
    mapPressure = 11.12; // Row 1

    drawDensityMap();
    int col = constrain((int)(rpm / 500), 0, HEATMAP_COLS - 1);
    float load = (mapPressure * throttlePos) / 100.0;
    int row = constrain((int)(load / 11.11), 0, HEATMAP_ROWS - 1);
    std::cout << "RPM: " << rpm << " -> Col: " << col << std::endl;
    std::cout << "Load: " << load << " -> Row: " << row << std::endl;

    assert(heatMap[col][row].count == 1);

    drawDensityMap();
    assert(heatMap[1][1].count == 2);

    std::cout << "Density Map test passed!" << std::endl;
}

int main() {
    arduino_setup();

    test_rpm_calculation();
    test_pulse_width_measurement();
    test_sensor_mapping();
    test_mode_switching();
    test_heatmap_update();
    test_color_functions();
    test_density_map();

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
