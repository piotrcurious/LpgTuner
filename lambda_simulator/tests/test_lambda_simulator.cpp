#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <cstdlib>
#include "mock_arduino.h"

// Namespace-like trick to avoid name collisions between narrowband and wideband simulations
namespace NB {
    #define setup setup_nb
    #define loop loop_nb
    #include "../new/simulator_narrowband.ino"
    #undef setup
    #undef loop
}

namespace WB {
    #define setup setup_wb
    #define loop loop_wb
    #include "../new/simulator_wideband.ino"
    #undef setup
    #undef loop
}

bool is_close(float a, float b, float epsilon = 0.01f) {
    return std::abs(a - b) < epsilon;
}

void test_rpm_dependent_delay() {
    std::cout << "Testing RPM-dependent transport delay..." << std::endl;

    g_mockState = MockState();
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 0; // Lean
    g_mockState.analogValues[NB::POT_MAP_PIN] = 2048;
    g_mockState.analogValues[NB::POT_RPM_PIN] = 0; // 500 RPM -> delaySteps = 50

    NB::filteredInjVal = 0.0f; // Force steady state lean
    NB::filteredMapVal = 2048.0f;
    NB::filteredRpmVal = 0.0f;

    NB::setup_nb();
    delay(6000); // Warm up

    // Fill buffer and reach steady state lean
    for(int i=0; i<200; i++) NB::loop_nb();

    float v_steady_lean = (float)g_mockState.dacValues[DAC_CHANNEL_1] / 255.0f * 3.3f;
    assert(v_steady_lean < 0.2f);

    std::cout << "  Transitioning to rich..." << std::endl;
    // Change to rich
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 4095;

    // Expect lean output for 50 steps
    for(int i=0; i<50; i++) {
        NB::loop_nb();
        float v = (float)g_mockState.dacValues[DAC_CHANNEL_1] / 255.0f * 3.3f;
        // Should still be lean because delay is 50 steps
        assert(v < 0.4f);
    }

    // Step 51 should start showing rich
    for(int i=0; i<100; i++) NB::loop_nb();
    float v_rich = (float)g_mockState.dacValues[DAC_CHANNEL_1] / 255.0f * 3.3f;
    assert(v_rich > 0.6f);

    std::cout << "  RPM-dependent delay test passed!" << std::endl;
}

void test_sensor_aging() {
    std::cout << "Testing sensor aging (sluggishness)..." << std::endl;

    g_mockState = MockState();
    NB::setup_nb();
    delay(6000);

    NB::agedOutputVoltage = 0.1f;
    float target = 0.9f;
    // Apply aging logic manually to verify formula
    float nextV = (NB::SENSOR_AGING_FACTOR * target) + ((1.0f - NB::SENSOR_AGING_FACTOR) * NB::agedOutputVoltage);
    assert(is_close(nextV, 0.42f));

    std::cout << "  Sensor aging test passed!" << std::endl;
}

int main() {
    test_rpm_dependent_delay();
    test_sensor_aging();

    std::cout << "\nAll advanced lambda simulator tests passed!" << std::endl;
    return 0;
}
