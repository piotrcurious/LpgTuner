#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
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

void test_narrowband_improved() {
    std::cout << "Testing improved simulator_narrowband.ino..." << std::endl;

    // Reset state
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 2048;
    g_mockState.analogValues[NB::POT_MAP_PIN] = 2048;
    NB::filteredInjVal = 2048.0f;
    NB::filteredMapVal = 2048.0f;

    NB::setup_nb();

    // 1. Test Filtering (EMA)
    // Send a step change
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 4095;
    NB::loop_nb();
    // After 1 loop with alpha=0.2, filtered should be 0.2*4095 + 0.8*2048 = 819 + 1638.4 = 2457.4
    assert(is_close(NB::filteredInjVal, 2457.4f, 1.0f));

    // 2. Test Transport Delay
    // Fill buffer with lean values
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 0; // Very lean
    for (int i = 0; i < 20; i++) NB::loop_nb(); // Alpha is 0.2, so after 20 loops it should be very close to 0

    // Now switch to rich
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 4095;
    NB::loop_nb();
    // Instantaneous should be richer, but output should still be lean due to delay
    // (Actual check depends on buffer state, let's verify delay buffer logic)
    float firstRichLambda = NB::lambdaBuffer[(NB::bufferIndex + NB::DELAY_BUFFER_SIZE - 1) % NB::DELAY_BUFFER_SIZE];
    assert(firstRichLambda < 1.0f); // Should be rich-ish (low lambda)

    // 3. Test Sigmoid Output
    // Lambda 1.0 -> 0.1 + 0.8/(1+1) = 0.1 + 0.4 = 0.5V
    float v_at_1 = NB::getNarrowbandVoltage(1.0f);
    assert(is_close(v_at_1, 0.5f));

    // Lambda 1.1 (Lean) -> 0.1 + 0.8/(1+exp(25*0.1)) = 0.1 + 0.8/(1+12.18) ~ 0.16V
    float v_lean = NB::getNarrowbandVoltage(1.1f);
    assert(v_lean < 0.2f && v_lean > 0.1f);

    std::cout << "  Narrowband improved tests passed!" << std::endl;
}

void test_wideband_improved() {
    std::cout << "Testing improved simulator_wideband.ino..." << std::endl;

    WB::filteredInjVal = 2048.0f;
    WB::filteredMapVal = 2048.0f;
    WB::setup_wb();

    // Scenario: Rich step
    g_mockState.analogValues[WB::POT_INJ_PULSE_PIN] = 4095; // 15ms
    g_mockState.analogValues[WB::POT_MAP_PIN] = 2048; // ~60kPa

    for(int i=0; i<50; i++) WB::loop_wb(); // Reach steady state

    // WB V should be low (Rich)
    // 15ms - 0.8 dead time = 14.2ms. 14.2 * 8.5 = 120.7 mg fuel.
    // Air ~ 300mg. AFR ~ 300 / (120.7 * 0.98) ~ 2.5.
    // AFR 2.5 is way below 10.0 min. WB V should be 0V.
    assert(g_mockState.dacValues[DAC_CHANNEL_2] == 0);

    std::cout << "  Wideband improved tests passed!" << std::endl;
}

int main() {
    test_narrowband_improved();
    test_wideband_improved();

    std::cout << "\nAll improved lambda simulator tests passed!" << std::endl;
    return 0;
}
