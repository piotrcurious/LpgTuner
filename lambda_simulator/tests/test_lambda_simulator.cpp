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

void test_narrowband_warmup() {
    std::cout << "Testing narrowband warm-up simulation..." << std::endl;

    // Reset global state for mocks
    g_mockState = MockState();
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 2048;
    g_mockState.analogValues[NB::POT_MAP_PIN] = 2048;

    // Reset simulation state
    NB::filteredInjVal = 2048.0f;
    NB::filteredMapVal = 2048.0f;

    NB::setup_nb(); // startTime_ms = 0

    // 1. Initial state (t=0)
    NB::loop_nb();
    float v0 = (float)g_mockState.dacValues[DAC_CHANNEL_1] / 255.0f * 3.3f;
    std::cout << "  V at t=0: " << v0 << std::endl;
    assert(is_close(v0, 0.45f));

    // 2. Mid warm-up (t=2000ms)
    delay(2000);
    NB::loop_nb();
    float v2 = (float)g_mockState.dacValues[DAC_CHANNEL_1] / 255.0f * 3.3f;
    std::cout << "  V at t=2000: " << v2 << std::endl;
    assert(is_close(v2, 0.45f));

    // 3. Post warm-up (t=6000ms, WARMUP_TIME_MS=5000)
    delay(4000);
    NB::loop_nb();
    float v6 = (float)g_mockState.dacValues[DAC_CHANNEL_1] / 255.0f * 3.3f;
    std::cout << "  V at t=6000: " << v6 << std::endl;
    // At default values (2048, 2048), it should be stoichiometric lambda ~ 1.0 -> 0.5V
    assert(!is_close(v6, 0.45f)); // Should have changed
    assert(is_close(v6, 0.50f, 0.05f));

    std::cout << "  Narrowband warm-up test passed!" << std::endl;
}

void test_wideband_warmup() {
    std::cout << "Testing wideband warm-up simulation..." << std::endl;

    g_mockState = MockState();
    g_mockState.analogValues[WB::POT_INJ_PULSE_PIN] = 2048;
    g_mockState.analogValues[WB::POT_MAP_PIN] = 2048;

    WB::filteredInjVal = 2048.0f;
    WB::filteredMapVal = 2048.0f;

    WB::setup_wb(); // WARMUP_TIME_MS = 8000

    NB::loop_nb(); // Just to sync time if needed, though they use same global

    // t=0
    WB::loop_wb();
    float wb_v0 = (float)g_mockState.dacValues[DAC_CHANNEL_2] / 255.0f * 3.3f;
    assert(wb_v0 == 0.0f);

    // t=9000
    delay(9000);
    WB::loop_wb();
    float wb_v9 = (float)g_mockState.dacValues[DAC_CHANNEL_2] / 255.0f * 3.3f;
    assert(wb_v9 > 0.0f); // Should be active

    std::cout << "  Wideband warm-up test passed!" << std::endl;
}

void test_manifold_dynamics() {
    std::cout << "Testing manifold dynamics (filtering)..." << std::endl;

    g_mockState = MockState();
    NB::filteredMapVal = 2048.0f;
    g_mockState.analogValues[NB::POT_MAP_PIN] = 4095;

    NB::loop_nb();
    // ALPHA = 0.15. 0.15*4095 + 0.85*2048 = 614.25 + 1740.8 = 2355.05
    assert(is_close(NB::filteredMapVal, 2355.05f, 1.0f));

    std::cout << "  Manifold dynamics test passed!" << std::endl;
}

int main() {
    test_narrowband_warmup();
    test_wideband_warmup();
    test_manifold_dynamics();

    std::cout << "\nAll enhanced lambda simulator tests passed!" << std::endl;
    return 0;
}
