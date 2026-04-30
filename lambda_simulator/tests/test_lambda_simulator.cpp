#include <iostream>
#include <cassert>
#include <cmath>
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

void test_narrowband() {
    std::cout << "Testing simulator_narrowband.ino..." << std::endl;

    // Set some defaults to avoid division by zero or other issues before setup
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 2048;
    g_mockState.analogValues[NB::POT_MAP_PIN] = 2048;

    NB::setup_nb();

    // Scenario 1: Lean (Low pulse width)
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 0; // 1.0 ms -> ~4.5 mg
    g_mockState.analogValues[NB::POT_MAP_PIN] = 4095; // 100 kPa -> ~591 mg air
    std::cout << "  Running loop for Lean scenario..." << std::endl;
    NB::loop_nb();
    std::cout << "  DAC Value: " << (int)g_mockState.dacValues[DAC_CHANNEL_1] << std::endl;
    assert(g_mockState.dacValues[DAC_CHANNEL_1] < 10);

    // Scenario 2: Rich (High pulse width)
    g_mockState.analogValues[NB::POT_INJ_PULSE_PIN] = 4095; // 15.0 ms
    g_mockState.analogValues[NB::POT_MAP_PIN] = 4095; // 100 kPa
    std::cout << "  Running loop for Rich scenario..." << std::endl;
    NB::loop_nb();
    std::cout << "  DAC Value: " << (int)g_mockState.dacValues[DAC_CHANNEL_1] << std::endl;
    assert(g_mockState.dacValues[DAC_CHANNEL_1] > 60);

    std::cout << "  Narrowband tests passed!" << std::endl;
}

void test_wideband() {
    std::cout << "Testing simulator_wideband.ino..." << std::endl;

    g_mockState.analogValues[WB::POT_INJ_PULSE_PIN] = 2048;
    g_mockState.analogValues[WB::POT_MAP_PIN] = 2048;

    WB::setup_wb();

    // Scenario 1: High AFR (Lean)
    g_mockState.analogValues[WB::POT_INJ_PULSE_PIN] = 0; // 1.0 ms -> 4.5 mg
    g_mockState.analogValues[WB::POT_MAP_PIN] = 2048; // ~60 kPa
    std::cout << "  Running loop for Lean scenario..." << std::endl;
    WB::loop_wb();
    std::cout << "  WB DAC Value: " << (int)g_mockState.dacValues[DAC_CHANNEL_2] << std::endl;
    // WB V mapped 10-20 to 0-5V.
    // Air ~ 300mg. Fuel ~ 4.5mg. AFR ~ 66.
    // Map(66, 10, 20, 0, 5) = 5.0 + (66-20)*... >> 5. Constrained to 5.0.
    // DAC = min(5.0, 3.3)/3.3 * 255 = 255.
    assert(g_mockState.dacValues[DAC_CHANNEL_2] == 255);

    // Scenario 2: Intermediate AFR
    // We want AFR around 15.
    // Air mass at 60kPa, 20C, 50%RH, 0.5L, 0.85VE:
    // P = 60000. sat_P ~ 2338. vapor_P ~ 1169. dry_P ~ 58831.
    // dry_rho = 58831 / (287.05 * 293.15) = 0.699.
    // vapor_rho = 1169 / (461.5 * 293.15) = 0.0086.
    // total_rho = 0.708 kg/m3.
    // air_mass = 0.708 * 0.0005 * 0.85 = 0.000301 kg = 301 mg.
    // For AFR=15, fuel_mass * 0.95 = 301 / 15 = 20.06.
    // fuel_mass = 20.06 / 0.95 = 21.1 mg.
    // From table: 3.5ms -> 21.0 mg.
    // map(3.5, 1, 15, 0, 4095) -> 3.5-1 = 2.5. 2.5/14 * 4095 = 731.
    g_mockState.analogValues[WB::POT_INJ_PULSE_PIN] = 731;
    g_mockState.analogValues[WB::POT_MAP_PIN] = 2048; // (60-20)/80 * 4095 = 2047.5
    std::cout << "  Running loop for Intermediate AFR scenario..." << std::endl;
    WB::loop_wb();
    std::cout << "  WB DAC Value: " << (int)g_mockState.dacValues[DAC_CHANNEL_2] << std::endl;
    // AFR should be around 15.
    // WB V = map(15, 10, 20, 0, 5) = 2.5V.
    // DAC = 2.5/3.3 * 255 = 193.
    assert(g_mockState.dacValues[DAC_CHANNEL_2] > 180 && g_mockState.dacValues[DAC_CHANNEL_2] < 210);

    std::cout << "  Wideband tests passed!" << std::endl;
}

int main() {
    test_narrowband();
    test_wideband();

    std::cout << "\nAll lambda simulator tests passed!" << std::endl;
    return 0;
}
