#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <deque>
#include <iomanip>
#include <algorithm>
#include <string>

// Include the actual production logic
#include "../ControllerLogic.h"

// Mock Arduino definitions for the simulation
#define INPUT 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0

struct MockPin {
    int mode = INPUT;
    int state = LOW;
};
MockPin pins[40];
int analogValues[40];
int dacValues[40];

void pinMode(int pin, int mode) { pins[pin].mode = mode; }
void digitalWrite(int pin, int state) { pins[pin].state = state; }
void dacWrite(int pin, int val) { dacValues[pin] = val; }
int analogRead(int pin) { return analogValues[pin]; }
unsigned long _millis = 0;
unsigned long millis() { return _millis; }

// Link to the implementation in ControllerLogic.cpp
extern void runControllerStep();
extern float currentOffset1, currentOffset2, filteredLambda1, filteredLambda2;
extern unsigned long lastUpdate;
extern bool sensor1Active, sensor2Active;

void reset_controller() {
    currentOffset1 = DAC_MID;
    currentOffset2 = DAC_MID;
    filteredLambda1 = 0.5f;
    filteredLambda2 = 0.5f;
    sensor1Active = false;
    sensor2Active = false;
    lastUpdate = 0;
    _millis = 0;
    for(int i=0; i<40; i++) {
        pins[i].mode = INPUT;
        pins[i].state = LOW;
        dacValues[i] = DAC_MID;
    }
}

// --- Enhanced Physics Simulation ---

float afrToVoltage(float afr) {
    float lambda = afr / 14.7f;
    return 0.1f + 0.8f * (0.5f * (1.0f - std::tanh(20.0f * (lambda - 1.0f))));
}

struct EngineModel {
    float baseAfr1 = 14.7f;
    float baseAfr2 = 14.7f;
    float imbalance1 = 0.0f;
    float imbalance2 = 0.0f;
    float rpm = 800.0f;

    std::deque<float> pipe1, pipe2;
    float currentV1 = 0.5f, currentV2 = 0.5f;

    bool sensorsWarming = false;

    void update(float dac1, float dac2) {
        int delaySteps = std::max(2, (int)(24000.0f / rpm));

        float fuel1 = 1.0f + (dac1 - 128.0f) / 128.0f * 0.20f;
        float fuel2 = 1.0f + (dac2 - 128.0f) / 128.0f * 0.20f;

        fuel1 *= (1.0f + imbalance1);
        fuel2 *= (1.0f + imbalance2);

        // Add Cross-talk (10% of fuel from other bank affects this bank)
        float effectiveFuel1 = 0.9f * fuel1 + 0.1f * fuel2;
        float effectiveFuel2 = 0.9f * fuel2 + 0.1f * fuel1;

        float afr1 = baseAfr1 / effectiveFuel1;
        float afr2 = baseAfr2 / effectiveFuel2;

        pipe1.push_back(afr1);
        pipe2.push_back(afr2);

        if (pipe1.size() > delaySteps) {
            float dAfr1 = pipe1.front(); pipe1.pop_front();
            float dAfr2 = pipe2.front(); pipe2.pop_front();

            if (sensorsWarming && _millis < 3000) {
                // Simulate cold sensors at a fixed mid-voltage, no switching
                currentV1 = 0.455f;
                currentV2 = 0.455f;
            } else {
                currentV1 = afrToVoltage(dAfr1) + ((rand() % 100) - 50) / 2000.0f;
                currentV2 = afrToVoltage(dAfr2) + ((rand() % 100) - 50) / 2000.0f;
            }
        }
    }
};

struct TestResult {
    bool passed;
    std::string message;
};

EngineModel engine;

TestResult run_scenario(std::string name, float imb1, float imb2, int steps, bool logToCsv) {
    reset_controller();
    engine = EngineModel();
    engine.imbalance1 = imb1;
    engine.imbalance2 = imb2;

    if (name == "ColdStart") engine.sensorsWarming = true;

    std::ofstream csvFile;
    if (logToCsv) {
        csvFile.open("split_bank_lambda/sim/sim_output.csv");
        csvFile << "Step,Time,L1_V,L2_V,Off1,Off2,Drift_En,Lim_En" << std::endl;
    }

    std::cout << "--- Scenario: " << name << " ---" << std::endl;

    bool driftTriggered = false;
    bool limitTriggered = false;
    int engagementStep = -1;

    for (int i = 0; i < steps; ++i) {
        if (name == "Transient") {
             if (i < 500) engine.rpm = 2000.0f;
             else { engine.rpm = 4000.0f; engine.imbalance1 = -0.25f; }
        } else {
            engine.rpm = 2000.0f;
        }

        engine.update(dacValues[DAC1_PIN], dacValues[DAC2_PIN]);
        analogValues[LAMBDA1_PIN] = (int)(engine.currentV1 * 4095.0f / 3.3f);
        analogValues[LAMBDA2_PIN] = (int)(engine.currentV2 * 4095.0f / 3.3f);

        runControllerStep();

        if (engagementStep == -1 && sensor1Active && sensor2Active) engagementStep = i;

        if (pins[LAMBDA1_ENABLE_PIN].state == HIGH) driftTriggered = true;
        if (pins[LAMBDA2_ENABLE_PIN].state == HIGH) limitTriggered = true;

        if (logToCsv && i % 10 == 0) {
            csvFile << i << "," << i * 0.01f << "," << engine.currentV1 << "," << engine.currentV2 << ","
                    << dacValues[DAC1_PIN] << "," << dacValues[DAC2_PIN] << ","
                    << pins[LAMBDA1_ENABLE_PIN].state << "," << pins[LAMBDA2_ENABLE_PIN].state << std::endl;
        }

        _millis += 10;
    }

    std::cout << "  Engagement Step: " << engagementStep << std::endl;
    std::cout << "  Final Offsets: " << (int)currentOffset1 << ", " << (int)currentOffset2 << std::endl;
    std::cout << "  Drift Triggered: " << (driftTriggered ? "YES" : "NO") << std::endl;
    std::cout << "  Limit Triggered: " << (limitTriggered ? "YES" : "NO") << std::endl;

    if (logToCsv) csvFile.close();

    if (name == "ColdStart") {
        if (engagementStep < 300) return {false, "Control engaged before sensor warm-up"};
    } else if (name == "Transient") {
        if (!limitTriggered) return {false, "Limit not triggered on large imbalance"};
    }

    return {true, "Success"};
}

int main() {
    srand(42);
    std::vector<TestResult> results;

    results.push_back(run_scenario("Normal", 0.0f, 0.0f, 1000, false));
    results.push_back(run_scenario("ColdStart", 0.0f, 0.0f, 1000, false));
    results.push_back(run_scenario("Transient", 0.0f, 0.0f, 2000, true));

    int failed = 0;
    for (auto& r : results) {
        if (!r.passed) {
            std::cerr << ">> Test FAILED: " << r.message << std::endl;
            failed++;
        }
    }

    if (failed == 0) {
        std::cout << "\nALL ENHANCED TESTS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "\n" << failed << " TESTS FAILED" << std::endl;
        return 1;
    }
}
