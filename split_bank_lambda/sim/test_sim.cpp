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
    bool stuckValue1 = false, stuckValue2 = false;
    float stuckVoltage1 = 0.0f, stuckVoltage2 = 0.0f;

    void update(float dac1, float dac2) {
        if (stuckValue1) currentV1 = stuckVoltage1;
        if (stuckValue2) currentV2 = stuckVoltage2;
        if (stuckValue1 && stuckValue2) return;

        int delaySteps = std::max(2, (int)(24000.0f / rpm));

        float fuel1 = 1.0f + (dac1 - 128.0f) / 128.0f * 0.20f;
        float fuel2 = 1.0f + (dac2 - 128.0f) / 128.0f * 0.20f;

        fuel1 *= (1.0f + imbalance1);
        fuel2 *= (1.0f + imbalance2);

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
                currentV1 = 0.455f;
                currentV2 = 0.455f;
            } else {
                if (!stuckValue1) currentV1 = afrToVoltage(dAfr1) + ((rand() % 100) - 50) / 2000.0f;
                if (!stuckValue2) currentV2 = afrToVoltage(dAfr2) + ((rand() % 100) - 50) / 2000.0f;
            }
        }
    }
};

struct TestResult {
    bool passed;
    std::string message;
    std::string scenario;
};

EngineModel engine;

TestResult run_scenario(std::string name, float imb1, float imb2, int steps) {
    reset_controller();
    engine = EngineModel();
    engine.imbalance1 = imb1;
    engine.imbalance2 = imb2;

    if (name == "ColdStart") engine.sensorsWarming = true;
    if (name == "SensorStuckLean") { engine.stuckValue1 = true; engine.stuckVoltage1 = 0.1f; }
    if (name == "SensorStuckRich") { engine.stuckValue2 = true; engine.stuckVoltage2 = 0.9f; }

    std::ofstream csvFile("split_bank_lambda/sim/sim_" + name + ".csv");
    csvFile << "Step,Time,L1_V,L2_V,Off1,Off2,Drift_En,Lim_En" << std::endl;

    std::cout << "--- Scenario: " << name << " ---" << std::endl;

    bool driftTriggered = false;
    bool limitTriggered = false;
    int engagementStep = -1;

    for (int i = 0; i < steps; ++i) {
        if (name == "Transient") {
             if (i < 500) engine.rpm = 2000.0f;
             else { engine.rpm = 4000.0f; engine.imbalance1 = -0.25f; }
        } else if (name == "IdleOscillation") {
             engine.rpm = 800.0f;
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

        if (i % 10 == 0) {
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

    csvFile.close();

    if (name == "ColdStart") {
        if (engagementStep < 300) return {false, "Control engaged before sensor warm-up", name};
    } else if (name == "SensorStuckLean" || name == "SensorStuckRich" || name == "Transient") {
        if (!limitTriggered) return {false, "Limit not triggered in error state", name};
    }

    return {true, "Success", name};
}

int main() {
    srand(42);
    std::vector<TestResult> results;

    results.push_back(run_scenario("Normal", 0.0f, 0.0f, 1500));
    results.push_back(run_scenario("ColdStart", 0.0f, 0.0f, 1500));
    results.push_back(run_scenario("Transient", 0.0f, 0.0f, 2000));
    results.push_back(run_scenario("IdleOscillation", 0.0f, 0.0f, 2000));
    results.push_back(run_scenario("SensorStuckLean", 0.0f, 0.0f, 1500));
    results.push_back(run_scenario("SensorStuckRich", 0.0f, 0.0f, 1500));

    std::cout << "\nTest Results Table:" << std::endl;
    std::cout << "Scenario | Status | Message" << std::endl;
    std::cout << "--- | --- | ---" << std::endl;
    int failed = 0;
    for (auto& r : results) {
        std::cout << r.scenario << " | " << (r.passed ? "PASS" : "FAIL") << " | " << r.message << std::endl;
        if (!r.passed) failed++;
    }

    if (failed == 0) {
        std::cout << "\nALL SCENARIOS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "\n" << failed << " SCENARIOS FAILED" << std::endl;
        return 1;
    }
}
