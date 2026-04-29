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

void reset_controller() {
    currentOffset1 = DAC_MID;
    currentOffset2 = DAC_MID;
    filteredLambda1 = 0.5f;
    filteredLambda2 = 0.5f;
    lastUpdate = 0;
    _millis = 0;
    for(int i=0; i<40; i++) {
        pins[i].mode = INPUT;
        pins[i].state = LOW;
        dacValues[i] = DAC_MID;
    }
}

// --- Physics Simulation ---

float afrToVoltage(float afr) {
    float lambda = afr / 14.7f;
    return 0.1f + 0.8f * (0.5f * (1.0f - std::tanh(20.0f * (lambda - 1.0f))));
}

struct EngineSubgroup {
    float baseAfr = 14.7f;
    float offsetImbalance = 0.0f;
    std::deque<float> exhaustPipe;
    int transportDelaySteps = 15;
    float currentVoltage = 0.5f;

    void update(float dacOffset) {
        float fuelFactor = 1.0f + (dacOffset - 128.0f) / 128.0f * 0.20f;
        fuelFactor *= (1.0f + offsetImbalance);
        float actualAfr = baseAfr / fuelFactor;
        exhaustPipe.push_back(actualAfr);
        if (exhaustPipe.size() > transportDelaySteps) {
            float delayedAfr = exhaustPipe.front();
            exhaustPipe.pop_front();
            currentVoltage = afrToVoltage(delayedAfr);
            currentVoltage += ((rand() % 100) - 50) / 2000.0f; // Noise
        }
    }
};

struct TestResult {
    bool passed;
    std::string message;
};

TestResult run_scenario(std::string name, float imbalance1, float imbalance2, int steps, bool logToCsv, float transientImbalance1 = 0.0f, int transientStep = -1) {
    reset_controller();
    EngineSubgroup bank1, bank2;
    bank1.offsetImbalance = imbalance1;
    bank2.offsetImbalance = imbalance2;

    std::ofstream csvFile;
    if (logToCsv) {
        csvFile.open("split_bank_lambda/sim/sim_output.csv");
        csvFile << "Step,Time,L1_V,L2_V,Off1,Off2,Drift_En,Lim_En" << std::endl;
    }

    std::cout << "--- Scenario: " << name << " ---" << std::endl;

    bool driftTriggered = false;
    bool limitTriggered = false;
    float avgError1 = 0;
    float avgError2 = 0;
    int samples = 0;

    for (int i = 0; i < steps; ++i) {
        if (i == transientStep) {
            bank1.offsetImbalance = transientImbalance1;
            std::cout << "[Step " << i << "] Transient Imbalance Applied: " << transientImbalance1 << std::endl;
        }
        bank1.update(dacValues[DAC1_PIN]);
        bank2.update(dacValues[DAC2_PIN]);
        analogValues[LAMBDA1_PIN] = (int)(bank1.currentVoltage * 4095.0f / 3.3f);
        analogValues[LAMBDA2_PIN] = (int)(bank2.currentVoltage * 4095.0f / 3.3f);

        runControllerStep();

        if (pins[LAMBDA1_ENABLE_PIN].state == HIGH) driftTriggered = true;
        if (pins[LAMBDA2_ENABLE_PIN].state == HIGH) limitTriggered = true;

        if (i > steps / 2) {
            avgError1 += std::abs(bank1.currentVoltage - LEAN_TARGET);
            avgError2 += std::abs(bank2.currentVoltage - RICH_TARGET);
            samples++;
        }

        if (logToCsv && i % 10 == 0) {
            csvFile << i << "," << i * 0.01f << "," << bank1.currentVoltage << "," << bank2.currentVoltage << ","
                    << dacValues[DAC1_PIN] << "," << dacValues[DAC2_PIN] << ","
                    << pins[LAMBDA1_ENABLE_PIN].state << "," << pins[LAMBDA2_ENABLE_PIN].state << std::endl;
        }

        _millis += 10;
    }

    avgError1 /= samples;
    avgError2 /= samples;

    std::cout << "Final Offsets: " << (int)currentOffset1 << ", " << (int)currentOffset2 << std::endl;
    std::cout << "Avg Voltages: " << bank1.currentVoltage << ", " << bank2.currentVoltage << std::endl;
    std::cout << "Drift Triggered: " << (driftTriggered ? "YES" : "NO") << std::endl;
    std::cout << "Limit Triggered: " << (limitTriggered ? "YES" : "NO") << std::endl;

    if (logToCsv) csvFile.close();

    if (name == "Balanced") {
        if (driftTriggered) return {false, "Drift triggered on balanced engine"};
        if (limitTriggered) return {false, "Limit triggered on balanced engine"};
    } else if (name == "Imbalance_Large") {
        if (!driftTriggered) return {false, "Drift NOT triggered on imbalanced engine"};
    } else if (name == "Saturation") {
        if (!limitTriggered) return {false, "Limit NOT triggered on saturated engine"};
    }

    return {true, "Success"};
}

int main() {
    srand(42);
    std::vector<TestResult> results;

    results.push_back(run_scenario("Balanced", 0.0f, 0.0f, 1000, false));
    results.push_back(run_scenario("Imbalance_Large", -0.15f, 0.05f, 1000, false));
    results.push_back(run_scenario("Saturation", -0.35f, 0.15f, 2000, false));
    results.push_back(run_scenario("Transient", 0.0f, 0.0f, 2000, true, -0.20f, 500)); // Log transient for plotting

    int failed = 0;
    for (auto& r : results) {
        if (!r.passed) {
            std::cerr << "Test FAILED: " << r.message << std::endl;
            failed++;
        }
    }

    if (failed == 0) {
        std::cout << "ALL TESTS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << failed << " TESTS FAILED" << std::endl;
        return 1;
    }
}
