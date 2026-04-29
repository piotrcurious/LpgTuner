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

// --- Enhanced Physics Simulation ---

float afrToVoltage(float afr) {
    float lambda = afr / 14.7f;
    return 0.1f + 0.8f * (0.5f * (1.0f - std::tanh(20.0f * (lambda - 1.0f))));
}

struct EngineSubgroup {
    float baseAfr = 14.7f;
    float offsetImbalance = 0.0f;
    std::deque<float> exhaustPipe;

    // Engine State
    float rpm = 800.0f;
    bool stuckValue = false;
    float stuckVoltage = 0.0f;

    void update(float dacOffset) {
        if (stuckValue) {
            currentVoltage = stuckVoltage;
            return;
        }

        int transportDelaySteps = std::max(2, (int)(24000.0f / rpm));

        float fuelFactor = 1.0f + (dacOffset - 128.0f) / 128.0f * 0.20f;
        fuelFactor *= (1.0f + offsetImbalance);
        float actualAfr = baseAfr / fuelFactor;

        exhaustPipe.push_back(actualAfr);
        while (exhaustPipe.size() > transportDelaySteps) {
            float delayedAfr = exhaustPipe.front();
            exhaustPipe.pop_front();
            currentVoltage = afrToVoltage(delayedAfr);
            float noiseMag = 2000.0f;
            currentVoltage += ((rand() % 100) - 50) / noiseMag;
        }
    }

    float currentVoltage = 0.5f;
};

struct TestResult {
    bool passed;
    std::string message;
};

EngineSubgroup bank1, bank2;

TestResult run_scenario(std::string name, float imbalance1, float imbalance2, int steps, bool logToCsv) {
    reset_controller();
    bank1 = EngineSubgroup();
    bank2 = EngineSubgroup();
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
    float maxOsc1 = 0;
    float minV1 = 1.0, maxV1 = 0.0;
    int limitStep = -1;

    if (name == "SensorStuckLean") { bank1.stuckValue = true; bank1.stuckVoltage = 0.1f; }
    else if (name == "SensorStuckRich") { bank2.stuckValue = true; bank2.stuckVoltage = 0.9f; }

    for (int i = 0; i < steps; ++i) {
        if (name == "IdleOscillation") bank1.rpm = bank2.rpm = 800.0f;
        else if (name == "Transient") {
             if (i < 500) bank1.rpm = bank2.rpm = 2000.0f;
             else { bank1.rpm = bank2.rpm = 4000.0f; bank1.offsetImbalance = -0.20f; }
        }
        else bank1.rpm = bank2.rpm = 2000.0f;

        bank1.update(dacValues[DAC1_PIN]);
        bank2.update(dacValues[DAC2_PIN]);
        analogValues[LAMBDA1_PIN] = (int)(bank1.currentVoltage * 4095.0f / 3.3f);
        analogValues[LAMBDA2_PIN] = (int)(bank2.currentVoltage * 4095.0f / 3.3f);

        runControllerStep();

        if (pins[LAMBDA1_ENABLE_PIN].state == HIGH) driftTriggered = true;
        if (pins[LAMBDA2_ENABLE_PIN].state == HIGH) {
            if (!limitTriggered) limitStep = i;
            limitTriggered = true;
        }

        if (i > steps / 2) {
            maxOsc1 = std::max(maxOsc1, std::abs(bank1.currentVoltage - 0.45f));
            minV1 = std::min(minV1, bank1.currentVoltage);
            maxV1 = std::max(maxV1, bank1.currentVoltage);
        }

        if (logToCsv && i % 10 == 0) {
            csvFile << i << "," << i * 0.01f << "," << bank1.currentVoltage << "," << bank2.currentVoltage << ","
                    << dacValues[DAC1_PIN] << "," << dacValues[DAC2_PIN] << ","
                    << pins[LAMBDA1_ENABLE_PIN].state << "," << pins[LAMBDA2_ENABLE_PIN].state << std::endl;
        }

        _millis += 10;
    }

    std::cout << "  Final Offsets: " << (int)currentOffset1 << ", " << (int)currentOffset2 << std::endl;
    std::cout << "  Drift Triggered: " << (driftTriggered ? "YES" : "NO") << std::endl;
    std::cout << "  Limit Triggered: " << (limitTriggered ? "YES" : "NO");
    if (limitTriggered) std::cout << " at step " << limitStep;
    std::cout << std::endl;
    std::cout << "  Steady-State V1 Range: [" << minV1 << ", " << maxV1 << "] (Target 0.45)" << std::endl;

    if (logToCsv) csvFile.close();

    if (name == "SensorStuckLean" || name == "SensorStuckRich") {
        if (!limitTriggered) return {false, "Limit emulation failed to trigger on stuck sensor"};
    } else if (name == "IdleOscillation") {
        if (maxOsc1 > 0.15) return {false, "Oscillation at idle too high"};
    }

    return {true, "Success"};
}

int main() {
    srand(42);
    std::vector<TestResult> results;

    results.push_back(run_scenario("IdleOscillation", 0.0f, 0.0f, 1500, false));
    results.push_back(run_scenario("SensorStuckLean", 0.0f, 0.0f, 1500, false));
    results.push_back(run_scenario("SensorStuckRich", 0.0f, 0.0f, 1500, false));
    results.push_back(run_scenario("Transient", 0.0f, 0.0f, 2000, true));

    int failed = 0;
    for (auto& r : results) {
        if (!r.passed) {
            std::cerr << ">> Test FAILED: " << r.message << std::endl;
            failed++;
        }
    }

    if (failed == 0) {
        std::cout << "\nALL EDGE CASE TESTS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "\n" << failed << " TESTS FAILED" << std::endl;
        return 1;
    }
}
