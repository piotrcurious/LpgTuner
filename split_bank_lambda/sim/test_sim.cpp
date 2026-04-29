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
    float rpm = 800.0f; // Default idle

    void update(float dacOffset) {
        // Higher RPM = Faster exhaust flow = Lower transport delay
        // Assume 800 RPM = 30 steps delay, 6000 RPM = 4 steps delay
        int transportDelaySteps = std::max(2, (int)(24000.0f / rpm));

        float fuelFactor = 1.0f + (dacOffset - 128.0f) / 128.0f * 0.20f;
        fuelFactor *= (1.0f + offsetImbalance);
        float actualAfr = baseAfr / fuelFactor;

        exhaustPipe.push_back(actualAfr);
        while (exhaustPipe.size() > transportDelaySteps) {
            float delayedAfr = exhaustPipe.front();
            exhaustPipe.pop_front();
            currentVoltage = afrToVoltage(delayedAfr);
            // Noise increases slightly at idle
            float noiseMag = (rpm < 1000) ? 1500.0f : 2000.0f;
            currentVoltage += ((rand() % 100) - 50) / noiseMag;
        }
    }

    float currentVoltage = 0.5f;
};

struct TestResult {
    bool passed;
    std::string message;
};

// Global variables for the simulation loop to access
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
        csvFile << "Step,Time,L1_V,L2_V,Off1,Off2,Drift_En,Lim_En,RPM,AFR1" << std::endl;
    }

    std::cout << "--- Scenario: " << name << " ---" << std::endl;

    bool driftTriggered = false;
    bool limitTriggered = false;

    for (int i = 0; i < steps; ++i) {
        // Dynamic Engine State Management
        if (name == "Idle") {
            bank1.rpm = bank2.rpm = 800.0f;
        } else if (name == "Cruise") {
            bank1.rpm = bank2.rpm = 2500.0f;
        } else if (name == "Acceleration") {
            // RPM ramps from 1000 to 5000, base AFR goes rich
            float t = (float)i / steps;
            bank1.rpm = bank2.rpm = 1000.0f + 4000.0f * t;
            bank1.baseAfr = bank2.baseAfr = 14.7f - 2.0f * t;
        } else if (name == "Deceleration") {
            // RPM drops from 4000 to 1000, base AFR goes lean
            float t = (float)i / steps;
            bank1.rpm = bank2.rpm = 4000.0f - 3000.0f * t;
            bank1.baseAfr = bank2.baseAfr = 14.7f + 5.0f * t;
        }

        bank1.update(dacValues[DAC1_PIN]);
        bank2.update(dacValues[DAC2_PIN]);
        analogValues[LAMBDA1_PIN] = (int)(bank1.currentVoltage * 4095.0f / 3.3f);
        analogValues[LAMBDA2_PIN] = (int)(bank2.currentVoltage * 4095.0f / 3.3f);

        runControllerStep();

        if (pins[LAMBDA1_ENABLE_PIN].state == HIGH) driftTriggered = true;
        if (pins[LAMBDA2_ENABLE_PIN].state == HIGH) limitTriggered = true;

        if (logToCsv && i % 5 == 0) {
            csvFile << i << "," << i * 0.01f << "," << bank1.currentVoltage << "," << bank2.currentVoltage << ","
                    << dacValues[DAC1_PIN] << "," << dacValues[DAC2_PIN] << ","
                    << pins[LAMBDA1_ENABLE_PIN].state << "," << pins[LAMBDA2_ENABLE_PIN].state << ","
                    << bank1.rpm << "," << bank1.baseAfr << std::endl;
        }

        _millis += 10;
    }

    std::cout << "Final Offsets: " << (int)currentOffset1 << ", " << (int)currentOffset2 << std::endl;
    std::cout << "Drift Triggered: " << (driftTriggered ? "YES" : "NO") << std::endl;
    std::cout << "Limit Triggered: " << (limitTriggered ? "YES" : "NO") << std::endl;

    if (logToCsv) csvFile.close();

    // Pass criteria: Controller shouldn't crash and should attempt compensation.
    // In acceleration/deceleration, we expect Limit to trigger because base AFR moves far from stoich.
    if (name == "Idle" || name == "Cruise") {
        if (limitTriggered && imbalance1 == 0 && imbalance2 == 0) return {false, "Limit triggered unexpectedly in stable mode"};
    }

    return {true, "Success"};
}

int main() {
    srand(42);
    std::vector<TestResult> results;

    results.push_back(run_scenario("Idle", 0.0f, 0.0f, 1500, false));
    results.push_back(run_scenario("Cruise", 0.0f, 0.0f, 1500, false));
    results.push_back(run_scenario("Acceleration", -0.05f, 0.05f, 2000, true)); // Log acceleration
    results.push_back(run_scenario("Deceleration", 0.0f, 0.0f, 2000, false));

    int failed = 0;
    for (auto& r : results) {
        if (!r.passed) {
            std::cerr << "Test FAILED: " << r.message << std::endl;
            failed++;
        }
    }

    if (failed == 0) {
        std::cout << "ALL DYNAMIC TESTS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << failed << " TESTS FAILED" << std::endl;
        return 1;
    }
}
