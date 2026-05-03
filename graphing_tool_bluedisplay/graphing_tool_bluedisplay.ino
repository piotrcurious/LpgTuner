// Graphing Tool for Engine Tuning
// Visualizes injector pulse width on an RPM/Load map using BlueDisplay.

#ifndef UNIT_TEST
#include "BlueDisplay.hpp"
#else
#include "tests/mock_arduino.h"
#endif

// Constants
#define MAX_RPM 7000
#define MAX_PW 20.0
#define TIMEOUT_US 500000 // 0.5s timeout for pulses
#define MIN_CAM_PERIOD_US 4000 // Max ~15000 RPM equivalent for debounce

// Pins
const int throttlePin = A0;
const int mapPin = A1;
const int lambdaPin = A2;
const int camPin = 2;
const int injectorPin = 3;
const int injectorGaugePin = 9;

// Variables
float throttle = 0;
float engineLoad = 0;
float lambda = 0;
float rpm = 0;
float pulseWidthMs = 0;

float lastDrawnRpm = -1;
float lastDrawnLoad = -1;
unsigned long lastDrawTime = 0;
unsigned long lastTextUpdateTime = 0;
bool simulationMode = false;

volatile unsigned long lastCamTime = 0;
volatile unsigned long camPeriod = 0;
volatile unsigned long lastValidCamTime = 0;
volatile unsigned long injectorOnTime = 0;
volatile unsigned long pulseWidthUs = 0;
volatile unsigned long lastInjectorTime = 0;

int displayWidth, displayHeight;
int chartWidth, chartHeight, chartXOffset, chartYOffset;

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_BLUE 0x001F
#define COLOR_GREEN 0x07E0

BDButton clearButton;
BDButton simButton;
void initDisplay();
void drawGui();
void handleClear(BDButton *aButton, int16_t aValue);
void handleSim(BDButton *aButton, int16_t aValue);

void camISR() {
    unsigned long now = micros();
    unsigned long period = now - lastValidCamTime;
    if (lastValidCamTime == 0 || period > MIN_CAM_PERIOD_US) {
        if (lastValidCamTime > 0) camPeriod = period;
        lastValidCamTime = now;
        lastCamTime = now; // For timeout detection
    }
}

void injectorISR() {
    unsigned long now = micros();
    // Injectors are typically ground-switched (Active LOW)
    // If it goes LOW, it's ON. If it goes HIGH, it's OFF.
    if (digitalRead(injectorPin) == LOW) {
        injectorOnTime = now;
    } else {
        if (injectorOnTime > 0) {
            pulseWidthUs = now - injectorOnTime;
            lastInjectorTime = now;
        }
        injectorOnTime = 0;
    }
}

void readSensors() {
    if (simulationMode) {
        static float simPhase = 0;
        simPhase += 0.05;
        rpm = 1000 + 4000 * (0.5 + 0.5 * sin(simPhase));
        engineLoad = 20 + 60 * (0.5 + 0.5 * cos(simPhase * 0.7));
        pulseWidthMs = (engineLoad / 100.0) * 15.0 + 2.0;
        lambda = 0.9 + 0.2 * (0.5 + 0.5 * sin(simPhase * 1.3));
        return;
    }

    unsigned long now = micros();

    throttle = analogRead(throttlePin) / 10.23; // 0-100%
    engineLoad = analogRead(mapPin) / 10.23; // 0-100% (simplified)
    lambda = (analogRead(lambdaPin) / 1023.0) + 0.5; // 0.5-1.5 approx

    // RPM Timeout
    if (now - lastCamTime > TIMEOUT_US) {
        camPeriod = 0;
        rpm = 0;
    } else if (camPeriod > 0) {
        rpm = 60000000.0 / camPeriod;
    } else {
        rpm = 0;
    }

    // PulseWidth Timeout
    if (now - lastInjectorTime > TIMEOUT_US) {
        pulseWidthUs = 0;
    }
    pulseWidthMs = pulseWidthUs / 1000.0;
}

void drawChart() {
    if (rpm == 0 || engineLoad == 0) return;

    // Throttle drawing to every 50ms OR if there is a significant change
    unsigned long now = millis();
    if (now - lastDrawTime < 50 &&
        abs(rpm - lastDrawnRpm) < 50 &&
        abs(engineLoad - lastDrawnLoad) < 2) {
        return;
    }

    lastDrawTime = now;
    lastDrawnRpm = rpm;
    lastDrawnLoad = engineLoad;

#ifdef UNIT_TEST
    float rpm_c = arduino_constrain(rpm, 0, MAX_RPM);
    float load_c = arduino_constrain(engineLoad, 0, 100);
    float pw_c = arduino_constrain(pulseWidthMs, 0, MAX_PW);
#else
    float rpm_c = constrain(rpm, 0, MAX_RPM);
    float load_c = constrain(engineLoad, 0, 100);
    float pw_c = constrain(pulseWidthMs, 0, MAX_PW);
#endif

#ifdef UNIT_TEST
    int x = arduino_map(rpm_c, 0, MAX_RPM, chartXOffset, chartXOffset + chartWidth);
    int y = arduino_map(load_c, 0, 100, chartYOffset + chartHeight, chartYOffset);
#else
    int x = map(rpm_c, 0, MAX_RPM, chartXOffset, chartXOffset + chartWidth);
    int y = map(load_c, 0, 100, chartYOffset + chartHeight, chartYOffset);
#endif

    // Color mapping: Blue (short) -> Green (mid) -> Red (long)
    uint16_t color;
    if (pw_c < (MAX_PW / 2.0)) {
#ifdef UNIT_TEST
        int g = arduino_map(pw_c * 100, 0, (MAX_PW / 2.0) * 100, 0, 63);
        int b = 31 - arduino_map(pw_c * 100, 0, (MAX_PW / 2.0) * 100, 0, 31);
#else
        int g = map(pw_c * 100, 0, (MAX_PW / 2.0) * 100, 0, 63);
        int b = 31 - map(pw_c * 100, 0, (MAX_PW / 2.0) * 100, 0, 31);
#endif
        color = (g << 5) | b;
    } else {
#ifdef UNIT_TEST
        int r = arduino_map((pw_c - MAX_PW / 2.0) * 100, 0, (MAX_PW / 2.0) * 100, 0, 31);
        int g = 63 - arduino_map((pw_c - MAX_PW / 2.0) * 100, 0, (MAX_PW / 2.0) * 100, 0, 63);
#else
        int r = map((pw_c - MAX_PW / 2.0) * 100, 0, (MAX_PW / 2.0) * 100, 0, 31);
        int g = 63 - map((pw_c - MAX_PW / 2.0) * 100, 0, (MAX_PW / 2.0) * 100, 0, 63);
#endif
        color = (r << 11) | (g << 5);
    }

    BlueDisplay1.drawPixel(x, y, color);
    // Draw a small 2x2 square for better visibility
    BlueDisplay1.drawPixel(x+1, y, color);
    BlueDisplay1.drawPixel(x, y+1, color);
    BlueDisplay1.drawPixel(x+1, y+1, color);
}

void drawCurrentPulseWidth() {
    unsigned long now = millis();
    if (now - lastTextUpdateTime < 200) return;
    lastTextUpdateTime = now;

    char buf[64];
    char pwBuf[10];
    char lambdaBuf[10];
#ifdef UNIT_TEST
    sprintf(pwBuf, "%.2f", pulseWidthMs);
    sprintf(lambdaBuf, "%.2f", lambda);
#else
    dtostrf(pulseWidthMs, 4, 2, pwBuf);
    dtostrf(lambda, 4, 2, lambdaBuf);
#endif
    sprintf(buf, "RPM: %d  PW: %s ms  L: %s  %s", (int)rpm, pwBuf, lambdaBuf, simulationMode ? "[SIM]" : "");
    BlueDisplay1.drawText(10, displayHeight - 20, buf, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

void setup() {
    Serial.begin(115200);
    pinMode(camPin, INPUT_PULLUP);
    pinMode(injectorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING);
    attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE);
    BlueDisplay1.initCommunication(&Serial, &initDisplay, &drawGui, NULL);
}

void loop() {
    BlueDisplay1.checkAndHandleEvents();
    readSensors();
    drawChart();
    drawCurrentPulseWidth();
    // Update analog gauge
#ifdef UNIT_TEST
    int gaugeValue = arduino_constrain(pulseWidthMs * (255.0 / MAX_PW), 0, 255);
#else
    int gaugeValue = constrain(pulseWidthMs * (255.0 / MAX_PW), 0, 255);
#endif
    analogWrite(injectorGaugePin, gaugeValue);
}

void initDisplay() {
    displayWidth = BlueDisplay1.getDisplayWidth();
    displayHeight = BlueDisplay1.getDisplayHeight();
    chartWidth = displayWidth - 50;
    chartHeight = displayHeight - 80;
    chartXOffset = 40;
    chartYOffset = 30;

    clearButton.init(displayWidth - 140, 5, 60, 25, COLOR_RED, "Clear", TEXT_SIZE_11, 0, 0, &handleClear);
    simButton.init(displayWidth - 70, 5, 60, 25, COLOR_BLUE, "Sim", TEXT_SIZE_11, 0, 0, &handleSim);
}

void drawGui() {
    BlueDisplay1.clearDisplay(COLOR_WHITE);
    BlueDisplay1.drawText(10, 5, "Injector Map", TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawRectRel(chartXOffset, chartYOffset, chartWidth, chartHeight, COLOR_BLACK);

    // Draw Axis Labels
    BlueDisplay1.drawText(chartXOffset + chartWidth / 2 - 20, chartYOffset + chartHeight + 5, "RPM", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);

    // Vertical "LOAD" label
    BlueDisplay1.drawText(10, chartYOffset + chartHeight / 2 - 20, "L", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(10, chartYOffset + chartHeight / 2 - 10, "O", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(10, chartYOffset + chartHeight / 2, "A", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(10, chartYOffset + chartHeight / 2 + 10, "D", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);

    clearButton.drawButton();
    simButton.drawButton();
}

void handleClear(BDButton *aButton, int16_t aValue) {
    drawGui();
}

void handleSim(BDButton *aButton, int16_t aValue) {
    simulationMode = !simulationMode;
    drawGui();
}
