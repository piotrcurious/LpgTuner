// Graphing Tool for Engine Tuning
// Visualizes injector pulse width on an RPM/Load map using BlueDisplay.

#ifndef UNIT_TEST
#include "BlueDisplay.hpp"
#endif

// Constants
#define MAX_RPM 7000
#define MAX_PW 20.0
#define TIMEOUT_US 500000 // 0.5s timeout for pulses

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

volatile unsigned long lastCamTime = 0;
volatile unsigned long camPeriod = 0;
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

#ifndef UNIT_TEST
BDButton clearButton;
void initDisplay();
void drawGui();
void handleClear(BDButton *aButton, int16_t aValue);
#endif

void camISR() {
    unsigned long now = micros();
    if (lastCamTime > 0) camPeriod = now - lastCamTime;
    lastCamTime = now;
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

#ifndef UNIT_TEST
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

    int x = map(constrain(rpm, 0, MAX_RPM), 0, MAX_RPM, chartXOffset, chartXOffset + chartWidth);
    int y = map(constrain(engineLoad, 0, 100), 0, 100, chartYOffset + chartHeight, chartYOffset);

    // Color mapping: Blue (short) -> Green (mid) -> Red (long)
    uint16_t color;
    if (pulseWidthMs < (MAX_PW / 2.0)) {
        int g = map(pulseWidthMs * 100, 0, (MAX_PW / 2.0) * 100, 0, 63);
        int b = 31 - map(pulseWidthMs * 100, 0, (MAX_PW / 2.0) * 100, 0, 31);
        color = (g << 5) | b;
    } else {
        int r = map((pulseWidthMs - MAX_PW / 2.0) * 100, 0, (MAX_PW / 2.0) * 100, 0, 31);
        int g = 63 - map((pulseWidthMs - MAX_PW / 2.0) * 100, 0, (MAX_PW / 2.0) * 100, 0, 63);
        color = (r << 11) | (g << 5);
    }

    BlueDisplay1.drawPixel(x, y, color);
    // Draw a small 2x2 square for better visibility
    BlueDisplay1.drawPixel(x+1, y, color);
    BlueDisplay1.drawPixel(x, y+1, color);
    BlueDisplay1.drawPixel(x+1, y+1, color);
}

void drawCurrentPulseWidth() {
    char buf[48];
    char pwBuf[10];
#ifdef UNIT_TEST
    sprintf(pwBuf, "%.2f", pulseWidthMs);
#else
    dtostrf(pulseWidthMs, 4, 2, pwBuf);
#endif
    sprintf(buf, "RPM: %d  PW: %s ms  ", (int)rpm, pwBuf);
    BlueDisplay1.drawText(10, displayHeight - 20, buf, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

void setup() {
    Serial.begin(115200);
    pinMode(camPin, INPUT_PULLUP);
    pinMode(injectorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING);
    attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE);
    BlueDisplay1.initCommunication(&Serial, &initDisplay, &drawGui);
}

void loop() {
    BlueDisplay1.checkAndHandleEvents();
    readSensors();
    drawChart();
    drawCurrentPulseWidth();
    // Update analog gauge
    int gaugeValue = constrain(pulseWidthMs * (255.0 / MAX_PW), 0, 255);
    analogWrite(injectorGaugePin, gaugeValue);
}

void initDisplay() {
    displayWidth = BlueDisplay1.getDisplayWidth();
    displayHeight = BlueDisplay1.getDisplayHeight();
    chartWidth = displayWidth - 50;
    chartHeight = displayHeight - 80;
    chartXOffset = 40;
    chartYOffset = 30;

    clearButton.init(displayWidth - 70, 5, 60, 25, COLOR_RED, "Clear", TEXT_SIZE_11, 0, 0, &handleClear);
}

void drawGui() {
    BlueDisplay1.clearDisplay(COLOR_WHITE);
    BlueDisplay1.drawText(10, 5, "Injector Map", TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawRect(chartXOffset, chartYOffset, chartWidth + 1, chartHeight + 1, COLOR_BLACK);

    // Draw Axis Labels
    BlueDisplay1.drawText(chartXOffset + chartWidth / 2 - 20, chartYOffset + chartHeight + 5, "RPM", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(5, chartYOffset + chartHeight / 2, "L", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(5, chartYOffset + chartHeight / 2 + 10, "O", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(5, chartYOffset + chartHeight / 2 + 20, "A", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawText(5, chartYOffset + chartHeight / 2 + 30, "D", TEXT_SIZE_09, COLOR_BLACK, COLOR_WHITE);

    clearButton.drawButton();
}

void handleClear(BDButton *aButton, int16_t aValue) {
    drawGui();
}
#endif
