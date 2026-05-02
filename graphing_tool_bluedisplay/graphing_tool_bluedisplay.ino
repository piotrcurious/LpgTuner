// Graphing Tool for Engine Tuning
// Visualizes injector pulse width on an RPM/Load map using BlueDisplay.

#ifndef UNIT_TEST
#include "BlueDisplay.hpp"
#endif

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

volatile unsigned long lastCamTime = 0;
volatile unsigned long camPeriod = 0;
volatile unsigned long injectorOnTime = 0;
volatile unsigned long pulseWidthUs = 0;

int displayWidth, displayHeight;
int chartWidth, chartHeight, chartXOffset, chartYOffset;

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_BLUE 0x001F

#ifndef UNIT_TEST
void initDisplay();
void drawGui();
#endif

void camISR() {
    unsigned long now = micros();
    if (lastCamTime > 0) camPeriod = now - lastCamTime;
    lastCamTime = now;
}

void injectorISR() {
    unsigned long now = micros();
    if (digitalRead(injectorPin) == HIGH) {
        injectorOnTime = now;
    } else {
        if (injectorOnTime > 0) pulseWidthUs = now - injectorOnTime;
    }
}

void readSensors() {
    throttle = analogRead(throttlePin) / 10.23; // 0-100%
    engineLoad = analogRead(mapPin) / 10.23; // 0-100% (simplified)
    lambda = analogRead(lambdaPin) / 1023.0 + 0.5; // 0.5-1.5 approx

    if (camPeriod > 0) rpm = 60000000.0 / camPeriod;
    else rpm = 0;

    pulseWidthMs = pulseWidthUs / 1000.0;
}

#ifndef UNIT_TEST
void drawChart() {
    int x = map(rpm, 0, 6000, chartXOffset, chartXOffset + chartWidth);
    int y = map(engineLoad, 0, 100, chartYOffset + chartHeight, chartYOffset);

    // Simple color mapping blue -> red
    int r = map(pulseWidthMs, 0, 10, 0, 31);
    int b = 31 - r;
    uint16_t color = (r << 11) | b;

    BlueDisplay1.drawPixel(x, y, color);
}

void drawCurrentPulseWidth() {
    char buf[16];
    sprintf(buf, "%.2f ms", pulseWidthMs);
    BlueDisplay1.fillRect(displayWidth - 80, 0, 80, 20, COLOR_WHITE);
    BlueDisplay1.drawText(displayWidth - 80, 0, buf, TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
}

void setup() {
    Serial.begin(115200);
    pinMode(camPin, INPUT_PULLUP);
    pinMode(injectorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(camPin), camISR, RISING);
    attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE);
    BlueDisplay1.initCommunication(&initDisplay, &drawGui);
}

void loop() {
    BlueDisplay1.checkAndHandleEvents();
    readSensors();
    drawChart();
    drawCurrentPulseWidth();
    analogWrite(injectorGaugePin, map(pulseWidthMs, 0, 10, 0, 255));
}

void initDisplay() {
    displayWidth = BlueDisplay1.getDisplayWidth();
    displayHeight = BlueDisplay1.getDisplayHeight();
    chartWidth = displayWidth - 40;
    chartHeight = displayHeight - 60;
    chartXOffset = 30;
    chartYOffset = 30;
}

void drawGui() {
    BlueDisplay1.clearDisplay(COLOR_WHITE);
    BlueDisplay1.drawText(10, 5, "Injector Map", TEXT_SIZE_11, COLOR_BLACK, COLOR_WHITE);
    BlueDisplay1.drawRect(chartXOffset, chartYOffset, chartWidth, chartHeight, COLOR_BLACK);
}
#endif
