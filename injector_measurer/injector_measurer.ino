// Injector Pulse Width Measurer
// Measures actual injector pulse width via interrupts and provides smoothed estimates.

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Pins
const int injectorPin = 3;

// Variables
volatile unsigned long injectorOnTime = 0;
volatile uint32_t lastPulseWidthUs = 0;
float smoothedPulseWidthMs = 0;
const float alpha = 0.1; // Smoothing factor

void injectorISR() {
    unsigned long now = micros();
    if (digitalRead(injectorPin) == HIGH) {
        injectorOnTime = now;
    } else {
        if (injectorOnTime > 0) {
            lastPulseWidthUs = now - injectorOnTime;
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(injectorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(injectorPin), injectorISR, CHANGE);
}

void loop() {
    // Basic EMA filter for pulse width
    smoothedPulseWidthMs = (smoothedPulseWidthMs * (1.0 - alpha)) + ((lastPulseWidthUs / 1000.0) * alpha);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        Serial.print("Raw PW: "); Serial.print(lastPulseWidthUs / 1000.0);
        Serial.print(" ms | Smoothed: "); Serial.print(smoothedPulseWidthMs);
        Serial.println(" ms");
        lastPrint = millis();
    }
}
