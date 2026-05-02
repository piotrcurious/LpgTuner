// AFR and Duty Estimator with OLED Display
// Estimates AFR and injector duty cycle, displaying results on an OLED screen.

#ifndef UNIT_TEST
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#endif

// Pins
const int O2_PIN = A0;
const int INJ_PIN = 2;

// Variables
float o2_voltage = 0;
float inj_duty = 0;
float afr = 14.7;
float corr = 1.0;

bool o2_state = false;
bool o2_prev_state = false;
bool inj_state = false;
bool inj_prev_state = false;

unsigned long o2_rise_time = 0;
unsigned long o2_last_period_us = 0;
unsigned long inj_rise_time = 0;
unsigned long inj_last_period_us = 0;

#define GRAPH_WIDTH 128
float afr_buffer[GRAPH_WIDTH];
int afr_buffer_index = 0;

#ifndef UNIT_TEST
Adafruit_SH1107 oled(64, 128, &Wire); // Example SH1107 initialization
#endif

void setup() {
    Serial.begin(115200);
    pinMode(INJ_PIN, INPUT_PULLUP);
    for(int i=0; i<GRAPH_WIDTH; i++) afr_buffer[i] = 14.7;
#ifndef UNIT_TEST
    oled.begin(0x3C, true);
    oled.clearDisplay();
    oled.display();
#endif
}

void loop() {
    o2_voltage = analogRead(O2_PIN) * (5.0 / 1023.0);
    o2_state = (o2_voltage > 0.45);
    inj_state = (digitalRead(INJ_PIN) == LOW); // Active low injectors

    unsigned long now = micros();

    // O2 signal analysis
    if (o2_state != o2_prev_state) {
        if (o2_state) { // Low -> High (Lean -> Rich crossing)
            if (o2_rise_time > 0) o2_last_period_us = now - o2_rise_time;
            o2_rise_time = now;
        } else { // High -> Low (Rich -> Lean crossing)
            if (o2_rise_time > 0 && o2_last_period_us > 0) {
                float high_time = now - o2_rise_time;
                float duty = high_time / o2_last_period_us;
                afr = 14.7 + (duty - 0.5) * 10.0;
                afr_buffer[afr_buffer_index] = afr;
                afr_buffer_index = (afr_buffer_index + 1) % GRAPH_WIDTH;
            }
        }
        o2_prev_state = o2_state;
    }

    // Injector signal analysis
    if (inj_state != inj_prev_state) {
        if (inj_state) { // Inactive -> Active (Rising edge of duty)
            if (inj_rise_time > 0) {
                unsigned long p = now - inj_rise_time;
                if (p > 0) inj_last_period_us = p;
            }
            inj_rise_time = now;
        } else { // Active -> Inactive (Falling edge of duty)
            if (inj_rise_time > 0 && inj_last_period_us > 0) {
                float pulse_width = now - inj_rise_time;
                inj_duty = pulse_width / inj_last_period_us;
                corr = 1.0 - (inj_duty - 0.1);
            }
        }
        inj_prev_state = inj_state;
    }

#ifndef UNIT_TEST
    // Update display (every 100ms)
    static unsigned long last_disp = 0;
    if (millis() - last_disp > 100) {
        oled.clearDisplay();
        oled.setCursor(0,0);
        oled.print("AFR: "); oled.println(afr);
        oled.print("Duty: "); oled.print(inj_duty * 100); oled.println("%");

        for(int i=0; i<GRAPH_WIDTH; i++) {
            int val = (afr_buffer[(afr_buffer_index + i) % GRAPH_WIDTH] - 10) * 4;
            oled.drawPixel(i, 63 - val, SH110X_WHITE);
        }
        oled.display();
        last_disp = millis();
    }
#endif
}
