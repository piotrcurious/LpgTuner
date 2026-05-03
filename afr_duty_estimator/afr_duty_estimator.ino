// AFR and Duty Estimator with OLED Display
// Estimates AFR and injector duty cycle, displaying results on an OLED screen.

#ifndef UNIT_TEST
#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#endif

// Pins
const int O2_PIN = A0;
const int INJ_PIN = 2;

// Constants
const float EMA_ALPHA = 0.1;
const float O2_THRESHOLD = 0.45;

// Variables
float o2_voltage = 0.45;
float inj_duty = 0;
float inj_duty_ema = 0;
float afr = 14.7;
float afr_ema = 14.7;
float afr_min = 20.0;
float afr_max = 0.0;

bool o2_state = false;
bool o2_prev_state = false;
bool inj_state = false;
bool inj_prev_state = false;

unsigned long o2_rise_time = 0;
unsigned long o2_last_period_us = 0;
unsigned long inj_fall_time = 0; // Time when injector became active (Falling edge)
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

    // Initial read to stabilize filters
    float initial_o2 = analogRead(O2_PIN) * (5.0 / 1023.0);
    o2_voltage = initial_o2;
    o2_prev_state = (o2_voltage > O2_THRESHOLD);

    inj_state = (digitalRead(INJ_PIN) == LOW);
    inj_prev_state = inj_state;

    for(int i=0; i<GRAPH_WIDTH; i++) afr_buffer[i] = 14.7;

#ifndef UNIT_TEST
    oled.begin(0x3C, true);
    oled.clearDisplay();
    oled.display();
#endif
}

void loop() {
    float raw_o2 = analogRead(O2_PIN) * (5.0 / 1023.0);
    // EMA filtering for display voltage
    o2_voltage = (o2_voltage * (1.0 - EMA_ALPHA)) + (raw_o2 * EMA_ALPHA);

    o2_state = (raw_o2 > O2_THRESHOLD);
    inj_state = (digitalRead(INJ_PIN) == LOW); // Active low injectors

    unsigned long now = micros();

    // O2 signal analysis
    if (o2_state != o2_prev_state) {
        if (o2_state) { // Low -> High (Lean -> Rich crossing)
            if (o2_rise_time > 0) {
                unsigned long p = now - o2_rise_time;
                if (p > 50000) o2_last_period_us = p; // Sanity check > 50ms
            }
            o2_rise_time = now;
        } else { // High -> Low (Rich -> Lean crossing)
            if (o2_rise_time > 0 && o2_last_period_us > 0) {
                float high_time = now - o2_rise_time;
                float duty = high_time / o2_last_period_us;
                if (duty > 1.0) duty = 1.0;

                // AFR Estimation from Narrowband Duty Cycle
                // High Duty = More time Rich = Lower AFR
                afr = 14.7 - (duty - 0.5) * 10.0;
                afr_ema = (afr_ema * 0.8) + (afr * 0.2);

                if (afr_ema < afr_min) afr_min = afr_ema;
                if (afr_ema > afr_max) afr_max = afr_ema;

                afr_buffer[afr_buffer_index] = afr_ema;
                afr_buffer_index = (afr_buffer_index + 1) % GRAPH_WIDTH;
            }
        }
        o2_prev_state = o2_state;
    }

    // Injector signal analysis
    if (inj_state != inj_prev_state) {
        if (inj_state) { // Inactive -> Active (Falling edge)
            if (inj_fall_time > 0) {
                unsigned long p = now - inj_fall_time;
                if (p > 0) inj_last_period_us = p;
            }
            inj_fall_time = now;
        } else { // Active -> Inactive (Rising edge)
            if (inj_fall_time > 0 && inj_last_period_us > 0) {
                float pulse_width = now - inj_fall_time;
                inj_duty = pulse_width / inj_last_period_us;
                if (inj_duty > 1.0) inj_duty = 1.0;
                inj_duty_ema = (inj_duty_ema * 0.9) + (inj_duty * 0.1);
            }
        }
        inj_prev_state = inj_state;
    }

#ifndef UNIT_TEST
    // Update display (every 50ms)
    static unsigned long last_disp = 0;
    if (millis() - last_disp > 50) {
        oled.clearDisplay();
        oled.setTextColor(SH110X_WHITE);

        // AFR Reading
        oled.setCursor(0,0);
        oled.setTextSize(1);
        oled.print("AFR: ");
        oled.setTextSize(2);
        if (afr_ema < 13.5) oled.print("R ");
        else if (afr_ema > 15.5) oled.print("L ");
        oled.println(afr_ema, 1);

        // Stats
        oled.setTextSize(1);
        oled.print("Min: "); oled.print(afr_min, 1);
        oled.print(" Max: "); oled.println(afr_max, 1);

        // Duty Cycle
        oled.print("Duty: ");
        oled.print(inj_duty_ema * 100, 1); oled.println("%");

        // Rolling Graph
        for(int i=0; i<GRAPH_WIDTH; i++) {
            float val_afr = afr_buffer[(afr_buffer_index + i) % GRAPH_WIDTH];
            // Scale AFR 10-18 to pixels 20-63
            int y = 63 - (int)((val_afr - 10.0) * 6.0);
            if (y < 20) y = 20;
            if (y > 63) y = 63;
            oled.drawPixel(i, y, SH110X_WHITE);
        }

        // Stoich Line
        int stoich_y = 63 - (int)((14.7 - 10.0) * 6.0);
        oled.drawFastHLine(0, stoich_y, GRAPH_WIDTH, SH110X_WHITE);

        oled.display();
        last_disp = millis();
    }
#endif
}
