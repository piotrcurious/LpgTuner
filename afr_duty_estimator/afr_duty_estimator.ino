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
float rpm = 0;

bool o2_warm = false;
int o2_transitions = 0;

bool o2_state = false;
bool o2_prev_state = false;

unsigned long o2_rise_time = 0;
unsigned long o2_last_period_us = 0;

// Interrupt variables for Injector
volatile unsigned long last_inj_fall_us = 0;
volatile unsigned long last_inj_width_us = 0;
volatile unsigned long last_inj_period_us = 0;
volatile bool inj_updated = false;

#define GRAPH_WIDTH 128
float afr_buffer[GRAPH_WIDTH];
int afr_buffer_index = 0;

#ifndef UNIT_TEST
Adafruit_SH1107 oled(64, 128, &Wire); // Example SH1107 initialization
#endif

void IRAM_ATTR inj_isr() {
    unsigned long now = micros();
    bool state = (digitalRead(INJ_PIN) == LOW);
    static bool last_state = false;

    if (state != last_state) {
        if (state) { // Falling edge (Active)
            if (last_inj_fall_us > 0) {
                last_inj_period_us = now - last_inj_fall_us;
            }
            last_inj_fall_us = now;
        } else { // Rising edge (Inactive)
            if (last_inj_fall_us > 0) {
                last_inj_width_us = now - last_inj_fall_us;
                inj_updated = true;
            }
        }
        last_state = state;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(INJ_PIN, INPUT_PULLUP);

    // Initial read to stabilize filters
    float initial_o2 = analogRead(O2_PIN) * (5.0 / 1023.0);
    o2_voltage = initial_o2;
    o2_prev_state = (o2_voltage > O2_THRESHOLD);

    for(int i=0; i<GRAPH_WIDTH; i++) afr_buffer[i] = 14.7;

    attachInterrupt(digitalPinToInterrupt(INJ_PIN), inj_isr, CHANGE);

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
                float duty = high_time / (float)o2_last_period_us;
                if (duty > 1.0) duty = 1.0;

                afr = 14.7 - (duty - 0.5) * 10.0;
                afr_ema = (afr_ema * 0.8) + (afr * 0.2);

                o2_transitions++;
                if (o2_transitions > 4) o2_warm = true;

                if (o2_warm) {
                    if (afr_ema < afr_min) afr_min = afr_ema;
                    if (afr_ema > afr_max) afr_max = afr_ema;
                }

                afr_buffer[afr_buffer_index] = afr_ema;
                afr_buffer_index = (afr_buffer_index + 1) % GRAPH_WIDTH;
            }
        }
        o2_prev_state = o2_state;
    }

    // Injector signal analysis from ISR
    if (inj_updated) {
        unsigned long p, w;
        // Basic atomic read (may need critical section on some platforms)
        noInterrupts();
        p = last_inj_period_us;
        w = last_inj_width_us;
        inj_updated = false;
        interrupts();

        if (p > 0) {
            inj_duty = (float)w / p;
            if (inj_duty > 1.0) inj_duty = 1.0;
            inj_duty_ema = (inj_duty_ema * 0.9) + (inj_duty * 0.1);

            // RPM Calculation: 4-stroke, 2 revs per injection cycle per cylinder.
            // But usually for a single injector signal, 1 pulse = 2 revs?
            // Or 1 pulse per rev if it's wasted spark/group fire?
            // Assuming 1 injection per 2 revolutions (Standard 4-stroke sequential).
            // RPM = (1 / period_sec) * 60 * 2
            rpm = (2.0 * 60.0 * 1000000.0) / p;
        }
    } else {
        // Handle engine stop
        static unsigned long last_inj_time = 0;
        if (now - last_inj_fall_us > 500000) { // 0.5s timeout (120 RPM)
            rpm = 0;
            inj_duty_ema = 0;
        }
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
        if (!o2_warm) {
            oled.println("WARMUP");
        } else {
            if (afr_ema < 13.5) oled.print("R ");
            else if (afr_ema > 15.5) oled.print("L ");
            oled.println(afr_ema, 1);
        }

        // Stats
        oled.setTextSize(1);
        oled.print("Min: "); oled.print(afr_min, 1);
        oled.print(" Max: "); oled.println(afr_max, 1);

        // RPM and Duty
        oled.print("RPM:"); oled.print((int)rpm);
        oled.print(" D:");
        oled.print(inj_duty_ema * 100, 0); oled.println("%");

        // Rolling Graph
        for(int i=0; i<GRAPH_WIDTH; i++) {
            float val_afr = afr_buffer[(afr_buffer_index + i) % GRAPH_WIDTH];
            int y = 63 - (int)((val_afr - 10.0) * 6.0);
            if (y < 24) y = 24;
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
