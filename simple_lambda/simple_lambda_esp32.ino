// Simple Lambda Probe Monitor - ESP32 DMA Version
// This code monitors a narrowband lambda probe using ESP32's ADC continuous (DMA) driver.
// It features a sophisticated oscillation detector with peak/valley tracking and adaptive hysteresis.

#ifndef UNIT_TEST
#include <Arduino.h>
extern "C" {
  #include "esp_adc/adc_continuous.h"
}
#endif

// ---------- Configuration ----------
#define ADC_CHANNEL          ADC_CHANNEL_0 // GPIO 36 (VP) on many ESP32 boards
#define ADC_UNIT             ADC_UNIT_1
#define SAMPLING_RATE_HZ     20000
#define READ_BUF_BYTES       2048
#define DECIMATION_FACTOR    20    // 20000 / 20 = 1000Hz effective processing rate
#define ADC_ATTEN            ADC_ATTEN_DB_11 // 0-3.3V range

// Stoichiometric constants
const float stoichRatio = 14.7;

// ---------- Globals ----------
#ifndef UNIT_TEST
static adc_continuous_handle_t adc_handle = NULL;
#endif

volatile float currentAFR = 14.7;
volatile float currentVoltage = 0.45;
volatile float currentFreq = 0;
volatile float currentDuty = 0;
volatile float signalPeak = 0.8;
volatile float signalValley = 0.2;

// Oscillation detector state
struct OscState {
    float peak = 0.7;
    float valley = 0.3;
    float threshold = 0.5;
    float hysteresis = 0.05;
    bool isHigh = false;
    uint32_t lastEdgeTimeUs = 0;
    uint32_t highDurationUs = 0;
    uint32_t lowDurationUs = 0;

    // EMA filter for peak/valley tracking
    float alpha = 0.005; // Slow tracking
} osc;

#ifdef UNIT_TEST
uint32_t micros();
#endif

void updateOscillation(float voltage) {
    uint32_t now = micros();

    // Track peak and valley with EMA for a "moving envelope"
    if (voltage > osc.peak) {
        osc.peak = voltage;
    } else {
        osc.peak = osc.peak * (1.0f - osc.alpha) + voltage * osc.alpha;
    }

    if (voltage < osc.valley) {
        osc.valley = voltage;
    } else {
        osc.valley = osc.valley * (1.0f - osc.alpha) + voltage * osc.alpha;
    }

    // Adaptive threshold and hysteresis
    osc.threshold = (osc.peak + osc.valley) / 2.0f;
    float swing = osc.peak - osc.valley;
    osc.hysteresis = swing * 0.15f; // 15% of swing as hysteresis
    if (osc.hysteresis < 0.02f) osc.hysteresis = 0.02f; // Minimum hysteresis

    // State machine for edge detection
    if (!osc.isHigh && voltage > (osc.threshold + osc.hysteresis)) {
        // Rising edge detected
        osc.isHigh = true;
        if (osc.lastEdgeTimeUs != 0) {
            osc.lowDurationUs = now - osc.lastEdgeTimeUs;
            uint32_t periodUs = osc.highDurationUs + osc.lowDurationUs;
            if (periodUs > 0) {
                currentFreq = 1000000.0f / periodUs;
                currentDuty = (float)osc.highDurationUs / periodUs;
            }
        }
        osc.lastEdgeTimeUs = now;
    } else if (osc.isHigh && voltage < (osc.threshold - osc.hysteresis)) {
        // Falling edge detected
        osc.isHigh = false;
        if (osc.lastEdgeTimeUs != 0) {
            osc.highDurationUs = now - osc.lastEdgeTimeUs;
        }
        osc.lastEdgeTimeUs = now;
    }

    // Export tracking values
    signalPeak = osc.peak;
    signalValley = osc.valley;

    // Timeout: if no edge for 2 seconds, reset frequency/duty
    if (now - osc.lastEdgeTimeUs > 2000000) {
        currentFreq = 0;
        currentDuty = 0;
    }
}

#ifndef UNIT_TEST
void setupADC() {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size = READ_BUF_BYTES / 2,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = NULL,
        .sample_freq_hz = SAMPLING_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t pattern[1];
    pattern[0].atten = ADC_ATTEN;
    pattern[0].channel = ADC_CHANNEL;
    pattern[0].unit = ADC_UNIT;
    pattern[0].bit_width = ADC_BITWIDTH_12;
    dig_cfg.adc_pattern = pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void adcTask(void *pv) {
    uint8_t *result = (uint8_t *)malloc(READ_BUF_BYTES);
    if (result == NULL) {
        Serial.println("Failed to allocate ADC buffer");
        vTaskDelete(NULL);
        return;
    }
    uint32_t out_len = 0;

    static uint32_t decimateCount = 0;
    static float decimateSum = 0;

    while (1) {
        esp_err_t ret = adc_continuous_read(adc_handle, result, READ_BUF_BYTES, &out_len, pdMS_TO_TICKS(10));
        if (ret == ESP_OK) {
            for (int i = 0; i < out_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                uint32_t chan = p->type1.channel;
                uint32_t data = p->type1.data & 0xFFF;

                if (chan == ADC_CHANNEL) {
                    float voltage = data * (3.3f / 4095.0f);
                    decimateSum += voltage;
                    decimateCount++;

                    if (decimateCount >= DECIMATION_FACTOR) {
                        float avgVoltage = decimateSum / DECIMATION_FACTOR;
                        currentVoltage = avgVoltage;

                        // Sophisticated oscillation detector
                        updateOscillation(avgVoltage);

                        // Simple linear AFR mapping for display purposes (0.1V=15.4, 0.9V=14.0)
                        currentAFR = 14.7f - (avgVoltage - 0.45f) * 1.8375f;

                        decimateSum = 0;
                        decimateCount = 0;
                    }
                }
            }
        }
        vTaskDelay(1);
    }
}

void setup() {
    Serial.begin(115200);
    setupADC();
    xTaskCreatePinnedToCore(adcTask, "adcTask", 4096, NULL, 5, NULL, 1);
    Serial.println("Simple Lambda ESP32 DMA Initialized");
}

void loop() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 200) {
        Serial.printf("AFR: %.2f | V: %.3f | Freq: %.2f Hz | Duty: %.1f%% | P: %.2f V | V: %.2f V\n",
                      currentAFR, currentVoltage, currentFreq, currentDuty * 100.0f, signalPeak, signalValley);
        lastPrint = millis();
    }
    delay(10);
}
#endif
