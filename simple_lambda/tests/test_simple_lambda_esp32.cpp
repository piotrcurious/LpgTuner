#include "mock_arduino.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

// Mock some ESP32 specifics
#define ADC_CHANNEL_0 0
#define ADC_UNIT_1 1
#define ADC_ATTEN_DB_11 3
#define ADC_BITWIDTH_12 12
#define SOC_ADC_DIGI_RESULT_BYTES 4
#define ESP_OK 0

typedef int esp_err_t;
typedef void* adc_continuous_handle_t;

struct adc_continuous_handle_cfg_t {
    uint32_t max_store_buf_size;
    uint32_t conv_frame_size;
};

struct adc_digi_pattern_config_t {
    uint8_t atten;
    uint8_t channel;
    uint8_t unit;
    uint8_t bit_width;
};

struct adc_continuous_config_t {
    uint32_t pattern_num;
    adc_digi_pattern_config_t* adc_pattern;
    uint32_t sample_freq_hz;
    int conv_mode;
    int format;
};

typedef struct {
    struct {
        unsigned int data : 12;
        unsigned int channel : 4;
        unsigned int unit : 4;
        unsigned int reserved : 12;
    } type1;
} adc_digi_output_data_t;

#define ADC_CONV_SINGLE_UNIT_1 1
#define ADC_DIGI_OUTPUT_FORMAT_TYPE1 1

// Mock functions
esp_err_t adc_continuous_new_handle(adc_continuous_handle_cfg_t* cfg, adc_continuous_handle_t* handle) { return ESP_OK; }
esp_err_t adc_continuous_config(adc_continuous_handle_t handle, adc_continuous_config_t* cfg) { return ESP_OK; }
esp_err_t adc_continuous_start(adc_continuous_handle_t handle) { return ESP_OK; }

// We will manually call updateOscillation and other logic since it's hard to mock the task and DMA read perfectly without more effort.
// So we include the .ino but we will call its internal functions.

#define UNIT_TEST
#include "../simple_lambda_esp32.ino"

// Override micros() for test
uint32_t mock_micros = 0;
uint32_t micros() { return mock_micros; }

void test_esp32_oscillation() {
    std::cout << "Testing ESP32 oscillation logic..." << std::endl;

    // Reset state
    osc = OscState();
    currentFreq = 0;
    currentDuty = 0;
    mock_micros = 0;

    // Simulate 1Hz oscillation: 0.2V to 0.8V
    // 0.5s Rich (0.8V), 0.5s Lean (0.2V)

    // Start Rich
    for(int i=0; i<100; i++) {
        updateOscillation(0.8);
        mock_micros += 1000; // 1ms steps
    }
    std::cout << "After Rich soak: Peak=" << osc.peak << " Valley=" << osc.valley << " Thr=" << osc.threshold << " High=" << osc.isHigh << std::endl;
    assert(osc.peak > 0.7);

    // Switch to Lean
    for(int i=0; i<500; i++) {
        updateOscillation(0.2);
        mock_micros += 1000;
    }
    std::cout << "After Lean soak: Peak=" << osc.peak << " Valley=" << osc.valley << " Thr=" << osc.threshold << " High=" << osc.isHigh << std::endl;
    assert(!osc.isHigh);
    assert(osc.valley < 0.3);

    // Switch to Rich again (Rising Edge)
    for(int i=0; i<500; i++) {
        updateOscillation(0.8);
        mock_micros += 1000;
    }
    std::cout << "After 2nd Rich soak: Freq=" << currentFreq << " Duty=" << currentDuty * 100 << "%" << std::endl;

    // Switch to Lean again (Falling Edge)
    for(int i=0; i<500; i++) {
        updateOscillation(0.2);
        mock_micros += 1000;
    }

    // Switch to Rich again to complete 2nd period and get freq
    for(int i=0; i<500; i++) {
        updateOscillation(0.8);
        mock_micros += 1000;
    }

    std::cout << "Final ESP32 Freq: " << currentFreq << " Hz" << std::endl;
    std::cout << "Final ESP32 Duty: " << currentDuty * 100 << " %" << std::endl;

    assert(currentFreq > 0.9 && currentFreq < 1.1);
    assert(currentDuty > 0.45 && currentDuty < 0.55);
}

int main() {
    test_esp32_oscillation();
    std::cout << "ESP32 logic tests passed!" << std::endl;
    return 0;
}
