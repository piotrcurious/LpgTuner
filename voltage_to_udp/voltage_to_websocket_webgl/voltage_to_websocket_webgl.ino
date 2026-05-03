#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <Preferences.h>
#include "esp_adc_cal.h"
#include "frontend.h"

extern "C" {
  #include "esp_adc/adc_continuous.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

// ADC Configuration
#define ADC_CHANNELS_COUNT 4
static adc_channel_t adc_channels[ADC_CHANNELS_COUNT] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7};

volatile uint32_t adc_sample_freq = 50000;
#define ADC_READ_BUF_BYTES    (16 * 1024)
#define ADC_FRAME_SIZE        256
#define ADC_READ_TIMEOUT_MS   10

static adc_continuous_handle_t adc_handle = NULL;
static esp_adc_cal_characteristics_t *adc_chars;

// Simulation Mode
volatile bool simulation_mode = false;
volatile uint8_t sim_noise_level = 0; // 0-100
volatile bool adc_needs_reconfig = false;
float sim_phase = 0;

// Trigger Configuration
#define TRIGGER_PINS_COUNT 4
uint8_t trigger_pins[] = {12, 13, 14, 27};
volatile bool trigger_occurred[TRIGGER_PINS_COUNT] = {false, false, false, false};
volatile uint32_t last_trigger_event_time[TRIGGER_PINS_COUNT] = {0, 0, 0, 0};
#define TRIGGER_HOLDOFF_MS 10

enum TriggerMode { MODE_INDEPENDENT, MODE_CAM_WHEEL };
volatile TriggerMode currentMode = MODE_INDEPENDENT;
volatile int trigger_edge = RISING; // RISING or FALLING
volatile bool is_running = true;
volatile bool trigger_armed = true; // For "Normal" mode
volatile int run_mode = 0; // 0: Auto, 1: Normal, 2: Single
volatile uint8_t trigger_mask = 0x0F; // All 4 pins by default
volatile uint8_t cam_wheel_target = 0;

// Analog Trigger Configuration
volatile int8_t analog_trig_ch = -1; // -1: Disabled, 0-3: Channel index
volatile uint16_t analog_trig_level = 2048;
volatile uint16_t analog_trig_hyst = 50; // 50 units (~40mV)
volatile uint8_t trigger_pos_pct = 10; // 10% of window is pre-trigger
volatile bool hysteresis_armed = false;

// Data Buffering
#define SAMPLES_PER_PACKET 512
#pragma pack(push, 1)
struct ScopePacket {
  uint32_t sequence;
  uint32_t timestamp;
  uint16_t trigger_flags;
  uint16_t trigger_sample_idx;
  uint32_t free_heap;
  uint16_t loop_time_us; // Execution time of one processing loop
  uint16_t interval_us;  // Time between packet sends
  uint16_t data[SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT];
};
#pragma pack(pop)

static ScopePacket *packet_ptr;
uint32_t packet_seq = 0;

// Networking
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// Multi-core safe access
portMUX_TYPE trigger_mux = portMUX_INITIALIZER_UNLOCKED;

// Interrupts
void IRAM_ATTR handleTrigger(int idx) {
  uint32_t now = millis();
  if (now - last_trigger_event_time[idx] < TRIGGER_HOLDOFF_MS) return;
  last_trigger_event_time[idx] = now;

  portENTER_CRITICAL_ISR(&trigger_mux);
  if (idx == 0 && currentMode == MODE_CAM_WHEEL) {
    trigger_occurred[cam_wheel_target] = true;
    cam_wheel_target = (cam_wheel_target + 1) % ADC_CHANNELS_COUNT;
  } else if (currentMode == MODE_INDEPENDENT) {
    trigger_occurred[idx] = true;
  }
  portEXIT_CRITICAL_ISR(&trigger_mux);
}

void IRAM_ATTR handleTrigger0() { handleTrigger(0); }
void IRAM_ATTR handleTrigger1() { handleTrigger(1); }
void IRAM_ATTR handleTrigger2() { handleTrigger(2); }
void IRAM_ATTR handleTrigger3() { handleTrigger(3); }

void updateInterrupts() {
  for (int i = 0; i < TRIGGER_PINS_COUNT; i++) {
    detachInterrupt(digitalPinToInterrupt(trigger_pins[i]));
  }
  attachInterrupt(digitalPinToInterrupt(trigger_pins[0]), handleTrigger0, trigger_edge);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[1]), handleTrigger1, trigger_edge);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[2]), handleTrigger2, trigger_edge);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[3]), handleTrigger3, trigger_edge);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      if (len > 0 && data[0] == 'M') {
        currentMode = (data[1] == '1') ? MODE_CAM_WHEEL : MODE_INDEPENDENT;
      } else if (len > 0 && data[0] == 'R') {
        run_mode = data[1] - '0';
        trigger_armed = true;
      } else if (len > 0 && data[0] == 'F') {
        is_running = (data[1] == '1');
      } else if (len > 0 && data[0] == 'E') {
        trigger_edge = (data[1] == '1') ? RISING : FALLING;
        updateInterrupts();
      } else if (len > 0 && data[0] == 'K') { // Trigger MasK
        trigger_mask = data[1];
      } else if (len > 0 && data[0] == 'T') { // force Trigger
        trigger_occurred[0] = true;
      } else if (len > 0 && data[0] == 'A') { // Analog Trigger: A[CH][LEVEL_H][LEVEL_L]
        analog_trig_ch = (int8_t)data[1];
        if (len >= 4) analog_trig_level = (data[2] << 8) | data[3];
      } else if (len > 0 && data[0] == 'P') { // Position
        trigger_pos_pct = data[1];
      } else if (len > 0 && data[0] == 'H') { // sample rate (Hz)
        adc_sample_freq = atoi((char*)data + 1);
        adc_needs_reconfig = true;
      } else if (len > 0 && data[0] == 'S') {
        simulation_mode = (data[1] == '1');
      } else if (len > 0 && data[0] == 'N') {
        sim_noise_level = atoi((char*)data + 1);
      } else if (len > 0 && data[0] == 'W') {
        String cmd = String((char*)data).substring(2);
        int sep = cmd.indexOf(';');
        if (sep != -1) {
          preferences.begin("wifi", false);
          preferences.putString("ssid", cmd.substring(0, sep));
          preferences.putString("pass", cmd.substring(sep + 1));
          preferences.putInt("mode", 0);
          preferences.end();
          ESP.restart();
        }
      }
    }
  }
}

void setup_adc() {
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = ADC_READ_BUF_BYTES,
    .conv_frame_size = ADC_FRAME_SIZE,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

  adc_continuous_config_t dig_cfg = {
    .pattern_num = ADC_CHANNELS_COUNT,
    .adc_pattern = NULL,
    .sample_freq_hz = adc_sample_freq,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };

  adc_digi_pattern_config_t adc_pattern[ADC_CHANNELS_COUNT];
  for (int i = 0; i < ADC_CHANNELS_COUNT; i++) {
    adc_pattern[i].atten = ADC_ATTEN_DB_11;
    adc_pattern[i].channel = adc_channels[i];
    adc_pattern[i].unit = ADC_UNIT_1;
    adc_pattern[i].bit_width = ADC_BITWIDTH_12;
  }
  dig_cfg.adc_pattern = adc_pattern;

  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
  ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void scope_task(void *pv) {
  uint8_t *read_raw = (uint8_t*)malloc(ADC_READ_BUF_BYTES);
  const int CIRC_BUF_SIZE = 1024;
  uint16_t *circ_buf = (uint16_t*)malloc(CIRC_BUF_SIZE * ADC_CHANNELS_COUNT * sizeof(uint16_t));
  uint32_t circ_write_idx = 0;

  packet_ptr = (ScopePacket*)malloc(sizeof(ScopePacket));
  uint32_t last_packet_time = micros();
  bool triggered = false;
  uint32_t trigger_at_idx = 0;
  uint32_t samples_after_trigger = 0;
  uint16_t prev_analog_val[ADC_CHANNELS_COUNT] = {0,0,0,0};

  while (1) {
    if (adc_needs_reconfig) {
      adc_continuous_stop(adc_handle);
      adc_continuous_deinit(adc_handle);
      setup_adc();
      adc_needs_reconfig = false;
    }
    if (ws.count() == 0 || !is_running) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    uint32_t loop_start = micros();

    if (simulation_mode) {
      for (int i = 0; i < 64; i++) {
        uint16_t current_samples[ADC_CHANNELS_COUNT];
        for (int ch = 0; ch < ADC_CHANNELS_COUNT; ch++) {
          float val = 0;
          if (ch == 0) val = 2048 + 1200 * sin(sim_phase); // CH1: Engine Speed/Phase
          else if (ch == 1) val = 1500 + 800 * sin(sim_phase * 0.7); // CH2: Lambda/O2
          else if (ch == 2) val = (fmod(sim_phase, 4.0 * PI) / (4.0 * PI)) * 3800; // CH3: TPS
          else if (ch == 3) val = (sin(sim_phase * 4.0) > 0.95) ? 3800 : 400; // CH4: Ignition/Crank Pulse
          if (sim_noise_level > 0) val += (random(sim_noise_level * 15) - (sim_noise_level * 7));
          current_samples[ch] = (uint16_t)constrain(val, 0, 4095);
          circ_buf[circ_write_idx * ADC_CHANNELS_COUNT + ch] = current_samples[ch];
        }
        sim_phase += 0.05;

        if (!triggered) {
          bool trig = false;
          portENTER_CRITICAL(&trigger_mux);
          for (int t = 0; t < TRIGGER_PINS_COUNT; t++) {
             if (trigger_occurred[t] && (trigger_mask & (1 << t))) { trig = true; packet_ptr->trigger_flags |= (1 << t); }
          }
          portEXIT_CRITICAL(&trigger_mux);

          if (analog_trig_ch >= 0 && analog_trig_ch < ADC_CHANNELS_COUNT) {
            uint16_t val = current_samples[analog_trig_ch];
            if (trigger_edge == RISING) {
              if (!hysteresis_armed && val < (analog_trig_level - analog_trig_hyst)) hysteresis_armed = true;
              if (hysteresis_armed && val >= analog_trig_level) { trig = true; packet_ptr->trigger_flags |= (1 << 4); hysteresis_armed = false; }
            } else {
              if (!hysteresis_armed && val > (analog_trig_level + analog_trig_hyst)) hysteresis_armed = true;
              if (hysteresis_armed && val <= analog_trig_level) { trig = true; packet_ptr->trigger_flags |= (1 << 4); hysteresis_armed = false; }
            }
          }

          if (trig || run_mode == 0) {
            if (run_mode == 0 || trigger_armed) {
              triggered = true;
              trigger_at_idx = circ_write_idx;
              samples_after_trigger = 0;
            }
          }
        }

        for(int ch=0; ch<ADC_CHANNELS_COUNT; ch++) prev_analog_val[ch] = current_samples[ch];
        circ_write_idx = (circ_write_idx + 1) % CIRC_BUF_SIZE;

        if (triggered) {
          uint32_t post_trigger_needed = SAMPLES_PER_PACKET * (100 - trigger_pos_pct) / 100;
          samples_after_trigger++;
          if (samples_after_trigger >= post_trigger_needed) {
            uint32_t now = micros();
            packet_ptr->sequence = packet_seq++;
            packet_ptr->timestamp = now;
            packet_ptr->free_heap = ESP.getFreeHeap();
            packet_ptr->loop_time_us = (uint16_t)(micros() - loop_start);
            packet_ptr->interval_us = (uint16_t)(now - last_packet_time);
            last_packet_time = now;
            if (currentMode == MODE_CAM_WHEEL) packet_ptr->trigger_flags |= (1 << 7);

            uint32_t pre_trigger_samples = SAMPLES_PER_PACKET * trigger_pos_pct / 100;
            int start_idx = (int)trigger_at_idx - (int)pre_trigger_samples;
            if (start_idx < 0) start_idx += CIRC_BUF_SIZE;
            packet_ptr->trigger_sample_idx = (uint16_t)pre_trigger_samples;

            for (int s = 0; s < SAMPLES_PER_PACKET; s++) {
              int src = (start_idx + s) % CIRC_BUF_SIZE;
              for (int ch = 0; ch < ADC_CHANNELS_COUNT; ch++) {
                uint32_t raw = circ_buf[src * ADC_CHANNELS_COUNT + ch];
                packet_ptr->data[s * ADC_CHANNELS_COUNT + ch] = (uint16_t)esp_adc_cal_raw_to_voltage(raw, adc_chars);
              }
            }

            ws.binaryAll((uint8_t*)packet_ptr, sizeof(ScopePacket));
            if (run_mode == 2) trigger_armed = false;
            triggered = false;
            packet_ptr->trigger_flags = 0;
            portENTER_CRITICAL(&trigger_mux);
            for(int t=0; t<TRIGGER_PINS_COUNT; t++) trigger_occurred[t] = false;
            portEXIT_CRITICAL(&trigger_mux);
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    } else {
      uint32_t out_len = 0;
      esp_err_t ret = adc_continuous_read(adc_handle, read_raw, ADC_READ_BUF_BYTES, &out_len, ADC_READ_TIMEOUT_MS);
      if (ret == ESP_OK && out_len > 0) {
        size_t count = out_len / sizeof(adc_digi_output_data_t);
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)read_raw;
        for (size_t i = 0; i < count; i++) {
          uint32_t chan = p[i].type1.channel;
          uint32_t val = p[i].type1.data & 0xFFF;
          int chan_idx = -1;
          for(int j=0; j<ADC_CHANNELS_COUNT; j++) if(adc_channels[j] == (adc_channel_t)chan) { chan_idx = j; break; }
          if(chan_idx == -1) continue;
          circ_buf[circ_write_idx * ADC_CHANNELS_COUNT + chan_idx] = (uint16_t)val;

          if (chan_idx == ADC_CHANNELS_COUNT - 1) {
            if (!triggered) {
              bool trig = false;
              portENTER_CRITICAL(&trigger_mux);
              for (int t = 0; t < TRIGGER_PINS_COUNT; t++) {
                 if (trigger_occurred[t] && (trigger_mask & (1 << t))) { trig = true; packet_ptr->trigger_flags |= (1 << t); }
              }
              portEXIT_CRITICAL(&trigger_mux);

              if (analog_trig_ch >= 0 && analog_trig_ch < ADC_CHANNELS_COUNT) {
                uint16_t v = (uint16_t)val;
                if (trigger_edge == RISING) {
                  if (!hysteresis_armed && v < (analog_trig_level - analog_trig_hyst)) hysteresis_armed = true;
                  if (hysteresis_armed && v >= analog_trig_level) { trig = true; packet_ptr->trigger_flags |= (1 << 4); hysteresis_armed = false; }
                } else {
                  if (!hysteresis_armed && v > (analog_trig_level + analog_trig_hyst)) hysteresis_armed = true;
                  if (hysteresis_armed && v <= analog_trig_level) { trig = true; packet_ptr->trigger_flags |= (1 << 4); hysteresis_armed = false; }
                }
              }

              if (trig || run_mode == 0) {
                if (run_mode == 0 || trigger_armed) {
                  triggered = true;
                  trigger_at_idx = circ_write_idx;
                  samples_after_trigger = 0;
                }
              }
            }

            for(int ch=0; ch<ADC_CHANNELS_COUNT; ch++) {
               if(ch == chan_idx) prev_analog_val[ch] = (uint16_t)val;
               else prev_analog_val[ch] = circ_buf[circ_write_idx * ADC_CHANNELS_COUNT + ch];
            }
            circ_write_idx = (circ_write_idx + 1) % CIRC_BUF_SIZE;

            if (triggered) {
              uint32_t post_trigger_needed = SAMPLES_PER_PACKET * (100 - trigger_pos_pct) / 100;
              samples_after_trigger++;
              if (samples_after_trigger >= post_trigger_needed) {
                uint32_t now = micros();
                packet_ptr->sequence = packet_seq++;
                packet_ptr->timestamp = now;
                packet_ptr->free_heap = ESP.getFreeHeap();
                packet_ptr->loop_time_us = (uint16_t)(micros() - loop_start);
                packet_ptr->interval_us = (uint16_t)(now - last_packet_time);
                last_packet_time = now;
                if (currentMode == MODE_CAM_WHEEL) packet_ptr->trigger_flags |= (1 << 7);

                uint32_t pre_trigger_samples = SAMPLES_PER_PACKET * trigger_pos_pct / 100;
                int start_idx = (int)trigger_at_idx - (int)pre_trigger_samples;
                if (start_idx < 0) start_idx += CIRC_BUF_SIZE;
                packet_ptr->trigger_sample_idx = (uint16_t)pre_trigger_samples;

                for (int s = 0; s < SAMPLES_PER_PACKET; s++) {
                  int src = (start_idx + s) % CIRC_BUF_SIZE;
                  for (int ch = 0; ch < ADC_CHANNELS_COUNT; ch++) {
                    uint32_t raw = circ_buf[src * ADC_CHANNELS_COUNT + ch];
                    packet_ptr->data[s * ADC_CHANNELS_COUNT + ch] = (uint16_t)esp_adc_cal_raw_to_voltage(raw, adc_chars);
                  }
                }

                ws.binaryAll((uint8_t*)packet_ptr, sizeof(ScopePacket));
                if (run_mode == 2) trigger_armed = false;
                triggered = false;
                packet_ptr->trigger_flags = 0;
                portENTER_CRITICAL(&trigger_mux);
                for(int t=0; t<TRIGGER_PINS_COUNT; t++) trigger_occurred[t] = false;
                portEXIT_CRITICAL(&trigger_mux);
              }
            }
          }
        }
      }
      vTaskDelay(1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  preferences.begin("wifi", false);
  String saved_ssid = preferences.getString("ssid", "");
  String saved_pass = preferences.getString("pass", "");
  int mode = preferences.getInt("mode", 0);

  if (saved_ssid.length() > 0 && mode == 0) {
    preferences.putInt("mode", 1);
    preferences.end();
    WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { delay(500); Serial.print("."); }
    if (WiFi.status() == WL_CONNECTED) {
      preferences.begin("wifi", false);
      preferences.putInt("mode", 0);
      preferences.end();
    } else {
      preferences.begin("wifi", false);
      preferences.putInt("mode", 0);
      preferences.end();
      WiFi.softAP("SCOPE_PRO_AP", "12345678");
    }
  } else {
    preferences.putInt("mode", 0);
    preferences.end();
    WiFi.softAP("SCOPE_PRO_AP", "12345678");
  }

  for (int i = 0; i < TRIGGER_PINS_COUNT; i++) pinMode(trigger_pins[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[0]), handleTrigger0, RISING);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[1]), handleTrigger1, RISING);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[2]), handleTrigger2, RISING);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[3]), handleTrigger3, RISING);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", html_index); });
  server.begin();

  setup_adc();
  xTaskCreatePinnedToCore(scope_task, "scope", 8192, NULL, 5, NULL, 1);
}

void loop() {
  ws.cleanupClients();
  delay(100);
}
