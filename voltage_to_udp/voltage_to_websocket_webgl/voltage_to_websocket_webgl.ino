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

#define ADC_SAMPLE_FREQ_HZ    50000
#define ADC_READ_BUF_BYTES    (16 * 1024)
#define ADC_FRAME_SIZE        256
#define ADC_READ_TIMEOUT_MS   10

static adc_continuous_handle_t adc_handle = NULL;
static esp_adc_cal_characteristics_t *adc_chars;

// Simulation Mode
volatile bool simulation_mode = false;
volatile uint8_t sim_noise_level = 0; // 0-100
float sim_phase = 0;

// Trigger Configuration
#define TRIGGER_PINS_COUNT 4
uint8_t trigger_pins[] = {12, 13, 14, 27};
volatile bool trigger_occurred[TRIGGER_PINS_COUNT] = {false, false, false, false};
volatile uint32_t last_trigger_event_time[TRIGGER_PINS_COUNT] = {0, 0, 0, 0};
#define TRIGGER_HOLDOFF_MS 10

enum TriggerMode { MODE_INDEPENDENT, MODE_CAM_WHEEL };
volatile TriggerMode currentMode = MODE_INDEPENDENT;
volatile uint8_t cam_wheel_target = 0;

// Data Buffering
#define SAMPLES_PER_PACKET 512
#pragma pack(push, 1)
struct ScopePacket {
  uint32_t sequence;
  uint32_t timestamp;
  uint16_t trigger_flags;
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

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      if (len > 0 && data[0] == 'M') {
        currentMode = (data[1] == '1') ? MODE_CAM_WHEEL : MODE_INDEPENDENT;
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
    .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
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
  uint32_t sample_idx = 0;
  packet_ptr = (ScopePacket*)malloc(sizeof(ScopePacket));

  uint32_t last_packet_time = micros();

  while (1) {
    if (ws.count() == 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    uint32_t loop_start = micros();

    if (simulation_mode) {
      for (int i = 0; i < 64; i++) {
        for (int ch = 0; ch < ADC_CHANNELS_COUNT; ch++) {
          float val = 0;
          if (ch == 0) val = 2048 + 1000 * sin(sim_phase);
          else if (ch == 1) val = 2048 + 1000 * cos(sim_phase * 1.5);
          else if (ch == 2) val = (fmod(sim_phase, 2.0 * PI) / (2.0 * PI)) * 4095;
          else if (ch == 3) val = (sin(sim_phase * 5.0) > 0) ? 3500 : 500;
          if (sim_noise_level > 0) val += (random(sim_noise_level * 10) - (sim_noise_level * 5));
          packet_ptr->data[sample_idx * ADC_CHANNELS_COUNT + ch] = (uint16_t)constrain(val, 0, 4095);
        }
        sim_phase += 0.05;
        if (packet_seq % 10 == 0 && sample_idx == 0) trigger_occurred[0] = true;

        sample_idx++;
        if (sample_idx >= SAMPLES_PER_PACKET) {
          uint32_t now = micros();
          packet_ptr->sequence = packet_seq++;
          packet_ptr->timestamp = now;
          packet_ptr->free_heap = ESP.getFreeHeap();
          packet_ptr->loop_time_us = (uint16_t)(micros() - loop_start);
          packet_ptr->interval_us = (uint16_t)(now - last_packet_time);
          last_packet_time = now;
          packet_ptr->trigger_flags = (currentMode == MODE_CAM_WHEEL) ? (1 << 7) : 0;

          portENTER_CRITICAL(&trigger_mux);
          for (int t = 0; t < TRIGGER_PINS_COUNT; t++) if (trigger_occurred[t]) { packet_ptr->trigger_flags |= (1 << t); trigger_occurred[t] = false; }
          portEXIT_CRITICAL(&trigger_mux);

          ws.binaryAll((uint8_t*)packet_ptr, sizeof(ScopePacket));
          sample_idx = 0;
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
          packet_ptr->data[sample_idx * ADC_CHANNELS_COUNT + chan_idx] = (uint16_t)val;
          if (chan_idx == ADC_CHANNELS_COUNT - 1) {
            sample_idx++;
            if (sample_idx >= SAMPLES_PER_PACKET) {
              uint32_t now = micros();
              packet_ptr->sequence = packet_seq++;
              packet_ptr->timestamp = now;
              packet_ptr->free_heap = ESP.getFreeHeap();
              packet_ptr->loop_time_us = (uint16_t)(micros() - loop_start);
              packet_ptr->interval_us = (uint16_t)(now - last_packet_time);
              last_packet_time = now;
              packet_ptr->trigger_flags = (currentMode == MODE_CAM_WHEEL) ? (1 << 7) : 0;

              portENTER_CRITICAL(&trigger_mux);
              for (int t = 0; t < TRIGGER_PINS_COUNT; t++) if (trigger_occurred[t]) { packet_ptr->trigger_flags |= (1 << t); trigger_occurred[t] = false; }
              portEXIT_CRITICAL(&trigger_mux);

              ws.binaryAll((uint8_t*)packet_ptr, sizeof(ScopePacket));
              sample_idx = 0;
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
