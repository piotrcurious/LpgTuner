#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <Preferences.h>
#include "wifi_settings.h"
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

// Trigger Configuration
#define TRIGGER_PINS_COUNT 4
uint8_t trigger_pins[] = {12, 13, 14, 27};
volatile uint32_t last_trigger_time[TRIGGER_PINS_COUNT] = {0, 0, 0, 0};
volatile bool trigger_occurred[TRIGGER_PINS_COUNT] = {false, false, false, false};

enum TriggerMode { MODE_INDEPENDENT, MODE_CAM_WHEEL };
volatile TriggerMode currentMode = MODE_INDEPENDENT;
volatile uint8_t cam_wheel_target = 0;

// Data Buffering
#define SAMPLES_PER_PACKET 512
#pragma pack(push, 1)
struct ScopePacket {
  uint32_t sequence;
  uint32_t timestamp;
  uint8_t trigger_flags;
  uint16_t data[SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT];
};
#pragma pack(pop)

ScopePacket packet;
uint32_t packet_seq = 0;

// Networking
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// WiFi handling
String saved_ssid = "";
String saved_pass = "";

// Interrupts
void IRAM_ATTR handleTrigger0() {
  if (currentMode == MODE_INDEPENDENT) trigger_occurred[0] = true;
  else {
    trigger_occurred[cam_wheel_target] = true;
    cam_wheel_target = (cam_wheel_target + 1) % ADC_CHANNELS_COUNT;
  }
  last_trigger_time[0] = micros();
}
void IRAM_ATTR handleTrigger1() { trigger_occurred[1] = true; last_trigger_time[1] = micros(); }
void IRAM_ATTR handleTrigger2() { trigger_occurred[2] = true; last_trigger_time[2] = micros(); }
void IRAM_ATTR handleTrigger3() { trigger_occurred[3] = true; last_trigger_time[3] = micros(); }

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      if (len > 0 && data[0] == 'M') {
        currentMode = (data[1] == '1') ? MODE_CAM_WHEEL : MODE_INDEPENDENT;
      } else if (len > 0 && data[0] == 'W') {
        // WiFi Command: W;SSID;PASS
        String cmd = String((char*)data).substring(2);
        int sep = cmd.indexOf(';');
        if (sep != -1) {
          String s = cmd.substring(0, sep);
          String p = cmd.substring(sep + 1);
          preferences.begin("wifi", false);
          preferences.putString("ssid", s);
          preferences.putString("pass", p);
          preferences.putInt("mode", 0); // Reset fallback
          preferences.end();
          Serial.println("WiFi Config Saved. Restarting...");
          delay(500);
          ESP.restart();
        }
      }
    }
  }
}

void setup_adc() {
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
  while (1) {
    uint32_t out_len = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, read_raw, ADC_READ_BUF_BYTES, &out_len, ADC_READ_TIMEOUT_MS);
    if (ret == ESP_OK && out_len > 0) {
      size_t count = out_len / sizeof(adc_digi_output_data_t);
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)read_raw;
      for (size_t i = 0; i < count; i++) {
        uint32_t chan = p[i].type1.channel;
        uint32_t val = p[i].type1.data & 0xFFF;
        int chan_idx = -1;
        for(int j=0; j<ADC_CHANNELS_COUNT; j++) if(adc_channels[j] == chan) { chan_idx = j; break; }
        if(chan_idx == -1) continue;
        packet.data[sample_idx * ADC_CHANNELS_COUNT + chan_idx] = (uint16_t)val;
        if (chan_idx == ADC_CHANNELS_COUNT - 1) {
          sample_idx++;
          if (sample_idx >= SAMPLES_PER_PACKET) {
            packet.sequence = packet_seq++;
            packet.timestamp = micros();
            packet.trigger_flags = 0;
            for (int t = 0; t < TRIGGER_PINS_COUNT; t++) if (trigger_occurred[t]) { packet.trigger_flags |= (1 << t); trigger_occurred[t] = false; }
            ws.binaryAll((uint8_t*)&packet, sizeof(packet));
            sample_idx = 0;
          }
        }
      }
    }
    vTaskDelay(1);
  }
}

void start_ap() {
  Serial.println("Starting Access Point...");
  WiFi.softAP("SCOPE_PRO_AP", "12345678");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  preferences.begin("wifi", false);
  saved_ssid = preferences.getString("ssid", "");
  saved_pass = preferences.getString("pass", "");
  int mode = preferences.getInt("mode", 0); // 0 = Normal, 1 = Fallback AP

  bool try_sta = (saved_ssid.length() > 0 && mode == 0);

  if (try_sta) {
    Serial.println("Attempting STA connection to: " + saved_ssid);
    preferences.putInt("mode", 1); // Set fallback for next boot if we crash/reset
    preferences.end();

    WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { delay(500); Serial.print("."); }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nSTA Connected: " + WiFi.localIP().toString());
      preferences.begin("wifi", false);
      preferences.putInt("mode", 0); // Clear fallback on success
      preferences.end();
    } else {
      Serial.println("\nSTA Failed. Falling back to AP.");
      preferences.begin("wifi", false);
      preferences.putInt("mode", 0); // Clear fallback to allow STA next time
      preferences.end();
      start_ap();
    }
  } else {
    if (mode == 1) {
      Serial.println("Fallback AP Mode detected (reset during connection).");
      preferences.putInt("mode", 0); // Next boot will try STA again
    }
    preferences.end();
    start_ap();
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
