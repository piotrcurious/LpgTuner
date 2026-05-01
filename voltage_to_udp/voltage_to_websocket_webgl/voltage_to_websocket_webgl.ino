#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include "wifi_settings.h"
#include "frontend.h"

extern "C" {
  #include "esp_adc/adc_continuous.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

// ADC Configuration
#define ADC_CHANNELS_COUNT 4
// ADC1 Channels: 0, 3, 6, 7 -> GPIOs 36, 39, 34, 35
static adc_channel_t adc_channels[ADC_CHANNELS_COUNT] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7};

#define ADC_SAMPLE_FREQ_HZ    50000 // 50kHz total, so 12.5kHz per channel
#define ADC_READ_BUF_BYTES    (16 * 1024)
#define ADC_FRAME_SIZE        256
#define ADC_READ_TIMEOUT_MS   10

static adc_continuous_handle_t adc_handle = NULL;

// Trigger Configuration
#define TRIGGER_PINS_COUNT 4
uint8_t trigger_pins[] = {12, 13, 14, 27};
volatile uint32_t last_trigger_time[TRIGGER_PINS_COUNT] = {0, 0, 0, 0};
volatile bool trigger_occurred[TRIGGER_PINS_COUNT] = {false, false, false, false};

enum TriggerMode {
  MODE_INDEPENDENT,
  MODE_CAM_WHEEL
};
volatile TriggerMode currentMode = MODE_INDEPENDENT;
volatile uint8_t cam_wheel_target = 0;

// Data Buffering
#define SAMPLES_PER_PACKET 512
#pragma pack(push, 1)
struct ScopePacket {
  uint32_t sequence;
  uint32_t timestamp;
  uint8_t trigger_flags; // bit 0-3: trigger occurred for chan 0-3
  uint16_t data[SAMPLES_PER_PACKET * ADC_CHANNELS_COUNT];
};
#pragma pack(pop)

ScopePacket packet;
uint32_t packet_seq = 0;

// Networking
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Interrupts
void IRAM_ATTR handleTrigger0() {
  if (currentMode == MODE_INDEPENDENT) {
    trigger_occurred[0] = true;
  } else {
    trigger_occurred[cam_wheel_target] = true;
    cam_wheel_target = (cam_wheel_target + 1) % ADC_CHANNELS_COUNT;
  }
  last_trigger_time[0] = micros();
}
void IRAM_ATTR handleTrigger1() { trigger_occurred[1] = true; last_trigger_time[1] = micros(); }
void IRAM_ATTR handleTrigger2() { trigger_occurred[2] = true; last_trigger_time[2] = micros(); }
void IRAM_ATTR handleTrigger3() { trigger_occurred[3] = true; last_trigger_time[3] = micros(); }

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("Client %u connected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    if (len > 0 && data[0] == 'M') {
      if (data[1] == '0') currentMode = MODE_INDEPENDENT;
      else if (data[1] == '1') currentMode = MODE_CAM_WHEEL;
      Serial.printf("Mode set to: %s\n", currentMode == MODE_INDEPENDENT ? "Independent" : "Cam Wheel");
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
        for(int j=0; j<ADC_CHANNELS_COUNT; j++) {
            if(adc_channels[j] == chan) {
                chan_idx = j;
                break;
            }
        }
        if(chan_idx == -1) continue;

        packet.data[sample_idx * ADC_CHANNELS_COUNT + chan_idx] = (uint16_t)val;

        if (chan_idx == ADC_CHANNELS_COUNT - 1) {
          sample_idx++;
          if (sample_idx >= SAMPLES_PER_PACKET) {
            packet.sequence = packet_seq++;
            packet.timestamp = micros();
            packet.trigger_flags = 0;
            for (int t = 0; t < TRIGGER_PINS_COUNT; t++) {
              if (trigger_occurred[t]) {
                packet.trigger_flags |= (1 << t);
                trigger_occurred[t] = false;
              }
            }

            ws.binaryAll((uint8_t*)&packet, sizeof(packet));
            sample_idx = 0;
          }
        }
      }
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < TRIGGER_PINS_COUNT; i++) {
    pinMode(trigger_pins[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(trigger_pins[0]), handleTrigger0, RISING);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[1]), handleTrigger1, RISING);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[2]), handleTrigger2, RISING);
  attachInterrupt(digitalPinToInterrupt(trigger_pins[3]), handleTrigger3, RISING);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

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
