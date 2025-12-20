// UDP to BarPlotter DMA Display
// Original project by piotrcurious, refactored by Jules to use BarPlotter
#include "wifi_settings.h"
#include "display_settings.h" // For TFT_eSPI setup and pins
#include "BarPlotter.h"
#include <IRremote.h>
#include <AsyncUDP.h>


// --- BarPlotter Config ---
#define BAR_COUNT             TFT_WIDTH
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS TFT_HEIGHT
#define PLOT_X                0
#define PLOT_Y                0
#define PLOT_SPACING          0

// --- IR Config ---
#define IR_PIN 15

// ---------- IR KEY TABLE (from example) ----------
enum KeyCode {
  KEY_POWER = 0x02,
  KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06, KEY_4 = 0x08,
  KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D, KEY_9 = 0x0E,
  KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65, KEY_RIGHT = 0x62,
  KEY_OK = 0x68, KEY_MENU = 0x79, KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b,
  KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10
};

// --- Globals ---
BarPlotter plotter(tft);
AsyncUDP udp;

// --- Data Handling ---
#define PACKET_BUFFER_SIZE 2
#pragma pack(push, 1)
struct PulseData {
  float rpm[PACKET_BUFFER_SIZE];
  uint32_t length[PACKET_BUFFER_SIZE];
  uint8_t index;
};
#pragma pack(pop)

// Rolling buffer to hold the raw pulse length data
uint32_t rollingBuffer[BAR_COUNT];
uint32_t graphMin = 0;
uint32_t graphMax = 40000; // Start with a reasonable max

// Snapshot buffer to hold the scaled bar heights for the plotter
uint16_t snapshot[BAR_COUNT];

// --- Function Prototypes ---
void handlePacket(AsyncUDPPacket packet);
void serial_plotter_helper();
void handle_ir();

void setup() {
  Serial.begin(115200);
  delay(50);

  // --- TFT & Plotter Setup ---
  tft.init();
  tft.initDMA();
  pinMode(TFT_LED,OUTPUT);
  digitalWrite(TFT_LED,LOW);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);
  plotter.setFrameRate(30.0f);

  // Initialize rolling buffer
  for (int i = 0; i < BAR_COUNT; i++) {
    rollingBuffer[i] = 0;
  }

  // --- WiFi & UDP Setup ---
  WiFi.persistent(false);
#ifdef AP_mode_on
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
#else
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
#endif
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (udp.listenMulticast(multicastIP, multicastPort)) {
    Serial.println("UDP Listening");
    udp.onPacket(handlePacket);
  } else {
    Serial.println("UDP Setup Failed");
  }

  // --- IR Setup ---
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  Serial.printf("IR receiver started on pin %d\n", IR_PIN);
}

void loop() {
  serial_plotter_helper();
  handle_ir();
  yield();
}

IRAM_ATTR void handlePacket(AsyncUDPPacket packet) {
  PulseData pulseData;
  if (packet.length() != sizeof(pulseData)) {
    return; // Ignore malformed packets
  }
  memcpy(&pulseData, packet.data(), sizeof(pulseData));

  for (int i = 0; i < PACKET_BUFFER_SIZE; i++) {
    for (int j = 0; j < BAR_COUNT - 1; j++) {
      rollingBuffer[j] = rollingBuffer[j + 1];
    }
    rollingBuffer[BAR_COUNT - 1] = pulseData.length[i];
  }

  // Simplified auto-scaling
  graphMin = rollingBuffer[0];
  graphMax = rollingBuffer[0];
  for (int i = 1; i < BAR_COUNT; i++) {
    if (rollingBuffer[i] > graphMax) graphMax = rollingBuffer[i];
    if (rollingBuffer[i] < graphMin) graphMin = rollingBuffer[i];
  }

  if (graphMin == graphMax) {
      graphMax = graphMin + 1;
  }

  for (int i = 0; i < BAR_COUNT; i++) {
    snapshot[i] = map(rollingBuffer[i], graphMin, graphMax, 0, BAR_MAX_HEIGHT_PIXELS);
  }

  plotter.startBarPlot(snapshot, BAR_COUNT);
}

uint8_t selected = 1;
bool serial_debug_enabled = false;

void serial_plotter_helper() {
  if (Serial.available() == 0) return;
  char input = Serial.read();

  if (input >= '1' && input <= '9') {
    selected = input - '0';
  } else if (input == 'd' || input == 'D') {
    serial_debug_enabled = !serial_debug_enabled;
    plotter.enableDebug(serial_debug_enabled);
    Serial.printf("Debug %s\n", serial_debug_enabled ? "ENABLED" : "DISABLED");
    return;
  } else if (input == 's' || input == 'S') {
    plotter.printConfig();
    Serial.printf("Plotter state: %s\n", plotter.frameStateString());
    return;
  } else if (input == 'r' || input == 'R') {
    Serial.printf("Target plot generation rate: %.2f Hz\n", plotter._target_rate_hz);
    return;
  } else if (input == '-' || input == '_') {
    switch (selected) {
      case 1: { uint8_t v = plotter._even_frctr2; v--; plotter.setEvenTiming(v, plotter._even_fporch, plotter._even_bporch); break; }
      case 2: { uint8_t v = plotter._even_fporch; v--; plotter.setEvenTiming(plotter._even_frctr2, v, plotter._even_bporch); break; }
      case 3: { uint8_t v = plotter._even_bporch; v--; plotter.setEvenTiming(plotter._even_frctr2, plotter._even_fporch, v); break; }
      case 4: { uint8_t v = plotter._odd_frctr2; v--; plotter.setOddTiming(v, plotter._odd_fporch, plotter._odd_bporch); break; }
      case 5: { uint8_t v = plotter._odd_fporch; v--; plotter.setOddTiming(plotter._odd_frctr2, v, plotter._odd_bporch); break; }
      case 6: { uint8_t v = plotter._odd_bporch; v--; plotter.setOddTiming(plotter._odd_frctr2, plotter._odd_fporch, v); break; }
      case 7: { uint32_t p = plotter._phase_send_offset; p = (p > 100) ? p - 100 : 0; plotter.setPhaseOffsetUs(p); break; }
      case 8: { uint32_t m = plotter._mid_offset; m = (m > 50) ? m - 50 : 0; plotter.setMidOffsetUs(m); break; }
      case 9: { float r = plotter._target_rate_hz; r = max(1.0f, r - 0.5f); plotter.setFrameRate(r); break; }
    }
    plotter.printConfig();
  } else if (input == '=' || input == '+') {
    switch (selected) {
      case 1: { uint8_t v = plotter._even_frctr2; v++; plotter.setEvenTiming(v, plotter._even_fporch, plotter._even_bporch); break; }
      case 2: { uint8_t v = plotter._even_fporch; v++; plotter.setEvenTiming(plotter._even_frctr2, v, plotter._even_bporch); break; }
      case 3: { uint8_t v = plotter._even_bporch; v++; plotter.setEvenTiming(plotter._even_frctr2, plotter._even_fporch, v); break; }
      case 4: { uint8_t v = plotter._odd_frctr2; v++; plotter.setOddTiming(v, plotter._odd_fporch, plotter._odd_bporch); break; }
      case 5: { uint8_t v = plotter._odd_fporch; v++; plotter.setOddTiming(plotter._odd_frctr2, v, plotter._odd_bporch); break; }
      case 6: { uint8_t v = plotter._odd_bporch; v++; plotter.setOddTiming(plotter._odd_frctr2, plotter._odd_fporch, v); break; }
      case 7: { uint32_t p = plotter._phase_send_offset; p += 100; plotter.setPhaseOffsetUs(p); break; }
      case 8: { uint32_t m = plotter._mid_offset; m += 50; plotter.setMidOffsetUs(m); break; }
      case 9: { float r = plotter._target_rate_hz; r = min(120.0f, r + 0.5f); plotter.setFrameRate(r); break; }
    }
    plotter.printConfig();
  }
}

void handle_ir() {
  if (!IrReceiver.decode()) return;

  if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x07) {
    uint32_t cmd_raw = IrReceiver.decodedIRData.command;
    uint8_t key = (uint8_t)(cmd_raw & 0xFF);
    Serial.printf("IR (SAM+0x07) command: 0x%02X (raw 0x%08X)\n", key, (unsigned)cmd_raw);

    switch(key) {
        case KEY_VOL_UP: {
            float r = plotter._target_rate_hz;
            r = min(120.0f, r + 1.0f);
            plotter.setFrameRate(r);
            plotter.printConfig();
            break;
        }
        case KEY_VOL_DOWN: {
            float r = plotter._target_rate_hz;
            r = max(1.0f, r - 1.0f);
            plotter.setFrameRate(r);
            plotter.printConfig();
            break;
        }
        case KEY_OK:
            plotter.printConfig();
            break;
        default:
            Serial.printf("Unmapped key 0x%02X\n", key);
            break;
    }
  }

  IrReceiver.resume();
}
