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
#define RING_BUFFER_SIZE (BAR_COUNT * 4) // Increased for more history
#pragma pack(push, 1)
struct PulseData {
  float rpm[PACKET_BUFFER_SIZE];
  uint32_t length[PACKET_BUFFER_SIZE];
  uint8_t index;
};
#pragma pack(pop)

// Double buffer for ring buffer to prevent race conditions
uint32_t ringBufferA[RING_BUFFER_SIZE];
uint32_t ringBufferB[RING_BUFFER_SIZE];
volatile uint32_t* activeRingBuffer = ringBufferA;
volatile uint32_t* inactiveRingBuffer = ringBufferB;
volatile int ringBufferPos = 0;

uint32_t graphMin = 0;
uint32_t graphMax = 40000;

// Snapshot buffer to hold the scaled bar heights for the plotter
uint16_t snapshot[BAR_COUNT];

// --- Buffering and Triggering ---
uint16_t* pending_snapshot = nullptr;
volatile bool pending_snapshot_valid = false;
volatile bool trigger_enabled = false;
volatile uint32_t trigger_value = 20000; // Default trigger value
const size_t trigger_align_bar_index = BAR_COUNT / 2;
volatile int trigger_slope = 1; // 1 for rising, -1 for falling
volatile float triggered_copy_freq_hz = 30.0f;
volatile float fallback_copy_freq_hz = 5.0f;
volatile uint32_t trigger_hold_ms = 50;
volatile uint32_t last_trigger_time_ms = 0;
volatile uint32_t min_trigger_separation_ms = 50;
volatile uint32_t last_trigger_event_ms = 0;
volatile uint16_t trigger_hysteresis = 8;
volatile size_t last_trigger_ring_index = SIZE_MAX;
volatile uint32_t generated_frames = 0;
volatile uint32_t dropped_frames = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// --- Function Prototypes ---
void handlePacket(AsyncUDPPacket packet);
void serial_plotter_helper();
void handle_ir();
void try_flush_pending();
bool attempt_queue_snapshot(uint16_t* snapshot_data);
void produce_snapshot_triggered(int trigger_index);


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
  for (int i = 0; i < RING_BUFFER_SIZE; i++) {
    ringBufferA[i] = 0;
    ringBufferB[i] = 0;
  }

  // Allocate pending snapshot buffer
  pending_snapshot = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  if (!pending_snapshot) {
      Serial.println("Failed to allocate pending_snapshot buffer");
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
  try_flush_pending();
  serial_plotter_helper();
  handle_ir();

  int local_ringBufferPos;
  size_t local_last_trigger_ring_index;
  uint32_t local_last_trigger_time_ms;

  // Atomically swap buffers and copy indices
  portENTER_CRITICAL(&mux);
  volatile uint32_t* temp = activeRingBuffer;
  activeRingBuffer = inactiveRingBuffer;
  inactiveRingBuffer = temp;
  local_ringBufferPos = ringBufferPos;
  local_last_trigger_ring_index = last_trigger_ring_index;
  local_last_trigger_time_ms = last_trigger_time_ms;
  portEXIT_CRITICAL(&mux);

  uint32_t now = millis();
  static uint32_t last_triggered_copy_ms = 0;
  static uint32_t last_fallback_copy_ms = 0;
  static uint32_t last_scale_calc_ms = 0;

  // Recalculate min/max for scaling periodically
  if (now - last_scale_calc_ms > 500) {
      graphMin = inactiveRingBuffer[0];
      graphMax = inactiveRingBuffer[0];
      for (int i = 1; i < RING_BUFFER_SIZE; i++) {
        if (inactiveRingBuffer[i] > graphMax) graphMax = inactiveRingBuffer[i];
        if (inactiveRingBuffer[i] < graphMin) graphMin = inactiveRingBuffer[i];
      }

      if (graphMin == graphMax) {
        graphMax = graphMin + 1;
      }
      last_scale_calc_ms = now;
  }


  if (trigger_enabled) {
    if (now - local_last_trigger_time_ms <= trigger_hold_ms) {
      if (now - last_triggered_copy_ms >= (1000 / triggered_copy_freq_hz)) {
        if (local_last_trigger_ring_index != SIZE_MAX) {
          produce_snapshot_triggered(local_last_trigger_ring_index);
          attempt_queue_snapshot(snapshot);
        }
        last_triggered_copy_ms = now;
      }
    } else {
      if (now - last_fallback_copy_ms >= (1000 / fallback_copy_freq_hz)) {
        if (local_last_trigger_ring_index != SIZE_MAX) {
            produce_snapshot_triggered(local_last_trigger_ring_index);
        } else {
            produce_snapshot_triggered(local_ringBufferPos);
        }
        attempt_queue_snapshot(snapshot);
        last_fallback_copy_ms = now;
      }
    }
  } else {
    // Non-triggered mode: update at fallback frequency
    if (now - last_fallback_copy_ms >= (1000 / fallback_copy_freq_hz)) {
      produce_snapshot_triggered(local_ringBufferPos);
      attempt_queue_snapshot(snapshot);
      last_fallback_copy_ms = now;
    }
  }

  yield();
}

void try_flush_pending() {
  if (pending_snapshot_valid && pending_snapshot) {
    if (plotter.startBarPlot(pending_snapshot, BAR_COUNT)) {
      generated_frames++;
      pending_snapshot_valid = false;
    } else {
      dropped_frames++;
    }
  }
}

bool attempt_queue_snapshot(uint16_t* snapshot_data) {
  try_flush_pending();

  if (plotter.startBarPlot(snapshot_data, BAR_COUNT)) {
    generated_frames++;
    return true;
  } else {
    if (!pending_snapshot_valid && pending_snapshot) {
      memcpy(pending_snapshot, snapshot_data, sizeof(uint16_t) * BAR_COUNT);
      pending_snapshot_valid = true;
      return true;
    }
    dropped_frames++;
  }
  return false;
}

void produce_snapshot_triggered(int trigger_ring_index) {
  int start_index = trigger_ring_index - trigger_align_bar_index;

  for (int i = 0; i < BAR_COUNT; i++) {
    int source_index = (start_index + i + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
    snapshot[i] = map(inactiveRingBuffer[source_index], graphMin, graphMax, 0, BAR_MAX_HEIGHT_PIXELS);
  }
}

void handlePacket(AsyncUDPPacket packet) {
  PulseData pulseData;
  if (packet.length() != sizeof(pulseData)) {
    return;
  }
  memcpy(&pulseData, packet.data(), sizeof(pulseData));

  portENTER_CRITICAL(&mux);
  uint32_t prev_value = activeRingBuffer[(ringBufferPos - 1 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE];

  for (int i = 0; i < PACKET_BUFFER_SIZE; i++) {
    uint32_t current_value = pulseData.length[i];
    activeRingBuffer[ringBufferPos] = current_value;

    if (trigger_enabled) {
        uint32_t low_th = (trigger_value > trigger_hysteresis) ? (uint32_t)trigger_value - trigger_hysteresis : 0u;
        uint32_t high_th = (uint32_t)trigger_value + trigger_hysteresis;

        bool crossed = false;
        if (trigger_slope > 0 && prev_value <= low_th && current_value > high_th) {
            crossed = true;
        } else if (trigger_slope < 0 && prev_value >= high_th && current_value < low_th) {
            crossed = true;
        }

        if (crossed) {
            uint32_t now = millis();
            if (now - last_trigger_event_ms >= min_trigger_separation_ms) {
                last_trigger_time_ms = now;
                last_trigger_event_ms = now;
                last_trigger_ring_index = ringBufferPos;
            }
        }
    }

    ringBufferPos = (ringBufferPos + 1) % RING_BUFFER_SIZE;
    prev_value = current_value;
  }
  portEXIT_CRITICAL(&mux);
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
    Serial.printf("Frames generated: %u, dropped: %u\n", (unsigned)generated_frames, (unsigned)dropped_frames);
    Serial.printf("Plotter state: %s\n", plotter.frameStateString());
    return;
  } else if (input == 'r' || input == 'R') {
    Serial.printf("Target plot generation rate: %.2f Hz\n", plotter._target_rate_hz);
    return;
  } else if (input == '-' || input == '_') {
    portENTER_CRITICAL(&mux);
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
    portEXIT_CRITICAL(&mux);
    plotter.printConfig();
  } else if (input == '=' || input == '+') {
    portENTER_CRITICAL(&mux);
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
    portEXIT_CRITICAL(&mux);
    plotter.printConfig();
  }
}

void handle_ir() {
  if (!IrReceiver.decode()) return;

  if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x07) {
    uint32_t cmd_raw = IrReceiver.decodedIRData.command;
    uint8_t key = (uint8_t)(cmd_raw & 0xFF);
    Serial.printf("IR (SAM+0x07) command: 0x%02X (raw 0x%08X)\n", key, (unsigned)cmd_raw);

    portENTER_CRITICAL(&mux);
    switch(key) {
        case KEY_POWER:
            trigger_enabled = !trigger_enabled;
            if (!trigger_enabled) {
                last_trigger_ring_index = SIZE_MAX;
            }
            Serial.printf("Trigger %s\n", trigger_enabled ? "ENABLED" : "DISABLED");
            break;
        case KEY_UP:
            trigger_slope = 1;
            Serial.println("Trigger slope: RISING");
            break;
        case KEY_DOWN:
            trigger_slope = -1;
            Serial.println("Trigger slope: FALLING");
            break;
        case KEY_LEFT:
            trigger_value = (trigger_value > 500) ? trigger_value - 500 : 0;
            Serial.printf("Trigger value decreased -> %u\n", trigger_value);
            break;
        case KEY_RIGHT:
            trigger_value = (trigger_value < 4294967295) ? trigger_value + 500 : 4294967295;
            Serial.printf("Trigger value increased -> %u\n", trigger_value);
            break;
        case KEY_VOL_UP: {
            triggered_copy_freq_hz = min(200.0f, triggered_copy_freq_hz + 5.0f);
            Serial.printf("Triggered copy freq -> %.2f Hz\n", triggered_copy_freq_hz);
            break;
        }
        case KEY_VOL_DOWN: {
            triggered_copy_freq_hz = max(1.0f, triggered_copy_freq_hz - 5.0f);
            Serial.printf("Triggered copy freq -> %.2f Hz\n", triggered_copy_freq_hz);
            break;
        }
        case KEY_CH_UP: {
            fallback_copy_freq_hz = min(60.0f, fallback_copy_freq_hz + 1.0f);
            Serial.printf("Fallback copy freq -> %.2f Hz\n", fallback_copy_freq_hz);
            break;
        }
        case KEY_CH_DOWN: {
            fallback_copy_freq_hz = max(0.1f, fallback_copy_freq_hz - 1.0f);
            Serial.printf("Fallback copy freq -> %.2f Hz\n", fallback_copy_freq_hz);
            break;
        }
        case KEY_OK:
            Serial.printf("STATUS: trigger=%s slope=%d value=%u hold_ms=%u triggered_copy=%.2f fallback_copy=%.2f pending=%d\n",
                      trigger_enabled ? "ON" : "OFF", trigger_slope, trigger_value, (unsigned)trigger_hold_ms, triggered_copy_freq_hz, fallback_copy_freq_hz, pending_snapshot_valid ? 1 : 0);
            break;
        case KEY_MENU:
            if (pending_snapshot_valid) {
                pending_snapshot_valid = false;
                Serial.println("Pending snapshot CLEARED");
            } else {
                Serial.println("Pending snapshot slot EMPTY");
            }
            break;
        case KEY_1:
            trigger_hold_ms = (trigger_hold_ms > 10) ? trigger_hold_ms - 10 : 0;
            Serial.printf("Trigger hold decreased -> %u ms\n", trigger_hold_ms);
            break;
        case KEY_2:
            trigger_hold_ms += 10;
            Serial.printf("Trigger hold increased -> %u ms\n", trigger_hold_ms);
            break;
        case KEY_3:
            trigger_hysteresis = (trigger_hysteresis > 1) ? trigger_hysteresis - 1 : 0;
            Serial.printf("Trigger hysteresis decreased -> %u\n", trigger_hysteresis);
            break;
        case KEY_4:
            trigger_hysteresis++;
            Serial.printf("Trigger hysteresis increased -> %u\n", trigger_hysteresis);
            break;
        default:
            Serial.printf("Unmapped key 0x%02X\n", key);
            break;
    }
    portEXIT_CRITICAL(&mux);
  }

  IrReceiver.resume();
}
