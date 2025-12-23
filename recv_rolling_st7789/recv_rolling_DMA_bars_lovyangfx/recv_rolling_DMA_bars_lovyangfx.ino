// UDP to BarPlotter DMA Display
// Original project by piotrcurious, refactored by Jules to use BarPlotter
#include "wifi_settings.h"
#include "display_settings_lovyangfx.h" // For LovyanGFX setup and pins
#include "BarPlotter_Lovyan.h"
#include <IRremote.h>
#include <AsyncUDP.h>


// --- BarPlotter Config ---
#define BAR_COUNT             320
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS 240
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
  pinMode(TFT_LED,OUTPUT);
  digitalWrite(TFT_LED,LOW);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);
//  plotter.setFrameRate(23.62);
//  plotter.setFrameRate(24.3f);
//  plotter.setFrameRate(20.870);
  plotter.setFrameRate(16.48);

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

// ----------------- Frame timing helper -----------------
// Add this function anywhere above loop() (e.g. after serial_plotter_helper())
// Call as: printFrameTimingEstimates(); or printFrameTimingEstimates(20000000UL);

void printFrameTimingEstimates(uint32_t spi_hz = 20000000UL) {
  // Adjust these if your display uses different bytes-per-pixel
  const uint32_t BYTES_PER_PIXEL = 2; // 16-bit (RGB565)

  // Basic geometry taken from the running sketch
  const uint32_t cols = BAR_COUNT;                    // 320
  const uint32_t rows = BAR_MAX_HEIGHT_PIXELS;        // TFT_HEIGHT (240)
  const uint32_t pixels_per_frame = cols * rows;
  const uint32_t frame_bytes = pixels_per_frame * BYTES_PER_PIXEL;

  // Time to transfer raw pixel data over SPI (in microseconds)
  // SPI transfers 1 bit per clock, so total bits = bytes * 8
  const double pixel_transfer_us = (double)frame_bytes * 8.0 * 1e6 / (double)spi_hz;

  // Interpret the per-column "timing" fields as byte counts of overhead per column.
  // Even/odd columns count:
  const uint32_t even_cols = (cols + 1) / 2;
  const uint32_t odd_cols  = cols / 2;

  // Per-column overhead bytes (assumption: these fields count bytes)
  const uint32_t even_overhead_bytes_per_col = (uint32_t)plotter._even_frctr2 + (uint32_t)plotter._even_fporch + (uint32_t)plotter._even_bporch;
  const uint32_t odd_overhead_bytes_per_col  = (uint32_t)plotter._odd_frctr2  + (uint32_t)plotter._odd_fporch  + (uint32_t)plotter._odd_bporch;

  const uint32_t total_overhead_bytes =
    even_cols * even_overhead_bytes_per_col +
    odd_cols  * odd_overhead_bytes_per_col;

  const double overhead_transfer_us = (double)total_overhead_bytes * 8.0 * 1e6 / (double)spi_hz;

  // Phase offsets (already in microseconds in your code)
  const double phase_us = (double)plotter._phase_send_offset;
  const double mid_us   = (double)plotter._mid_offset;

  // Total estimated frame time and resulting rate
  const double total_frame_us = pixel_transfer_us + overhead_transfer_us; //+ phase_us + mid_us;
  const double estimated_hz = (total_frame_us > 0.0) ? (1e6 / total_frame_us) : 0.0;

  // Target rate from plotter
  const double target_hz = (double)plotter._target_rate_hz;

  // Print nicely
  Serial.println(F("---- Frame timing estimate ----"));
  Serial.printf("SPI clock: %u Hz\n", (unsigned)spi_hz);
  Serial.printf("Display geometry: %u x %u  (pixels/frame = %u)\n", (unsigned)cols, (unsigned)rows, (unsigned)pixels_per_frame);
  Serial.printf("Bytes/frame (pixels * %u Bpp): %u\n", (unsigned)BYTES_PER_PIXEL, (unsigned)frame_bytes);
  Serial.printf("Pixel data transfer time: %.3f ms (%.0f us)\n", pixel_transfer_us / 1000.0, pixel_transfer_us);
  Serial.printf("Even cols: %u, odd cols: %u\n", (unsigned)even_cols, (unsigned)odd_cols);
  Serial.printf("Even overhead bytes/col: %u  (frctr2=%u fporch=%u bporch=%u)\n",
                (unsigned)even_overhead_bytes_per_col,
                (unsigned)plotter._even_frctr2, (unsigned)plotter._even_fporch, (unsigned)plotter._even_bporch);
  Serial.printf("Odd  overhead bytes/col: %u  (frctr2=%u fporch=%u bporch=%u)\n",
                (unsigned)odd_overhead_bytes_per_col,
                (unsigned)plotter._odd_frctr2, (unsigned)plotter._odd_fporch, (unsigned)plotter._odd_bporch);
  Serial.printf("Total overhead bytes: %u\n", (unsigned)total_overhead_bytes);
  Serial.printf("Overhead transfer time: %.3f ms (%.0f us)\n", overhead_transfer_us / 1000.0, overhead_transfer_us);

  Serial.printf("Phase send offset: %.0f us\n", phase_us);
  Serial.printf("Mid offset:        %.0f us\n", mid_us);

  Serial.printf("Estimated total frame time: %.3f ms (%.0f us)\n", total_frame_us / 1000.0, total_frame_us);
  Serial.printf("Estimated refresh rate: %.3f Hz\n", estimated_hz);
  Serial.printf("plotter._target_rate_hz: %.3f Hz\n", target_hz);
  Serial.printf("Delta (est - target): %.3f Hz  (%.3f %% relative)\n",
                estimated_hz - target_hz,
                (target_hz > 0.0) ? ((estimated_hz - target_hz) / target_hz * 100.0) : 0.0);
  Serial.println(F("--------------------------------"));
  Serial.println(F("Notes: BYTES_PER_PIXEL and interpretation of frctr2/fporch/bporch are assumptions."));
  Serial.println(F("If your timing fields are not byte counts, adjust the helper accordingly."));
}

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
      case 9: { float r = plotter._target_rate_hz; r = max(1.0f, r - 0.01f); plotter.setFrameRate(r); break; }
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
      case 9: { float r = plotter._target_rate_hz; r = min(120.0f, r + 0.01f); plotter.setFrameRate(r); break; }
    }
    plotter.printConfig();
  }
     else if (input == 't' || input == 'T') {
     // optional: pass SPI clock in Hz if known, e.g. 40000000UL
     printFrameTimingEstimates(80000000UL);  // default 20 MHz
     return;
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
