// continuous_adc_plot_esp32.ino
// Trigger-aligned snapshotting + polite BarPlotter handoff
// ADC continuous (DMA) driver from ESP-IDF v5 + BarPlotter.
// IRremote-based control using provided Samsung key table.

#include <IRremote.h> // Arduino-IRremote (IrReceiver)
#include "BarPlotter.h"
#include <TFT_eSPI.h>

extern "C" {
  #include "esp_adc/adc_continuous.h"
  #include "driver/adc.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

#define IR_PIN 15

// ---------- USER CONFIG ----------
#define TFT_ROTATION          1
#define BAR_COUNT             320
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS 240
#define PLOT_X                0
#define PLOT_Y                0
#define PLOT_SPACING          0

// ADC continuous settings
#define ADC_SAMPLE_FREQ_HZ    21000UL
#define ADC_READ_BUF_BYTES    (8 * 1024)
#define ADC_FRAME_SIZE        128
#define ADC_READ_TIMEOUT_MS   50

// decimation: how many ADC samples to average per bar
#define DECIMATE_PER_FRAME    4

// ---------- IR KEY TABLE (user-supplied) ----------
enum KeyCode {
  KEY_POWER = 0x02,
  KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06, KEY_4 = 0x08,
  KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D, KEY_9 = 0x0E,
  KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65, KEY_RIGHT = 0x62,
  KEY_OK = 0x68, KEY_MENU = 0x79, KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b,
  KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10
};

// ---------- FrameState constants (mirror BarPlotter) ----------
#define FRAME_IDLE      0
#define FRAME_QUEUED    1
#define FRAME_CONSUMING 2
#define FRAME_SENT      3

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// ADC handle
static adc_continuous_handle_t adc_handle = NULL;
static uint8_t *adc_read_buffer = nullptr;
static volatile bool adc_running = false;

// Plot generation frequency (Hz) - TFT/frame sync ONLY.
static volatile float plot_rate_hz = 23.62f;

// Stats
static volatile uint32_t dropped_frames = 0;
static volatile uint32_t generated_frames = 0;

// Triggering / copy-management
static volatile bool trigger_enabled = false;
static volatile int trigger_slope = 1; // 1 = rising, -1 = falling
static volatile uint16_t trigger_value = 128; // 0..4095 ADC raw
static volatile uint32_t trigger_hold_ms = 50;
static volatile uint32_t last_trigger_time_ms = 0;

static volatile float triggered_copy_freq_hz = 30.0f;
static inline uint32_t triggered_copy_interval_ms() {
  float f = triggered_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(1000.0f / f + 0.5f);
}

static volatile float fallback_copy_freq_hz = 5.0f;
static inline uint32_t fallback_copy_interval_ms() {
  float f = fallback_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(1000.0f / f + 0.5f);
}
static inline uint32_t fallback_unreliable_threshold_ms() {
  float f = fallback_copy_freq_hz;
  if (f <= 0.0f) f = 1.0f;
  return (uint32_t)(2.0f * (1000.0f / f) + 0.5f);
}

static volatile uint32_t last_immediate_copy_ms = 0;
static volatile uint32_t minimum_immediate_copy_interval_ms = 20;

static uint16_t *pending_snapshot = nullptr;
static volatile bool pending_snapshot_valid = false;

// Trigger tuning additions
static volatile uint16_t trigger_hysteresis = 8;        // LSBs
static volatile uint32_t min_trigger_separation_ms = 50; // ms between accepted crossings

// Trigger alignment configuration
// This is the bar index at which we want the triggering sample to appear.
// Default is the center bar so trigger appears visually centered.
static const size_t trigger_align_bar_index = BAR_COUNT / 2;

// store last detected trigger ring index for repeat/fallback alignment
static volatile size_t last_trigger_ring_index = SIZE_MAX; // invalid initial

// ---------- ADC setup ----------
void setup_adc_continuous()
{
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  adc_continuous_handle_cfg_t handle_cfg = {
      .max_store_buf_size = ADC_READ_BUF_BYTES,
      .conv_frame_size = ADC_FRAME_SIZE,
      .flags = {0}
  };

  esp_err_t r = adc_continuous_new_handle(&handle_cfg, &adc_handle);
  if (r != ESP_OK) {
    Serial.print("adc_continuous_new_handle failed: ");
    Serial.println(r);
    return;
  }

  static adc_digi_pattern_config_t pattern[1];
  pattern[0].atten = ADC_ATTEN_DB_11;
  pattern[0].channel = ADC_CHANNEL_0;
  pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dig_cfg = {
      .pattern_num = 1,
      .adc_pattern = pattern,
      .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
  };

  r = adc_continuous_config(adc_handle, &dig_cfg);
  if (r != ESP_OK) {
    Serial.print("adc_continuous_config failed: ");
    Serial.println(r);
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  adc_read_buffer = (uint8_t*)malloc(ADC_READ_BUF_BYTES);
  if (!adc_read_buffer) {
    Serial.println("adc_read_buffer malloc failed");
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  r = adc_continuous_start(adc_handle);
  if (r != ESP_OK) {
    Serial.print("adc_continuous_start failed: ");
    Serial.println(r);
    free(adc_read_buffer);
    adc_read_buffer = nullptr;
    adc_continuous_deinit(adc_handle);
    adc_handle = NULL;
    return;
  }

  adc_running = true;
  Serial.println("ADC continuous started");
}

// ---------- Snapshot production helpers ----------

// produce snapshot (existing behavior) -- decimate using ring_pos as "end"
static void produce_snapshot_from_ring(uint16_t *ring, size_t ringSize, size_t ring_pos, uint16_t *out) {
  size_t samples_needed = (size_t)BAR_COUNT * DECIMATE_PER_FRAME;
  ssize_t start_idx = (ssize_t)ring_pos - (ssize_t)samples_needed;
  if (start_idx < 0) start_idx += ringSize;

  for (size_t b = 0; b < BAR_COUNT; ++b) {
    uint32_t acc = 0;
    for (size_t d = 0; d < DECIMATE_PER_FRAME; ++d) {
      size_t idx = (start_idx + b * DECIMATE_PER_FRAME + d) % ringSize;
      acc += ring[idx];
    }
    uint16_t avg = (uint16_t)(acc / DECIMATE_PER_FRAME);
    uint32_t height = ((uint32_t)avg * BAR_MAX_HEIGHT_PIXELS) / 4095u;
    if (height > BAR_MAX_HEIGHT_PIXELS) height = BAR_MAX_HEIGHT_PIXELS;
    out[b] = (uint16_t)height;
  }
}

// produce snapshot aligned to a trigger sample index (so trigger sample maps to 'trigger_bar_index').
// If the trigger index is too old (overwritten) this falls back to the latest snapshot mode.
static void produce_snapshot_from_ring_triggered(uint16_t *ring, size_t ringSize, size_t ring_pos,
                                                 uint16_t *out, size_t trigger_ring_idx, size_t trigger_bar_index)
{
  const size_t samples_needed = (size_t)BAR_COUNT * DECIMATE_PER_FRAME;

  // Quick sanity: ringSize must be > samples_needed
  if (ringSize <= samples_needed || trigger_ring_idx == SIZE_MAX) {
    // fallback to normal behaviour
    produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
    return;
  }

  // Compute how many samples have been written since trigger (age)
  size_t age = (ring_pos + ringSize - trigger_ring_idx) % ringSize;

  // If trigger sample has been overwritten (i.e., age is greater than ringSize - samples_needed),
  // we cannot reliably produce a window centered on the original trigger; fall back to normal.
  if (age > (ringSize - samples_needed)) {
    produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
    return;
  }

  // Compute starting index such that trigger_ring_idx becomes the center-sample of bar 'trigger_bar_index'.
  // We want the start index of the snapshot's raw-sample window:
  //   start_idx = trigger_ring_idx - (trigger_bar_index * DECIMATE_PER_FRAME)
  // so that bar 0 aggregates samples [start_idx .. start_idx+DECIMATE_PER_FRAME-1],
  // and bar trigger_bar_index's DECIMATE window will include trigger_ring_idx within its window.
  ssize_t start_idx = (ssize_t)trigger_ring_idx - (ssize_t)(trigger_bar_index * DECIMATE_PER_FRAME);

  // Normalize start_idx into [0..ringSize-1]
  start_idx %= (ssize_t)ringSize;
  if (start_idx < 0) start_idx += ringSize;

  for (size_t b = 0; b < BAR_COUNT; ++b) {
    uint32_t acc = 0;
    for (size_t d = 0; d < DECIMATE_PER_FRAME; ++d) {
      size_t idx = (start_idx + b * DECIMATE_PER_FRAME + d) % ringSize;
      acc += ring[idx];
    }
    uint16_t avg = (uint16_t)(acc / DECIMATE_PER_FRAME);
    uint32_t height = ((uint32_t)avg * BAR_MAX_HEIGHT_PIXELS) / 4095u;
    if (height > BAR_MAX_HEIGHT_PIXELS) height = BAR_MAX_HEIGHT_PIXELS;
    out[b] = (uint16_t)height;
  }
}

// If a pending snapshot exists, attempt to push it and clear on success.
static void try_flush_pending() {
  if (pending_snapshot_valid && pending_snapshot) {
    if (plotter.startBarPlot(pending_snapshot, BAR_COUNT)) {
      generated_frames++;
      pending_snapshot_valid = false;
    } else {
      // still busy; keep pending, but count the failed attempt for stats
      dropped_frames++;
    }
  }
}

// Polite, frame-aware attempt to queue (or stash) a snapshot.
// Returns true if accepted by plotter or saved to pending; false if dropped.
static bool attempt_queue_snapshot(uint16_t *snapshot) {
  // try to push an older pending first
  try_flush_pending();

  int fs = plotter.getFrameState();
  bool queueEmpty = plotter.isQueueEmpty();

  // If plotter is consuming a frame, prefer to stash to pending (if empty)
  if (fs == FRAME_CONSUMING) {
    if (!pending_snapshot_valid && pending_snapshot) {
      memcpy(pending_snapshot, snapshot, sizeof(uint16_t) * BAR_COUNT);
      pending_snapshot_valid = true;
      return true;
    } else {
      dropped_frames++;
      return false;
    }
  }

  // If idle or recently sent a frame -> attempt to queue (higher chance of success)
  if ((fs == FRAME_IDLE) || (fs == FRAME_SENT) || queueEmpty) {
    if (plotter.startBarPlot(snapshot, BAR_COUNT)) {
      generated_frames++;
      return true;
    } else {
      // failed to copy into plotter; try pending
      if (!pending_snapshot_valid && pending_snapshot) {
        memcpy(pending_snapshot, snapshot, sizeof(uint16_t) * BAR_COUNT);
        pending_snapshot_valid = true;
        return true;
      } else {
        dropped_frames++;
        return false;
      }
    }
  }

  // final fallback: save to pending if possible
  if (!pending_snapshot_valid && pending_snapshot) {
    memcpy(pending_snapshot, snapshot, sizeof(uint16_t) * BAR_COUNT);
    pending_snapshot_valid = true;
    return true;
  }

  dropped_frames++;
  return false;
}

// ---------- ADC reader task (producer) ----------
void adc_reader_task(void *pv)
{
  const size_t ringSize = BAR_COUNT * DECIMATE_PER_FRAME * 4; // 4x headroom
  uint16_t *ring = (uint16_t*)malloc(sizeof(uint16_t) * ringSize);
  if (!ring) {
    Serial.println("ring alloc failed");
    vTaskDelete(NULL);
    return;
  }
  size_t ring_pos = 0;

  uint16_t *out = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  if (!out) {
    Serial.println("out alloc failed");
    free(ring);
    vTaskDelete(NULL);
    return;
  }

  // allocate pending snapshot buffer
  pending_snapshot = (uint16_t*)malloc(sizeof(uint16_t) * BAR_COUNT);
  pending_snapshot_valid = false;

  // Periodic plotting tick uses plot_rate_hz
  TickType_t lastPlotTick = xTaskGetTickCount();
  uint32_t local_period_ms = (uint32_t)(1000.0f / plot_rate_hz + 0.5f);
  TickType_t period_ticks = pdMS_TO_TICKS(local_period_ms);

  // For trigger detection: keep previous sample
  uint16_t prev_sample = 0;

  // track enable state locally to prime prev_sample on enable transition
  bool last_trigger_enabled_local = false;

  // guard rapid-fire trigger events
  uint32_t last_trigger_event_ms = 0;

  // For triggered repeat-copy scheduling:
  uint32_t last_triggered_copy_ms_local = 0;
  // For fallback repeat-copy scheduling:
  uint32_t last_fallback_copy_ms_local = 0;

  while (1) {
    if (!adc_running) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Give priority to pending
    try_flush_pending();

    // Detect trigger enable/disable transitions and prime prev_sample
    if (trigger_enabled != last_trigger_enabled_local) {
      last_trigger_enabled_local = trigger_enabled;
      if (last_trigger_enabled_local) {
        // On enable: take the most recent sample already in ring as prev_sample
        size_t last_idx = (ring_pos == 0) ? (ringSize - 1) : (ring_pos - 1);
        prev_sample = ring[last_idx];
        // avoid immediate fallback snaps until some time passes
        last_trigger_time_ms = millis();
        last_immediate_copy_ms = millis();
        // clear stale pending
        pending_snapshot_valid = false;
        last_trigger_event_ms = 0;
      }
    }

    // Read ADC bytes
    uint32_t out_len = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, adc_read_buffer, ADC_READ_BUF_BYTES, &out_len, ADC_READ_TIMEOUT_MS);
    if (ret == ESP_ERR_TIMEOUT) {
      // no data this iteration
    } else if (ret == ESP_OK && out_len > 0) {
      size_t item_size = sizeof(adc_digi_output_data_t);
      size_t count = out_len / item_size;
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)adc_read_buffer;

      for (size_t i = 0; i < count; ++i) {
        uint32_t raw = p[i].type1.data & 0xFFF; // 12-bit
        // write sample into ring and advance ring_pos
        ring[ring_pos] = (uint16_t)raw;
        // index of this sample (useful if a crossing occurs now)
        size_t this_sample_index = ring_pos;
        ring_pos++;
        if (ring_pos >= ringSize) ring_pos = 0;

        // Trigger detection logic (sample-level) with hysteresis + min separation
        if (trigger_enabled) {
          uint16_t s = (uint16_t)raw;
          bool crossed = false;

          uint32_t low_th = (trigger_value > trigger_hysteresis) ? (uint32_t)trigger_value - trigger_hysteresis : 0u;
          uint32_t high_th = (uint32_t)trigger_value + trigger_hysteresis;
          if (high_th > 4095u) high_th = 4095u;

          if (trigger_slope > 0) {
            if ((uint32_t)prev_sample < low_th && (uint32_t)s >= high_th) crossed = true;
          } else {
            if ((uint32_t)prev_sample > high_th && (uint32_t)s <= low_th) crossed = true;
          }

          if (crossed) {
            uint32_t nowms = millis();
            if ((nowms - last_trigger_event_ms) >= min_trigger_separation_ms) {
              last_trigger_time_ms = nowms;
              last_trigger_event_ms = nowms;

              // record the ring index where crossing happened
              last_trigger_ring_index = this_sample_index;

              // Attempt an immediate snapshot copy aligned to the trigger index (respecting minimum interval)
              if ((nowms - last_immediate_copy_ms) >= minimum_immediate_copy_interval_ms) {
                produce_snapshot_from_ring_triggered(ring, ringSize, ring_pos, out, last_trigger_ring_index, trigger_align_bar_index);
                attempt_queue_snapshot(out);
                last_immediate_copy_ms = nowms;
                last_triggered_copy_ms_local = nowms;
              }
            } // else suppressed
          } // end crossed
          prev_sample = s;
        } else {
          prev_sample = (uint16_t)raw;
        }
      }
    } else if (ret != ESP_OK) {
      Serial.print("adc_continuous_read error: ");
      Serial.println(ret);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // recalc period_ticks if plot_rate_hz changed externally
    {
      float hz = plot_rate_hz;
      uint32_t msec = (uint32_t)(1000.0f / hz + 0.5f);
      if (msec == 0) msec = 1;
      TickType_t new_period = pdMS_TO_TICKS(msec);
      if (new_period != period_ticks) {
        period_ticks = new_period;
      }
    }

    // Triggered-repeat copying while inside hold_ms: align to last_trigger_ring_index if available
    if (trigger_enabled && (millis() - last_trigger_time_ms) <= trigger_hold_ms) {
      uint32_t nowms = millis();
      uint32_t trig_interval = triggered_copy_interval_ms();
      if ((nowms - last_triggered_copy_ms_local) >= trig_interval) {
        if (last_trigger_ring_index != SIZE_MAX) {
          produce_snapshot_from_ring_triggered(ring, ringSize, ring_pos, out, last_trigger_ring_index, trigger_align_bar_index);
        } else {
          produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
        }
        attempt_queue_snapshot(out);
        last_triggered_copy_ms_local = nowms;
      }
    }

    // Fallback-unreliable mode: when no crossing for > threshold, produce aligned snapshots using last_trigger index if possible
    if (trigger_enabled) {
      uint32_t nowms = millis();
      uint32_t threshold_ms = fallback_unreliable_threshold_ms();
      if ((nowms - last_trigger_time_ms) > threshold_ms) {
        uint32_t fb_interval = fallback_copy_interval_ms();
        if ((nowms - last_fallback_copy_ms_local) >= fb_interval) {
          if ((nowms - last_immediate_copy_ms) >= minimum_immediate_copy_interval_ms) {
            if (last_trigger_ring_index != SIZE_MAX) {
              produce_snapshot_from_ring_triggered(ring, ringSize, ring_pos, out, last_trigger_ring_index, trigger_align_bar_index);
            } else {
              produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
            }
            attempt_queue_snapshot(out);
            last_immediate_copy_ms = nowms;
            last_fallback_copy_ms_local = nowms;
            Serial.printf("Fallback copy (no crossing for %u ms, threshold %u ms)\n",
                          (unsigned)(nowms - last_trigger_time_ms), (unsigned)threshold_ms);
          } else {
            last_fallback_copy_ms_local = nowms;
          }
        }
      }
    }

    // Periodic plotting synchronized with TFT frame
    TickType_t now = xTaskGetTickCount();
    if ((now - lastPlotTick) >= period_ticks) {
      TickType_t delta = now - lastPlotTick;
      TickType_t steps = delta / period_ticks;
      lastPlotTick += steps * period_ticks;

      // If we have a recent trigger index (recent enough), prefer aligned snapshot; otherwise normal
      uint32_t nowms = millis();
      uint16_t ms_since_trigger = (last_trigger_ring_index == SIZE_MAX) ? 0xFFFF : (uint16_t)(nowms - last_trigger_time_ms);
      // Choose aligned snapshot if trigger within last 2 * (1000/plot_rate_hz) ms (heuristic)
      if (last_trigger_ring_index != SIZE_MAX && ms_since_trigger < (uint16_t)(2.0f * (1000.0f / plot_rate_hz))) {
        produce_snapshot_from_ring_triggered(ring, ringSize, ring_pos, out, last_trigger_ring_index, trigger_align_bar_index);
      } else {
        produce_snapshot_from_ring(ring, ringSize, ring_pos, out);
      }
      attempt_queue_snapshot(out);
    }

    taskYIELD();
  }

  // never reached
  free(out);
  free(ring);
  if (pending_snapshot) free(pending_snapshot);
  vTaskDelete(NULL);
}

// ---------- main/setup/loop ----------
void setPlotRateHz(float hz) {
  if (hz < 1.0f) hz = 1.0f;
  if (hz > 120.0f) hz = 120.0f;
  plot_rate_hz = hz;
  plotter.setFrameRate(hz);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  tft.init();
  tft.initDMA();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(0x0000);

  plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);
  setPlotRateHz(plot_rate_hz);

  setup_adc_continuous();

  xTaskCreatePinnedToCore(adc_reader_task, "adc_reader", 8192, NULL, 2, NULL, 1);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  Serial.printf("IR receiver started on pin %d\n", IR_PIN);
}

// Serial tuning helpers (same semantics as before)
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
    Serial.printf("Target plot generation rate (TFT sync): %.2f Hz\n", plot_rate_hz);
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
      case 9: { float r = plot_rate_hz; r = max(1.0f, r - 0.01f); setPlotRateHz(r); break; }
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
      case 9: { float r = plot_rate_hz; r = min(120.0f, r + 0.01f); setPlotRateHz(r); break; }
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

    switch (key) {
      case KEY_POWER:
        trigger_enabled = !trigger_enabled;
        Serial.printf("Trigger %s\n", trigger_enabled ? "ENABLED" : "DISABLED");
        if (!trigger_enabled) {
          last_trigger_time_ms = 0;
          last_trigger_ring_index = SIZE_MAX;
        } else {
          last_immediate_copy_ms = millis();
          last_trigger_time_ms = millis();
        }
        break;

      case KEY_UP:
        trigger_slope = 1;
        Serial.println("Trigger slope: RISING");
        break;
      case KEY_DOWN:
        trigger_slope = -1;
        Serial.println("Trigger slope: FALLING");
        break;

      case KEY_LEFT: {
        uint16_t dec = 64;
        uint32_t nv = (uint32_t)trigger_value;
        if (nv > dec) nv -= dec; else nv = 0;
        trigger_value = (uint16_t)nv;
        Serial.printf("Trigger value decreased -> %u\n", trigger_value);
        break;
      }
      case KEY_RIGHT: {
        uint16_t inc = 64;
        uint32_t nv = (uint32_t)trigger_value + inc;
        if (nv > 4095) nv = 4095;
        trigger_value = (uint16_t)nv;
        Serial.printf("Trigger value increased -> %u\n", trigger_value);
        break;
      }

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
        Serial.printf("Fallback copy freq -> %.2f Hz (thresh %u ms)\n", fallback_copy_freq_hz, (unsigned)fallback_unreliable_threshold_ms());
        break;
      }
      case KEY_CH_DOWN: {
        fallback_copy_freq_hz = max(0.1f, fallback_copy_freq_hz - 1.0f);
        Serial.printf("Fallback copy freq -> %.2f Hz (thresh %u ms)\n", fallback_copy_freq_hz, (unsigned)fallback_unreliable_threshold_ms());
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

      // quick presets 0..9
      case KEY_0: case KEY_1: case KEY_2: case KEY_3: case KEY_4:
      case KEY_5: case KEY_6: case KEY_7: case KEY_8: case KEY_9: {
        uint8_t digit = 0;
        if (key == KEY_0) digit = 0; else if (key == KEY_1) digit = 1;
        else if (key == KEY_2) digit = 2; else if (key == KEY_3) digit = 3;
        else if (key == KEY_4) digit = 4; else if (key == KEY_5) digit = 5;
        else if (key == KEY_6) digit = 6; else if (key == KEY_7) digit = 7;
        else if (key == KEY_8) digit = 8; else if (key == KEY_9) digit = 9;
        trigger_value = (uint16_t)(((uint32_t)digit * 4095) / 9);
        Serial.printf("Trigger quick-preset -> digit=%u value=%u\n", digit, trigger_value);
        break;
      }

      default:
        Serial.printf("Unmapped key 0x%02X\n", key);
        break;
    }
  }

  IrReceiver.resume();
}

void loop() {
  serial_plotter_helper();
  handle_ir();

  static uint32_t last_print = 0;
  if (millis() - last_print >= 1000) {
    last_print = millis();
    Serial.printf("Plot (TFT) rate: %.2f Hz | generated: %u | dropped: %u | trigger=%s | val=%u | fb_thresh_ms=%u | pending=%d\n",
                  plot_rate_hz, (unsigned)generated_frames, (unsigned)dropped_frames,
                  trigger_enabled ? "ON" : "OFF", trigger_value, (unsigned)fallback_unreliable_threshold_ms(), pending_snapshot_valid ? 1 : 0);
  }
  yield();
}
