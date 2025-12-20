#include "BarPlotter.h"
#include "esp_heap_caps.h" // MALLOC_CAP_DMA
#include "esp_timer.h"     // esp_timer_get_time()
#include <string.h>        // memcpy

const uint8_t BarPlotter::_pattern[BarPlotter::_patternH][BarPlotter::_patternW] = {
    {1,1,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}
};

// -------------------- New timing/PLL helper defaults --------------------
static const float DEFAULT_RATE_HZ = 24.3f;    // default host frame target
static const uint32_t DEFAULT_PHASE_OFFSET_US = 0; // send timing command 2ms before expected frame start (tweak)
static const uint8_t DEFAULT_PORCH_STEP = 1;   // step size when nudging porches

// Frame state enum
enum FrameState {
    FRAME_IDLE = 0,
    FRAME_QUEUED,
    FRAME_CONSUMING,
    FRAME_SENT
};

BarPlotter::BarPlotter(TFT_eSPI& tft) :
    _tft(tft),
    _barCount(0),
    _barWidth(0),
    _barMaxHeight(0),
    _plotX(0),
    _plotY(0),
    _plotSpacing(0),
    _plotInProgress(false),
    _plotQueueInProgress(false),
    _plotDataArray(nullptr),
    _plotDataCount(0),
    _currentBarIndex(0),
    _plotTaskHandle(NULL),
    _barSourceCopies(nullptr),
    _barSourcePixelsSingle(0),
    _barSourceCopiesCount(0),
    _barSourceRowsSingle(0),
    _rowWidth(0),
    _dmaBarBuffer(nullptr),
    _dmaBarBufferPixels(0),
    // newly added defaults:
    _frame_us((uint64_t)(1000000.0f / DEFAULT_RATE_HZ)),
    _phase_send_offset(DEFAULT_PHASE_OFFSET_US),
    _mid_offset((uint32_t)((uint64_t)(1000000.0f / DEFAULT_RATE_HZ) / 2)),
    _porch_step(DEFAULT_PORCH_STEP),
    _target_rate_hz(DEFAULT_RATE_HZ),
    _even_frctr2(0x1f), // for 320x240 st7789
    _even_fporch(0x38),
    _even_bporch(0x36),
    _odd_frctr2(0x1a),
    _odd_fporch(0x40),
    _odd_bporch(0x3f),
    _debug_mode(false),
    _debug_fill_state(false),
    // new members
    _frameBufA(nullptr),
    _frameBufB(nullptr),
    _activeBuf(nullptr),
    _queuedBuf(nullptr),
    _hasQueuedFrame(false),
    _frameMutex(NULL),
    _frameState(FRAME_IDLE),
    _taskRunning(false)
{
    // existing color defaults...
    _colorFill = 0x07E0;   // green
    _colorExtra = 0x0700;   // for debug (spacing clear)
    _colorFg = 0xFFFF;   // white (background of bar area)
    _screenBg = 0x0000;   // black screen background
    _gridColor = 0x5502; // color used for the pattern (grid)
}

BarPlotter::~BarPlotter() {
    if (_barSourceCopies) {
        heap_caps_free(_barSourceCopies);
    }
    if (_dmaBarBuffer) {
        heap_caps_free(_dmaBarBuffer);
    }
    if (_frameBufA) heap_caps_free(_frameBufA);
    if (_frameBufB) heap_caps_free(_frameBufB);
    if (_frameMutex) vSemaphoreDelete(_frameMutex);
    if (_plotTaskHandle) {
        vTaskDelete(_plotTaskHandle);
    }
}

// -------------------- setters --------------------
void BarPlotter::setFrameRate(float hz) {
    if (hz <= 0.1f) return;
    _target_rate_hz = hz;
    _frame_us = (uint64_t)(1000000.0f / hz);
    _mid_offset = (uint32_t)(_frame_us / 2);
}

// --- getters for external synchronization
int BarPlotter::getFrameState() const {
    return _frameState;
}
const char* BarPlotter::frameStateString() const {
    switch(_frameState) {
        case FRAME_IDLE: return "IDLE";
        case FRAME_QUEUED: return "QUEUED";
        case FRAME_CONSUMING: return "CONSUMING";
        case FRAME_SENT: return "SENT";
        default: return "UNKNOWN";
    }
}

// --- other setters/getters implementations
void BarPlotter::setEvenTiming(uint8_t frctr2, uint8_t fporch, uint8_t bporch) {
    _even_frctr2 = frctr2;
    _even_fporch = fporch;
    _even_bporch = bporch;
}
void BarPlotter::setOddTiming(uint8_t frctr2, uint8_t fporch, uint8_t bporch) {
    _odd_frctr2 = frctr2;
    _odd_fporch = fporch;
    _odd_bporch = bporch;
}
void BarPlotter::setPhaseOffsetUs(uint32_t us) {
    _phase_send_offset = us;
}
void BarPlotter::setMidOffsetUs(uint32_t us) {
    _mid_offset = us;
}

void BarPlotter::setPorchStep(uint8_t step) {
    _porch_step = step ? step : 1;
}
void BarPlotter::enableDebug(bool en) {
    _debug_mode = en;
    if (!en) _debug_fill_state = false;
}
void BarPlotter::printConfig() {
    Serial.println("=== BarPlotter config ===");
    Serial.printf("even_frctr2=0x%02X, even_fporch=0x%02X, even_bporch=0x%02X\n", _even_frctr2, _even_fporch, _even_bporch);
    Serial.printf("odd_frctr2=0x%02X, odd_fporch=0x%02X, odd_bporch=0x%02X\n", _odd_frctr2, _odd_fporch, _odd_bporch);
    Serial.printf("frame_rate=%.2f Hz, frame_us=%llu us\n", _target_rate_hz, (unsigned long long)_frame_us);
    Serial.printf("phase_send_offset=%u us, mid_offset=%u us\n", _phase_send_offset, _mid_offset);
    Serial.printf("porch_step=%u, debug=%u\n", _porch_step, (int)_debug_mode);
    Serial.printf("frameState=%s\n", frameStateString());
    Serial.println("=========================");
}


bool BarPlotter::begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing) {
    _barCount = barCount;
    _barWidth = barWidth;
    _barMaxHeight = barMaxHeight;
    _plotX = plotX;
    _plotY = plotY;
    _plotSpacing = plotSpacing;

    if (createTallBarSourceCopiesWithPattern()) {
        Serial.printf("Using tall precomputed barSource copies with pattern (%u copies)\n", (unsigned)_barSourceCopiesCount);
    } else {
        Serial.println("Tall precompute copies with pattern failed; attempting per-bar DMA buffer");
        if (createPerBarDmaBuffer()) {
            Serial.println("Using per-bar DMA buffer fallback");
        } else {
            Serial.println("No DMA buffers available; will use fillRect fallback");
        }
    }

    // allocate ping-pong frame buffers (for heights only)
    size_t frameBytes = sizeof(uint16_t) * (size_t)_barCount;
    _frameBufA = (uint16_t*) heap_caps_malloc(frameBytes, MALLOC_CAP_8BIT);
    _frameBufB = (uint16_t*) heap_caps_malloc(frameBytes, MALLOC_CAP_8BIT);
    if (!_frameBufA || !_frameBufB) {
        Serial.println("Frame buffer allocation failed");
        if (_frameBufA) heap_caps_free(_frameBufA);
        if (_frameBufB) heap_caps_free(_frameBufB);
        _frameBufA = _frameBufB = nullptr;
        // still return true; fallbacks might still allow plotting with direct reads
    } else {
        // init
        for (size_t i=0;i<(size_t)_barCount;++i) { _frameBufA[i]=0; _frameBufB[i]=0; }
        _activeBuf = _frameBufA;
        _queuedBuf = _frameBufB;
    }

    // create frame mutex
    _frameMutex = xSemaphoreCreateMutex();
    if (!_frameMutex) {
        Serial.println("Frame mutex creation failed");
    }

    // create persistent plot task (runs continuously)
    if (_plotTaskHandle == NULL) {
        const uint32_t stackSize = 8192;
        BaseType_t ok = xTaskCreatePinnedToCore(plotTask, "plotTask", stackSize, this, 5, &_plotTaskHandle, 0);
        if (ok != pdPASS) {
            Serial.println("Failed to create plotTask");
            _plotTaskHandle = NULL;
            return false;
        } else {
            _taskRunning = true;
        }
    }

    return true;
}

// -------------------- startBarPlot (queue snapshot; persistent task consumes) --------------------
// returns true if queued (or accepted); false if the call failed (dropped)
bool BarPlotter::startBarPlot(const uint16_t* dataArray, size_t count) {
    if (!dataArray || count == 0 || count != (size_t)_barCount) {
        Serial.println("Invalid data input to startBarPlot");
        return false;
    }

    // If internal ping-pong buffers exist, copy into queued buffer under mutex.
    const uint32_t default_max_wait_ms = 5; // small bounded wait for producer
    bool copied = false;

    if (_frameBufA && _frameBufB && _frameMutex) {
        // try to acquire quickly; block for a short bounded time to reduce frame drops
        if (xSemaphoreTake(_frameMutex, pdMS_TO_TICKS(default_max_wait_ms)) == pdTRUE) {
            // copy into queuedBuf
            memcpy(_queuedBuf, dataArray, sizeof(uint16_t) * (size_t)_barCount);
            _hasQueuedFrame = true;
            _frameState = FRAME_QUEUED;
            xSemaphoreGive(_frameMutex);
            copied = true;
        } else {
            // couldn't acquire in time -> drop
            copied = false;
        }
    } else {
        // No internal buffers: fallback to original pointer-swap behavior (dangerous but compat)
        // We will simply set the pointer and let plot task access it. This is less safe; prefer buffers.
        _plotDataArray = dataArray;
        _plotDataCount = count;
        _plotInProgress = true;
        _plotQueueInProgress = true;
        _frameState = FRAME_QUEUED;
        copied = true;
    }

    if (!copied) {
        // drop & signal
        return false;
    }

    return true;
}


bool BarPlotter::isPlotting() const {
    // "plotting" means the plot task is actively consuming a frame
    return (_frameState == FRAME_CONSUMING);
}

bool BarPlotter::isQueueEmpty() const {
    // queue empty if no queued frame pending
    return !_hasQueuedFrame;
}

void BarPlotter::endWrite() {
    _tft.endWrite();
}


// -------------------- Helper: write the timing registers --------------------
// This writes the FRCTR2 and PORCTRL settings for a given 3-byte parameter set.
// We expect the caller to call within startWrite()/endWrite() context or we do it ourselves.
static inline void write_timing_registers(TFT_eSPI &tft, uint8_t frctr2_val, uint8_t por_fporch, uint8_t por_bporch) {
    tft.writecommand(ST7789_FRCTR2);
    tft.writedata(frctr2_val);
    tft.writecommand(ST7789_PORCTRL);
    tft.writedata(por_fporch);
    tft.writedata(por_bporch);
    tft.writedata(0x01); // enable separate porch control
    tft.writedata(0x33);
    tft.writedata(0x33);
}


bool BarPlotter::createTallBarSourceCopiesWithPattern() {
    const size_t copies = (size_t)_patternH;
    if (copies == 0) return false;

    const size_t w = (size_t)_barWidth;
    const size_t h = (size_t)_barMaxHeight;
    const size_t rowsPerCopy = 2 * h;
    _rowWidth = w;
    _barSourceRowsSingle = rowsPerCopy;

    uint64_t pixelsSingle = (uint64_t)_rowWidth * (uint64_t)rowsPerCopy;
    if (pixelsSingle == 0 || pixelsSingle > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute copies: size check failed");
        return false;
    }
    _barSourcePixelsSingle = (size_t)pixelsSingle;

    uint64_t totalPixels = (uint64_t)_barSourcePixelsSingle * (uint64_t)copies;
    if (totalPixels == 0 || totalPixels > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute copies: total size check failed");
        return false;
    }

    size_t totalBytes = (size_t) totalPixels * sizeof(uint16_t);
    Serial.printf("Precompute (tall copies with pattern, for static pattern): allocating %u bytes for %u pixels (w=%u, rowsPerCopy=%u, copies=%u)\n",
                  (unsigned)totalBytes, (unsigned)totalPixels, (unsigned)w, (unsigned)rowsPerCopy, (unsigned)copies);

    _barSourceCopies = (uint16_t*) heap_caps_malloc(totalBytes, MALLOC_CAP_DMA);
    if (!_barSourceCopies) {
        Serial.println("Precompute copies: allocation failed (no DMA-capable memory)");
        return false;
    }

    for (size_t k = 0; k < copies; ++k) {
        uint16_t* basePtr = _barSourceCopies + (k * _barSourcePixelsSingle);

        for (size_t r = 0; r < rowsPerCopy; ++r) {
            uint16_t baseColor = (r < h) ? _colorFg : _colorFill;
            size_t base = r * _rowWidth;
            size_t patternRow = (r + k) % _patternH;

            for (size_t c = 0; c < _rowWidth; ++c) {
                size_t patternCol = c % _patternW;
                uint16_t color = baseColor;
                if (_pattern[patternRow][patternCol]) {
                    color = _gridColor;
                }
                basePtr[base + c] = color;
            }
        }
    }

    _barSourceCopiesCount = copies;
    return true;
}

bool BarPlotter::createPerBarDmaBuffer() {
    const size_t w = (size_t)_barWidth;
    const size_t h = (size_t)_barMaxHeight;
    _dmaBarBufferPixels = w * h;
    size_t bytes = _dmaBarBufferPixels * sizeof(uint16_t);
    _dmaBarBuffer = (uint16_t*) heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!_dmaBarBuffer) {
        Serial.println("Fallback DMA: allocation failed");
        return false;
    }
    for (size_t i = 0; i < _dmaBarBufferPixels; ++i) _dmaBarBuffer[i] = _colorFg;
    return true;
}

void BarPlotter::prepareDmaBarBufferForValue(uint16_t v) {
    if (!_dmaBarBuffer) return;
    const size_t w = (size_t)_barWidth;
    const size_t h = (size_t)_barMaxHeight;
    if (v > (uint16_t)h) v = (uint16_t)h;
    for (size_t r = 0; r < h; ++r) {
        bool fillRow = (r >= (h - v));
        uint16_t color = fillRow ? _colorFill : _colorFg;
        size_t base = r * w;
        for (size_t c = 0; c < w; ++c) _dmaBarBuffer[base + c] = color;
    }
}

uint16_t* BarPlotter::windowPtrForValueAndCopy(uint16_t v, size_t copyIndex) {
    if (!_barSourceCopies) return nullptr;
    if (copyIndex >= _barSourceCopiesCount) copyIndex = 0;
    if (v > (uint16_t)_barMaxHeight) v = (uint16_t)_barMaxHeight;
    return _barSourceCopies + (copyIndex * _barSourcePixelsSingle) + ((size_t)v * _rowWidth);
}

// Helper: calibrate transfer time for one worst-case bar (full height).
// Returns microseconds per bar (uint64_t).
uint64_t BarPlotter::calibrate_bar_transfer_time(BarPlotter *self) {
//uint64_t calibrate_bar_transfer_time(BarPlotter *self) {

    const int w = self->_barWidth;
    const int h = self->_barMaxHeight;
    if (w == 0 || h == 0) return 2000; // fallback safe guess (2 ms)

    uint16_t *src = nullptr;
    uint16_t tmp_color = 0xFFFF;
    // prefer precomputed full-height window if available
    if (self->_barSourceCopies && self->_barSourceCopiesCount > 0) {
        src = self->windowPtrForValueAndCopy((uint16_t)h, 0);
    } else if (self->_dmaBarBuffer) {
        // ensure dma buffer contains full-height bar
        self->prepareDmaBarBufferForValue((uint16_t)h);
        src = self->_dmaBarBuffer;
    } else {
        // no DMA buffer available; create a tiny local DMA-capable buffer for calibration
        // but avoid malloc here; use a single-row fallback (estimate)
        // fall back to heuristic value
        return 2000; // 2 ms fallback
    }

    // Ensure previous DMA finished
    while (self->_tft.dmaBusy()) taskYIELD();

    // perform measured push
    uint64_t t0 = esp_timer_get_time();
    self->_tft.startWrite();
    self->_tft.pushImageDMA(self->_plotX, self->_plotY, w, h, src, nullptr);
    self->_tft.endWrite();
    // wait for DMA completion to measure its duration
    while (self->_tft.dmaBusy()) taskYIELD();
    uint64_t t1 = esp_timer_get_time();
    uint64_t elapsed = (t1 > t0) ? (t1 - t0) : 2000;
    // protect lower bound
    if (elapsed < 200) elapsed = 200; // don't assume <200us per bar
    return elapsed;
}


void BarPlotter::plotTask(void* pvParameters) {
    BarPlotter* self = (BarPlotter*)pvParameters;

    // Two timing parameter sets (tweak to suit panel)
    self->_even_frctr2 = 0x1f;// for 320x240 st7789
    self->_even_fporch = 0x38;
    self->_even_bporch = 0x36;

    self->_odd_frctr2 = 0x1a;
    self->_odd_fporch = 0x40;
    self->_odd_bporch = 0x3f;

    bool use_even_set = true;

    // Perform a single calibration to estimate per-bar DMA time (µs)
    uint64_t per_bar_us = calibrate_bar_transfer_time(self);
    // Smooth a bit / cap if needed
    if (per_bar_us < 200) per_bar_us = 200;
    // Optional: log result
    Serial.printf("BarPlotter: calibrated per-bar transfer time = %llu us\n", (unsigned long long)per_bar_us);

    uint64_t last_frame_start_us = esp_timer_get_time();

    while (1) {
        // continuous task: wait for either queued frame or for scheduled next frame slot
        // If no frames queued, we still run to keep timing stable.
        // compute schedule next frame start without depending on external trigger
        if (!self->_taskRunning && !self->_hasQueuedFrame) {
            // shouldn't happen - just sleep a bit
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // If a queued frame exists in ping-pong buffers, swap it in for consumption
        if (self->_frameBufA && self->_frameBufB && self->_frameMutex) {
            if (self->_hasQueuedFrame) {
                // fast swap under mutex
                if (xSemaphoreTake(self->_frameMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    // swap active and queued
                    uint16_t* tmp = self->_activeBuf;
                    self->_activeBuf = self->_queuedBuf;
                    self->_queuedBuf = tmp;
                    self->_hasQueuedFrame = false;
                    self->_frameState = FRAME_CONSUMING;
                    // expose active to plot code
                    self->_plotDataArray = self->_activeBuf;
                    self->_plotDataCount = (size_t)self->_barCount;
                    xSemaphoreGive(self->_frameMutex);
                }
            }
        } else {
            // fallback: if no internal buffers, plotter uses pointer provided by startBarPlot
            if (self->_plotInProgress) {
                self->_frameState = FRAME_CONSUMING;
            }
        }

        const int totalBars = (int)self->_plotDataCount;
        if (totalBars <= 0) {
            // nothing to display; yield a touch to timing
            self->_frameState = FRAME_IDLE;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // compute mid_bar_index from mid_offset and calibrated per-bar time
        size_t mid_bar_index = 0;
        if (per_bar_us > 0) {
            uint64_t idx = (uint64_t)(self->_mid_offset / per_bar_us);
            if (idx > (uint64_t)totalBars) idx = (uint64_t)totalBars;
            mid_bar_index = (size_t)idx;
        } else {
            mid_bar_index = (size_t)(totalBars / 2);
        }
        // clamp
        if (mid_bar_index > (size_t)totalBars) mid_bar_index = (size_t)totalBars;

        // schedule wake time: next_frame_deadline - phase_offset
        uint64_t now = esp_timer_get_time();
        uint64_t next_frame_deadline = last_frame_start_us + self->_frame_us;
        int64_t wake_at = (int64_t)next_frame_deadline - (int64_t)self->_phase_send_offset;
        if ((int64_t)now < wake_at) {
            int64_t delay_us = wake_at - (int64_t)now;
            if (delay_us > 2000) {
                vTaskDelay(pdMS_TO_TICKS((uint32_t)(delay_us / 1000)));
            } else {
                while ((int64_t)esp_timer_get_time() < wake_at) { taskYIELD(); }
            }
        }

        // ensure previous DMA finished before sending start-of-frame timing
        while (self->_tft.dmaBusy()) taskYIELD();

        // ----------------- Stage A: start-of-frame timing + push bars before mid index
        self->_tft.startWrite();
        if (use_even_set) {
            write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
        } else {
            write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
        }

        // determine frame contents based on debug mode
        bool debug_force_fill = false;
        if (self->_debug_mode) {
            // alternate full / empty each frame for tearing visualization
            debug_force_fill = self->_debug_fill_state;
            self->_debug_fill_state = !self->_debug_fill_state;
        }

        const int baseX = self->_plotX;
        const int baseY = self->_plotY;
        const int w = self->_barWidth;
        const int h = self->_barMaxHeight;

        size_t i = 0;
        for (; i < mid_bar_index; ++i) {
            uint16_t value;
            if (self->_debug_mode) {
                value = debug_force_fill ? (uint16_t)h : 0;
            } else {
                value = self->_plotDataArray[i];
            }
            if (value > (uint16_t)h) value = (uint16_t)h;
            int dx = baseX + (int)i * (w + self->_plotSpacing);
            int dy = baseY;

            size_t copyIndex = 0;
            if (self->_barSourceCopiesCount >= (size_t)self->_patternH) {
                size_t anchor = (size_t)(self->_plotY % self->_patternH);
                size_t v_mod = (size_t)(value % (uint16_t)self->_patternH);
                copyIndex = (anchor + self->_patternH - v_mod) % self->_patternH;
            } else if (self->_barSourceCopiesCount > 0) {
                copyIndex = i % self->_barSourceCopiesCount;
            }

            if (self->_barSourceCopies) {
                uint16_t* src = self->windowPtrForValueAndCopy(value, copyIndex);
                self->_tft.pushImageDMA(dx, dy, w, h, src, nullptr);
            } else if (self->_dmaBarBuffer) {
                self->prepareDmaBarBufferForValue(value);
                self->_tft.pushImageDMA(dx, dy, w, h, self->_dmaBarBuffer, nullptr);
            } else {
                if (value > 0) {
                    int fillY = dy + (h - (int)value);
                    self->_tft.fillRect(dx, fillY, w, (int)value, self->_colorFill);
                }
                if (value < (uint16_t)h) {
                    int clearH = h - (int)value;
                    self->_tft.fillRect(dx, dy, w, clearH, self->_colorFg);
                }
            }
        }
        self->_tft.endWrite(); // finish chunk A pushes

        // record approximate push time (we'll treat next stage relative to this)
        last_frame_start_us = esp_timer_get_time();

        // ----------------- Stage B: wait for last DMA to finish and send mid-frame timing
        if (mid_bar_index >= (size_t)totalBars) {
            // All bars already pushed in Stage A. Wait for DMA and then send mid (late).
            while (self->_tft.dmaBusy()) taskYIELD();
            self->_tft.startWrite();
            if (use_even_set) {
                write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
            } else {
                write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
            }
            self->_tft.endWrite();
            // nothing left to push
        } else {
            // Need to send mid between two chunks: wait for current DMA to finish
            while (self->_tft.dmaBusy()) taskYIELD();

            // send mid-frame timing
            self->_tft.startWrite();
            if (use_even_set) {
                write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
            } else {
                write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
            }
            self->_tft.endWrite();

            // ----------------- Stage C: push remaining bars [mid_bar_index .. totalBars-1]
            self->_tft.startWrite();
            for (size_t j = mid_bar_index; j < (size_t)totalBars; ++j) {
                uint16_t value;
                if (self->_debug_mode) {
                    value = debug_force_fill ? (uint16_t)h : 0;
                } else {
                    value = self->_plotDataArray[j];
                }
                if (value > (uint16_t)h) value = (uint16_t)h;
                int dx = baseX + (int)j * (w + self->_plotSpacing);
                int dy = baseY;

                size_t copyIndex = 0;
                if (self->_barSourceCopiesCount >= (size_t)self->_patternH) {
                    size_t anchor = (size_t)(self->_plotY % self->_patternH);
                    size_t v_mod = (size_t)(value % (uint16_t)self->_patternH);
                    copyIndex = (anchor + self->_patternH - v_mod) % self->_patternH;
                } else if (self->_barSourceCopiesCount > 0) {
                    copyIndex = j % self->_barSourceCopiesCount;
                }

                if (self->_barSourceCopies) {
                    uint16_t* src = self->windowPtrForValueAndCopy(value, copyIndex);
                    self->_tft.pushImageDMA(dx, dy, w, h, src, nullptr);
                } else if (self->_dmaBarBuffer) {
                    self->prepareDmaBarBufferForValue(value);
                    self->_tft.pushImageDMA(dx, dy, w, h, self->_dmaBarBuffer, nullptr);
                } else {
                    if (value > 0) {
                        int fillY = dy + (h - (int)value);
                        self->_tft.fillRect(dx, fillY, w, (int)value, self->_colorFill);
                    }
                    if (value < (uint16_t)h) {
                        int clearH = h - (int)value;
                        self->_tft.fillRect(dx, dy, w, clearH, self->_colorFg);
                    }
                }
            }
            self->_tft.endWrite();
        }

        // Mark frame consumed and toggle state
        self->_frameState = FRAME_SENT;
        // keep plotInProgress flags for backward compatibility
        self->_plotQueueInProgress = false;
        self->_plotInProgress = false; // single frame consumed; still task runs continuously

        use_even_set = !use_even_set;

        // small pause to let producer run (non-blocking)
        taskYIELD();
    } // end while

    vTaskDelete(NULL);
}




void BarPlotter::clearSpacingAreas() {
    if (_plotSpacing == 0) return;
    for (size_t i = 0; i < _barCount; ++i) {
        int dx = _plotX + (int)i * (_barWidth + _plotSpacing);
        int gapX = dx + _barWidth;
        if (_plotSpacing > 0) {
            _tft.fillRect(gapX, _plotY, _plotSpacing, _barMaxHeight, _screenBg);
        }
    }
}
