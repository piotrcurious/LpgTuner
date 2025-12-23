// BarPlotter.cpp  (replacement - drop in)
#include "BarPlotter.h"
#include "esp_heap_caps.h" // MALLOC_CAP_DMA
#include "esp_timer.h"     // esp_timer_get_time()
#include <string.h>
#include <math.h>
#include <stdio.h>

// define the static instance pointer declared in the header
BarPlotter* BarPlotter::_instance = nullptr;

// -------------------- default pattern --------------------
const uint8_t BarPlotter::_pattern[BarPlotter::_patternH][BarPlotter::_patternW] = {
    {1,1,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}
};

// -------------------- Open-loop scheduler defaults --------------------
static const double DEFAULT_TARGET_HZ = 16.45;
static const uint32_t DEFAULT_WRITE_LATENCY_US = 700; // conservative default
static const uint32_t DEFAULT_SAFETY_MARGIN_US = 1200; // margin to keep DMA inside vblank

// -------------------- helper: write timing registers (ST7789) --------------------
static inline void write_timing_registers(TFT_eSPI &tft, uint8_t frctr2_val, uint8_t por_fporch, uint8_t por_bporch) {
    // Use the ST7789 commands: FRCTRL2 (0xC6) and PORCTRL (0xB2)
    tft.writecommand(ST7789_FRCTR2);
    tft.writedata(frctr2_val);
    tft.writecommand(ST7789_PORCTRL);
    tft.writedata(por_fporch);
    tft.writedata(por_bporch);
    // enable separate porch control (fields used by some panels)
    tft.writedata(0x01);
    tft.writedata(0x33);
    tft.writedata(0x33);
}

// -------------------- static timer callback (small ISR) --------------------
void BarPlotter::send_timer_cb(void* arg) {
    // Wake the plot task via notification. It's safe to access _instance because this
    // is a static member of the class and _instance is the registered object.
    if (BarPlotter::_instance && BarPlotter::_instance->_plotTaskHandle) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(BarPlotter::_instance->_plotTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
    }
}

// -------------------- calibration helper --------------------
uint64_t BarPlotter::calibrate_bar_transfer_time(BarPlotter *self) {
    const int w = self->_barWidth;
    const int h = self->_barMaxHeight;
    if (w == 0 || h == 0) return 2000; // fallback safe guess (2 ms)

    uint16_t *src = nullptr;
    if (self->_barSourceCopies && self->_barSourceCopiesCount > 0) {
        src = self->windowPtrForValueAndCopy((uint16_t)h, 0);
    } else if (self->_dmaBarBuffer) {
        self->prepareDmaBarBufferForValue((uint16_t)h);
        src = self->_dmaBarBuffer;
    } else {
        return 2000; // fallback
    }

    while (self->_tft.dmaBusy()) taskYIELD();

    uint64_t t0 = esp_timer_get_time();
    self->_tft.startWrite();
    self->_tft.pushImageDMA(self->_plotX, self->_plotY, w, h, src, nullptr);
    self->_tft.endWrite();
    while (self->_tft.dmaBusy()) taskYIELD();
    uint64_t t1 = esp_timer_get_time();
    uint64_t elapsed = (t1 > t0) ? (t1 - t0) : 2000;
    if (elapsed < 200) elapsed = 200;
    return elapsed;
}

// -------------------- compute safe mid index (static class method) --------------------
size_t BarPlotter::compute_safe_mid_index_static(BarPlotter* self, uint64_t per_bar_us, size_t totalBars) {
    uint64_t safety_margin_us = self->_safety_margin_us; // conservative
    uint64_t half_frame_available_us = 0;
    if (self->_frame_us > safety_margin_us) half_frame_available_us = (self->_frame_us / 2) - safety_margin_us;
    else half_frame_available_us = 0;
    size_t max_bars_half = (per_bar_us > 0) ? (size_t)(half_frame_available_us / per_bar_us) : 1;
    if (max_bars_half < 1) max_bars_half = 1;
    if (max_bars_half > totalBars) max_bars_half = totalBars;
    return max_bars_half;
}

// -------------------- PDM chooser --------------------
uint64_t BarPlotter::choose_next_duration_us() {
    _pdm_acc += _p_fraction;
    if (_pdm_acc >= 1.0) { _pdm_acc -= 1.0; _next_frame_choice_long = true; return _long_duration_us; }
    _next_frame_choice_long = false; return _short_duration_us;
}

uint64_t BarPlotter::choose_next_duration_us_no_side_effects() const {
    if (_short_duration_us == 0) return _frame_us;
    if (_long_duration_us == 0) return _frame_us * 2;
    // This helper isn't used in the current code path but left for API parity.
    return _short_duration_us;
}

// -------------------- constructor / destructor --------------------
BarPlotter::BarPlotter(TFT_eSPI& tft) :
    _tft(tft),
    _barCount(0), _barWidth(0), _barMaxHeight(0), _plotX(0), _plotY(0), _plotSpacing(0),
    _plotInProgress(false), _plotQueueInProgress(false), _plotDataArray(nullptr), _plotDataCount(0),
    _currentBarIndex(0), _plotTaskHandle(NULL),
    _barSourceCopies(nullptr), _barSourcePixelsSingle(0), _barSourceCopiesCount(0), _barSourceRowsSingle(0), _rowWidth(0),
    _dmaBarBuffer(nullptr), _dmaBarBufferPixels(0),
    _frame_us((uint64_t)(1000000.0 / DEFAULT_TARGET_HZ)),
    _porch_step(2),
    _target_rate_hz(DEFAULT_TARGET_HZ),
    _even_frctr2(0x1c), _even_fporch(0x3c), _even_bporch(0x3d),
    _odd_frctr2(0x1f), _odd_fporch(0x8d), _odd_bporch(0x3f),
    _debug_mode(false), _debug_fill_state(false),
    _frameBufA(nullptr), _frameBufB(nullptr), _activeBuf(nullptr), _queuedBuf(nullptr),
    _hasQueuedFrame(false), _frameMutex(NULL), _frameState(FRAME_IDLE),
    _taskRunning(false),
    _send_timer(nullptr), _pdm_acc(0.0), _p_fraction(0.0),
    _current_frame_is_long(false), _next_frame_choice_long(false),
    _short_duration_us(0), _long_duration_us(0),
    _last_cmd_time_us(0), _write_latency_us(DEFAULT_WRITE_LATENCY_US),
    _safety_margin_us(DEFAULT_SAFETY_MARGIN_US), _per_bar_us(0),
    _ramwr_start_us(0), _ramwr_end_us(0),
    _estimated_frame_period_us(0.0)
{
    _colorFill = 0x07E0;   // green
    _colorExtra = 0x0700;  // debug
    _colorFg = 0xFFFF;     // white
    _screenBg = 0x0000;    // black
    _gridColor = 0x5502;
    _phase_send_offset = DEFAULT_WRITE_LATENCY_US / 2;
    _mid_offset = (uint32_t)((uint64_t)(1000000.0 / DEFAULT_TARGET_HZ) / 1.1);
    // register instance for static callback
    BarPlotter::_instance = this;
}

BarPlotter::~BarPlotter() {
    if (_barSourceCopies) heap_caps_free(_barSourceCopies);
    if (_dmaBarBuffer) heap_caps_free(_dmaBarBuffer);
    if (_frameBufA) heap_caps_free(_frameBufA);
    if (_frameBufB) heap_caps_free(_frameBufB);
    if (_frameMutex) vSemaphoreDelete(_frameMutex);
    if (_plotTaskHandle) vTaskDelete(_plotTaskHandle);
    if (_send_timer) esp_timer_delete(_send_timer);
    BarPlotter::_instance = nullptr;
}

// -------------------- setters --------------------
void BarPlotter::setFrameRate(float hz) {
    if (hz <= 0.1f) return;
    _target_rate_hz = hz;
    _frame_us = (uint64_t)(1000000.0f / hz);
    _mid_offset = (uint32_t)(_frame_us * 0.76f);
}

void BarPlotter::setEvenTiming(uint8_t frctr2, uint8_t fporch, uint8_t bporch) {
    _even_frctr2 = frctr2; _even_fporch = fporch; _even_bporch = bporch;
}
void BarPlotter::setOddTiming(uint8_t frctr2, uint8_t fporch, uint8_t bporch) {
    _odd_frctr2 = frctr2; _odd_fporch = fporch; _odd_bporch = bporch;
}
void BarPlotter::setPhaseOffsetUs(uint32_t us) { _phase_send_offset = us; }
void BarPlotter::setMidOffsetUs(uint32_t us) { _mid_offset = us; }
void BarPlotter::setPorchStep(uint8_t step) { _porch_step = step ? step : 1; }
void BarPlotter::enableDebug(bool en) { _debug_mode = en; if (!en) _debug_fill_state = false; }

void BarPlotter::printConfig() {
    Serial.println("=== BarPlotter config ===");
    Serial.printf("even_frctr2=0x%02X, even_fporch=0x%02X, even_bporch=0x%02X\n", _even_frctr2, _even_fporch, _even_bporch);
    Serial.printf("odd_frctr2=0x%02X, odd_fporch=0x%02X, odd_bporch=0x%02X\n", _odd_frctr2, _odd_fporch, _odd_bporch);
    Serial.printf("frame_rate=%.2f Hz, frame_us=%llu us\n", _target_rate_hz, (unsigned long long)_frame_us);
    Serial.printf("write_latency_us=%u, safety_margin_us=%u\n", (unsigned)_write_latency_us, (unsigned)_safety_margin_us);
    Serial.printf("p_fraction=%.4f, pdm_acc=%.4f\n", _p_fraction, _pdm_acc);
    Serial.printf("frameState=%s\n", frameStateString());
    Serial.println("=========================");
}
// return human-readable frame state (must match header: const method)
const char* BarPlotter::frameStateString() const {
    switch (_frameState) {
        case FRAME_IDLE:      return "IDLE";
        case FRAME_QUEUED:    return "QUEUED";
        case FRAME_CONSUMING: return "CONSUMING";
        case FRAME_SENT:      return "SENT";
        default:              return "UNKNOWN";
    }
}

// -------------------- allocation and startup --------------------
bool BarPlotter::begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing) {
    _barCount = barCount; _barWidth = barWidth; _barMaxHeight = barMaxHeight; _plotX = plotX; _plotY = plotY; _plotSpacing = plotSpacing;

    if (createTallBarSourceCopiesWithPattern()) {
        Serial.printf("Using tall precomputed barSource copies with pattern (%u copies)\n", (unsigned)_barSourceCopiesCount);
    } else {
        Serial.println("Tall precompute failed; attempting per-bar DMA buffer");
        if (createPerBarDmaBuffer()) Serial.println("Using per-bar DMA buffer fallback");
        else Serial.println("No DMA buffers; will use fillRect fallback");
    }

    size_t frameBytes = sizeof(uint16_t) * (size_t)_barCount;
    _frameBufA = (uint16_t*) heap_caps_malloc(frameBytes, MALLOC_CAP_8BIT);
    _frameBufB = (uint16_t*) heap_caps_malloc(frameBytes, MALLOC_CAP_8BIT);
    if (!_frameBufA || !_frameBufB) {
        Serial.println("Frame buffer allocation failed");
        if (_frameBufA) heap_caps_free(_frameBufA);
        if (_frameBufB) heap_caps_free(_frameBufB);
        _frameBufA = _frameBufB = nullptr;
    } else {
        for (size_t i=0;i<(size_t)_barCount;++i) { _frameBufA[i]=0; _frameBufB[i]=0; }
        _activeBuf = _frameBufA; _queuedBuf = _frameBufB;
    }

    _frameMutex = xSemaphoreCreateMutex();
    if (!_frameMutex) Serial.println("Frame mutex creation failed");

    // Create persistent plot task pinned to core 1.
    if (_plotTaskHandle == NULL) {
        const uint32_t stackSize = 16 * 1024;
        BaseType_t ok = xTaskCreatePinnedToCore(plotTask, "plotTask", stackSize, this, 7, &_plotTaskHandle, 1);
        if (ok != pdPASS) {
            Serial.println("Failed to create plotTask");
            _plotTaskHandle = NULL; return false;
        } else _taskRunning = true;
    }

    // create send_timer (will be armed by plotTask)
    esp_timer_create_args_t args = { .callback = BarPlotter::send_timer_cb, .arg = nullptr, .name = "bp_send" };
    esp_timer_create(&args, &_send_timer);

    return true;
}

// -------------------- startBarPlot (queue snapshot) --------------------
bool BarPlotter::startBarPlot(const uint16_t* dataArray, size_t count) {
    if (!dataArray || count == 0 || count != (size_t)_barCount) return false;
    const uint32_t default_max_wait_ms = 0;
    bool copied = false;
    if (_frameBufA && _frameBufB && _frameMutex) {
        if (xSemaphoreTake(_frameMutex, pdMS_TO_TICKS(default_max_wait_ms)) == pdTRUE) {
            memcpy(_queuedBuf, dataArray, sizeof(uint16_t) * (size_t)_barCount);
            _hasQueuedFrame = true; _frameState = FRAME_QUEUED; xSemaphoreGive(_frameMutex);
            copied = true;
        } else copied = false;
    } else {
        // fallback pointer path: accept external pointer (const OK)
        _plotDataArray = dataArray; _plotDataCount = count; _plotInProgress = true; _plotQueueInProgress = true; _frameState = FRAME_QUEUED; copied = true;
    }
    return copied;
}

bool BarPlotter::isPlotting() const { return (_frameState == FRAME_CONSUMING); }
bool BarPlotter::isQueueEmpty() const { return !_hasQueuedFrame; }
void BarPlotter::endWrite() { _tft.endWrite(); }

// -------------------- DMA helpers --------------------
bool BarPlotter::createTallBarSourceCopiesWithPattern() {
    const size_t copies = (size_t)_patternH; if (copies == 0) return false;
    const size_t w = (size_t)_barWidth; const size_t h = (size_t)_barMaxHeight; const size_t rowsPerCopy = 2 * h;
    _rowWidth = w; _barSourceRowsSingle = rowsPerCopy;
    uint64_t pixelsSingle = (uint64_t)_rowWidth * (uint64_t)rowsPerCopy;
    if (pixelsSingle == 0 || pixelsSingle > SIZE_MAX / sizeof(uint16_t)) return false;
    _barSourcePixelsSingle = (size_t)pixelsSingle;
    uint64_t totalPixels = (uint64_t)_barSourcePixelsSingle * (uint64_t)copies;
    if (totalPixels == 0 || totalPixels > SIZE_MAX / sizeof(uint16_t)) return false;
    size_t totalBytes = (size_t) totalPixels * sizeof(uint16_t);
    _barSourceCopies = (uint16_t*) heap_caps_malloc(totalBytes, MALLOC_CAP_DMA);
    if (!_barSourceCopies) return false;
    for (size_t k = 0; k < copies; ++k) {
        uint16_t* basePtr = _barSourceCopies + (k * _barSourcePixelsSingle);
        for (size_t r = 0; r < rowsPerCopy; ++r) {
            uint16_t baseColor = (r < h) ? _colorFg : _colorFill;
            size_t base = r * _rowWidth;
            size_t patternRow = (r + k) % _patternH;
            for (size_t c = 0; c < _rowWidth; ++c) {
                size_t patternCol = c % _patternW;
                uint16_t color = baseColor;
                if (_pattern[patternRow][patternCol]) color = _gridColor;
                basePtr[base + c] = color;
            }
        }
    }
    _barSourceCopiesCount = copies; return true;
}

bool BarPlotter::createPerBarDmaBuffer() {
    const size_t w = (size_t)_barWidth; const size_t h = (size_t)_barMaxHeight;
    _dmaBarBufferPixels = w * h; size_t bytes = _dmaBarBufferPixels * sizeof(uint16_t);
    _dmaBarBuffer = (uint16_t*) heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!_dmaBarBuffer) return false;
    for (size_t i = 0; i < _dmaBarBufferPixels; ++i) _dmaBarBuffer[i] = _colorFg; return true;
}

void BarPlotter::prepareDmaBarBufferForValue(uint16_t v) {
    if (!_dmaBarBuffer) return; const size_t w = (size_t)_barWidth; const size_t h = (size_t)_barMaxHeight; if (v > (uint16_t)h) v = (uint16_t)h;
    for (size_t r = 0; r < h; ++r) {
        bool fillRow = (r >= (h - v)); uint16_t color = fillRow ? _colorFill : _colorFg; size_t base = r * w; for (size_t c = 0; c < w; ++c) _dmaBarBuffer[base + c] = color;
    }
}

uint16_t* BarPlotter::windowPtrForValueAndCopy(uint16_t v, size_t copyIndex) {
    if (!_barSourceCopies) return nullptr; if (copyIndex >= _barSourceCopiesCount) copyIndex = 0; if (v > (uint16_t)_barMaxHeight) v = (uint16_t)_barMaxHeight;
    return _barSourceCopies + (copyIndex * _barSourcePixelsSingle) + ((size_t)v * _rowWidth);
}

void BarPlotter::clearSpacingAreas() {
    if (_plotSpacing == 0) return; for (size_t i = 0; i < _barCount; ++i) {
        int dx = _plotX + (int)i * (_barWidth + _plotSpacing); int gapX = dx + _barWidth; if (_plotSpacing > 0) _tft.fillRect(gapX, _plotY, _plotSpacing, _barMaxHeight, _screenBg);
    }
}

// -------------------- rendering --------------------
void BarPlotter::renderNextFrame() {
    // Swap in queued frame if present
    if (_frameBufA && _frameBufB && _frameMutex) {
        if (_hasQueuedFrame) {
            if (xSemaphoreTake(_frameMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                uint16_t* tmp = _activeBuf; _activeBuf = _queuedBuf; _queuedBuf = tmp; _hasQueuedFrame = false; _frameState = FRAME_CONSUMING; _plotDataArray = _activeBuf; _plotDataCount = (size_t)_barCount; xSemaphoreGive(_frameMutex);
            }
        }
    }

    const int totalBars = (int)_plotDataCount;
    if (totalBars <= 0) { _frameState = FRAME_IDLE; return; }

    // compute safe mid index
    size_t mid_bar_index = this->compute_safe_mid_index(_per_bar_us, (size_t)totalBars);
    if (mid_bar_index > (size_t)totalBars) mid_bar_index = (size_t)totalBars;

    // record ramwr start timestamp
    _ramwr_start_us = esp_timer_get_time();

    // Stage A: push first chunk
    _tft.startWrite();
    const int baseX = _plotX; const int baseY = _plotY; const int w = _barWidth; const int h = _barMaxHeight;
    bool debug_force_fill = false;
    if (_debug_mode) { debug_force_fill = _debug_fill_state; _debug_fill_state = !_debug_fill_state; }

    for (size_t i = 0; i < mid_bar_index; ++i) {
        uint16_t value = _debug_mode ? (debug_force_fill ? (uint16_t)h : 0) : _plotDataArray[i];
        if (value > (uint16_t)h) value = (uint16_t)h;
        int dx = baseX + (int)i * (w + _plotSpacing); int dy = baseY;
        size_t copyIndex = 0;
        if (_barSourceCopiesCount >= (size_t)_patternH) {
            size_t anchor = (size_t)(_plotY % _patternH);
            size_t v_mod = (size_t)(value % (uint16_t)_patternH);
            copyIndex = (anchor + _patternH - v_mod) % _patternH;
        } else if (_barSourceCopiesCount > 0) copyIndex = i % _barSourceCopiesCount;

        if (_barSourceCopies) {
            uint16_t* src = windowPtrForValueAndCopy(value, copyIndex);
            _tft.pushImageDMA(dx, dy, w, h, src, nullptr);
        } else if (_dmaBarBuffer) {
            prepareDmaBarBufferForValue(value);
            _tft.pushImageDMA(dx, dy, w, h, _dmaBarBuffer, nullptr);
        } else {
            if (value > 0) { int fillY = dy + (h - (int)value); _tft.fillRect(dx, fillY, w, (int)value, _colorFill); }
            if (value < (uint16_t)h) { int clearH = h - (int)value; _tft.fillRect(dx, dy, w, clearH, _colorFg); }
        }
    }
    //_tft.endWrite();

    // wait for DMA completion before possibly starting second chunk
    while (_tft.dmaBusy()) taskYIELD();

    // Stage C: push remaining bars

    if (mid_bar_index < (size_t)totalBars) {
/*
           
           // All bars already pushed in Stage A. Wait for DMA and then send mid (late).
            while (self->_tft.dmaBusy()) taskYIELD();
            //self->_tft.startWrite();
            //if (!use_even_set) {
                write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
            //} else {
                //write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
            //}
            //self->_tft.endWrite();
            // nothing left to push
        } else {
            // Need to send mid between two chunks: wait for current DMA to finish
            while (self->_tft.dmaBusy()) taskYIELD();

            // send mid-frame timing
            //self->_tft.startWrite();
            //if (use_even_set) {
                write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
            //} else {
            //    write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
            //}
            //self->_tft.endWrite();
*/    
      
      
      
      
      
      
      //  _tft.startWrite(); 
        for (size_t j = mid_bar_index; j < (size_t)totalBars; ++j) {
            uint16_t value = _debug_mode ? (debug_force_fill ? (uint16_t)h : 0) : _plotDataArray[j];
            if (value > (uint16_t)h) value = (uint16_t)h;
            int dx = baseX + (int)j * (w + _plotSpacing); int dy = baseY;
            size_t copyIndex = 0;
            if (_barSourceCopiesCount >= (size_t)_patternH) {
                size_t anchor = (size_t)(_plotY % _patternH);
                size_t v_mod = (size_t)(value % (uint16_t)_patternH);
                copyIndex = (anchor + _patternH - v_mod) % _patternH;
            } else if (_barSourceCopiesCount > 0) copyIndex = j % _barSourceCopiesCount;

            if (_barSourceCopies) {
                uint16_t* src = windowPtrForValueAndCopy(value, copyIndex);
                _tft.pushImageDMA(dx, dy, w, h, src, nullptr);
            } else if (_dmaBarBuffer) {
                prepareDmaBarBufferForValue(value);
                _tft.pushImageDMA(dx, dy, w, h, _dmaBarBuffer, nullptr);
            } else {
                if (value > 0) { int fillY = dy + (h - (int)value); _tft.fillRect(dx, fillY, w, (int)value, _colorFill); }
                if (value < (uint16_t)h) { int clearH = h - (int)value; _tft.fillRect(dx, dy, w, clearH, _colorFg); }
            }
        }
        _tft.endWrite();
    }

    // wait for DMA finish and record ramwr end
    while (_tft.dmaBusy()) taskYIELD();
    _ramwr_end_us = esp_timer_get_time();

    // frame done
    _frameState = FRAME_SENT; _plotQueueInProgress = false; _plotInProgress = false;
}

// -------------------- plotTask: main loop (waits for timer notify) --------------------
void BarPlotter::plotTask(void* pvParameters) {
    BarPlotter* self = (BarPlotter*)pvParameters;

    // calibrate per-bar DMA time once at startup
    self->_per_bar_us = (uint32_t) self->calibrate_bar_transfer_time(self);
    if (self->_per_bar_us < 200) self->_per_bar_us = 200;
    Serial.printf("BarPlotter: calibrated per-bar transfer time = %u us\n", (unsigned)self->_per_bar_us);

    // derive short/long durations if not explicitly set
    if (self->_short_duration_us == 0) self->_short_duration_us = self->_frame_us;
    if (self->_long_duration_us == 0) self->_long_duration_us = self->_frame_us * 2;

    // compute PDM fraction
    double Tt = (double)self->_frame_us;
    double Ts = (double)self->_short_duration_us;
    double Tl = (double)self->_long_duration_us;
    if (Tl == Ts) self->_p_fraction = 0.0; else self->_p_fraction = (Tt - Ts) / (Tl - Ts);
    if (self->_p_fraction < 0.0) self->_p_fraction = 0.0; if (self->_p_fraction > 1.0) self->_p_fraction = 1.0;
    self->_pdm_acc = 0.0;

    // initial timing write for the upcoming frame
    self->choose_next_duration_us(); // sets _next_frame_choice_long
    self->_tft.startWrite();
    if (self->_next_frame_choice_long) write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
    else write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
    self->_tft.endWrite();
    self->_last_cmd_time_us = esp_timer_get_time();

    // schedule the first timer using next duration from PDM
    uint64_t nextDur = self->choose_next_duration_us(); // for following frame
    uint64_t send_at = self->_last_cmd_time_us + nextDur - self->_write_latency_us;
    int64_t delay = (int64_t)send_at - (int64_t)esp_timer_get_time(); if (delay < 0) delay = 0;
    esp_timer_start_once(self->_send_timer, (uint64_t)delay);

    // draw the very first frame now
    self->renderNextFrame();

    // main loop: wait for notification from send_timer (ISR)
    for (;;) {
        // block until notified by timer
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // timer fired: perform the single write for "next frame"
        self->_tft.startWrite();
        // pick and write the timing set for the next frame
        if (self->choose_next_duration_us(), self->_next_frame_choice_long) {
            write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch);
            self->_current_frame_is_long = true;
        } else {
            write_timing_registers(self->_tft, self->_even_frctr2, self->_even_fporch, self->_even_bporch);
            self->_current_frame_is_long = false;
        }
        self->_tft.endWrite();

        // record when we issued the command
        self->_last_cmd_time_us = esp_timer_get_time();

        // schedule next timer using next duration from PDM
        uint64_t chosen_next = self->choose_next_duration_us();
        uint64_t next_send_at = self->_last_cmd_time_us + chosen_next - self->_write_latency_us;
        int64_t d = (int64_t)next_send_at - (int64_t)esp_timer_get_time(); if (d < 0) d = 0;
        esp_timer_start_once(self->_send_timer, (uint64_t)d);

        // Now render the current frame (we just prepared the "next" timing)
        self->renderNextFrame();
        write_timing_registers(self->_tft, self->_odd_frctr2, self->_odd_fporch, self->_odd_bporch); // set long frame
    }

    vTaskDelete(NULL);
}
