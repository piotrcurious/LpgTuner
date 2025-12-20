#ifndef BAR_PLOTTER_H
#define BAR_PLOTTER_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// BarPlotter: drives a vertical bar plot area using TFT_eSPI.
// - Uses DMA-capable precomputed bar bitmaps or per-bar DMA buffer
// - Provides ping-pong frame buffering and a persistent plot task
// - Exposes frame-state for producer/consumer synchronization

class BarPlotter {
public:
    BarPlotter(TFT_eSPI& tft);
    ~BarPlotter();

    // initialize plotter (allocations, precomputes, and starts persistent plot task)
    bool begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing);

    // queue a frame of bar heights (0..barMaxHeight).
    // Returns true if the frame was accepted (queued or copied), false if dropped.
    bool startBarPlot(const uint16_t* dataArray, size_t count);

    // quick state info
    bool isPlotting() const;    // currently consuming/plotting a frame
    bool isQueueEmpty() const;  // true if no queued frame waiting
    void endWrite();

    // tuning API
    void setEvenTiming(uint8_t frctr2, uint8_t fporch, uint8_t bporch);
    void setOddTiming(uint8_t frctr2, uint8_t fporch, uint8_t bporch);
    void setPhaseOffsetUs(uint32_t us);
    void setMidOffsetUs(uint32_t us);
    void setFrameRate(float hz);
    void setPorchStep(uint8_t step);
    void enableDebug(bool en);
    void printConfig(); // prints current tuning state to Serial

    // Exposed tuning field (kept public for easy live tweaking from sketches)
    uint32_t _phase_send_offset;  // microseconds before expected frame start to send "start-of-frame" timing

    // timing parameters exposed for live tuning
    uint8_t _even_frctr2;
    uint8_t _even_fporch;
    uint8_t _even_bporch;

    uint8_t _odd_frctr2;
    uint8_t _odd_fporch;
    uint8_t _odd_bporch;

    bool    _debug_mode;
    bool    _debug_fill_state; // flips each frame for debug alternation

    uint32_t _mid_offset;         // offset (us) from push time to send alternate timing write (default frame_us/2)
    float    _target_rate_hz;     // convenience

    // Frame-state introspection (for synchronization)
    // return integer code (use frameStateString() for readable)
    int getFrameState() const;
    const char* frameStateString() const;

private:
    static void plotTask(void* pvParameters);

    TFT_eSPI& _tft;

    int _barCount;
    int _barWidth;
    int _barMaxHeight;
    int _plotX;
    int _plotY;
    int _plotSpacing;

    // backward-compatible flags
    volatile bool _plotInProgress;
    volatile bool _plotQueueInProgress;

    // pointer to active data used by plot task (may point to internal ping buffer)
    const uint16_t* _plotDataArray;
    size_t _plotDataCount;
    size_t _currentBarIndex;
    TaskHandle_t _plotTaskHandle;

    // precomputed tall copies (patterned)
    uint16_t* _barSourceCopies;
    size_t _barSourcePixelsSingle;
    size_t _barSourceCopiesCount;
    size_t _barSourceRowsSingle;
    size_t _rowWidth;

    // per-bar DMA fallback buffer
    uint16_t* _dmaBarBuffer;
    size_t _dmaBarBufferPixels;

    // Pattern definition
    static const uint8_t _patternW = 4;
    static const uint8_t _patternH = 16;
    static const uint8_t _pattern[_patternH][_patternW];

    // Colors
    uint16_t _colorFill;
    uint16_t _colorExtra;
    uint16_t _colorFg;
    uint16_t _screenBg;
    uint16_t _gridColor;

    // helpers
    bool createTallBarSourceCopiesWithPattern();
    bool createPerBarDmaBuffer();
    void prepareDmaBarBufferForValue(uint16_t v);
    uint16_t* windowPtrForValueAndCopy(uint16_t v, size_t copyIndex);
    void clearSpacingAreas();
    static uint64_t calibrate_bar_transfer_time(BarPlotter *self);

    // timing / scheduler fields
    uint64_t _frame_us;           // frame interval in microseconds (1/refresh)
    uint8_t  _porch_step;         // tuning step for porch adjustments

    // --- ping-pong frame buffering for safe producer/consumer handoff
    uint16_t* _frameBufA;         // internal buffer A (heights)
    uint16_t* _frameBufB;         // internal buffer B (heights)
    uint16_t* _activeBuf;         // buffer currently being consumed by plot task
    uint16_t* _queuedBuf;         // buffer owned by producer until swapped
    volatile bool _hasQueuedFrame; // true when queuedBuf contains a fresh frame
    SemaphoreHandle_t _frameMutex; // protects swap/copy of ping-pong buffers

    // frame-state enum (internal)
    enum FrameState {
        FRAME_IDLE = 0,
        FRAME_QUEUED,
        FRAME_CONSUMING,
        FRAME_SENT
    };
    volatile FrameState _frameState;

    // internal flags
    volatile bool _taskRunning;
};

#endif // BAR_PLOTTER_H
