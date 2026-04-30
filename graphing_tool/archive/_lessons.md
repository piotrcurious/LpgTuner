# Graphing Tool Archive

## graphing_esp32_tft.ino
This was the initial version of the engine tuning map tool.

### Lessons Learned:
- **Coupled Updates:** Data collection was tied to the display refresh rate, causing statistical inaccuracies if the display loop was slow or fast.
- **Blocking UI:** Button handling used `delay()` or blocking loops, which paused engine data collection during mode switches.
- **Library Dependency:** It relied on `TFT_eSPI`, which is being phased out in favor of `LovyanGFX` for better performance and hardware abstraction.
- **Safety:** It lacked proper critical sections for multi-core ESP32 interrupt handling.

### Superseded By:
`graphing_esp32_tft_more.ino` which implements non-blocking UI, decoupled data processing, persistent storage, and statistical overlays.
