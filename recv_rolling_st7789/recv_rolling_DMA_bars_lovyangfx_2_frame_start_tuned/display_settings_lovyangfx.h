#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void)
  {
    auto cfg = _bus_instance.config();

    cfg.spi_host = VSPI_HOST;
    cfg.spi_mode = 0;
    cfg.freq_write = 80000000;
    cfg.freq_read  = 16000000;
    cfg.pin_sclk = 18;
    cfg.pin_mosi = 23;
    cfg.pin_miso = -1;
    cfg.pin_dc   = 16;

    _bus_instance.config(cfg);
    _panel_instance.setBus(&_bus_instance);

    auto panel_cfg = _panel_instance.config();

    panel_cfg.pin_cs           = 5;
    panel_cfg.pin_rst          = 17;
    panel_cfg.pin_busy         = -1;
    panel_cfg.panel_width      = 240;
    panel_cfg.panel_height     = 320;
    panel_cfg.offset_x         = 0;
    panel_cfg.offset_y         = 0;
    panel_cfg.offset_rotation  = 0;
    panel_cfg.dummy_read_pixel = 8;
    panel_cfg.dummy_read_bits  = 1;
    panel_cfg.readable         = true;
    panel_cfg.invert           = true;
    panel_cfg.rgb_order        = false;
    panel_cfg.dlen_16bit       = false;
    panel_cfg.bus_shared       = true;

    _panel_instance.config(panel_cfg);

    setPanel(&_panel_instance);
  }
};

extern LGFX tft;

#define TFT_LED 27
