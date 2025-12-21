//#define MONO_OLED_128x64 // for 128x64_mono_oled
#define st7735_tft

#ifdef st7735_tft

#define TFT_WIDTH 320 // display width, in pixels
#define TFT_HEIGHT 240 // display height, in pixels
#define COLOR_DEPTH 16 // must be 16 if using DMA.

#include <SPI.h>
#include <TFT_eSPI.h>

//esp32
#define TFT_CS     4
#define TFT_LED    27 // LED must be connected via pnp transistor. active low.

TFT_eSPI    tft = TFT_eSPI();
//#define IWIDTH  128
//#define IHEIGHT 160 // sprite/framebuffer size. 181x181 is max for dma (64k)

#define USE_DMA_TO_TFT  // this is faster but does not allow 2bpp framebuffer. only 16bit framebuffer allowed for DMA


// Option 2: use any pins but a little slower!
//#define TFT_SCLK 13   // set these to be whatever pins you like!
//#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#endif //st7735_tft
