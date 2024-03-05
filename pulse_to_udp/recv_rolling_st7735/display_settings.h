//#define MONO_OLED_128x64 // for 128x64_mono_oled
#define st7735_tft

#ifdef MONO_OLED_128x64
// Define the OLED display parameters
#define OLED_WIDTH 128 // OLED display width, in pixels
#define OLED_HEIGHT 64 // OLED display height, in pixels
//#define OLED_ADDRESS 0x3C // OLED display I2C address
#define OLED_BITRATE  80000000 // OLED SPI bitrate 80Mhz max

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);
#endif // MONO_OLED_128x64

#ifdef st7735_tft

#define TFT_WIDTH 128 // OLED display width, in pixels
#define TFT_HEIGHT 160 // OLED display height, in pixels
#define TFT_BITRATE  70000000 // TFT SPI bitrate 80Mhz max
#define COLOR_DEPTH 16


//#include <Adafruit_GFX.h>    // Core graphics library
//#include <Adafruit_ST7735.h> // Hardware-specific library

#include <SPI.h>
#include <TFT_eSPI.h>

/*
//esp8266
//#define TFT_CS     D8
#define TFT_CS     15
//#define TFT_RST    D4  // you can also connect this to the Arduino reset
#define TFT_RST    2  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
//#define TFT_DC     D3
#define TFT_DC     0

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
*/

//esp32
#define TFT_CS     4   
#define TFT_LED    27 // LED must be connected via pnp transistor. active low. 

//#define TFT_RST    17  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
//#define TFT_DC     16

//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

TFT_eSPI    tft = TFT_eSPI();
#define IWIDTH  128
#define IHEIGHT 160 // sprite/framebuffer size. 181x181 is max for dma (64k)

// Create two sprites for a DMA toggle buffer
TFT_eSprite spr[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft) };

#define USE_DMA_TO_TFT  // this is faster but does not allow 2bpp framebuffer. only 16bit framebuffer allowed for DMA

// Toggle buffer selection
bool sprSel = 0;

// Pointers to start of Sprites in RAM
uint16_t* sprPtr[2];


// Option 2: use any pins but a little slower!
//#define TFT_SCLK 13   // set these to be whatever pins you like!
//#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#endif //st7735_tft
