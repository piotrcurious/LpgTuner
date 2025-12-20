#define st7735_tft

#ifdef st7735_tft

#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define COLOR_DEPTH 16

#include <SPI.h>
#include <TFT_eSPI.h>

#define TFT_CS     4
#define TFT_LED    27

TFT_eSPI    tft = TFT_eSPI();

#endif //st7735_tft
