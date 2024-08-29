#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};

#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define COLOR_DEPTH 16  // 16-bit color
#define DISPLAY_REFRESH_RATE 16  // ~60Hz (1000ms / 60fps = 16.67ms)

uint8_t sprSel = 0;
bool odd_frame = false;
bool odd_frame_adjusted = false;
unsigned long last_display_time = 0;

// Function prototype for the drawing callback
typedef void (*DrawCallback)(TFT_eSprite& sprite);

void setup() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Initialize sprites
  for (int i = 0; i < 2; i++) {
    spr[i].createSprite(TFT_WIDTH, TFT_HEIGHT);
    spr[i].setColorDepth(COLOR_DEPTH);
  }

  // Enable DMA for faster transfers
  tft.initDMA();

  // Set initial frame rate
  tft.startWrite();
  tft.writecommand(ST7735_FRMCTR1); // frame rate control
  tft.writedata(0x0b);  // fastest refresh
  tft.writedata(0x16);  // 6 lines front porch
  tft.writedata(0x3f);  // 3 lines back porch
  tft.endWrite();
}

void adjustRefreshRate(bool isOddFrame) {
  tft.startWrite();
  tft.writecommand(ST7735_FRMCTR1); // frame rate control
  
  if (isOddFrame) {
    tft.writedata(0x0a);  // slightly slower refresh
    tft.writedata(0x20);  // 32 lines front porch
    tft.writedata(0x3f);  // 63 lines back porch
  } else {
    tft.writedata(0x0b);  // fastest refresh
    tft.writedata(0x16);  // 22 lines front porch
    tft.writedata(0x3f);  // 63 lines back porch
  }
  
  tft.endWrite();
}

void renderFrame(DrawCallback drawFunc) {
  unsigned long current_time = millis();
  
  if (current_time - last_display_time >= DISPLAY_REFRESH_RATE) {
    // Draw to the back buffer
    drawFunc(spr[!sprSel]);

    // Push the sprite to the display using DMA
    tft.startWrite();
    tft.pushImageDMA(0, 0, TFT_WIDTH, TFT_HEIGHT, spr[!sprSel].getPointer());

    // Adjust refresh rate based on odd/even frame
    odd_frame = !odd_frame;
    if (odd_frame && !odd_frame_adjusted) {
      adjustRefreshRate(true);
      odd_frame_adjusted = true;
    } else if (!odd_frame) {
      adjustRefreshRate(false);
      odd_frame_adjusted = false;
    }

    // Swap buffers
    sprSel = !sprSel;

    last_display_time = current_time;
  }

  // Non-blocking check for DMA completion
  if (!tft.dmaBusy()) {
    tft.endWrite();
  }
}

// Example drawing function
void drawGraphics(TFT_eSprite& sprite) {
  static int x = 0;
  
  sprite.fillSprite(TFT_BLACK);
  sprite.drawRect(x, 50, 20, 20, TFT_WHITE);
  sprite.drawLine(0, x, TFT_WIDTH, TFT_HEIGHT - x, TFT_RED);
  
  x = (x + 1) % TFT_WIDTH;
}

void loop() {
  renderFrame(drawGraphics);
  // Other non-blocking tasks can be performed here
}
