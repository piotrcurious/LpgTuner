#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define DISPLAY_REFRESH_RATE 60 // Target refresh rate

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr[3] = {TFT_eSprite(&tft), TFT_eSprite(&tft), TFT_eSprite(&tft)};

uint16_t* sprPtr[3];
uint8_t currentBuffer = 0;
uint8_t drawingBuffer = 1;
uint8_t pendingBuffer = 2;

bool odd_frame = false;
bool odd_frame_adjusted = false;
unsigned long last_display_time = 0;

// This is your asynchronous drawing function
void drawGraphics(void* parameter) {
  while (true) {
    // Clear the drawing buffer
    spr[drawingBuffer].fillSprite(TFT_BLACK);

    // Your drawing code here
    spr[drawingBuffer].fillRect(random(TFT_WIDTH), random(TFT_HEIGHT), 20, 20, TFT_RED);
    spr[drawingBuffer].drawLine(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_GREEN);

    // Signal that drawing is complete by swapping pending and drawing buffers
    uint8_t temp = pendingBuffer;
    pendingBuffer = drawingBuffer;
    drawingBuffer = temp;

    vTaskDelay(1); // Give other tasks a chance to run
  }
}

void setup() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  for (int i = 0; i < 3; i++) {
    spr[i].createSprite(TFT_WIDTH, TFT_HEIGHT);
    spr[i].fillSprite(TFT_BLACK);
    sprPtr[i] = (uint16_t*)spr[i].getPointer();
  }

  // Start the drawing task
  xTaskCreatePinnedToCore(drawGraphics, "DrawGraphics", 4096, NULL, 1, NULL, 0);

  // Initial display timing setup
  tft.writecommand(ST7735_FRMCTR1);
  tft.writedata(0x0b);
  tft.writedata(0x16);
  tft.writedata(0x3f);
}

void loop() {
  unsigned long current_time = millis();

  if (current_time - last_display_time >= 1000 / DISPLAY_REFRESH_RATE) {
    last_display_time = current_time;

    tft.startWrite();

    // Set frame timing for even frames
    tft.writecommand(ST7735_FRMCTR1);
    tft.writedata(0x0b);
    tft.writedata(0x16);
    tft.writedata(0x3f);

    odd_frame = !odd_frame;
    odd_frame_adjusted = false;

    // Wait for any ongoing DMA transfer to complete
    while (tft.dmaBusy()) {
      yield();
    }

    // Swap buffers
    uint8_t temp = currentBuffer;
    currentBuffer = pendingBuffer;
    pendingBuffer = temp;

    // Push the current buffer to the display
    tft.pushImageDMA(0, 0, TFT_WIDTH, TFT_HEIGHT, sprPtr[currentBuffer]);

    tft.endWrite();
  }

  // Adjust frame timing for odd frames
  if (millis() - last_display_time >= (1000 / DISPLAY_REFRESH_RATE) / 2) {
    if (odd_frame && !odd_frame_adjusted) {
      tft.startWrite();
      tft.writecommand(ST7735_FRMCTR1);
      tft.writedata(0x0a);
      tft.writedata(0x20);
      tft.writedata(0x3f);
      tft.endWrite();
      odd_frame_adjusted = true;
    }
  }

  // Other non-blocking tasks can be performed here
}
