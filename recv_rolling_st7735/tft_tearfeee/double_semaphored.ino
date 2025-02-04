#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define DISPLAY_REFRESH_RATE 60  // Target refresh rate (fps)

TFT_eSPI tft = TFT_eSPI();

// Two sprite buffers: one is being displayed, one is drawn into.
TFT_eSprite spr[2] = { TFT_eSprite(&tft), TFT_eSprite(&tft) };
uint16_t* sprPtr[2];

volatile uint8_t currentBuffer = 0;   // Buffer currently displayed
volatile uint8_t drawingBuffer = 1;   // Buffer used for drawing next frame

// Binary semaphore to signal when the drawing buffer is complete.
SemaphoreHandle_t xSemaphore;

// Variables for frame timing and anti-tearing control.
volatile unsigned long lastDisplayTime = 0;
volatile bool oddFrame = false;   // toggles each frame

// -----------------------------------------------------------------------------
// Drawing Task: waits for permission to draw, then updates the drawing buffer.
// When finished, it signals completion via the semaphore.
void drawGraphics(void * parameter) {
  while (true) {
    // Wait until allowed to draw.
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      // Clear the drawing buffer.
      spr[drawingBuffer].fillSprite(TFT_BLACK);

      // --- Your drawing code goes here ---
      spr[drawingBuffer].fillRect(random(TFT_WIDTH), random(TFT_HEIGHT), 20, 20, TFT_RED);
      spr[drawingBuffer].drawLine(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_GREEN);

      // Signal that drawing is complete.
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(1); // Yield to other tasks.
  }
}

// -----------------------------------------------------------------------------
// setup(): initializes the TFT, the sprite buffers, the semaphore, and starts
// the drawing task.
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Create the two sprite buffers.
  for (int i = 0; i < 2; i++) {
    spr[i].createSprite(TFT_WIDTH, TFT_HEIGHT);
    spr[i].fillSprite(TFT_BLACK);
    sprPtr[i] = (uint16_t*)spr[i].getPointer();
  }

  // Create the binary semaphore.
  xSemaphore = xSemaphoreCreateBinary();
  // Initially allow drawing.
  xSemaphoreGive(xSemaphore);

  // Start the drawing task on core 0 (or adjust core as needed).
  xTaskCreatePinnedToCore(drawGraphics, "DrawGraphics", 4096, NULL, 1, NULL, 0);

  // Set initial display timing (for an even frame).
  tft.startWrite();
  tft.writecommand(ST7735_FRMCTR1);
  tft.writedata(0x0B);
  tft.writedata(0x16);
  tft.writedata(0x3F);
  tft.endWrite();

  lastDisplayTime = millis();
}

// -----------------------------------------------------------------------------
// loop(): continuously runs the anti-tearing adjustment (when in an odd frame)
// and, if the drawing buffer is complete and the frame period has elapsed,
// pushes the current image via DMA and swaps buffers.
void loop() {
  unsigned long currentTime = millis();
  unsigned long framePeriod = 1000 / DISPLAY_REFRESH_RATE;

  // --- Continuous frame adjustment ---
  // When in an odd frame, continuously send adjusted timing commands.
  if (oddFrame) {
    tft.startWrite();
    tft.writecommand(ST7735_FRMCTR1);
    // Adjusted parameters (example values) for anti-tearing.
    tft.writedata(0x0A);
    tft.writedata(0x20);
    tft.writedata(0x3F);
    tft.endWrite();
  }
  
  // --- DMA Push and Buffer Swap ---
  // Only attempt to update the display when one frame period has elapsed...
  if (currentTime - lastDisplayTime >= framePeriod) {
    // ...and only if the drawing buffer is complete (i.e. semaphore available).
    if (xSemaphoreTake(xSemaphore, 0) == pdTRUE) {
      // Wait for any ongoing DMA transfer to finish.
      while (tft.dmaBusy()) {
        yield();
      }
      
      tft.startWrite();
      // For even frames, send standard timing parameters.
      if (!oddFrame) {
        tft.writecommand(ST7735_FRMCTR1);
        tft.writedata(0x0B);
        tft.writedata(0x16);
        tft.writedata(0x3F);
      }
      tft.endWrite();
      
      // Push the current buffer to the display via DMA.
      tft.pushImageDMA(0, 0, TFT_WIDTH, TFT_HEIGHT, sprPtr[currentBuffer]);

      // Swap the buffers.
      uint8_t temp = currentBuffer;
      currentBuffer = drawingBuffer;
      drawingBuffer = temp;

      // Signal the drawing task to start drawing the next frame.
      xSemaphoreGive(xSemaphore);

      // Reset frame timer and toggle the oddFrame flag.
      lastDisplayTime = currentTime;
      oddFrame = !oddFrame;
    }
  }

  // Other non-blocking tasks can be performed here.
}
