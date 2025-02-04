#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define DISPLAY_REFRESH_RATE 60  // Target refresh rate (fps)

TFT_eSPI tft = TFT_eSPI();

// Two sprite buffers: one for display, one for drawing.
TFT_eSprite spr[2] = { TFT_eSprite(&tft), TFT_eSprite(&tft) };
uint16_t* sprPtr[2];

volatile uint8_t currentBuffer = 0;   // Currently displayed buffer.
volatile uint8_t drawingBuffer = 1;   // Buffer into which drawing is done.

// Semaphore signaling that drawing is complete.
SemaphoreHandle_t drawDoneSemaphore;
// Semaphore signaling that buffer swap is done (i.e. the drawing buffer is free).
SemaphoreHandle_t swapDoneSemaphore;

// For continuous frame adjustment.
volatile unsigned long lastFrameTime = 0;
volatile bool oddFrame = false;
volatile bool oddFrameAdjusted = false;

// -----------------------------------------------------------------------------
// Drawing task: It waits until the previous frame has been swapped (via swapDoneSemaphore),
// then draws into the drawing buffer, and signals completion with drawDoneSemaphore.
void drawGraphics(void* parameter) {
  for (;;) {
    // Wait until the main loop signals that the drawing buffer is free.
    xSemaphoreTake(swapDoneSemaphore, portMAX_DELAY);
    
    // Draw into the designated drawing buffer.
    spr[drawingBuffer].fillSprite(TFT_BLACK);
    
    // Example drawing code – replace with your custom drawing:
    spr[drawingBuffer].fillRect(random(TFT_WIDTH), random(TFT_HEIGHT), 20, 20, TFT_RED);
    spr[drawingBuffer].drawLine(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_GREEN);
    
    // Signal that drawing is complete.
    xSemaphoreGive(drawDoneSemaphore);
    
    // Yield to allow main loop to process.
    vTaskDelay(1);
  }
}

// -----------------------------------------------------------------------------
// Setup: initialize the TFT, create sprites, and start the drawing task.
void setup() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  
  for (int i = 0; i < 2; i++) {
    spr[i].createSprite(TFT_WIDTH, TFT_HEIGHT);
    spr[i].fillSprite(TFT_BLACK);
    sprPtr[i] = (uint16_t*)spr[i].getPointer();
  }
  
  // Create the semaphores.
  drawDoneSemaphore = xSemaphoreCreateBinary();
  swapDoneSemaphore = xSemaphoreCreateBinary();
  // Allow the drawing task to start (buffer is free).
  xSemaphoreGive(swapDoneSemaphore);
  
  // Launch the drawing task (pinned to core 0 as an example).
  xTaskCreatePinnedToCore(drawGraphics, "DrawGraphics", 4096, NULL, 1, NULL, 0);
  
  // Set the initial frame timing (even frame parameters).
  tft.startWrite();
  tft.writecommand(ST7735_FRMCTR1);
  tft.writedata(0x0B);
  tft.writedata(0x16);
  tft.writedata(0x3F);
  tft.endWrite();
  
  lastFrameTime = millis();
}

// -----------------------------------------------------------------------------
// Main loop: continuously adjust frame timing while, once per frame period,
// if the drawing is complete, push the image via DMA and swap buffers.
void loop() {
  unsigned long currentTime = millis();
  unsigned long framePeriod = 1000 / DISPLAY_REFRESH_RATE;
  
  // Continuous frame adjustment: regardless of drawing state.
  if (oddFrame && !oddFrameAdjusted && (currentTime - lastFrameTime >= framePeriod / 2)) {
    tft.startWrite();
    tft.writecommand(ST7735_FRMCTR1);
    // Adjusted timing parameters for mid-frame update.
    tft.writedata(0x0A);
    tft.writedata(0x20);
    tft.writedata(0x3F);
    tft.endWrite();
    oddFrameAdjusted = true;
  }
  
  // When the full frame period has elapsed:
  if (currentTime - lastFrameTime >= framePeriod) {
    // Restart frame timing.
    lastFrameTime = currentTime;
    oddFrame = !oddFrame;
    oddFrameAdjusted = false;
    
    // Only proceed if the drawing task has completed its drawing.
    if (xSemaphoreTake(drawDoneSemaphore, 0) == pdTRUE) {
      // Wait until any ongoing DMA transfer is finished.
      while (tft.dmaBusy()) {
        yield();
      }
      
      // Optionally, you may re-send the standard (even frame) timing command here.
      tft.startWrite();
      tft.writecommand(ST7735_FRMCTR1);
      tft.writedata(0x0B);
      tft.writedata(0x16);
      tft.writedata(0x3F);
      tft.endWrite();
      
      // Initiate DMA transfer: push the just-drawn drawingBuffer to the display.
      tft.pushImageDMA(0, 0, TFT_WIDTH, TFT_HEIGHT, sprPtr[drawingBuffer]);
      
      // Swap the buffers so that the just-drawn image becomes current.
      uint8_t temp = currentBuffer;
      currentBuffer = drawingBuffer;
      drawingBuffer = temp;
      
      // Signal the drawing task that the swap is done and it can begin drawing the next frame.
      xSemaphoreGive(swapDoneSemaphore);
    }
  }
  
  // Other non-blocking tasks may be processed here.
}
