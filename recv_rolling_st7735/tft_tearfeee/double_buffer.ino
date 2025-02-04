#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define DISPLAY_REFRESH_RATE 60

struct BufferState {
    bool ready;
    uint32_t frameCount;
};

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};

// Buffer management
uint16_t* sprPtr[2];
volatile uint8_t displayBuffer = 0;
volatile uint8_t drawBuffer = 1;
volatile BufferState bufferStates[2] = {{false, 0}, {false, 0}};

// Synchronization
SemaphoreHandle_t bufferMutex;
unsigned long lastDisplayTime = 0;

// Frame timing control
bool oddFrame = false;
bool oddFrameAdjusted = false;

// Drawing task
void drawGraphics(void* parameter) {
    while (true) {
        // Clear the drawing buffer
        spr[drawBuffer].fillSprite(TFT_BLACK);

        // Your drawing code here
        spr[drawBuffer].fillRect(random(TFT_WIDTH), random(TFT_HEIGHT), 20, 20, TFT_RED);
        spr[drawBuffer].drawLine(0, 0, TFT_WIDTH, TFT_HEIGHT, TFT_GREEN);

        // Signal that drawing is complete
        xSemaphoreTake(bufferMutex, portMAX_DELAY);
        bufferStates[drawBuffer].ready = true;
        bufferStates[drawBuffer].frameCount++;
        xSemaphoreGive(bufferMutex);

        // Wait until the current drawing buffer is displayed
        while (drawBuffer == displayBuffer) {
            vTaskDelay(1);
        }

        // Switch to the other buffer for drawing
        drawBuffer = (drawBuffer == 0) ? 1 : 0;
        
        vTaskDelay(1);
    }
}

void setup() {
    // Initialize display
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);

    // Create sprites and get pointers
    for (int i = 0; i < 2; i++) {
        spr[i].createSprite(TFT_WIDTH, TFT_HEIGHT);
        spr[i].fillSprite(TFT_BLACK);
        sprPtr[i] = (uint16_t*)spr[i].getPointer();
    }

    // Create synchronization primitive
    bufferMutex = xSemaphoreCreateMutex();

    // Initial display timing setup for even frames
    tft.startWrite();
    tft.writecommand(ST7735_FRMCTR1);
    tft.writedata(0x0b);
    tft.writedata(0x16);
    tft.writedata(0x3f);
    tft.endWrite();

    // Start drawing task
    xTaskCreatePinnedToCore(drawGraphics, "DrawGraphics", 4096, NULL, 1, NULL, 0);
}

void loop() {
    unsigned long currentTime = millis();

    // Check if it's time for the next frame
    if (currentTime - lastDisplayTime >= 1000 / DISPLAY_REFRESH_RATE) {
        lastDisplayTime = currentTime;

        tft.startWrite();

        // Set frame timing for even frames
        tft.writecommand(ST7735_FRMCTR1);
        tft.writedata(0x0b);
        tft.writedata(0x16);
        tft.writedata(0x3f);

        oddFrame = !oddFrame;
        oddFrameAdjusted = false;

        xSemaphoreTake(bufferMutex, portMAX_DELAY);
        bool bufferReady = bufferStates[drawBuffer].ready;
        xSemaphoreGive(bufferMutex);

        // Only swap and display if the drawing buffer is ready
        if (bufferReady) {
            // Wait for any ongoing DMA transfer
            while (tft.dmaBusy()) {
                yield();
            }

            // Push the current drawing buffer to display
            tft.pushImageDMA(0, 0, TFT_WIDTH, TFT_HEIGHT, sprPtr[drawBuffer]);

            // Update buffer states
            xSemaphoreTake(bufferMutex, portMAX_DELAY);
            bufferStates[drawBuffer].ready = false;
            displayBuffer = drawBuffer;
            xSemaphoreGive(bufferMutex);
        }

        tft.endWrite();
    }

    // Adjust frame timing for odd frames halfway through the frame time
    if (millis() - lastDisplayTime >= (1000 / DISPLAY_REFRESH_RATE) / 2) {
        if (oddFrame && !oddFrameAdjusted) {
            tft.startWrite();
            tft.writecommand(ST7735_FRMCTR1);
            tft.writedata(0x0a);
            tft.writedata(0x20);
            tft.writedata(0x3f);
            tft.endWrite();
            oddFrameAdjusted = true;
        }
    }

    // Other non-blocking tasks can be performed here
}
