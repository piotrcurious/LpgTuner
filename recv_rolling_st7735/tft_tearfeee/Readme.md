Yes. The code avoids tearing by combining a double-buffering approach with careful timing adjustments to the display’s refresh rate. Here’s how these techniques work together:

1. Double Buffering:
The code uses two sprite buffers. One buffer is actively drawn to by a background task while the other buffer is sent to the display using DMA. By swapping only when the drawing is fully complete, the code ensures that the display never shows a partially rendered frame. This separation is a primary method for avoiding tearing.


2. Timing Synchronization with Frame Refresh Commands:
The code sends specific commands (using ST7735_FRMCTR1) to set the display’s refresh parameters. At the beginning of each frame, it writes a set of parameters (e.g., 0x0b, 0x16, 0x3f) to define the timing for even frames. Then, halfway through the frame period, if it’s an odd frame, it adjusts the timing by writing a different set of parameters (e.g., 0x0a, 0x20, 0x3f).

This mid-frame adjustment effectively shifts the speed at which the display refreshes. By synchronizing this adjustment with the buffer swapping and the DMA transfer completion, the display is “instructed” to update its visible content only after the new frame is fully rendered. This coordination helps to ensure that the refresh happens during a period when the content is stable, avoiding any visual tearing.


3. Waiting for DMA Completion:
Before pushing a new frame to the display, the code waits for any ongoing DMA transfer to finish (using while (tft.dmaBusy()) { yield(); }). This ensures that the frame update only begins once the previous frame has been completely transferred, which further prevents tearing that could occur if updates overlapped.



In summary:
The combination of double buffering (which isolates the drawing process from the display process) and the shifting of display refresh timing (through specific command sequences that adjust when the display refreshes) ensures that each new frame is fully rendered and properly synchronized with the display update cycle. This coordinated approach prevents the display from showing partially updated frames, thereby avoiding tearing.
