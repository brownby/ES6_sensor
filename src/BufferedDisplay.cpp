#include "BufferedDisplay.h"
void (*HW_startWrite)() = 0;
void (*HW_setAddressWindow)(int x, int y, int width, int height) = 0;
void (*HW_writePixels)(uint16_t *colors, uint16_t count) = 0;
void (*HW_endWrite)() = 0;

#define _swap_(a, b) \
    (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

uint16_t *screenBuffer;

BufferedDisplay::BufferedDisplay(
    Adafruit_GFX &hw,
    void (*startWrite)(),
    void (*setAddressWindow)(int x, int y, int width, int height),
    void (*writePixels)(uint16_t *colors, uint16_t count),
    void (*endWrite)()) : Adafruit_GFX(hw.width(), hw.height())
{
    this->HW = &hw;
    HW_startWrite = startWrite;
    HW_setAddressWindow = setAddressWindow;
    HW_writePixels = writePixels;
    HW_endWrite = endWrite;
    screenBuffer = new uint16_t[hw.width() * hw.height()];
}

void BufferedDisplay::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    /*drawPixelFunc(x, y, color);
}
void Adafruit_TFTLCD_ESP32::drawPixelFunc_Actual(int16_t x, int16_t y, uint16_t color) {*/
    // Serial.printf("DrawPx(%d, %d, %d)\n", x, y, color);
    if ((x >= 0) && (x < width()) && (y >= 0) && (y < height()))
    {
        // Pixel is in-bounds. Rotate coordinates if needed.
        switch (getRotation())
        {
        case 1:
            _swap_(x, y);
            x = WIDTH - x - 1;
            break;
        case 2:
            x = WIDTH - x - 1;
            y = HEIGHT - y - 1;
            break;
        case 3:
            _swap_(x, y);
            y = HEIGHT - y - 1;
            break;
        }
        if (screenBuffer[x + y * WIDTH] != color)
        {
            // Serial.printf("screenBuffer[%d] = %d\n", x + y * WIDTH, color);
            screenBuffer[x + y * WIDTH] = color; // we never give 16 bit colors to this functions
            updateRequired = true;

            if (x < updateX0)
            {
                updateX0 = x;
                // Serial.printf("updateX0 set to %d\n", updateX0);
            }
            if (x > updateX1)
            {
                updateX1 = x;
                // Serial.printf("updateX1 set to %d\n", updateX1);
            }
            if (y < updateY0)
            {
                updateY0 = y;
                // Serial.printf("updateY0 set to %d\n", updateY0);
            }
            if (y > updateY1)
            {
                updateY1 = y;
                // Serial.printf("updateY1 set to %d\n", updateY1);
            }
        }
    }
}
// Because this function is used infrequently, it configures the ports for
// the read operation, reads the data, then restores the ports to the write
// configuration.  Write operations happen a LOT, so it's advantageous to
// leave the ports in that state as a default.
uint8_t BufferedDisplay::readPixel(int16_t x, int16_t y)
{
    if ((x >= 0) && (x < width()) && (y >= 0) && (y < height()))
    {
        // Pixel is in-bounds. Rotate coordinates if needed.
        switch (getRotation())
        {
        case 1:
            _swap_(x, y);
            x = WIDTH - x - 1;
            break;
        case 2:
            x = WIDTH - x - 1;
            y = HEIGHT - y - 1;
            break;
        case 3:
            _swap_(x, y);
            y = HEIGHT - y - 1;
            break;
        }
        return screenBuffer[x + y * WIDTH];
    }
    return 0;
}
void BufferedDisplay::clearDisplay(uint16_t color){
    fillScreen(color);
}
void BufferedDisplay::display(bool forceFullWidth, bool forceFullHeight){
    update(forceFullWidth, forceFullHeight);
}
void BufferedDisplay::update(bool forceFullWidth, bool forceFullHeight)
{
    long st = millis();
    InUpdate = true;
    if (forceFullWidth)
    {
        //Serial.println("Force full width");
        updateX0 = 0;
        updateX1 = HW->width() - 1;
    }
    if (forceFullHeight)
    {
        //Serial.println("Force full Height");
        updateY0 = 0;
        updateY1 = HW->height() - 1;
    }
    if (updateX1 < updateX0 || updateY1 < updateY0)
    {
        //Serial.println("Nothing to update");
        return;
    }
    int updateX0 = this->updateX0;
    int updateX1 = this->updateX1;
    int updateY0 = this->updateY0;
    int updateY1 = this->updateY1;
    int w = updateX1 - updateX0 + 1;
    int h = updateY1 - updateY0 + 1;
    //Serial.printf("Update (%d, %d, %d, %d)\n", updateX0, updateY0, w, h);
    if (w == HW->width()) {// Dump whole color buffer at once
        //Serial.println("Update full width");
        HW_startWrite();
        HW_setAddressWindow(0, updateY0, w, h);
        HW_writePixels(screenBuffer + w * updateY0, w * h);
        HW_endWrite();
    }
    else {
        //Serial.println("Update partial width");
        for (int yi = 0; yi < h; yi++)
        {
            //Serial.printf("Update line (%d, %d, %d, %d)\n", updateX0, updateY0 + yi, w, 1);
            HW_startWrite();
            HW_setAddressWindow(updateX0, updateY0 + yi, w, 1);
            HW_writePixels(screenBuffer + HW->width() * (updateY0 + yi) + updateX0, w);
            HW_endWrite();
        }
    }
    this->updateX0 = HW->width();
    this->updateX1 = -1;
    this->updateY0 = HW->height();
    this->updateY1 = -1;
    InUpdate = false;
    //Serial.print("update took: ");
    //Serial.println(millis() - st);
}