#pragma once
#include <Adafruit_GFX.h>
#define ForceFullWidthUpdate

class BufferedDisplay : public Adafruit_GFX
{
private:
    Adafruit_GFX *HW;
    bool InUpdate = false;
    bool updateRequired = false;
    int updateX0 = 1000, updateX1 = -1, updateY0 = 1000, updateY1 = -1;
public:
    BufferedDisplay(
        Adafruit_GFX &hw,
        void (*startWrite)(),
        void (*setAddressWindow)(int x, int y, int width, int height),
        void (*writePixels)(uint16_t *colors, uint16_t count),
        void (*endWrite)());
    void drawPixel(int16_t x, int16_t y, uint16_t color) override;
    uint8_t readPixel(int16_t x, int16_t y);
    void update(bool forceFullWidth = true, bool forceFullHeight = false);
    void display(bool forceFullWidth = true, bool forceFullHeight = false);
    void clearDisplay(uint16_t color = 0);
};