#include "Display.h"
#include <Arduino.h>
#include <AnimatedGIF.h>
#include <JPEGDecoder.h>     // JPEG decoder library
#include <WiFi.h>
#include <LittleFS.h>
#include "pinconfig.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_10.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_12.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_15.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_20.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_25.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_30.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_35.h"
#include "Fonts/DejaVu_LGC_Sans_Mono_40.h"

#define minimum(a,b) ((a)>(b)?(b):(a))
// Defs
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 160

String animationNowPlaying = "";
int lastSetBrightness = -1;
int brightnessTarget = 0;

void JITPOverlay();
void DisplayLoop();
void show_JPEG(const char *path, int x, int y);
void jpegRender(int xpos, int ypos);
void *GIFOpenFile(const char *fname, int32_t *pSize);
void GIFCloseFile(void *pHandle);
int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen);
int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition);
void GIFDraw(GIFDRAW *pDraw);
void screen_Home();
void drawProgressbar(int x, int y, int width, int height, int progress);
void drawStringCenter(const String &buf, int y, int size);
uint16_t Color16(uint8_t R, uint8_t G, uint8_t B);

// Screens
void DisplayHomeLoop();
void DisplayUpdateFWLoop();
bool DisplayAnimationLoop(String splashFilName, bool looping, void (*OverLay)() = 0);

bool SplashAnimIsDone = false;
int TFT_BRIGHTNESS = 255;
TaskHandle_t displayLoopTaskHandle = NULL;
Adafruit_ST7735 st_tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
void TFT_startWrite()
{
    st_tft.startWrite();
}
void TFT_setAddressWindow(int x, int y, int width, int height)
{
    st_tft.setAddrWindow(x, y, width, height);
}
void TFT_writePixels(uint16_t *colors, uint16_t count)
{
    st_tft.writePixels(colors, count, false, false);
}
void TFT_endWrite()
{
    st_tft.endWrite();
}
BufferedDisplay display(st_tft, TFT_startWrite, TFT_setAddressWindow, TFT_writePixels, TFT_endWrite);
AnimatedGIF gif;
void DisplaySetup()
{
#if defined(DebugDisplay)
    Serial.print("Display setup ");
#endif
    st_tft.initR(INITR_BLACKTAB);
    display.fillScreen(ST77XX_BLACK);
    gif.begin(LITTLE_ENDIAN_PIXELS);

    // Clear screen before turning on backlight
    display.fillScreen(ST7735_WHITE);
    display.update();
    // Brightness
    setLCDBacklight(TFT_BRIGHTNESS);
    ledcAttachPin(TFT_LED, LCD_CHANNEL);
    ledcSetup(LCD_CHANNEL, LCD_FREQ, LCD_RESOLUTION);
    ledcWrite(LCD_CHANNEL, 255);
    // Create loop task
    // xTaskCreatePinnedToCore(
    //     DisplayLoop,            /* Function to implement the task */
    //     "DispLoop",             /* Name of the task */
    //     80000,                   /* Stack size in words */
    //     NULL,                   /* Task input parameter */
    //     0,                      /* Priority of the task */
    //     &displayLoopTaskHandle, /* Task handle. */
    //     0);                     /* Core where the task should run */
#if defined(DebugDisplay)
    Serial.println("done");
#endif
}

#if defined(MultiThreadDisplayLoop)
void DisplayLoopInf(void *parameter)
{
    while (1)
    {
        DisplayLoop();
        delay(40);
    }
}
#endif
void DisplayLoop()
{
    if (brightnessTarget != lastSetBrightness)
    {
        if (brightnessTarget > lastSetBrightness)
        {
            lastSetBrightness += 10;
            if (lastSetBrightness > brightnessTarget)
                lastSetBrightness = brightnessTarget;
        }
        else
        {
            lastSetBrightness -= 10;
            if (lastSetBrightness < brightnessTarget)
                lastSetBrightness = brightnessTarget;
        }
#if defined(DebugDisplay)
        Serial.println(lastSetBrightness);
#endif
        float toSet = 255 - pow(sqrt(255 - lastSetBrightness) - 15, 2);
        if (toSet < 0)
            toSet = 0;
        else if (toSet > 255)
            toSet = 255;
        ledcWrite(LCD_CHANNEL, (int)toSet);
#if defined(DebugDisplay)
        Serial.print("Set brightness: ");
        Serial.print(lastSetBrightness);
        Serial.print(", Write: ");
        Serial.println((int)toSet);
#endif
    }
#if defined(DebugDisplay)
// Serial.print("DataToDisplay.CurrentScreenMode: ");
// Serial.println(DataToDisplay.CurrentScreenMode);
#endif
}

void PlaySplashAnimation()
{
    SplashAnimIsDone = false;
    while (!SplashAnimIsDone)
    {
        DisplayLoop();
        delay(1);
    }
    display.setTextColor(ST7735_BLACK);
    display.setFont(&DejaVu_LGC_Sans_Mono_10);
    display.setCursor(22, 140);
    display.print("VERSION " + String(FRIMWARE_VERSION));
    display.update();
}
/* SCREENS defs */
bool DisplayAnimationLoop(String animationFileName, bool looping, void (*OverLay)())
{
#if defined(DebugDisplay)
    //Serial.println("DisplayAnimationLoop");
#endif
    if (animationNowPlaying == animationFileName + "_end")
    { // do nothing
#if defined(DebugDisplay)
        // Serial.println("Do Nothing");
#endif
        return true;
    }
    else if (animationNowPlaying == "")
    {
#if defined(DebugDisplay)
        Serial.println("Open gif");
#endif
        animationNowPlaying = animationFileName;
        // No anim playing. Start now
        if (gif.open(animationNowPlaying.c_str(), GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw))
        {

            GIFINFO gi;
#if defined(DebugDisplay)
            Serial.printf("Successfully opened GIF; Canvas size = %d x %d\n", gif.getCanvasWidth(), gif.getCanvasHeight());
#endif
            if (gif.getInfo(&gi))
            {
#if defined(DebugDisplay)
                Serial.printf("frame count: %d\n", gi.iFrameCount);
                Serial.printf("duration: %d ms\n", gi.iDuration);
                Serial.printf("max delay: %d ms\n", gi.iMaxDelay);
                Serial.printf("min delay: %d ms\n", gi.iMinDelay);
#endif
            }
        }
        else
        {
            animationNowPlaying = animationFileName + "_end";

#if defined(DebugDisplay)
            Serial.printf("Error opening animation file. Error %d\n", gif.getLastError());
#endif
        }
    }
    else if (animationNowPlaying == animationFileName)
    { // play frames
        // Serial.println("play frames");
        if (!gif.playFrame(true, NULL))
        {
            // end of animation. Check if we need to loop
            if (looping)
            {
                // animationNowPlaying = "";
                gif.reset();
#if defined(DebugDisplay)
                Serial.println("Reset anim loop");
#endif
            }
            else
            {
                gif.close();
                animationNowPlaying = animationFileName + "_end";
#if defined(DebugDisplay)
                Serial.println("Anim End");
#endif
            }
        }
        else
        {
            if (OverLay)
                OverLay();
            display.update();
#if defined(DebugDisplay)
            // Serial.println("DrawFrame");
#endif
        }
    }
    else
    {
#if defined(DebugDisplay)
        Serial.println("Abort other anim");
#endif
        // Some other anim is playing. Stop it first
        gif.close();
        animationNowPlaying = ""; // for the next loop
    }
    return false;
}
// GIF Externals
File gifFile;
/* ANIMATION */
void *GIFOpenFile(const char *fname, int32_t *pSize)
{
#if defined(DebugDisplay)
    Serial.print("GIFOpenFile: ");
    Serial.print(fname);
    Serial.print(" > ");
#endif
    gifFile = LittleFS.open(fname);
    if (gifFile)
    {
#if defined(DebugDisplay)
        Serial.println("opened");
#endif
        *pSize = gifFile.size();
        return (void *)&gifFile;
    }

#if defined(DebugDisplay)
    Serial.println("failed");
#endif
    return NULL;
} /* GIFOpenFile() */

void GIFCloseFile(void *pHandle)
{
    File *f = static_cast<File *>(pHandle);
    if (f != NULL)
        f->close();
} /* GIFCloseFile() */

int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    int32_t iBytesRead;
    iBytesRead = iLen;
    File *f = static_cast<File *>(pFile->fHandle);
    // Note: If you read a file all the way to the last byte, seek() stops working
    if ((pFile->iSize - pFile->iPos) < iLen)
        iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
    if (iBytesRead <= 0)
        return 0;
    iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
    pFile->iPos = f->position();
    return iBytesRead;
} /* GIFReadFile() */

int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition)
{
    int i = micros();
    File *f = static_cast<File *>(pFile->fHandle);
    f->seek(iPosition);
    pFile->iPos = (int32_t)f->position();
    i = micros() - i;
    //  Serial.printf("Seek time = %d us\n", i);
    return pFile->iPos;
} /* GIFSeekFile() */

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s;
    uint16_t *d, *usPalette, usTemp[320];
    int x, y, iWidth;

    iWidth = pDraw->iWidth;
    if (iWidth + pDraw->iX > DISPLAY_WIDTH)
        iWidth = DISPLAY_WIDTH - pDraw->iX;
    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line
    if (y >= DISPLAY_HEIGHT || pDraw->iX >= DISPLAY_WIDTH || iWidth < 1)
        return;
    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
        for (x = 0; x < iWidth; x++)
        {
            if (s[x] == pDraw->ucTransparent)
                s[x] = pDraw->ucBackground;
        }
        pDraw->ucHasTransparency = 0;
    }

    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) // if transparency used
    {
        uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
        int x, iCount;
        pEnd = s + iWidth;
        x = 0;
        iCount = 0; // count non-transparent pixels
        while (x < iWidth)
        {
            c = ucTransparent - 1;
            d = usTemp;
            while (c != ucTransparent && s < pEnd)
            {
                c = *s++;
                if (c == ucTransparent) // done, stop
                {
                    s--; // back up to treat it like transparent
                }
                else // opaque
                {
                    *d++ = usPalette[c];
                    iCount++;
                }
            }           // while looking for opaque pixels
            if (iCount) // any opaque pixels?
            {
                // tft.setAddrWindow(pDraw->iX + x, y, iCount, 1);
                // tft.writePixels(usTemp, iCount, false, false);
                for (int i = 0; i < iCount; i++)
                    display.drawPixel(pDraw->iX + x + i, y, usTemp[i]);
                x += iCount;
                iCount = 0;
            }
            // no, look for a run of transparent pixels
            c = ucTransparent;
            while (c == ucTransparent && s < pEnd)
            {
                c = *s++;
                if (c == ucTransparent)
                    iCount++;
                else
                    s--;
            }
            if (iCount)
            {
                x += iCount; // skip these
                iCount = 0;
            }
        }
    }
    else
    {
        s = pDraw->pPixels;
        // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
        for (x = 0; x < iWidth; x++)
            usTemp[x] = usPalette[*s++];
        // tft.setAddrWindow(pDraw->iX, y, iWidth, 1);
        // tft.writePixels(usTemp, iWidth, false, false);
        for (int i = 0; i < iWidth; i++)
            display.drawPixel(pDraw->iX + i, y, usTemp[i]);
    }
} /* GIFDraw() */

/* ANIMATIONS, GRAPHICS, TFT */
// Set LCD Brightness
void setLCDBacklight(int x)
{
    brightnessTarget = x;
}
// Draw Progress Bar
void drawProgressbar(int x, int y, int width, int height, int progress)
{
    display.fillRect(x, y, width, height, ST7735_BLACK);

    progress = progress > 100 ? 100 : progress;
    progress = progress < 0 ? 0 : progress;

    double bar = ((double)(width - 1) / 100) * progress;

    display.drawRect(x, y, width, height, ST7735_WHITE);
    display.fillRect(x + 2, y + 2, bar, height - 4, ST7735_WHITE);

    // Display progress text
    if (height >= 15)
    {
        display.setCursor((width / 2) - 3, y + 11);
        display.setTextSize(1);
        display.setTextColor(ST7735_WHITE);
        if (progress >= 50)
        {
            display.setTextColor(ST7735_BLACK, ST7735_WHITE); // 'inverted' text
        }
        display.print(progress);
        display.print("%");
    }
}

// Clear Display
void clearDisplay()
{
    // Display
    // tft.initR(INITR_BLACKTAB);
    display.fillScreen(ST77XX_BLACK);
}

// Draw String Center of Screen
void drawStringCenter(const String &buf, int y, int size)
{
    const int SCREEN_WIDTH = 128;
    display.fillRect(SCREEN_WIDTH, y, 128, 20, ST7735_BLACK);
    int str_len = strlen(buf.c_str()) * 6;
    int x = (SCREEN_WIDTH - str_len) / 2;

    display.setCursor(x, y);
    display.setTextSize(size);
    display.setTextColor(ST7735_WHITE, ST7735_BLACK);
    display.print(buf);
}

// Draw String Center of Screen
void drawStringCenterBlack(const String &buf, int y)
{
    const int SCREEN_WIDTH = 128;
    display.fillRect(SCREEN_WIDTH, y, 128, 20, ST7735_BLACK);
    int str_len = strlen(buf.c_str()) * 6;
    int x = (SCREEN_WIDTH - str_len) / 2;

    display.setCursor(x, y);
    display.setTextSize(1);
    display.setTextColor(ST7735_BLACK, ST7735_WHITE);
    display.print(buf);
}

// Show JPEG
void show_JPEG(const char *path, int x, int y)
{
    // open the image file
    File jpgFile = LittleFS.open(path, FILE_READ);
    if (!jpgFile)
    {

#if defined(DebugDisplay)
        Serial.print("Could not open JPEG file: ");
        Serial.println(path);
#endif
    }
    // initialise the decoder to give access to image information
    JpegDec.decodeSdFile(jpgFile);

    // render the image onto the screen at coordinate 0,0
    jpegRender(x, y);

    jpgFile.close();
}

// Render JPEG
void jpegRender(int xpos, int ypos)
{
    uint16_t *pImg;
    uint16_t mcu_w = JpegDec.MCUWidth;
    uint16_t mcu_h = JpegDec.MCUHeight;
    uint32_t max_x = JpegDec.width;
    uint32_t max_y = JpegDec.height;
    uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
    uint32_t min_h = minimum(mcu_h, max_y % mcu_h);
    uint32_t win_w = mcu_w;
    uint32_t win_h = mcu_h;
    max_x += xpos;
    max_y += ypos;
    while (JpegDec.read())
    {
        pImg = JpegDec.pImage;
        int mcu_x = JpegDec.MCUx * mcu_w + xpos;
        int mcu_y = JpegDec.MCUy * mcu_h + ypos;
        if (mcu_x + mcu_w <= max_x)
            win_w = mcu_w;
        else
            win_w = min_w;
        if (mcu_y + mcu_h <= max_y)
            win_h = mcu_h;
        else
            win_h = min_h;

        if (win_w != mcu_w)
        {
            for (int h = 1; h < win_h - 1; h++)
            {
                memcpy(pImg + h * win_w, pImg + (h + 1) * mcu_w, win_w << 1);
            }
        }

        if ((mcu_x + win_w) <= display.width() && (mcu_y + win_h) <= display.height())
        {
            display.drawRGBBitmap(mcu_x, mcu_y, pImg, win_w, win_h);
        }
        else if ((mcu_y + win_h) >= display.height())
            JpegDec.abort();
    }
}

/* REFERENCE */

/* PUB SUB */
/*
if (aws_retries == 0) {
Serial.println("Setting AWS Parameters");
// Configure WiFiClientSecure to use the AWS IoT device credentials
net.setCACert(AWS_ROOT_CA);
net.setCertificate(aws_cert.c_str());
net.setPrivateKey(aws_key.c_str());

// Connect to the MQTT broker on the AWS endpoint we defined earlier
client.setServer(AWS_IOT_ENDPOINT, 8883);

aws_retries++;
return;
}

Serial.print("Device Name ");
Serial.println(device_name.c_str());

//Attempt Connection
if (!client.connect(device_name.c_str()) && aws_retries < 4) {
    Serial.print("Attempting Wifi Connection: ");
    Serial.println(aws_retries);
    aws_retries++;
    return;
}

if (!client.connected()) {
Serial.println("AWS IoT Timeout!");
return;
}
*/
uint16_t Color16(uint8_t R, uint8_t G, uint8_t B)
{
    return (((((uint32_t)R * 0b11111) / 0xFF) % 0b11111) << 11) |
           (((((uint32_t)G * 0b111111) / 0xFF) % 0b111111) << 5) |
           (((((uint32_t)B * 0b11111) / 0xFF) % 0b11111) << 0);
}