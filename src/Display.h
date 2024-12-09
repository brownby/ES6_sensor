#pragma once
#include <Arduino.h>
#include <Adafruit_ST7735.h> //Hardware-specific library for ST7735
#include "BufferedDisplay.h"
//#define MultiThreadDisplayLoop

// Headers for the main logiv
void DisplaySetup();
void setLCDBacklight(int x = 0);
void DisplayLoop();
#if defined(MultiThreadDisplayLoop)
void DisplayLoopInf(void * parameter );
#endif
void PlaySplashAnimation();
extern BufferedDisplay display;