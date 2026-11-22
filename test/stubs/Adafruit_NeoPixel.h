#pragma once
#include <stdint.h>
#define NEO_GRB 0
#define NEO_KHZ800 0
struct Adafruit_NeoPixel {
    Adafruit_NeoPixel(int, int, int) {}
    void begin() {} void setBrightness(int) {} void show() {}
    void setPixelColor(int, uint32_t) {}
    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }
};
