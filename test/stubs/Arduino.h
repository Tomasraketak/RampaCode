// Minimal Arduino / arduino-pico API for running Rampa_MCU.ino on a PC (test only).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <deque>
#include <vector>
#include <functional>
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#include <math.h>
#include <string>
extern uint32_t g_ms;
extern uint32_t g_us_sub;          // sub-millisecond part (the test steps core 1 several times per ms)
extern int g_pin[32];
extern int g_adc;                  // GP26 reading
inline uint32_t millis() { return g_ms; }
inline uint32_t micros() { return g_ms * 1000u + g_us_sub; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t us) { g_us_sub += us; while (g_us_sub >= 1000) { g_us_sub -= 1000; g_ms++; } }   // busy waits pass time
inline int analogRead(int) { return g_adc; }
inline void analogReadResolution(int) {}
inline void pinMode(int, int) {}
extern void (*g_pin_hook)(int, int);
inline void digitalWrite(int p, int v) { g_pin[p] = v; if (g_pin_hook) g_pin_hook(p, v); }
inline int digitalRead(int p) { return g_pin[p]; }
inline void noInterrupts() {}
inline void interrupts() {}
struct StubPrint {
    bool quiet = true;
    std::string out;                 // everything written (the test reads it back)
    std::deque<char> in;             // host -> rampa
    void begin(unsigned long) {}
    explicit operator bool() const { return true; }
    int availableForWrite() { return 4096; }
    int available() { return (int)in.size(); }
    int read() { char c = in.front(); in.pop_front(); return c; }
    int printf(const char *f, ...) { char b[1200]; va_list a; va_start(a, f); int r = vsnprintf(b, sizeof b, f, a); va_end(a); out += b; if (!quiet) fputs(b, stdout); return r; }
    void print(const char *s) { out += s; if (!quiet) fputs(s, stdout); }
    void println(const char *s) { out += s; out += "\r\n"; if (!quiet) puts(s); }
    void host(const char *l) { while (*l) in.push_back(*l++); in.push_back('\n'); }
};
struct StubUart {
    std::deque<uint8_t> rx; std::vector<uint8_t> tx; bool on = false;
    void setTX(int) {} void setRX(int) {} void setFIFOSize(int) {}
    void begin(unsigned long) { on = true; rx.clear(); }
    void end() { on = false; }
    int available() { return on ? (int)rx.size() : 0; }
    int read() { int b = rx.front(); rx.pop_front(); return b; }
    int availableForWrite() { return 4096; }
    size_t write(const uint8_t *b, size_t n) { if (on) tx.insert(tx.end(), b, b + n); return n; }
};
extern StubPrint Serial;
extern StubUart Serial1;
