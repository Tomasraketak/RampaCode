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
extern uint32_t g_ms;
extern int g_pin[32];
inline uint32_t millis() { return g_ms; }
inline uint32_t micros() { return g_ms * 1000u; }
inline void delay(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { g_pin[p] = v; }
inline int digitalRead(int p) { return g_pin[p]; }
inline void noInterrupts() {}
inline void interrupts() {}
struct StubPrint {
    bool quiet = true;
    void begin(unsigned long) {}
    int printf(const char *f, ...) { if (quiet) return 0; va_list a; va_start(a, f); int r = vprintf(f, a); va_end(a); return r; }
    void println(const char *s) { if (!quiet) puts(s); }
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
