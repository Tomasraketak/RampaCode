#pragma once
#include "Arduino.h"
// Wire  = umbilical slave (driven by the test as the Ariadna I2C master)
// Wire1 = internal bus with simulated TCA9534 / INA226 / ADS1115
struct TwoWire {
    bool internal = false;
    // slave side
    bool slave_on = false; uint8_t slave_addr = 0;
    void (*on_rx)(int) = nullptr; void (*on_req)() = nullptr;
    std::deque<uint8_t> srx; std::vector<uint8_t> stx;
    // master side (internal bus)
    uint8_t cur = 0, reg = 0; std::vector<uint8_t> wbuf; std::deque<uint8_t> rbuf;
    explicit TwoWire(bool i) : internal(i) {}
    void setSDA(int) {} void setSCL(int) {} void setClock(uint32_t) {} void setTimeout(int) {}
    void begin() {}
    void begin(uint8_t a) { slave_on = true; slave_addr = a; }
    void end() { slave_on = false; }
    void onReceive(void (*f)(int)) { on_rx = f; }
    void onRequest(void (*f)()) { on_req = f; }
    void beginTransmission(uint8_t a) { cur = a; wbuf.clear(); }
    size_t write(uint8_t b) { if (internal) wbuf.push_back(b); else stx.push_back(b); return 1; }
    size_t write(const uint8_t *b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
    uint8_t endTransmission(bool = true);
    int requestFrom(int a, int n);
    int available() { return internal ? (int)rbuf.size() : (int)srx.size(); }
    int read() { std::deque<uint8_t> &q = internal ? rbuf : srx; int b = q.front(); q.pop_front(); return b; }
};
extern TwoWire Wire, Wire1;
