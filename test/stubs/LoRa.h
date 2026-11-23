#pragma once
#include "SPI.h"
#include <deque>
#include <vector>
struct StubLoRaPkt { std::vector<uint8_t> b; uint32_t t_ms; };
struct StubLoRa {
    int txc = 0;
    std::deque<std::vector<uint8_t>> rx;     // heard on the next parsePacket()
    std::vector<StubLoRaPkt> tx;             // transmitted
    std::vector<uint8_t> cur, txbuf; size_t pos = 0;
    void setSPI(StubSPI &) {} void setPins(int, int, int) {}
    int begin(long) { return 1; } void end() {}
    void setSpreadingFactor(int) {} void setSignalBandwidth(long) {} void setCodingRate4(int) {}
    void setPreambleLength(int) {} void setSyncWord(int) {} void setTxPower(int) {} void enableCrc() {}
    void idle() {} void receive() {}
    int parsePacket() { if (rx.empty()) return 0; cur = rx.front(); rx.pop_front(); pos = 0; return (int)cur.size(); }
    int available() { return (int)(cur.size() - pos); }
    int read() { return pos < cur.size() ? cur[pos++] : -1; }
    int packetRssi() { return -60; }
    int beginPacket() { txbuf.clear(); return 1; }
    size_t write(const uint8_t *b, size_t n) { txbuf.insert(txbuf.end(), b, b + n); return n; }
    int endPacket(bool = false);
};
extern StubLoRa LoRa;
