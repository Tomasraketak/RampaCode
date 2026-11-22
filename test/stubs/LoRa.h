#pragma once
#include "SPI.h"
struct StubLoRa {
    int txc = 0;
    void setSPI(StubSPI &) {} void setPins(int, int, int) {}
    int begin(long) { return 1; } void end() {}
    void setSpreadingFactor(int) {} void setSignalBandwidth(long) {} void setCodingRate4(int) {}
    void setPreambleLength(int) {} void setSyncWord(int) {} void setTxPower(int) {} void enableCrc() {}
    void idle() {} void receive() {} int parsePacket() { return 0; } int available() { return 0; } int read() { return 0; }
    int beginPacket() { return 1; } size_t write(const uint8_t *, size_t n) { return n; } int endPacket(bool) { txc++; return 1; }
};
extern StubLoRa LoRa;
