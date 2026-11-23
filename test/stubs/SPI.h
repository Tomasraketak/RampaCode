#pragma once
#include <stdint.h>
#define MSBFIRST 1
#define SPI_MODE0 0
#define SPI_MODE1 1
struct SPISettings { uint32_t clk; int order, mode; SPISettings(uint32_t c = 1000000, int o = MSBFIRST, int m = SPI_MODE0) : clk(c), order(o), mode(m) {} };
// SPI1 = LoRa (unused by the test), SPI = SPI0 with the simulated ADS1220 + W25Q128
struct StubSPI {
    uint8_t (*xfer)(uint8_t) = nullptr;   // device model installed by the test
    int mode = 0;
    void setSCK(int) {} void setTX(int) {} void setRX(int) {} void end() {} void begin() {}
    void beginTransaction(const SPISettings &s) { mode = s.mode; }
    void endTransaction() {}
    uint8_t transfer(uint8_t b) { return xfer ? xfer(b) : 0xFF; }
};
extern StubSPI SPI1, SPI;
