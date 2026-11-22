#pragma once
struct StubSPI { void setSCK(int) {} void setTX(int) {} void setRX(int) {} void end() {} void begin() {} };
extern StubSPI SPI1;
