// Simulated SPI0 devices of the rampa PCB for the host test:
//   ADS1220 (CS GP15, DRDY GP21) with a load cell, two NTC thermistors,
//   W25Q128JV (CS GP5) with real erase/program semantics and busy times,
//   and the pressure transducer on GP26.
#pragma once
#include <vector>

// ---------------------------------------------------------------- physical world
static double g_force_n = 0.0;        // force on the load cell
static double g_psi = 0.0;            // chamber pressure
static double g_t1_c = 25.0, g_t2_c = 40.0;
static bool   g_ads_present = true, g_flash_present = true;
static const double SIM_LC_OFFSET = 20000.0;   // untared code
static const double SIM_CODES_PER_N = 850.0;   // ~0.8 mV/V at 100 N

static uint64_t sim_us(void) { return (uint64_t)g_ms * 1000u + g_us_sub; }

static int32_t sim_ntc_code(double tc)
{
    const double r = 10000.0 * exp(3950.0 * (1.0 / (tc + 273.15) - 1.0 / 298.15));
    const double ratio = 10000.0 / (r + 10000.0);
    return (int32_t)(ratio * 8388608.0);
}

// ---------------------------------------------------------------- ADS1220
static struct {
    uint8_t reg[4]; bool running; uint64_t next_us; int idx; uint8_t cmd; uint8_t out[4]; int nout; int32_t sample;
} ads;

static uint32_t ads_period_us(void) { return (ads.reg[1] & 0x10) ? 500u : 1000u; }   // turbo 2000 / normal 1000

static void ads_restart(void) { ads.next_us = sim_us() + ads_period_us(); }

static int32_t ads_value(void)
{
    const uint8_t mux = ads.reg[0] >> 4;
    if (mux == 0x6) return (int32_t)(SIM_LC_OFFSET + g_force_n * SIM_CODES_PER_N + ((g_ms * 7919u) % 11) - 5);
    if (mux == 0xA) return sim_ntc_code(g_t1_c);
    if (mux == 0xB) return sim_ntc_code(g_t2_c);
    return 0;
}

static uint8_t ads_xfer(uint8_t b)
{
    if (!g_ads_present) return 0xFF;
    uint8_t r = 0;
    if (ads.idx == 0) {
        ads.cmd = b;
        if (b == 0x06) { memset(ads.reg, 0, 4); ads.running = false; }
        else if (b == 0x08) { ads.running = true; ads_restart(); }
        else if (b == 0x10) {
            const int32_t v = ads.sample;
            ads.out[0] = (uint8_t)(v >> 16); ads.out[1] = (uint8_t)(v >> 8); ads.out[2] = (uint8_t)v; ads.nout = 0;
            // conversion consumed: next one a period after the previous
            uint64_t n = ads.next_us + ads_period_us();
            if (n < sim_us()) n = sim_us() + ads_period_us();
            ads.next_us = n;
        }
    } else {
        if ((ads.cmd & 0xF0) == 0x40 && ads.idx == 1) { ads.reg[(ads.cmd >> 2) & 3] = b; if (ads.running) ads_restart(); }
        else if ((ads.cmd & 0xF0) == 0x20 && ads.idx == 1) r = ads.reg[(ads.cmd >> 2) & 3];
        else if (ads.cmd == 0x10 && ads.nout < 3) r = ads.out[ads.nout++];
    }
    ads.idx++;
    return r;
}

// ---------------------------------------------------------------- W25Q128
static std::vector<uint8_t> g_flash(16u * 1024u * 1024u, 0xFF);
static struct { int idx; uint8_t cmd; uint32_t addr; bool wel; uint64_t busy_until; uint32_t progs, erases; } fl;

static uint8_t fl_xfer(uint8_t b)
{
    if (!g_flash_present) return 0xFF;
    uint8_t r = 0;
    const bool busy = sim_us() < fl.busy_until;
    if (fl.idx == 0) {
        fl.cmd = b; fl.addr = 0;
        if (b == 0x06 && !busy) fl.wel = true;
    } else if (fl.cmd == 0x9F) {
        static const uint8_t id[3] = { 0xEF, 0x40, 0x18 };
        r = fl.idx <= 3 ? id[fl.idx - 1] : 0;
    } else if (fl.cmd == 0x05) {
        r = (uint8_t)((busy ? 1 : 0) | (fl.wel ? 2 : 0));
    } else if (fl.idx <= 3) {
        fl.addr = (fl.addr << 8) | b;
        if (fl.idx == 3 && !busy && fl.wel) {
            if (fl.cmd == 0xD8) { const uint32_t a = fl.addr & ~0xFFFFu; memset(&g_flash[a], 0xFF, 0x10000); fl.busy_until = sim_us() + 20000; fl.wel = false; fl.erases++; }
            if (fl.cmd == 0x20) { const uint32_t a = fl.addr & ~0xFFFu; memset(&g_flash[a], 0xFF, 0x1000); fl.busy_until = sim_us() + 5000; fl.wel = false; }
        }
    } else {
        if (fl.cmd == 0x03) { r = busy ? 0xFF : g_flash[fl.addr % g_flash.size()]; fl.addr++; }
        else if (fl.cmd == 0x02 && fl.wel && !busy) {
            const uint32_t a = (fl.addr & ~0xFFu) | ((fl.addr + (uint32_t)(fl.idx - 4)) & 0xFFu);
            g_flash[a] &= b;
        }
    }
    fl.idx++;
    return r;
}

static void sim_pin_hook(int p, int v)
{
    if (p == 15 && v == LOW) ads.idx = 0;
    if (p == 5) {
        if (v == LOW) fl.idx = 0;
        else if (fl.cmd == 0x02 && fl.idx > 4 && fl.wel) { fl.busy_until = sim_us() + 700; fl.wel = false; fl.progs++; }
    }
}
static uint8_t sim_spi(uint8_t b)
{
    if (g_pin[15] == LOW) return ads_xfer(b);
    if (g_pin[5] == LOW) return fl_xfer(b);
    return 0xFF;
}

static void sim_step(void)
{
    // DRDY low while a conversion result is waiting
    if (g_ads_present && ads.running && sim_us() >= ads.next_us) {
        if (g_pin[21] != LOW) ads.sample = ads_value();
        g_pin[21] = LOW;
    } else g_pin[21] = HIGH;
    // pressure transducer -> R46/R47 -> 12-bit ADC (3.3 V)
    const double v = 5.0 * (0.1 + 0.8 * g_psi / 500.0);
    g_adc = (int)(v / 1.47 / 3.3 * 4095.0 + 0.5);
}

static void sim_reset(void)
{
    memset(&ads, 0, sizeof ads);
    memset(&fl, 0, sizeof fl);
    g_force_n = 0; g_psi = 0; g_t1_c = 25.0; g_t2_c = 40.0;
    g_ads_present = g_flash_present = true;
    g_pin[15] = g_pin[5] = HIGH;
    g_pin_hook = sim_pin_hook;
    SPI.xfer = sim_spi;
}
