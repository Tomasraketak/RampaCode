// =====================================================================
//  thrust_stand.h — measurement + recording engine of the rampa
//                   (runs on CORE 1 of the RP2350, owns SPI0)
//
//  Hardware (RampaNetlist_PCB1, hardware/RampaPCBnet.enet):
//    SPI0  GP2 SCK, GP3 MOSI, GP4 MISO, shared by
//      U14 ADS1220   CS GP15 (R59 PU), DRDY GP21, AVDD = REFP0 = 5V_SENS
//            AIN1-AIN0  load cell J8 (LC_SP / LC_SN, 1k + 10n/1n RC)
//            AIN2       thermistor 1 J10 (NTC from 5V_SENS, R48 10k to GND)
//            AIN3       thermistor 2 J11 (NTC from 5V_SENS, R49 10k to GND)
//      U9  W25Q128JV CS GP5 (R17 PU), 16 MB
//    GP26  ADC0  pressure transducer J9 (0.5-4.5 V at 5 V, ratiometric)
//          via R46 4k7 / R47 10k (x 0.680), clamped D6A/D6B
//    5V_SENS (U5 LP2985-5.0) is the load cell excitation, the ADS1220
//          reference and the transducer supply; core 0 measures it on the
//          ADS1115 AIN3 (R50/R51 = 1/2) for the ratiometric pressure.
//
//  CORE 1 NEVER TOUCHES THE PYRO PINS. It only measures and records; every
//  decision to fire lives on core 0 (Rampa_MCU.ino). Core 0 talks to it
//  through one request mailbox (ts_request) and reads results from
//  volatile variables / a single-producer ring of 100 Hz live points.
//
//  Sampling: ADS1220 continuous, load cell at 2000 SPS (turbo) or 1000 SPS,
//  PGA 128, ratiometric to the excitation. Every load-cell sample makes one
//  16-byte frame (time, load cell, pressure = sum of 8 ADC reads x 2,
//  thermistors). The thermistors are read in between: every 100 ms (250 ms
//  while recording) the multiplexer is switched to one of them for two
//  conversions (~1.5 ms gap in the load-cell stream, visible in the time
//  stamps). Frames go into 256-byte pages; full pages are programmed while
//  the ADS1220 converts (page program 0.4-3 ms, one page per 8 ms at
//  2000 SPS).
//
//  Flash layout:
//    0x000000 / 0x001000  configuration A / B (newest valid copy wins)
//    0x010000 ...         records, each on 64 KB block boundaries:
//                         +0 header (128 B), +128 footer (written at the end),
//                         +256 frames
//  The blocks of a record are erased when it is armed (ARM / RECORD), so
//  older records are never touched. A record whose footer is missing (power
//  lost) is still readable: frames end at the first one with a bad CRC.
// =====================================================================
#ifndef THRUST_STAND_H
#define THRUST_STAND_H

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "stand_link.h"

// ---------------------------------------------------------------- pins
static const uint8_t TS_PIN_SCK   = 2;
static const uint8_t TS_PIN_MOSI  = 3;
static const uint8_t TS_PIN_MISO  = 4;
static const uint8_t TS_PIN_FL_CS = 5;
static const uint8_t TS_PIN_ADS_CS = 15;
static const uint8_t TS_PIN_DRDY  = 21;
static const uint8_t TS_PIN_PRESS = 26;

// ---------------------------------------------------------------- flash
#define TS_FLASH_SIZE   (16UL * 1024UL * 1024UL)
#define TS_CFG_A        0x000000UL
#define TS_CFG_B        0x001000UL
#define TS_REC_BASE     0x010000UL
#define TS_BLOCK        0x10000UL
#define TS_PAGE         256u
#define TS_FRAMES_OFF   256u
#define TS_HDR_MAGIC    0x31535452UL   // "RTS1"
#define TS_END_MAGIC    0x31444E45UL   // "END1"
#define TS_CFG_MAGIC    0x31464352UL   // "RCF1"
#define TS_MAX_RECS     96
#define TS_JEDEC_W25Q128 0xEF4018UL

#define TS_FRAME_SIZE   16u
#define TS_PAGE_RING    32u            // pages buffered (8 KB = 0.25 s at 2000 SPS)
#define TS_CURVE_MAX    8192u          // 100 Hz points kept in RAM (81 s)
#define TS_CURVE_PRE    100u           // points before T-0 kept (1 s)
#define TS_LIVE_RING    256u

// frame.flags
#define TS_FR_FIRE      (1u << 0)
#define TS_FR_TH1       (1u << 1)      // th1 updated in this frame
#define TS_FR_TH2       (1u << 2)
#define TS_FR_NOLC      (1u << 3)      // ADS1220 not converting: lc invalid, timer-paced frame
// bits 4..7: stand state (sl_state_t)

#pragma pack(push, 1)
typedef struct {
    uint32_t t_us;      // micros since record start
    int32_t  lc;        // ADS1220 code, AIN1-AIN0, PGA 128 (24-bit, sign-extended)
    uint16_t p;         // 8 x 12-bit ADC reads, summed, x2  (0..65520)
    uint16_t th1;       // thermistor ratio x 65535 (latest)
    uint16_t th2;
    uint8_t  flags;     // TS_FR_*
    uint8_t  crc8;      // over bytes 0..14
} ts_frame_t;

typedef struct {                // persisted configuration (calibration + test defaults)
    uint32_t magic;             // TS_CFG_MAGIC
    uint16_t ver;               // 2 (1 = before the arming servo; migrated on load)
    uint16_t seq;               // newer copy wins
    int32_t  lc_offset;         // tare, ADS codes
    float    lc_scale;          // N per code, 0 = not calibrated (force shown in uV/V)
    float    p_zero;            // transducer output / supply at 0 psi (0.1 = 0.5 V of 5 V)
    float    p_span;            // ratio span for full scale (0.8 = 4.0 V of 5 V)
    float    p_full_psi;        // 500
    float    adc_vref;          // Pico ADC reference, V (3.30)
    float    p_div;             // divider gain inverse (R46+R47)/R47 = 1.47
    float    th_r0;             // NTC R at th_t0_c, ohm
    float    th_beta;
    float    th_t0_c;
    float    th_rfix;           // R48 / R49
    float    ign_thr_n;         // minimum thrust that counts as burning
    uint16_t fire_ms;           // default igniter pulse
    uint16_t rec_s;             // recorded after T-0
    uint8_t  pyro_ch;           // default igniter
    uint8_t  rate_2k;           // 1 = 2000 SPS turbo, 0 = 1000 SPS
    uint16_t cd_s;              // default countdown (GUI hint)
    uint8_t  arm_servo;         // servo 1..4 that closes the remote arming switch, 0 = none (key alone)
    uint8_t  _r;
    uint16_t arm_us;            // arming servo pulse: switch closed
    uint16_t safe_us;           // arming servo pulse: switch open
    uint16_t arm_settle_ms;     // the key sense must close within this after the servo moved
    uint16_t crc;
} ts_cfg_t;

typedef struct {                // 128 B at record start
    uint32_t magic;             // TS_HDR_MAGIC
    uint16_t ver;               // 2 (cfg v2)
    uint16_t test_no;
    uint32_t blocks;            // 64 KB blocks reserved
    uint16_t sps;
    uint8_t  cold;              // 1 = RECORD without pyro
    uint8_t  pyro_ch;
    uint16_t fire_ms;
    uint16_t rec_s;
    uint16_t cd_s;
    uint16_t v5_mv;
    ts_cfg_t cfg;               // calibration in force (offline conversion)
    uint8_t  _pad[128 - 24 - sizeof(ts_cfg_t) - 2];
    uint16_t crc;
} ts_hdr_t;

typedef struct {                // 128 B at record start + 128
    uint32_t magic;             // TS_END_MAGIC
    uint32_t n_frames;
    int32_t  t0_frame;          // frame of T-0 (-1: none)
    uint8_t  flags;             // SL_RF_*
    uint8_t  _r;
    uint16_t v5_mv;
    float    peak_n, impulse_ns, burn_s, avg_n, t_peak_s, ign_delay_s, pmax_psi;
    int16_t  t1max_dc, t2max_dc;
    uint32_t dropped;           // frames lost (page ring overflow)
    uint8_t  _pad[128 - 52 - 2];
    uint16_t crc;
} ts_end_t;
#pragma pack(pop)

static_assert(sizeof(ts_frame_t) == TS_FRAME_SIZE, "frame layout");
static_assert(sizeof(ts_hdr_t) == 128, "header layout");
static_assert(sizeof(ts_end_t) == 128, "footer layout");

typedef struct {                // record directory entry (RAM)
    uint32_t addr;
    uint32_t blocks;
    uint16_t test_no;
    uint16_t sps;
    uint8_t  closed;            // footer present
    uint8_t  flags;             // SL_RF_* from the footer
    uint32_t n_frames;
    float    peak_n, impulse_ns, burn_s;
} ts_dir_t;

typedef struct {                // 100 Hz live point (core 1 -> core 0)
    uint32_t t_ms;
    float    f, fmax;           // force: mean / max over the 10 ms (N or uV/V)
    float    p, pmax;           // psi
    float    t1, t2;            // degC, NAN = not connected
    float    p_v;               // transducer output, V (diagnostics)
    float    uvv;               // load cell uV/V (mean, untared)
} ts_live_t;

// ---------------------------------------------------------------- requests (core 0 -> core 1)
enum {
    TQ_NONE = 0, TQ_TARE, TQ_CAL, TQ_PZERO, TQ_REC_START, TQ_REC_STOP, TQ_READ, TQ_ERASE_ALL,
    TQ_SET_CFG, TQ_SAVE_CFG, TQ_RESCAN, TQ_REC_GO
};
enum { TQR_PENDING = 0, TQR_OK = 1, TQR_FAIL = 2 };
// recorder state
enum { TR_IDLE = 0, TR_ERASING, TR_READY, TR_RECORDING, TR_ERROR };   // READY = slot erased, waiting for GO

// ---------------------------------------------------------------- shared state
static ts_cfg_t        ts_cfg;                     // written by core 1 only (inside requests)
static ts_cfg_t        ts_cfg_new;                 // TQ_SET_CFG payload (core 0)
static volatile uint8_t  ts_req = TQ_NONE;
static volatile uint32_t ts_req_id = 0, ts_ack_id = 0;
static volatile uint8_t  ts_ack_res = TQR_OK;
static volatile int32_t  ts_req_arg = 0;           // TQ_CAL: mN; TQ_REC_START: blocks; TQ_READ: addr
static volatile int32_t  ts_req_arg2 = 0;          // TQ_REC_START: 1 = cold; TQ_READ: length
static uint8_t           ts_rd_buf[TS_PAGE];

static volatile bool     ts_ready = false;         // core 1 initialised
static volatile uint32_t ts_alive = 0;             // core 1 loop counter
static volatile bool     ts_ads_ok = false, ts_flash_ok = false;
static volatile uint32_t ts_jedec = 0;
static volatile uint16_t ts_sps = 0;               // measured load-cell rate
static volatile uint8_t  ts_rec = TR_IDLE;
static volatile uint8_t  ts_progress = 0;          // erase / record %
static volatile uint16_t ts_test_no = 0;           // last record number
static volatile uint32_t ts_free_addr = TS_REC_BASE;
static volatile uint32_t ts_dropped = 0;
static volatile uint32_t ts_rec_frames = 0;
static volatile bool     ts_res_ready = false;     // results of ts_test_no valid
static volatile bool     ts_tared = false;
static volatile uint8_t  ts_stand_state = 0;       // written by core 0, stamped into frames
static volatile bool     ts_fire_on = false;       // written by core 0, stamped into frames
static volatile bool     ts_t0_req = false;        // core 0: "T-0 is now" (set once)
static volatile uint16_t ts_v5_mv = 5000;          // written by core 0 (ADS1115)

static ts_dir_t          ts_dir[TS_MAX_RECS];
static volatile uint8_t  ts_ndir = 0;

static ts_live_t         ts_live[TS_LIVE_RING];
static volatile uint32_t ts_live_w = 0;            // producer index (core 1)

// results of the last record
static sl_res_t          ts_res;
// 100 Hz curve of the last record (RAM)
static float             ts_cf[TS_CURVE_MAX];      // force
static int16_t           ts_cp[TS_CURVE_MAX];      // pressure 0.1 psi
static float             ts_ci[TS_CURVE_MAX];      // cumulative impulse at the end of the point
static volatile uint16_t ts_cn = 0;
static volatile int16_t  ts_ct0 = -1;

static inline uint8_t ts_crc8(const uint8_t *p, size_t n)
{
    uint8_t c = 0;
    while (n--) { c ^= *p++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
    return c;
}

// ---------------------------------------------------------------- conversions (pure)
static const float TS_UVV_PER_CODE = 1.0e6f / (128.0f * 8388608.0f);

static inline float ts_force(const ts_cfg_t &c, float code)
{
    const float d = code - (float)c.lc_offset;
    return c.lc_scale != 0.0f ? d * c.lc_scale : d * TS_UVV_PER_CODE;
}
static inline float ts_press_v(const ts_cfg_t &c, float psum)   // transducer output, V
{
    return psum / (16.0f * 4095.0f) * c.adc_vref * c.p_div;
}
static inline float ts_press_ratio(const ts_cfg_t &c, float psum, uint16_t v5_mv)
{
    const float v5 = (v5_mv > 4000 && v5_mv < 5600) ? v5_mv * 0.001f : 5.0f;
    return ts_press_v(c, psum) / v5;
}
static inline float ts_press_psi(const ts_cfg_t &c, float psum, uint16_t v5_mv)
{
    if (c.p_span <= 0.0f) return NAN;
    return (ts_press_ratio(c, psum, v5_mv) - c.p_zero) / c.p_span * c.p_full_psi;
}
/** Thermistor ratio (0..1) -> degC; NAN when open / shorted. */
static inline float ts_therm_c(const ts_cfg_t &c, float ratio)
{
    if (!(ratio > 0.005f && ratio < 0.995f)) return NAN;
    const float r = c.th_rfix * (1.0f / ratio - 1.0f);
    const float inv = 1.0f / (c.th_t0_c + 273.15f) + logf(r / c.th_r0) / c.th_beta;
    return 1.0f / inv - 273.15f;
}

static void ts_cfg_defaults(ts_cfg_t &c)
{
    memset(&c, 0, sizeof c);
    c.magic = TS_CFG_MAGIC; c.ver = 2; c.seq = 0;
    c.lc_offset = 0; c.lc_scale = 0.0f;
    c.p_zero = 0.10f; c.p_span = 0.80f; c.p_full_psi = 500.0f; c.adc_vref = 3.30f; c.p_div = 14.7f / 10.0f;
    c.th_r0 = 10000.0f; c.th_beta = 3950.0f; c.th_t0_c = 25.0f; c.th_rfix = 10000.0f;
    c.ign_thr_n = 2.0f;
    c.fire_ms = 1000; c.rec_s = 20; c.pyro_ch = 1; c.rate_2k = 1; c.cd_s = 10;
    c.arm_servo = 0; c.arm_us = 2000; c.safe_us = 1000; c.arm_settle_ms = 1500;
}
static bool ts_servo_us_ok(uint16_t us) { return us >= 500 && us <= 2500; }
static bool ts_cfg_valid(const ts_cfg_t &c)
{
    if (c.magic != TS_CFG_MAGIC || c.ver != 2) return false;
    if (sl_crc16((const uint8_t *)&c, sizeof c - 2) != c.crc) return false;
    return c.p_span > 0.01f && c.adc_vref > 1.0f && c.th_beta > 100.0f && c.th_rfix > 10.0f && c.th_r0 > 10.0f &&
           c.pyro_ch >= 1 && c.pyro_ch <= 3 && c.fire_ms >= SL_FIRE_MS_MIN && c.fire_ms <= SL_FIRE_MS_MAX &&
           c.rec_s >= SL_REC_MIN_S && c.rec_s <= SL_REC_MAX_S &&
           c.arm_servo <= 4 && ts_servo_us_ok(c.arm_us) && ts_servo_us_ok(c.safe_us) &&
           c.arm_settle_ms >= 200 && c.arm_settle_ms <= 10000;
}
/** A version-1 copy (68 B, before the arming servo): keep its calibration, add the servo defaults. */
static bool ts_cfg_migrate_v1(const uint8_t *raw, ts_cfg_t &out)
{
    ts_cfg_t c;
    memcpy(&c, raw, sizeof c);
    const size_t v1 = 68;
    if (c.magic != TS_CFG_MAGIC || c.ver != 1) return false;
    if (sl_crc16(raw, v1 - 2) != (uint16_t)(raw[v1 - 2] | (raw[v1 - 1] << 8))) return false;
    ts_cfg_t d; ts_cfg_defaults(d);
    memcpy(&d, raw, offsetof(ts_cfg_t, arm_servo));      // everything up to cd_s is the same
    d.ver = 2;
    d.crc = sl_crc16((const uint8_t *)&d, sizeof d - 2);
    if (!ts_cfg_valid(d)) return false;
    out = d;
    return true;
}

// =====================================================================
//  SPI devices (core 1 only)
// =====================================================================
static SPISettings ts_ads_spi(4000000, MSBFIRST, SPI_MODE1);
static SPISettings ts_fl_spi(16000000, MSBFIRST, SPI_MODE0);

// ---- W25Q128
static inline void fl_sel(void) { SPI.beginTransaction(ts_fl_spi); digitalWrite(TS_PIN_FL_CS, LOW); }
static inline void fl_desel(void) { digitalWrite(TS_PIN_FL_CS, HIGH); SPI.endTransaction(); }
static void fl_cmd(uint8_t c) { fl_sel(); SPI.transfer(c); fl_desel(); }
static uint8_t fl_status(void) { fl_sel(); SPI.transfer(0x05); const uint8_t s = SPI.transfer(0); fl_desel(); return s; }
static bool fl_busy(void) { return (fl_status() & 0x01) != 0; }
static uint32_t fl_jedec(void)
{
    fl_sel(); SPI.transfer(0x9F);
    uint32_t id = (uint32_t)SPI.transfer(0) << 16; id |= (uint32_t)SPI.transfer(0) << 8; id |= SPI.transfer(0);
    fl_desel(); return id;
}
static void fl_addr(uint8_t c, uint32_t a)
{
    SPI.transfer(c); SPI.transfer((uint8_t)(a >> 16)); SPI.transfer((uint8_t)(a >> 8)); SPI.transfer((uint8_t)a);
}
static void fl_read(uint32_t a, uint8_t *b, size_t n)
{
    fl_sel(); fl_addr(0x03, a);
    for (size_t i = 0; i < n; i++) b[i] = SPI.transfer(0);
    fl_desel();
}
/** Starts a page program (n <= 256, must not cross a page). Returns immediately. */
static void fl_prog_start(uint32_t a, const uint8_t *b, size_t n)
{
    fl_cmd(0x06);
    fl_sel(); fl_addr(0x02, a);
    for (size_t i = 0; i < n; i++) SPI.transfer(b[i]);
    fl_desel();
}
static void fl_erase_start(uint8_t cmd, uint32_t a) { fl_cmd(0x06); fl_sel(); fl_addr(cmd, a); fl_desel(); }
static bool fl_wait(uint32_t timeout_ms)
{
    const uint32_t t = millis();
    while (fl_busy()) { if ((uint32_t)(millis() - t) > timeout_ms) return false; delayMicroseconds(20); }
    return true;
}

// ---- ADS1220
static uint8_t ts_ads_reg0_lc = 0x6E;     // MUX 0110 = AIN1-AIN0, gain 128, PGA on
static uint8_t ts_ads_reg1 = 0xD4;        // 2000 SPS turbo, continuous
static const uint8_t TS_ADS_REG0_TH1 = 0xA1;   // AIN2-AVSS, gain 1, PGA bypassed
static const uint8_t TS_ADS_REG0_TH2 = 0xB1;   // AIN3-AVSS
static const uint8_t TS_ADS_REG2 = 0x40;       // external REFP0/REFN0 (= excitation)
static const uint8_t TS_ADS_REG3 = 0x00;

static inline void ads_sel(void) { SPI.beginTransaction(ts_ads_spi); digitalWrite(TS_PIN_ADS_CS, LOW); }
static inline void ads_desel(void) { digitalWrite(TS_PIN_ADS_CS, HIGH); SPI.endTransaction(); }
static void ads_cmd(uint8_t c) { ads_sel(); SPI.transfer(c); ads_desel(); }
static void ads_wreg(uint8_t r, uint8_t v) { ads_sel(); SPI.transfer((uint8_t)(0x40 | (r << 2))); SPI.transfer(v); ads_desel(); }
static uint8_t ads_rreg(uint8_t r) { ads_sel(); SPI.transfer((uint8_t)(0x20 | (r << 2))); const uint8_t v = SPI.transfer(0); ads_desel(); return v; }
static int32_t ads_read(void)
{
    ads_sel();
    SPI.transfer(0x10);                                   // RDATA
    uint32_t v = (uint32_t)SPI.transfer(0) << 16; v |= (uint32_t)SPI.transfer(0) << 8; v |= SPI.transfer(0);
    ads_desel();
    if (v & 0x800000u) v |= 0xFF000000u;
    return (int32_t)v;
}
static bool ads_begin(void)
{
    ts_ads_reg1 = ts_cfg.rate_2k ? 0xD4 : 0xC4;          // DR 110: 2000 turbo / 1000 normal
    ads_cmd(0x06);                                        // RESET
    delayMicroseconds(100);
    ads_wreg(0, ts_ads_reg0_lc); ads_wreg(1, ts_ads_reg1); ads_wreg(2, TS_ADS_REG2); ads_wreg(3, TS_ADS_REG3);
    const bool ok = ads_rreg(0) == ts_ads_reg0_lc && ads_rreg(1) == ts_ads_reg1 && ads_rreg(2) == TS_ADS_REG2;
    if (ok) ads_cmd(0x08);                                // START/SYNC
    return ok;
}

// =====================================================================
//  configuration on flash (core 1)
// =====================================================================
static bool ts_cfg_store(void)
{
    ts_cfg.seq = (uint16_t)(ts_cfg.seq + 1);
    ts_cfg.crc = sl_crc16((const uint8_t *)&ts_cfg, sizeof ts_cfg - 2);
    const uint32_t a = (ts_cfg.seq & 1u) ? TS_CFG_B : TS_CFG_A;  // alternate: the other copy survives a power cut
    fl_erase_start(0x20, a);
    if (!fl_wait(500)) return false;
    fl_prog_start(a, (const uint8_t *)&ts_cfg, sizeof ts_cfg);
    if (!fl_wait(10)) return false;
    ts_cfg_t chk; fl_read(a, (uint8_t *)&chk, sizeof chk);
    return memcmp(&chk, &ts_cfg, sizeof chk) == 0;
}
static void ts_cfg_load(void)
{
    ts_cfg_t a, b;
    fl_read(TS_CFG_A, (uint8_t *)&a, sizeof a);
    fl_read(TS_CFG_B, (uint8_t *)&b, sizeof b);
    bool va = ts_cfg_valid(a), vb = ts_cfg_valid(b);
    if (!va) { uint8_t r[sizeof a]; memcpy(r, &a, sizeof r); va = ts_cfg_migrate_v1(r, a); }
    if (!vb) { uint8_t r[sizeof b]; memcpy(r, &b, sizeof r); vb = ts_cfg_migrate_v1(r, b); }
    if (va && vb) ts_cfg = ((int16_t)(b.seq - a.seq) > 0) ? b : a;
    else if (va) ts_cfg = a;
    else if (vb) ts_cfg = b;
    else ts_cfg_defaults(ts_cfg);
}

// =====================================================================
//  record directory
// =====================================================================
static void ts_scan(void)
{
    uint8_t n = 0;
    uint32_t a = TS_REC_BASE;
    uint16_t last = 0;
    while (a + TS_BLOCK <= TS_FLASH_SIZE) {
        ts_hdr_t h;
        fl_read(a, (uint8_t *)&h, sizeof h);
        if (h.magic != TS_HDR_MAGIC || sl_crc16((const uint8_t *)&h, sizeof h - 2) != h.crc ||
            h.blocks == 0 || a + h.blocks * TS_BLOCK > TS_FLASH_SIZE) break;
        ts_end_t e;
        fl_read(a + 128, (uint8_t *)&e, sizeof e);
        const bool closed = e.magic == TS_END_MAGIC && sl_crc16((const uint8_t *)&e, sizeof e - 2) == e.crc;
        if (n < TS_MAX_RECS) {
            ts_dir_t &d = ts_dir[n++];
            d.addr = a; d.blocks = h.blocks; d.test_no = h.test_no; d.sps = h.sps; d.closed = closed;
            d.flags = closed ? e.flags : 0; d.n_frames = closed ? e.n_frames : 0;
            d.peak_n = closed ? e.peak_n : 0; d.impulse_ns = closed ? e.impulse_ns : 0; d.burn_s = closed ? e.burn_s : 0;
        }
        last = h.test_no;
        a += h.blocks * TS_BLOCK;
    }
    ts_ndir = n;
    ts_free_addr = a;
    ts_test_no = last;
}

// =====================================================================
//  recorder (core 1)
// =====================================================================
static uint8_t  ts_pages[TS_PAGE_RING][TS_PAGE];
static volatile uint16_t ts_pg_w = 0, ts_pg_r = 0;       // page ring (core 1 only)
static uint16_t ts_pg_fill = 0;                          // bytes in the page being filled
static uint32_t ts_rec_addr = 0, ts_rec_blocks = 0, ts_rec_wr = 0;   // record start, size, next program address
static uint32_t ts_rec_t0us = 0;
static int32_t  ts_rec_t0_frame = -1;
static bool     ts_rec_cold = false;
static uint32_t ts_erase_next = 0, ts_erase_end = 0;
static bool     ts_erase_all = false;
static bool     ts_prog_busy = false;
// statistics of the running record (full rate)
static float    ts_st_peak = 0, ts_st_tpeak = 0, ts_st_pmax = 0, ts_st_t1max = NAN, ts_st_t2max = NAN;
static double   ts_st_imp = 0;
static uint32_t ts_st_last_us = 0;
static float    ts_st_last_f = 0;
static bool     ts_st_fired = false;
// curve accumulation (10 ms buckets)
static double   ts_cb_f = 0, ts_cb_p = 0; static uint16_t ts_cb_n = 0;

static void ts_rec_begin(uint32_t blocks, bool cold)
{
    if (ts_rec != TR_IDLE) return;
    if (!ts_flash_ok || blocks == 0 || ts_free_addr + blocks * TS_BLOCK > TS_FLASH_SIZE) { ts_rec = TR_ERROR; return; }
    ts_rec_addr = ts_free_addr;
    ts_rec_blocks = blocks;
    ts_rec_cold = cold;
    ts_erase_next = ts_rec_addr; ts_erase_end = ts_rec_addr + blocks * TS_BLOCK;
    ts_erase_all = false;
    ts_progress = 0;
    ts_res_ready = false;
    ts_rec = TR_ERASING;
}

static void ts_rec_start_frames(void)
{
    ts_hdr_t h;
    memset(&h, 0xFF, sizeof h);
    h.magic = TS_HDR_MAGIC; h.ver = 2;
    h.test_no = (uint16_t)(ts_test_no + 1);
    h.blocks = ts_rec_blocks;
    h.sps = ts_cfg.rate_2k ? 2000 : 1000;
    h.cold = ts_rec_cold; h.pyro_ch = ts_cfg.pyro_ch; h.fire_ms = ts_cfg.fire_ms; h.rec_s = ts_cfg.rec_s;
    h.cd_s = ts_cfg.cd_s; h.v5_mv = ts_v5_mv; h.cfg = ts_cfg;
    h.crc = sl_crc16((const uint8_t *)&h, sizeof h - 2);
    fl_prog_start(ts_rec_addr, (const uint8_t *)&h, sizeof h);
    fl_wait(10);
    ts_test_no = h.test_no;
    ts_rec_wr = ts_rec_addr + TS_FRAMES_OFF;
    ts_pg_w = ts_pg_r = 0; ts_pg_fill = 0;
    ts_rec_frames = 0; ts_dropped = 0;
    ts_rec_t0us = micros();
    ts_rec_t0_frame = -1;
    ts_st_peak = 0; ts_st_tpeak = 0; ts_st_pmax = 0; ts_st_imp = 0; ts_st_last_us = 0; ts_st_last_f = 0;
    ts_st_t1max = NAN; ts_st_t2max = NAN; ts_st_fired = false;
    ts_cn = 0; ts_ct0 = -1; ts_cb_f = ts_cb_p = 0; ts_cb_n = 0;
    ts_t0_req = false;
    ts_rec = TR_RECORDING;
}

/** Programs one buffered page if the chip is idle. Returns true if something was written. */
static bool ts_flush_one(void)
{
    if (ts_pg_r == ts_pg_w) return false;
    if (ts_prog_busy) { if (fl_busy()) return false; ts_prog_busy = false; }
    if (ts_rec_wr + TS_PAGE > ts_rec_addr + ts_rec_blocks * TS_BLOCK) { ts_pg_r = ts_pg_w; return false; }
    fl_prog_start(ts_rec_wr, ts_pages[ts_pg_r % TS_PAGE_RING], TS_PAGE);
    ts_prog_busy = true;
    ts_rec_wr += TS_PAGE;
    ts_pg_r = (uint16_t)(ts_pg_r + 1);
    return true;
}

static void ts_push_frame(const ts_frame_t &fr)
{
    if ((uint16_t)(ts_pg_w - ts_pg_r) >= TS_PAGE_RING) { ts_dropped = ts_dropped + 1; return; }
    uint8_t *pg = ts_pages[ts_pg_w % TS_PAGE_RING];
    memcpy(pg + ts_pg_fill, &fr, sizeof fr);
    ts_pg_fill += sizeof fr;
    ts_rec_frames = ts_rec_frames + 1;
    if (ts_pg_fill >= TS_PAGE) { ts_pg_fill = 0; ts_pg_w = (uint16_t)(ts_pg_w + 1); }
}

static void ts_curve_point(float f, float p_psi)
{
    if (ts_cn >= TS_CURVE_MAX) return;
    ts_cf[ts_cn] = f;
    ts_cp[ts_cn] = sl_sat16(p_psi * 10.0f);
    ts_ci[ts_cn] = (float)ts_st_imp;
    ts_cn = (uint16_t)(ts_cn + 1);
}
static void ts_curve_t0(void)
{
    // keep the last second before T-0
    if (ts_cn > TS_CURVE_PRE) {
        const uint16_t k = (uint16_t)(ts_cn - TS_CURVE_PRE);
        memmove(ts_cf, ts_cf + k, TS_CURVE_PRE * sizeof ts_cf[0]);
        memmove(ts_cp, ts_cp + k, TS_CURVE_PRE * sizeof ts_cp[0]);
        memmove(ts_ci, ts_ci + k, TS_CURVE_PRE * sizeof ts_ci[0]);
        ts_cn = TS_CURVE_PRE;
    }
    ts_ct0 = (int16_t)ts_cn;
}

/** Burn statistics from the 100 Hz curve (impulse uses the full-rate integral). */
static void ts_results(void)
{
    sl_res_t &r = ts_res;
    memset(&r, 0, sizeof r);
    r.test_no = ts_test_no;
    r.pyro_ch = ts_cfg.pyro_ch;
    r.flags = (uint8_t)((ts_st_fired ? SL_RF_FIRED : 0) | (ts_rec_cold ? SL_RF_COLD : 0) |
                        (ts_cfg.lc_scale != 0.0f ? SL_RF_LC_CAL : 0) | (ts_cn ? SL_RF_CURVE : 0));
    r.peak_n = ts_st_peak; r.pmax_psi = ts_st_pmax;
    r.t1max_dc = sl_sat16(ts_st_t1max * 10.0f); r.t2max_dc = sl_sat16(ts_st_t2max * 10.0f);
    r.curve_n = ts_cn; r.curve_hz = SL_CURVE_HZ;
    r.n_frames = ts_rec_frames; r.sps = ts_cfg.rate_2k ? 2000 : 1000;
    r.t0_idx = ts_ct0;
    float fmax = 0;
    for (uint16_t i = 0; i < ts_cn; i++) if (fabsf(ts_cf[i]) > fmax) fmax = fabsf(ts_cf[i]);
    r.curve_fexp = sl_fexp_for(fmax);
    const int s0 = ts_ct0 >= 0 ? ts_ct0 : 0;
    r.t_peak_s = ts_st_tpeak;
    float thr = 0.05f * ts_st_peak;
    if (thr < ts_cfg.ign_thr_n) thr = ts_cfg.ign_thr_n;
    int a = -1, b = -1;
    for (int i = s0; i < (int)ts_cn; i++) if (ts_cf[i] > thr) { if (a < 0) a = i; b = i; }
    if (a >= 0 && ts_st_peak > ts_cfg.ign_thr_n) {
        r.flags |= SL_RF_IGNITION;
        r.burn_s = (b - a + 1) / (float)SL_CURVE_HZ;
        r.impulse_ns = ts_ci[b] - (a > 0 ? ts_ci[a - 1] : 0.0f);
        r.avg_n = r.burn_s > 0 ? r.impulse_ns / r.burn_s : 0;
        r.ign_delay_s = ts_ct0 >= 0 ? (a - ts_ct0) / (float)SL_CURVE_HZ : 0;
    }
}

static void ts_rec_finish(bool aborted)
{
    if (ts_rec != TR_RECORDING) { if (ts_rec == TR_ERASING || ts_rec == TR_ERROR) ts_rec = TR_IDLE; return; }
    // close the partial page (unwritten bytes stay 0xFF = invalid frame)
    if (ts_pg_fill) {
        memset(ts_pages[ts_pg_w % TS_PAGE_RING] + ts_pg_fill, 0xFF, TS_PAGE - ts_pg_fill);
        ts_pg_fill = 0; ts_pg_w = (uint16_t)(ts_pg_w + 1);
    }
    uint32_t guard = millis();
    while (ts_pg_r != ts_pg_w && (uint32_t)(millis() - guard) < 500) { if (!ts_flush_one()) delayMicroseconds(20); }
    fl_wait(10); ts_prog_busy = false;
    ts_results();
    if (aborted) ts_res.flags |= SL_RF_ABORTED;
    ts_end_t e;
    memset(&e, 0xFF, sizeof e);
    e.magic = TS_END_MAGIC; e.n_frames = ts_rec_frames; e.t0_frame = ts_rec_t0_frame;
    e.flags = (uint8_t)(ts_res.flags | SL_RF_SAVED); e._r = 0; e.v5_mv = ts_v5_mv;
    e.peak_n = ts_res.peak_n; e.impulse_ns = ts_res.impulse_ns; e.burn_s = ts_res.burn_s; e.avg_n = ts_res.avg_n;
    e.t_peak_s = ts_res.t_peak_s; e.ign_delay_s = ts_res.ign_delay_s; e.pmax_psi = ts_res.pmax_psi;
    e.t1max_dc = ts_res.t1max_dc; e.t2max_dc = ts_res.t2max_dc; e.dropped = ts_dropped;
    e.crc = sl_crc16((const uint8_t *)&e, sizeof e - 2);
    fl_prog_start(ts_rec_addr + 128, (const uint8_t *)&e, sizeof e);
    const bool ok = fl_wait(10);
    ts_end_t chk; fl_read(ts_rec_addr + 128, (uint8_t *)&chk, sizeof chk);
    if (ok && memcmp(&chk, &e, sizeof e) == 0) ts_res.flags |= SL_RF_SAVED;
    ts_res.crc = 0;
    ts_res_ready = true;
    ts_rec = TR_IDLE;
    ts_progress = 100;
    ts_scan();
}

// =====================================================================
//  sampling (core 1)
// =====================================================================
static uint8_t  ts_th_phase = 0;          // 0 load cell, 1/2 thermistor discard/take, 3 load cell discard
static uint8_t  ts_th_which = 0;
static uint32_t ts_th_next = 0;
static uint16_t ts_th[2] = { 0, 0 };
static uint8_t  ts_th_new = 0;
static uint32_t ts_last_drdy_ms = 0, ts_ads_retry_ms = 0;
static uint32_t ts_fallback_us = 0;
static uint32_t ts_sps_n = 0, ts_sps_ms = 0;
// live bucket (10 ms)
static uint32_t ts_lb_ms = 0;
static double   ts_lb_lc = 0, ts_lb_p = 0; static uint16_t ts_lb_n = 0;
static float    ts_lb_fmax = -1e30f, ts_lb_pmax = -1e30f;
// tare / calibration averaging
static uint8_t  ts_avg_what = TQ_NONE;
static double   ts_avg_sum = 0; static uint32_t ts_avg_n = 0, ts_avg_need = 0;

static uint16_t ts_read_pressure(void)
{
    uint32_t s = 0;
    for (int i = 0; i < 8; i++) s += (uint32_t)analogRead(TS_PIN_PRESS);
    s *= 2u;
    return s > 65535u ? 65535u : (uint16_t)s;
}

static void ts_ack(uint8_t res) { ts_ack_res = res; ts_ack_id = ts_req_id; ts_req = TQ_NONE; }

/** One load-cell sample (or a timer-paced frame without the ADS). */
static void ts_sample(int32_t lc, bool lc_valid)
{
    const uint32_t now_us = micros();
    const uint16_t p = ts_read_pressure();
    const ts_cfg_t &c = ts_cfg;
    const float f = lc_valid ? ts_force(c, (float)lc) : 0.0f;
    const float psi = ts_press_psi(c, p, ts_v5_mv);

    // frame
    if (ts_rec == TR_RECORDING) {
        ts_frame_t fr;
        fr.t_us = now_us - ts_rec_t0us;
        fr.lc = lc; fr.p = p; fr.th1 = ts_th[0]; fr.th2 = ts_th[1];
        fr.flags = (uint8_t)((ts_fire_on ? TS_FR_FIRE : 0) | ((ts_th_new & 1) ? TS_FR_TH1 : 0) |
                             ((ts_th_new & 2) ? TS_FR_TH2 : 0) | (lc_valid ? 0 : TS_FR_NOLC) |
                             ((ts_stand_state & 0x0F) << 4));
        fr.crc8 = ts_crc8((const uint8_t *)&fr, sizeof fr - 1);
        ts_th_new = 0;
        if (ts_t0_req && ts_rec_t0_frame < 0) {
            ts_rec_t0_frame = (int32_t)ts_rec_frames;
            ts_st_fired = true;
            ts_curve_t0();
        }
        ts_push_frame(fr);
        // full-rate statistics
        const bool after_t0 = ts_rec_cold || ts_rec_t0_frame >= 0;
        if (ts_st_last_us && after_t0) ts_st_imp += 0.5 * (f + ts_st_last_f) * (double)(now_us - ts_st_last_us) * 1e-6;
        ts_st_last_us = now_us; ts_st_last_f = f;
        if (after_t0 && f > ts_st_peak) {
            ts_st_peak = f;
            ts_st_tpeak = ts_rec_cold ? fr.t_us * 1e-6f : (ts_cn > (uint16_t)ts_ct0 ? (ts_cn - ts_ct0) / (float)SL_CURVE_HZ : 0.0f);
        }
        if (psi == psi && psi > ts_st_pmax) ts_st_pmax = psi;
        ts_cb_f += f; ts_cb_p += (psi == psi ? psi : 0.0); ts_cb_n++;
        if (ts_cb_n >= (ts_cfg.rate_2k ? 20u : 10u)) {
            ts_curve_point((float)(ts_cb_f / ts_cb_n), (float)(ts_cb_p / ts_cb_n));
            ts_cb_f = ts_cb_p = 0; ts_cb_n = 0;
        }
        const uint32_t cap = (ts_rec_blocks * TS_BLOCK - TS_FRAMES_OFF) / TS_FRAME_SIZE;
        ts_progress = (uint8_t)(ts_rec_frames * 100ull / (cap ? cap : 1));
        if (ts_rec_frames >= cap) ts_rec_finish(false);
    }

    // tare / calibration averages
    if (ts_avg_what != TQ_NONE) {
        if (ts_avg_what == TQ_PZERO) { ts_avg_sum += ts_press_ratio(c, p, ts_v5_mv); ts_avg_n++; }
        else if (lc_valid) { ts_avg_sum += lc; ts_avg_n++; }
        if (ts_avg_n >= ts_avg_need) {
            const double m = ts_avg_sum / ts_avg_n;
            uint8_t res = TQR_OK;
            if (ts_avg_what == TQ_TARE) { ts_cfg.lc_offset = (int32_t)lround(m); ts_tared = true; }
            else if (ts_avg_what == TQ_CAL) {
                const double d = m - ts_cfg.lc_offset;
                if (fabs(d) < 2000.0 || ts_req_arg <= 0) res = TQR_FAIL;     // < ~1.9 uV/V: no weight on it
                else ts_cfg.lc_scale = (float)(ts_req_arg * 0.001 / d);
            } else if (ts_avg_what == TQ_PZERO) {
                if (m < 0.04 || m > 0.25) res = TQR_FAIL;                   // not a vented 0.5 V transducer
                else ts_cfg.p_zero = (float)m;
            }
            ts_avg_what = TQ_NONE;
            if (res == TQR_OK && ts_rec != TR_RECORDING) res = ts_cfg_store() ? TQR_OK : TQR_FAIL;
            ts_ack(res);
        }
    }

    // live 100 Hz point
    const uint32_t ms = now_us / 1000u;
    if (!ts_lb_n) ts_lb_ms = ms;
    ts_lb_lc += lc; ts_lb_p += p; ts_lb_n++;
    if (f > ts_lb_fmax) ts_lb_fmax = f;
    if (psi == psi && psi > ts_lb_pmax) ts_lb_pmax = psi;
    if ((uint32_t)(ms - ts_lb_ms) >= 10u) {
        ts_live_t &L = ts_live[ts_live_w % TS_LIVE_RING];
        const float lcm = (float)(ts_lb_lc / ts_lb_n), pm = (float)(ts_lb_p / ts_lb_n);
        L.t_ms = millis();
        L.f = lc_valid ? ts_force(c, lcm) : NAN;
        L.fmax = lc_valid ? ts_lb_fmax : NAN;
        L.p = ts_press_psi(c, pm, ts_v5_mv);
        L.pmax = ts_lb_pmax > -1e29f ? ts_lb_pmax : L.p;
        L.p_v = ts_press_v(c, pm);
        L.t1 = ts_therm_c(c, ts_th[0] / 65535.0f);
        L.t2 = ts_therm_c(c, ts_th[1] / 65535.0f);
        L.uvv = lc_valid ? lcm * TS_UVV_PER_CODE : NAN;
        if (ts_rec == TR_RECORDING) {
            if (L.t1 == L.t1 && !(L.t1 <= ts_st_t1max)) ts_st_t1max = L.t1;
            if (L.t2 == L.t2 && !(L.t2 <= ts_st_t2max)) ts_st_t2max = L.t2;
        }
        __sync_synchronize();
        ts_live_w = ts_live_w + 1;
        ts_lb_lc = ts_lb_p = 0; ts_lb_n = 0; ts_lb_fmax = -1e30f; ts_lb_pmax = -1e30f;
    }

    ts_sps_n++;
}

static void ts_ads_service(void)
{
    const uint32_t now = millis();
    if (!ts_ads_ok) {
        // timer-paced frames (1 kHz) keep pressure recorded; retry the ADS every second
        const uint32_t us = micros();
        if ((uint32_t)(us - ts_fallback_us) >= 1000u) { ts_fallback_us = us; ts_sample(0, false); }
        if ((uint32_t)(now - ts_ads_retry_ms) > 1000u) {
            ts_ads_retry_ms = now;
            ts_th_phase = 0;
            ts_ads_ok = ads_begin();
            ts_last_drdy_ms = now;
        }
        return;
    }
    if (digitalRead(TS_PIN_DRDY) != LOW) {
        if ((uint32_t)(now - ts_last_drdy_ms) > 50u) { ts_ads_ok = false; ts_ads_retry_ms = now; }   // stopped converting
        return;
    }
    ts_last_drdy_ms = now;
    const int32_t v = ads_read();
    switch (ts_th_phase) {
    case 0:
        ts_sample(v, true);
        if ((int32_t)(now - ts_th_next) >= 0) {
            ts_th_next = now + (ts_rec == TR_RECORDING ? 125u : 50u);   // each thermistor 4 Hz / 10 Hz
            ads_wreg(0, ts_th_which ? TS_ADS_REG0_TH2 : TS_ADS_REG0_TH1);
            ads_cmd(0x08);
            ts_th_phase = 1;
        }
        break;
    case 1: ts_th_phase = 2; break;                            // first conversion after the switch: discard
    case 2: {
        int32_t r = v < 0 ? 0 : v;
        ts_th[ts_th_which] = (uint16_t)((uint32_t)r >> 7);     // 23-bit positive -> 16 bit
        ts_th_new |= (uint8_t)(1u << ts_th_which);
        ts_th_which ^= 1;
        ads_wreg(0, ts_ads_reg0_lc);
        ads_cmd(0x08);
        ts_th_phase = 3;
        break; }
    default: ts_th_phase = 0; break;                           // settle the load cell
    }
}

static void ts_handle_request(void)
{
    const uint8_t q = ts_req;
    if (q == TQ_NONE || ts_ack_id == ts_req_id) return;
    switch (q) {
    case TQ_TARE: case TQ_CAL: case TQ_PZERO:
        if (ts_avg_what == TQ_NONE) {
            if (q != TQ_PZERO && !ts_ads_ok) { ts_ack(TQR_FAIL); break; }
            ts_avg_what = q; ts_avg_sum = 0; ts_avg_n = 0;
            ts_avg_need = q == TQ_PZERO ? 500u : (ts_cfg.rate_2k ? 1000u : 500u);   // 0.5 s
        }
        break;                                                  // acked by ts_sample()
    case TQ_REC_START:
        if (ts_rec == TR_RECORDING || ts_rec == TR_ERASING || ts_rec == TR_READY) { ts_ack(TQR_FAIL); break; }
        ts_rec = TR_IDLE;
        ts_rec_begin((uint32_t)ts_req_arg, ts_req_arg2 != 0);
        ts_ack(ts_rec == TR_ERASING ? TQR_OK : TQR_FAIL);
        if (ts_rec == TR_ERROR) ts_rec = TR_IDLE;
        break;
    case TQ_REC_GO:
        if (ts_rec != TR_READY) { ts_ack(TQR_FAIL); break; }
        ts_rec_start_frames();
        ts_ack(TQR_OK);
        break;
    case TQ_REC_STOP:
        if (ts_rec == TR_READY) { ts_rec = TR_IDLE; ts_ack(TQR_OK); break; }   // erased slot stays free
        ts_rec_finish(ts_req_arg != 0);
        ts_ack(TQR_OK);
        break;
    case TQ_READ: {
        if (ts_rec != TR_IDLE) { ts_ack(TQR_FAIL); break; }
        const uint32_t n = (uint32_t)ts_req_arg2 > TS_PAGE ? TS_PAGE : (uint32_t)ts_req_arg2;
        fl_wait(10);
        fl_read((uint32_t)ts_req_arg, ts_rd_buf, n);
        ts_ack(TQR_OK);
        break; }
    case TQ_ERASE_ALL:
        if (ts_rec != TR_IDLE) { ts_ack(TQR_FAIL); break; }
        ts_erase_next = TS_REC_BASE; ts_erase_end = TS_FLASH_SIZE; ts_erase_all = true;
        ts_progress = 0; ts_rec = TR_ERASING;
        ts_ack(TQR_OK);
        break;
    case TQ_SET_CFG:
        if (ts_rec != TR_IDLE) { ts_ack(TQR_FAIL); break; }
        {
            const uint16_t seq = ts_cfg.seq;
            const uint8_t old_rate = ts_cfg.rate_2k;
            ts_cfg = ts_cfg_new; ts_cfg.magic = TS_CFG_MAGIC; ts_cfg.ver = 2; ts_cfg.seq = seq;
            const bool ok = ts_cfg_store();
            if (ts_cfg.rate_2k != old_rate) ts_ads_ok = ads_begin();
            ts_ack(ok ? TQR_OK : TQR_FAIL);
        }
        break;
    case TQ_SAVE_CFG:
        ts_ack(ts_cfg_store() ? TQR_OK : TQR_FAIL);
        break;
    case TQ_RESCAN:
        ts_scan(); ts_ack(TQR_OK);
        break;
    default: ts_ack(TQR_FAIL); break;
    }
}

static void ts_erase_service(void)
{
    if (ts_rec != TR_ERASING) return;
    if (fl_busy()) return;
    if (ts_erase_next >= ts_erase_end) {
        if (ts_erase_all) { ts_erase_all = false; ts_rec = TR_IDLE; ts_scan(); ts_progress = 100; }
        else { ts_rec = TR_READY; ts_progress = 100; }
        return;
    }
    fl_erase_start(0xD8, ts_erase_next);
    ts_erase_next += TS_BLOCK;
    const uint32_t tot = ts_erase_end - (ts_erase_all ? TS_REC_BASE : ts_rec_addr);
    ts_progress = (uint8_t)((uint64_t)(ts_erase_next - (ts_erase_end - tot)) * 100u / (tot ? tot : 1));
}

// ---------------------------------------------------------------- core 1 entry points
static void ts_core1_setup(void)
{
    pinMode(TS_PIN_FL_CS, OUTPUT); digitalWrite(TS_PIN_FL_CS, HIGH);
    pinMode(TS_PIN_ADS_CS, OUTPUT); digitalWrite(TS_PIN_ADS_CS, HIGH);
    pinMode(TS_PIN_DRDY, INPUT);
    analogReadResolution(12);
    SPI.setRX(TS_PIN_MISO); SPI.setTX(TS_PIN_MOSI); SPI.setSCK(TS_PIN_SCK);
    SPI.begin();
    fl_cmd(0xAB);                                          // release from power-down
    delayMicroseconds(50);
    ts_jedec = fl_jedec();
    ts_flash_ok = ts_jedec == TS_JEDEC_W25Q128;
    if (ts_flash_ok) { ts_cfg_load(); ts_scan(); }
    else ts_cfg_defaults(ts_cfg);
    ts_ads_ok = ads_begin();
    ts_last_drdy_ms = millis();
    ts_sps_ms = millis();
    ts_ready = true;
}

static void ts_core1_loop(void)
{
    ts_alive = ts_alive + 1;
    ts_handle_request();
    ts_ads_service();
    if (ts_rec == TR_RECORDING) {
        ts_flush_one();
        if (!ts_flash_ok) ts_rec_finish(false);
    }
    ts_erase_service();
    const uint32_t now = millis();
    if ((uint32_t)(now - ts_sps_ms) >= 1000u) {
        ts_sps = (uint16_t)(ts_sps_n * 1000u / (now - ts_sps_ms));
        ts_sps_n = 0; ts_sps_ms = now;
        if (ts_rec == TR_IDLE && !ts_flash_ok) {                // hot-retry a flash that did not answer at boot
            ts_jedec = fl_jedec();
            if (ts_jedec == TS_JEDEC_W25Q128) { ts_flash_ok = true; ts_cfg_load(); ts_scan(); }
        }
    }
}

// ---------------------------------------------------------------- core 0 helpers
/** Posts a request; false if the previous one is still pending. */
static bool ts_request(uint8_t q, int32_t arg = 0, int32_t arg2 = 0)
{
    if (ts_ack_id != ts_req_id) return false;
    ts_req_arg = arg; ts_req_arg2 = arg2;
    __sync_synchronize();
    ts_req_id = ts_req_id + 1;
    ts_req = q;
    return true;
}
static bool ts_request_done(void) { return ts_ack_id == ts_req_id; }

/** Flash bytes free for new records. */
static uint32_t ts_flash_free(void) { return ts_flash_ok ? TS_FLASH_SIZE - ts_free_addr : 0; }

/** 64 KB blocks a record of `seconds` needs (frames + header). */
static uint32_t ts_blocks_for(uint32_t seconds)
{
    const uint32_t sps = ts_cfg.rate_2k ? 2000u : 1000u;
    const uint64_t bytes = (uint64_t)seconds * sps * TS_FRAME_SIZE * 105u / 100u + TS_FRAMES_OFF;
    return (uint32_t)((bytes + TS_BLOCK - 1) / TS_BLOCK);
}

#endif // THRUST_STAND_H
