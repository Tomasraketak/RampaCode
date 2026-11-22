// =====================================================================
//  stand_link.h — RAMPA as a STATIC-FIRE THRUST STAND  <->  GROUND STATION
//
//  Owner: RampaCode (Rampa_MCU/stand_link.h). The ground station keeps a
//  byte-identical copy (GroundStation/firmware/common + firmware/Ground_Station,
//  checked by GroundStation/tools/sync_common.py).
//
//  Use case: no rocket on the pogo. The rampa PCB fires a motor on the test
//  stand, measures thrust (load cell on the ADS1220), chamber pressure
//  (0.5-4.5 V ratiometric transducer on GP26) and two thermistors (ADS1220
//  AIN2/AIN3), records everything at the full ADC rate into the W25Q128 and
//  streams a decimated live view to the ground station.
//
//  RADIO: the same modulation and sync word as the Zenith link (433 MHz,
//  SF7, BW 250 kHz, CR 4/5, preamble 8, sync 0x2B), so the rampa hears the
//  station without retuning. The packets cannot be confused with Zenith ones:
//  own magic bytes (0xD1..0xD9), own sizes, CRC-16 over every byte.
//
//  TDMA (no fixed windows — nothing else uses the channel at a test stand):
//    rampa  --TLM-->  station          5 Hz when armed / counting / burning,
//                                      2 Hz when safe
//    rampa  <--CMD--  station          right after each TLM (the rampa listens
//                                      whenever it is not transmitting);
//                                      without TLM the station sends blind
//    every CMD (also PING) is a heartbeat; COUNTDOWN aborts by itself when no
//    valid packet came from the station for SL_LINK_TIMEOUT_MS.
//
//  Commands are acknowledged in the next TLM (ack_seq + ack_rc); a repeated
//  seq is acknowledged again but never executed twice, so retries are safe.
// =====================================================================
#ifndef STAND_LINK_H
#define STAND_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#if defined(__cplusplus)
#define SL_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define SL_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif

// ------------------------------------------------------------------ radio
#define SL_LORA_FREQ_HZ        433000000UL
#define SL_LORA_SF             7
#define SL_LORA_BW_HZ          250000UL
#define SL_LORA_CR4            5
#define SL_LORA_PREAMBLE       8
#define SL_LORA_TX_DBM         20
#define SL_LORA_SYNCWORD       0x2B

#define SL_VER                 1u
#define SL_TLM_MAGIC           0xD1u
#define SL_RES_MAGIC           0xD2u
#define SL_CRV_MAGIC           0xD3u
#define SL_CMD_MAGIC           0xD9u

// ------------------------------------------------------------------ timing
#define SL_TLM_FAST_MS         200u     // armed / countdown / firing / recording
#define SL_TLM_SLOW_MS         500u     // safe
#define SL_LINK_TIMEOUT_MS     2500u    // COUNTDOWN aborts without a station packet
#define SL_ARMED_LINK_MS       30000u   // ARMED disarms without a station packet
#define SL_ARMED_MAX_MS        600000u  // ARMED disarms after 10 min without FIRE
#define SL_MODE_EXIT_MS        60000u   // SAFE + silent station -> back to rocket mode
#define SL_CD_MIN_S            3
#define SL_CD_MAX_S            60
#define SL_REC_MIN_S           2
#define SL_REC_MAX_S           120
#define SL_FIRE_MS_MIN         50
#define SL_FIRE_MS_MAX         5000
#define SL_SUBS                10       // force sub-samples per TLM
#define SL_CRV_PTS             16       // curve points per CURVE packet
#define SL_CURVE_HZ            100

// ------------------------------------------------------------------ states
typedef enum {
    SL_ST_SAFE      = 0,   // pyro off, measuring, ready for commands
    SL_ST_ARMING    = 1,   // ARM accepted, flash slot being erased
    SL_ST_ARMED     = 2,   // waiting for FIRE (pyro still off)
    SL_ST_COUNTDOWN = 3,   // recording, T-0 counting on the rampa
    SL_ST_FIRING    = 4,   // igniter current flowing (fire_ms)
    SL_ST_BURN      = 5,   // pyro off, recording until T0 + rec_s
    SL_ST_ABORT     = 6,   // latched until DISARM
    SL_ST_RECORD    = 7,   // cold recording, no pyro (calibration, leak test)
    SL_ST_COUNT
} sl_state_t;

// tlm.flags
#define SL_F_KEY        (1u << 0)    // key switch J4 closed (pyro supply)
#define SL_F_CONT       (1u << 1)    // selected igniter has continuity
#define SL_F_ARM_OUT    (1u << 2)    // high-side switch on
#define SL_F_FIRE_OUT   (1u << 3)    // igniter low-side on
#define SL_F_REC        (1u << 4)    // recording to flash
#define SL_F_ADS_OK     (1u << 5)    // ADS1220 converting
#define SL_F_FLASH_OK   (1u << 6)    // W25Q128 answers and has room
#define SL_F_LC_CAL     (1u << 7)    // load cell calibrated: force in N (else uV/V)
#define SL_F_TH1        (1u << 8)    // thermistor 1 connected
#define SL_F_TH2        (1u << 9)    // thermistor 2 connected
#define SL_F_PRESS      (1u << 10)   // pressure transducer connected
#define SL_F_RESULT     (1u << 11)   // results of test_no available
#define SL_F_HANGFIRE   (1u << 12)   // fired, but no thrust seen: DO NOT APPROACH
#define SL_F_BTN        (1u << 13)   // local ABORT button held
#define SL_F_BUZZER     (1u << 14)   // locate buzzer on
#define SL_F_TARED      (1u << 15)   // tared since power-up

// faults: why ARM was refused / why an ABORT happened (live | latched)
#define SL_FLT_VBAT     (1u << 0)
#define SL_FLT_KEY      (1u << 1)
#define SL_FLT_CONT     (1u << 2)
#define SL_FLT_LINK     (1u << 3)
#define SL_FLT_ADS      (1u << 4)
#define SL_FLT_FLASH    (1u << 5)    // missing, full, or write error
#define SL_FLT_BTN      (1u << 6)
#define SL_FLT_SYS      (1u << 7)    // measurement core stalled / I2C hardware

// ------------------------------------------------------------------ commands
typedef enum {
    SL_C_PING     = 0,    // heartbeat (also switches the rampa into stand mode)
    SL_C_ARM      = 1,    // arg = igniter 1..3, arg2 = fire pulse ms        (token)
    SL_C_DISARM   = 2,    // back to SAFE, clears a latched ABORT
    SL_C_FIRE     = 3,    // arg = countdown s (SL_CD_MIN_S..SL_CD_MAX_S)    (token)
    SL_C_ABORT    = 4,    //                                                  (token)
    SL_C_TARE     = 5,    // zero the load cell (SAFE / ARMED)
    SL_C_RECORD   = 6,    // arg = s: cold recording without pyro (SAFE)
    SL_C_STOP     = 7,    // end a recording now
    SL_C_RESULT   = 8,    // resend the RES packet
    SL_C_CURVE    = 9,    // arg = first point: answer with one CURVE packet
    SL_C_BUZZER   = 10,   // arg 1/0
    SL_C_RECTIME  = 11,   // arg = s recorded after T-0 (SL_REC_MIN_S..MAX), saved
    SL_C_CAL      = 12,   // arg = known force on the load cell, mN (SAFE)     (token)
    SL_C_PZERO    = 13,   // pressure transducer zero = now (vented, SAFE)
    SL_C_COUNT
} sl_cmd_t;

#define SL_TOK_NONE     0x0000u
#define SL_TOK_ARM      0xA53Du
#define SL_TOK_FIRE     0xF14Eu
#define SL_TOK_ABORT    0xAB07u
#define SL_TOK_CAL      0xCA1Bu

static inline uint16_t sl_token_for(uint8_t cmd)
{
    switch (cmd) {
    case SL_C_ARM:   return SL_TOK_ARM;
    case SL_C_FIRE:  return SL_TOK_FIRE;
    case SL_C_ABORT: return SL_TOK_ABORT;
    case SL_C_CAL:   return SL_TOK_CAL;
    default:         return SL_TOK_NONE;
    }
}

typedef enum {
    SL_RC_NONE      = 0,
    SL_RC_OK        = 1,
    SL_RC_REJ_STATE = 2,   // not in a state where this is allowed
    SL_RC_REJ_INTLK = 3,   // interlock (key, igniter, battery, sensors, flash): see faults
    SL_RC_REJ_ARG   = 4,   // bad token / argument
    SL_RC_ABORTED   = 5,   // abort executed
    SL_RC_BUSY      = 6    // flash busy / not ready
} sl_rc_t;

// RES.flags
#define SL_RF_FIRED     (1u << 0)
#define SL_RF_IGNITION  (1u << 1)    // thrust above threshold seen
#define SL_RF_ABORTED   (1u << 2)
#define SL_RF_CURVE     (1u << 3)    // curve in RAM, CURVE requests answered
#define SL_RF_SAVED     (1u << 4)    // record closed properly on flash
#define SL_RF_COLD      (1u << 5)    // cold recording (no pyro)
#define SL_RF_LC_CAL    (1u << 6)    // force values in N (else uV/V)

#define SL_T_NONE       ((int16_t)-32768)

#pragma pack(push, 1)
/** rampa -> station, 56 B. force_N = f * 0.1 * 2^fexp (uV/V when !SL_F_LC_CAL). */
typedef struct {
    uint8_t  magic;          //  0 SL_TLM_MAGIC
    uint8_t  ver;            //  1
    uint8_t  state;          //  2 sl_state_t
    uint8_t  faults;         //  3 SL_FLT_* (live | latched)
    uint16_t flags;          //  4 SL_F_*
    uint8_t  ack_seq;        //  6
    uint8_t  ack_rc;         //  7 sl_rc_t
    uint16_t vbat_mv;        //  8
    uint16_t v5_mv;          // 10 sensor supply (5V_SENS)
    int32_t  t_rel_ms;       // 12 <0: to T-0, >=0: since T-0 (or since record start)
    int16_t  f[SL_SUBS];     // 16 force, oldest first, mean of each slice
    int16_t  fpeak;          // 36 max force since the previous TLM
    int8_t   fexp;           // 38
    uint8_t  pyro;           // 39 bits0-1 igniter 1..3, bits4-6 continuity ch1..3
    int16_t  p_dpsi;         // 40 chamber pressure, 0.1 psi
    int16_t  ppeak_dpsi;     // 42 max since previous TLM
    int16_t  t1_dc;          // 44 0.1 degC (SL_T_NONE)
    int16_t  t2_dc;          // 46
    uint16_t test_no;        // 48 last / current record
    uint8_t  flash_free_pct; // 50
    uint8_t  progress;       // 51 % (erase while ARMING, recording)
    uint16_t sps;            // 52 load-cell samples per second
    uint16_t crc;            // 54
} sl_tlm_t;

/** rampa -> station after every test and on SL_C_RESULT, 52 B. */
typedef struct {
    uint8_t  magic;          //  0 SL_RES_MAGIC
    uint8_t  ver;            //  1
    uint16_t test_no;        //  2
    uint8_t  flags;          //  4 SL_RF_*
    uint8_t  pyro_ch;        //  5
    float    peak_n;         //  6
    float    impulse_ns;     // 10 integral of force over the burn
    float    burn_s;         // 14 above threshold (5 % of peak, >= 2 N)
    float    avg_n;          // 18 impulse / burn time
    float    t_peak_s;       // 22 from T-0
    float    ign_delay_s;    // 26 T-0 -> threshold crossed
    float    pmax_psi;       // 30
    int16_t  t1max_dc;       // 34
    int16_t  t2max_dc;       // 36
    uint16_t curve_n;        // 38 points in the curve
    uint8_t  curve_hz;       // 40
    int8_t   curve_fexp;     // 41 curve force = f * 0.1 * 2^curve_fexp
    uint32_t n_frames;       // 42 full-rate frames on flash
    uint16_t sps;            // 46
    int16_t  t0_idx;         // 48 curve index of T-0 (-1: cold)
    uint16_t crc;            // 50
} sl_res_t;

/** rampa -> station on SL_C_CURVE, 74 B. Points [offset, offset+n). */
typedef struct {
    uint8_t  magic;          //  0 SL_CRV_MAGIC
    uint8_t  ver;            //  1
    uint16_t test_no;        //  2
    uint16_t offset;         //  4
    uint8_t  n;              //  6 valid points (0 = past the end)
    int8_t   fexp;           //  7
    int16_t  f[SL_CRV_PTS];  //  8
    int16_t  p[SL_CRV_PTS];  // 40 pressure, 0.1 psi
    uint16_t crc;            // 72
} sl_curve_t;

/** station -> rampa, 14 B. */
typedef struct {
    uint8_t  magic;          //  0 SL_CMD_MAGIC
    uint8_t  ver;            //  1
    uint8_t  seq;            //  2 1..255
    uint8_t  cmd;            //  3 sl_cmd_t
    int32_t  arg;            //  4
    uint16_t token;          //  8
    uint16_t arg2;           // 10
    uint16_t crc;            // 12
} sl_cmd_pkt_t;
#pragma pack(pop)

SL_STATIC_ASSERT(sizeof(sl_tlm_t) == 56, "sl_tlm_t layout");
SL_STATIC_ASSERT(sizeof(sl_res_t) == 52, "sl_res_t layout");
SL_STATIC_ASSERT(sizeof(sl_curve_t) == 74, "sl_curve_t layout");
SL_STATIC_ASSERT(sizeof(sl_cmd_pkt_t) == 14, "sl_cmd_pkt_t layout");

/** CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). */
static inline uint16_t sl_crc16(const uint8_t *p, size_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; i++) c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
    return c;
}

/** Fill magic + version and the trailing CRC of any packet above. */
static inline void sl_seal(void *pkt, size_t size, uint8_t magic)
{
    uint8_t *p = (uint8_t *)pkt;
    p[0] = magic;
    p[1] = SL_VER;
    const uint16_t c = sl_crc16(p, size - 2u);
    p[size - 2u] = (uint8_t)(c & 0xFFu);
    p[size - 1u] = (uint8_t)(c >> 8);
}

/** 1 when buf holds exactly one valid packet of `size` bytes with `magic`. */
static inline int sl_check(const uint8_t *buf, size_t n, size_t size, uint8_t magic)
{
    if (n != size || buf[0] != magic || buf[1] != SL_VER) return 0;
    const uint16_t c = (uint16_t)(buf[size - 2u] | ((uint16_t)buf[size - 1u] << 8));
    return sl_crc16(buf, size - 2u) == c;
}

/** Smallest exponent e (0..15) such that |max_n| fits int16 at 0.1 * 2^e N. */
static inline int8_t sl_fexp_for(float max_abs_n)
{
    int8_t e = 0;
    float lim = 3276.0f;
    while (e < 15 && max_abs_n > lim) { e++; lim *= 2.0f; }
    return e;
}
static inline int16_t sl_fpack(float v, int8_t e)
{
    float r = v * 10.0f;
    for (int8_t i = 0; i < e; i++) r *= 0.5f;
    if (r > 32767.0f) r = 32767.0f;
    if (r < -32767.0f) r = -32767.0f;
    return (int16_t)(r >= 0 ? r + 0.5f : r - 0.5f);
}
static inline float sl_funpack(int16_t f, int8_t e)
{
    float r = f * 0.1f;
    for (int8_t i = 0; i < e; i++) r *= 2.0f;
    return r;
}
static inline int16_t sl_sat16(float v)
{
    if (v != v) return SL_T_NONE;
    if (v > 32767.0f) return 32767;
    if (v < -32767.0f) return -32767;
    return (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}

#endif // STAND_LINK_H
