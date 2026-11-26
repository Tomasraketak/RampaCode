// =====================================================================
//  RAMPA PCB1 — launch pad controller for ZENITH *and* DAIDALOS (Ariadna V2)
//
//  ONE firmware, no reflashing: the rampa recognises by itself which rocket
//  is on the pogo umbilical and speaks its protocol.
//
//    Zenith  (zenith_link.h)        UART0 115200 on GP0 (TX) / GP1 (RX),
//                                   rocket = master, 20 Hz command/status frames,
//                                   rampa powers the rocket over UMB_5V.
//    Ariadna (ariadna_rampa_link.h) I2C0 slave 0x2B on GP0 (SDA) / GP1 (SCL),
//                                   Navigation MCU = I2C master @ 100 kHz, 20 Hz,
//                                   UMB_5V stays OFF (the Ariadna harness leaves
//                                   pin 2 unconnected), hot-detach at COMMIT —
//                                   exactly the behaviour of the Daidalos
//                                   Rampa_MCU.ino (fw 3.34).
//
//  Target: Raspberry Pi Pico 2 (RP2350, U7), arduino-pico (rp2040:rp2040:rpipico2).
//  Needs : LoRa (sandeepmistry), Adafruit NeoPixel.
//
//  ---------------------------------------------------------------------
//  HOW THE ROCKET IS RECOGNISED
//  ---------------------------------------------------------------------
//  GP0/GP1 can be UART0 or I2C0 on the RP2350, and both rockets use the
//  same pogo pins (J3.3 / J3.4). While nobody is locked in, the rampa
//  listens alternately 600 ms as an I2C slave and 600 ms as a UART. The
//  first frame that passes its protocol's magic + CRC locks that protocol;
//  it stays locked while frames keep coming (and always through a countdown
//  and firing). Neither protocol can be mistaken for the other: a UART byte
//  stream never produces an I2C START (SDA is not driven by the rocket in
//  UART mode), and I2C traffic seen by the UART is framing garbage that
//  fails the 0x5A 0xE7 sync + CRC-16.
//
//  At power-up the rampa first listens 1.5 s as an I2C slave with the
//  rocket supply OFF. Only if no Ariadna answers does it switch UMB_5V on
//  (a Zenith with its battery switched off needs it to boot).
//
//  Ariadna expects the COMMITTED answer in the SAME I2C transaction as its
//  COMMIT write (25 ms timeout), so Ariadna commands are processed directly
//  in the I2C receive interrupt; the main loop does its own state changes
//  with interrupts masked, so the two never interleave.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/watchdog.h"
#include "zenith_link.h"
#include "ariadna_rampa_link.h"

// =====================================================================
//  1. PINOUT (RampaNetlist_PCB1_2026-11-19.enet)
// =====================================================================
static const uint8_t PIN_UMB_0      = 0;   // U7.1  -> R21 1k   -> J3.4   (UART0 TX | I2C0 SDA)
static const uint8_t PIN_UMB_1      = 1;   // U7.2  <- R22 100R <- J3.3   (UART0 RX | I2C0 SCL), R23 100k PU
static const uint8_t PIN_INT_SDA    = 6;   // I2C1 internal bus (R18 4.7k)
static const uint8_t PIN_INT_SCL    = 7;   //                   (R19 4.7k)
static const uint8_t PIN_LORA_SCK   = 10;  // SPI1 -> M1 Ra-02 (via 1k)
static const uint8_t PIN_LORA_MOSI  = 11;
static const uint8_t PIN_LORA_MISO  = 12;
static const uint8_t PIN_LORA_CS    = 13;
static const uint8_t PIN_FIRE[3]    = { 16, 17, 18 };   // Q22/Q23/Q24 low side -> J5/J6/J7
static const uint8_t PIN_PYRO_ARM   = 19;  // Q21 -> Q20 AO4407A high side (VPYRO_ARMED)
static const uint8_t PIN_RGB        = 20;  // SK6812 via R56

static const uint8_t ADDR_TCA       = 0x20;
static const uint8_t ADDR_INA_BAT   = 0x40;
static const uint8_t ADDR_INA_UMB   = 0x41;
static const uint8_t ADDR_ADS       = 0x48;

static const uint8_t EXP_BUZZER     = 1u << 0;  // OUT H = buzzer
static const uint8_t EXP_ROCKET_EN  = 1u << 1;  // OUT H = UMB_5V buck on (rocket power)
static const uint8_t EXP_KEY        = 1u << 2;  // IN  H = key switch J4 closed
static const uint8_t EXP_LORA_RST   = 1u << 3;  // OUT L = LoRa reset
static const uint8_t EXP_LORA_EN    = 1u << 4;  // OUT H = LoRa 3V3 on (U12)
static const uint8_t EXP_LORA_DIO0  = 1u << 5;  // IN
static const uint8_t EXP_FIRE_LED   = 1u << 6;  // OUT H = red light
static const uint8_t EXP_BTN        = 1u << 7;  // IN  L = local abort pressed

// =====================================================================
//  2. CONSTANTS
// =====================================================================
static const uint16_t VBAT_MIN_MV      = 6600;   // 2S
static const uint16_t CONT_MIN_MV      = 350;    // ADS1115 CONTx threshold
static const uint32_t POST_FIRE_MS     = 5000;   // FIRED -> IDLE (as Daidalos RAMPA_POST_FIRE_MS)
static const uint16_t ARIADNA_FIRE_MS  = 1000;   // Daidalos RAMPA_FIRE_PULSE_MS
static const uint32_t LINK_TIMEOUT_MS  = 1500;   // both protocols
static const uint32_t UNLOCK_MS        = 3000;   // forget the protocol after this much silence
static const uint32_t SCAN_SLOT_MS     = 600;    // alternate I2C / UART listening
static const uint32_t BOOT_I2C_MS      = 1500;   // first listen for Ariadna with UMB_5V off
static const uint16_t UMB_OVERCURRENT  = 2500;   // mA on UMB_5V -> switch it off
static const uint32_t RADIO_QUIET_AFTER_FIRE_MS = 30UL * 60UL * 1000UL;

enum Proto : uint8_t { P_NONE = 0, P_ZENITH = 1, P_ARIADNA = 2 };
enum Port  : uint8_t { PORT_OFF = 0, PORT_UART = 1, PORT_I2C = 2 };

// =====================================================================
//  3. STATE  (volatile: touched by the I2C slave interrupt too)
// =====================================================================
static Adafruit_NeoPixel s_rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static zl_rx_t  s_rx;

static volatile uint8_t  s_proto = P_NONE;
static volatile uint8_t  s_port = PORT_OFF;
static volatile uint8_t  s_state = ZR_ST_IDLE;     // numerically identical to RAMPA_STATE_*
static volatile uint8_t  s_faults = 0;             // live interlock faults (same bits in both protocols)
static volatile uint8_t  s_latched = 0;            // faults that caused an ABORT
static volatile uint8_t  s_flags = 0;              // Zenith-format flags (ZL_RMP_*)
static volatile uint8_t  s_last_seq = 0;
static volatile uint8_t  s_pyro_ch = 1;
static volatile uint16_t s_fire_ms = 1000;
static volatile uint8_t  s_rocket_go = 0;
static volatile uint32_t s_last_cmd_ms = 0;
static volatile bool     s_cd = false;
static volatile uint32_t s_t0_ms = 0;
static volatile uint32_t s_fire_end_ms = 0, s_fired_ms = 0;
static volatile bool     s_detach_req = false;     // Ariadna: float GP0/GP1 after the COMMITTED read
static volatile uint32_t s_evt = 0;                // events to print from the main loop

static uint8_t  s_exp_out = 0, s_exp_in = 0;
static bool     s_exp_ok = false, s_ina_ok = false, s_ina_umb_ok = false, s_ads_ok = false;
static volatile uint16_t s_vbat_mv = 0, s_ibat_ma = 0, s_umb_mv = 0, s_umb_ma = 0, s_v5_mv = 0;
static volatile uint16_t s_cont_mv[3] = { 0, 0, 0 };
static volatile bool     s_key = false, s_btn = false;
static bool     s_rocket_pwr = true;               // user wish (GS command); actual output also needs proto
static bool     s_umb_trip = false;

static bool     s_detached = false;
static uint32_t s_detach_ms = 0;
static uint32_t s_scan_slot_ms = 0;
static uint32_t s_boot_ms = 0;

static bool     s_lora_on = false, s_lora_ok = false;
static uint32_t s_lora_tx_cnt = 0, s_lora_rx_cnt = 0;
static bool     s_lora_beacon_now = false;
static uint8_t  s_ul_seq = 0, s_ul_rc = ZL_RC_NONE;
static bool     s_locate = false;

// USB standalone diagnostics (never active while a rocket is locked or in countdown/fire)
static bool     s_usb_stream = false;
static bool     s_diag = false;
static bool     s_diag_arm = false;
static uint8_t  s_diag_fire_ch = 0;
static uint32_t s_diag_fire_end = 0;
static uint32_t s_diag_buzz_end = 0;
static int8_t   s_diag_led = -1;                   // -1 auto, 0 off, 1 on
static int8_t   s_diag_rgb = -1;                   // -1 auto, 0 off, 1..7 R/G/B/Y/C/M/W
static int8_t   s_diag_lora = -1;                  // -1 auto, 0 off, 1 on

static uint8_t  s_i2c_tx[ARIADNA_RAMPA_STAT_SIZE];  // Ariadna status served from the ISR

#define EVT_ABORT     (1u << 0)
#define EVT_COMMIT    (1u << 1)
#define EVT_FIRE      (1u << 2)
#define EVT_LOCK_Z    (1u << 3)
#define EVT_LOCK_A    (1u << 4)
#define EVT_UNLOCK    (1u << 5)
#define EVT_DETACH    (1u << 6)
#define EVT_UMB_TRIP  (1u << 7)

// =====================================================================
//  4. INTERNAL I2C1 (TCA9534, INA226 x2, ADS1115)
// =====================================================================
static bool i2c_w8(uint8_t a, uint8_t r, uint8_t v)
{ Wire1.beginTransmission(a); Wire1.write(r); Wire1.write(v); return Wire1.endTransmission() == 0; }
static bool i2c_r8(uint8_t a, uint8_t r, uint8_t *v)
{
    Wire1.beginTransmission(a); Wire1.write(r);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)a, 1) != 1) return false;
    *v = (uint8_t)Wire1.read(); return true;
}
static bool i2c_w16(uint8_t a, uint8_t r, uint16_t v)
{ Wire1.beginTransmission(a); Wire1.write(r); Wire1.write((uint8_t)(v >> 8)); Wire1.write((uint8_t)v); return Wire1.endTransmission() == 0; }
static bool i2c_r16(uint8_t a, uint8_t r, uint16_t *v)
{
    Wire1.beginTransmission(a); Wire1.write(r);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)a, 2) != 2) return false;
    const uint8_t h = (uint8_t)Wire1.read(), l = (uint8_t)Wire1.read();
    *v = (uint16_t)((h << 8) | l); return true;
}

static void exp_write(uint8_t out)
{
    if (out == s_exp_out && s_exp_ok) return;
    s_exp_ok = i2c_w8(ADDR_TCA, 0x01, out);
    if (s_exp_ok) s_exp_out = out;
}
static bool exp_begin(void)
{
    s_exp_out = 0;                                     // everything off, rocket supply OFF until we know
    bool ok = i2c_w8(ADDR_TCA, 0x01, s_exp_out);
    ok = ok && i2c_w8(ADDR_TCA, 0x03, (uint8_t)(EXP_KEY | EXP_LORA_DIO0 | EXP_BTN));
    s_exp_ok = ok;
    return ok;
}
static void ads_start(uint8_t ch)
{
    const uint16_t cfg = (uint16_t)(0x8000u | ((uint16_t)(4u + (ch & 3u)) << 12) | (1u << 9) | (1u << 8) | (6u << 5) | 0x0003u);
    s_ads_ok = i2c_w16(ADDR_ADS, 0x01, cfg);
}

static void sensors_update(uint32_t now)
{
    static uint8_t ch = 0;
    static bool started = false;
    static uint32_t next = 0;
    if (!started) { ads_start(ch); started = s_ads_ok; }
    else {
        uint16_t cfg, raw;
        if (i2c_r16(ADDR_ADS, 0x01, &cfg) && (cfg & 0x8000u) && i2c_r16(ADDR_ADS, 0x00, &raw)) {
            int16_t sr = (int16_t)raw; if (sr < 0) sr = 0;
            if (ch < 3) s_cont_mv[ch] = (uint16_t)((uint32_t)sr >> 3);
            else s_v5_mv = (uint16_t)(((uint32_t)sr >> 3) * 2u);   // AIN3 = 5V_SENS/2
            ch = (uint8_t)((ch + 1u) & 3u);
            ads_start(ch);
        }
    }
    if ((int32_t)(now - next) < 0) return;
    next = now + 20;
    uint8_t in = 0;
    if (i2c_r8(ADDR_TCA, 0x00, &in)) {
        s_exp_ok = true;
        s_exp_in = in;
        s_key = (in & EXP_KEY) != 0;
        s_btn = (in & EXP_BTN) == 0;
    } else { s_key = false; s_exp_ok = false; }
    uint16_t v = 0, ib = 0, uv = 0, c = 0;
    s_ina_ok = i2c_r16(ADDR_INA_BAT, 0x02, &v);
    if (s_ina_ok) {
        s_vbat_mv = (uint16_t)(((uint32_t)v * 125u) / 100u);
        if (i2c_r16(ADDR_INA_BAT, 0x04, &ib)) s_ibat_ma = (int16_t)ib < 0 ? 0 : ib;
    }
    s_ina_umb_ok = i2c_r16(ADDR_INA_UMB, 0x02, &uv);
    if (s_ina_umb_ok) s_umb_mv = (uint16_t)(((uint32_t)uv * 125u) / 100u);
    if (i2c_r16(ADDR_INA_UMB, 0x04, &c)) s_umb_ma = (int16_t)c < 0 ? 0 : c;
}

// =====================================================================
//  5. INTERLOCKS & COMMON STATE MACHINE (called from loop and from the I2C ISR)
// =====================================================================
static void pyro_off(void)
{
    for (int i = 0; i < 3; i++) digitalWrite(PIN_FIRE[i], LOW);
    digitalWrite(PIN_PYRO_ARM, LOW);
}

static void compute_flags(void)
{
    uint8_t f = 0;
    if (s_key) f |= ZL_RMP_KEY;
    for (int i = 0; i < 3; i++) if (s_cont_mv[i] >= CONT_MIN_MV) f |= (uint8_t)(ZL_RMP_CONT1 << i);
    if (digitalRead(PIN_PYRO_ARM)) f |= ZL_RMP_PYRO_ARMED;
    if (!s_lora_on) f |= ZL_RMP_LORA_OFF;
    if (s_exp_out & EXP_ROCKET_EN) f |= ZL_RMP_ROCKET_PWR;
    uint8_t fm = 0;
    if (!s_exp_ok || !s_ina_ok || !s_ads_ok) fm |= ZL_RMPF_HW;
    if (s_vbat_mv < VBAT_MIN_MV) fm |= ZL_RMPF_LOW_VBAT;
    if (!s_key) fm |= ZL_RMPF_NO_KEY;
    const uint8_t ch = (s_pyro_ch >= 1 && s_pyro_ch <= 3) ? s_pyro_ch : 1;
    if (!(f & (ZL_RMP_CONT1 << (ch - 1)))) fm |= ZL_RMPF_NO_CONT;
    if (s_btn) fm |= ZL_RMPF_BTN_ABORT;
    if (fm == 0) f |= ZL_RMP_READY;
    s_flags = f;
    s_faults = fm;
}

static void enter_abort(uint8_t why)
{
    pyro_off();
    s_cd = false;
    s_latched |= why;
    s_state = ZR_ST_ABORT;
    s_evt |= EVT_ABORT;
}

/**
 * One command from either rocket. The command numbers 0..4 mean the same in
 * both protocols (PING, PREPARE, COUNTDOWN, COMMIT, ABORT); 5 is SAFE (Zenith)
 * or DETACH (Ariadna). Where Daidalos' rampa behaved differently, the Ariadna
 * branch reproduces it.
 */
static void rampa_cmd(uint8_t proto, uint8_t cmd, uint8_t seq, int32_t t_rem, bool go,
                      uint8_t pyro_ch, uint16_t fire_ms, uint32_t rx_ms)
{
    s_last_seq = seq;
    s_last_cmd_ms = rx_ms ? rx_ms : 1;
    s_rocket_go = go;
    if (pyro_ch >= 1 && pyro_ch <= 3 && s_state != ZR_ST_COMMITTED && s_state != ZR_ST_FIRED) s_pyro_ch = pyro_ch;
    if (fire_ms >= 50 && fire_ms <= 5000) s_fire_ms = fire_ms;
    compute_flags();
    const bool ariadna = proto == P_ARIADNA;

    switch (cmd) {
    case ZR_CMD_ABORT:
        if (s_state != ZR_ST_FIRED) enter_abort(0);
        break;

    case 5:
        if (ariadna) {                                   // RAMPA_CMD_DETACH
            pyro_off(); s_cd = false; s_state = ZR_ST_IDLE; s_detach_req = true;
        } else {                                         // ZR_CMD_SAFE
            pyro_off(); s_cd = false; s_latched = 0;
            if (s_state != ZR_ST_FIRED || (uint32_t)(rx_ms - s_fired_ms) > 2000u) s_state = ZR_ST_IDLE;
        }
        break;

    case ZR_CMD_PING:
        if (s_state == ZR_ST_FIRED) break;
        if (ariadna) {
            // Daidalos: a PING clears TIMEOUT/ROCKET/CLOCK and re-evaluates (also out of ABORT)
            pyro_off(); s_cd = false;
            s_latched &= (uint8_t)~(ZL_RMPF_TIMEOUT | ZL_RMPF_ROCKET | ZL_RMPF_CLOCK);
            s_state = (s_faults | s_latched) ? ZR_ST_FAULT : ZR_ST_IDLE;
        } else {
            if (s_state == ZR_ST_ABORT) break;           // Zenith: abort latched until SAFE
            pyro_off(); s_cd = false;
            const uint8_t setup_faults = ZL_RMPF_NO_CONT | ZL_RMPF_NO_KEY;   // normal while setting up the pad
            s_state = (s_faults & ~setup_faults) ? ZR_ST_FAULT : ZR_ST_IDLE;
        }
        break;

    case ZR_CMD_PREPARE:
        if (s_state == ZR_ST_FIRED && s_fire_end_ms) break;
        if (s_state == ZR_ST_ABORT && !ariadna) break;
        pyro_off(); s_cd = false;
        if (s_faults == 0 && go) { s_latched &= (uint8_t)~ZL_RMPF_ROCKET; s_state = ZR_ST_READY; }
        else { if (!go) s_latched |= ZL_RMPF_ROCKET; s_state = ZR_ST_FAULT; }
        break;

    case ZR_CMD_COUNTDOWN:
        if (s_state == ZR_ST_ABORT || s_state == ZR_ST_FIRED || s_state == ZR_ST_COMMITTED) break;
        if (t_rem <= 0 || t_rem > (int32_t)ZL_CD_MAX_MS) { enter_abort(ZL_RMPF_CLOCK); break; }
        if (!go) { enter_abort(ZL_RMPF_ROCKET); break; }
        if (s_faults) { enter_abort(s_faults); break; }
        {
            const uint32_t t0 = rx_ms + (uint32_t)t_rem;
            if (s_cd && labs((long)(int32_t)(t0 - s_t0_ms)) > 200) { enter_abort(ZL_RMPF_CLOCK); break; }
            s_t0_ms = t0;
        }
        s_cd = true;
        s_state = ZR_ST_COUNTDOWN;
        break;

    case ZR_CMD_COMMIT:
        if (s_state == ZR_ST_COMMITTED || s_state == ZR_ST_FIRED) break;    // repeated commits
        if (s_state != ZR_ST_COUNTDOWN) { enter_abort(ZL_RMPF_CLOCK); break; }
        if (!go) { enter_abort(ZL_RMPF_ROCKET); break; }
        if (s_faults) { enter_abort(s_faults); break; }
        if (t_rem < 5 || t_rem > (int32_t)ZL_RMP_COMMIT_MS + 50) { enter_abort(ZL_RMPF_CLOCK); break; }
        s_t0_ms = rx_ms + (uint32_t)t_rem;
        s_cd = true;
        s_state = ZR_ST_COMMITTED;
        s_evt |= EVT_COMMIT;
        break;

    default: break;
    }
}

// =====================================================================
//  6. ARIADNA (Daidalos): I2C0 slave 0x2B
// =====================================================================
static void ariadna_refresh_tx(uint32_t now)
{
    ariadna_rampa_status_t st;
    memset(&st, 0, sizeof st);
    st.state = s_state;
    st.last_seq = s_last_seq;
    uint8_t f = (uint8_t)(s_flags & (ARIADNA_RMP_KEY_ARMED | ARIADNA_RMP_CONT1 | ARIADNA_RMP_CONT2 |
                                     ARIADNA_RMP_CONT3 | ARIADNA_RMP_READY));
    f |= ARIADNA_RMP_LORA_OFF;                               // radio is always off with an Ariadna on the pogo
    if (s_state == ZR_ST_FIRED) f |= ARIADNA_RMP_PYRO_ARMED;
    if (s_state == ZR_ST_COMMITTED) f |= ARIADNA_RMP_DETACHING;
    st.flags = f;
    st.fault_mask = (uint8_t)(s_faults | s_latched);         // identical bit meanings
    st.vbat_mv = s_vbat_mv;
    st.t_rem_ms = s_cd ? (int32_t)(s_t0_ms - now) : -1;
    st.cont_mv[0] = s_cont_mv[0]; st.cont_mv[1] = s_cont_mv[1]; st.cont_mv[2] = s_cont_mv[2];
    ariadna_rampa_status_seal(&st);
    memcpy(s_i2c_tx, &st, sizeof st);
}

static void on_i2c_receive(int n)
{
    uint8_t buf[ARIADNA_RAMPA_CMD_SIZE];
    size_t got = 0;
    while (n-- > 0 && Wire.available()) {
        const int c = Wire.read();
        if (got < sizeof buf) buf[got++] = (uint8_t)c;
    }
    ariadna_rampa_cmd_t c;
    if (got != sizeof buf || ariadna_rampa_cmd_decode(buf, got, &c) != ARIADNA_RAMPA_OK) return;
    const uint32_t now = millis();
    if (s_proto != P_ARIADNA) { s_proto = P_ARIADNA; s_evt |= EVT_LOCK_A; }
    const uint8_t need = ARIADNA_RK_POST_OK | ARIADNA_RK_NAV_VALID | ARIADNA_RK_POS_OK |
                         ARIADNA_RK_NAV_OK | ARIADNA_RK_NO_ABORT;
    const bool go = (c.rocket_flags & need) == need;
    // processed HERE so that the read which follows in the same transaction
    // already carries the answer (Ariadna aborts if COMMITTED is not back in 25 ms)
    rampa_cmd(P_ARIADNA, c.cmd, c.seq, c.t_rem_ms, go, c.pyro_ch, ARIADNA_FIRE_MS, now);
    ariadna_refresh_tx(now);
}

static void on_i2c_request(void)
{
    Wire.write(s_i2c_tx, sizeof s_i2c_tx);
    if (s_state == ZR_ST_COMMITTED) s_detach_req = true;     // float the pins before T-0 (Daidalos)
}

// =====================================================================
//  7. ZENITH: UART0 115200
// =====================================================================
static void zenith_send_status(uint32_t now)
{
    zl_rmp_status_t st = {};
    st.t_rem_ms = s_cd ? (int32_t)(s_t0_ms - now) : -1;
    st.vbat_mv = s_vbat_mv;
    st.umb_ma = s_umb_ma;
    st.cont_mv[0] = s_cont_mv[0]; st.cont_mv[1] = s_cont_mv[1]; st.cont_mv[2] = s_cont_mv[2];
    st.state = s_state;
    st.flags = s_flags;
    st.faults = (uint8_t)(s_faults | s_latched);
    st.last_seq = s_last_seq;
    st.fw = (uint8_t)(ZENITH_FW_VERSION & 0xFF);
    uint8_t f[ZL_FRAME_MAX];
    const uint16_t n = zl_encode(f, ZL_PKT_RMP_STATUS, &st, sizeof st);
    if (Serial1.availableForWrite() >= (int)n) Serial1.write(f, n);
}

static void zenith_poll(void)
{
    int budget = 256;
    while (Serial1.available() && budget-- > 0) {
        const uint8_t *pl; uint8_t n;
        if (zl_rx_byte(&s_rx, (uint8_t)Serial1.read(), &pl, &n) != ZL_PKT_RMP_CMD) continue;
        zl_rmp_cmd_t c;
        if (!zl_take(&c, sizeof c, pl, n)) continue;
        const uint32_t now = millis();
        if (s_proto != P_ZENITH) { s_proto = P_ZENITH; s_evt |= EVT_LOCK_Z; }
        // the frame travelled ~1.7 ms (20 B @ 115200) after the rocket stamped t_rem
        const int32_t t_rem = c.t_rem_ms > 0 ? c.t_rem_ms - 2 : c.t_rem_ms;
        noInterrupts();
        rampa_cmd(P_ZENITH, c.cmd, c.seq, t_rem, (c.rocket_flags & ZL_RK_GO) != 0, c.pyro_ch, c.fire_ms, now);
        interrupts();
        zenith_send_status(now);
    }
}

// =====================================================================
//  8. UMBILICAL PORT: who owns GP0/GP1
// =====================================================================
static void port_close(void)
{
    if (s_port == PORT_UART) Serial1.end();
    if (s_port == PORT_I2C) Wire.end();
    pinMode(PIN_UMB_0, INPUT);
    pinMode(PIN_UMB_1, INPUT);
    s_port = PORT_OFF;
}

static void port_open(uint8_t p)
{
    if (s_port == p) return;
    port_close();
    if (p == PORT_UART) {
        Serial1.setTX(PIN_UMB_0); Serial1.setRX(PIN_UMB_1);
        Serial1.setFIFOSize(512);
        Serial1.begin(ZL_UMB_BAUD);
        zl_rx_init(&s_rx);
    } else if (p == PORT_I2C) {
        ariadna_refresh_tx(millis());
        Wire.setSDA(PIN_UMB_0); Wire.setSCL(PIN_UMB_1);
        Wire.begin((uint8_t)ARIADNA_RAMPA_I2C_ADDR);
        Wire.onReceive(on_i2c_receive);
        Wire.onRequest(on_i2c_request);
    }
    s_port = p;
}

static bool critical_state(void)
{
    return s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_COMMITTED ||
           (s_state == ZR_ST_FIRED && (uint32_t)(millis() - s_fired_ms) < POST_FIRE_MS);
}

static void umbilical_service(uint32_t now)
{
    // ---- Ariadna hot-detach (after COMMITTED was read, or 20 ms before T-0) --
    if (s_proto == P_ARIADNA && !s_detached) {
        const bool late = s_state == ZR_ST_COMMITTED && (int32_t)(s_t0_ms - now) <= 20;
        if (s_detach_req || late) {
            port_close();
            s_detached = true; s_detach_ms = now; s_detach_req = false;
            s_evt |= EVT_DETACH;
        }
    }
    if (s_detached) {
        // re-attach once the launch is over (or did not happen): Ariadna re-enables
        // its side 5 s after an unanswered commit
        const bool done = s_state != ZR_ST_COMMITTED &&
                          !(s_state == ZR_ST_FIRED && (uint32_t)(now - s_fired_ms) < POST_FIRE_MS);
        if (done && (uint32_t)(now - s_detach_ms) > 2000u) {
            s_detached = false;
            s_last_cmd_ms = now;                          // give the rocket time to come back
            port_open(PORT_I2C);
        }
        return;
    }

    // ---- locked: keep the port, drop the lock after a long silence ------------
    if (s_proto != P_NONE) {
        port_open(s_proto == P_ZENITH ? PORT_UART : PORT_I2C);
        if ((uint32_t)(now - s_last_cmd_ms) > UNLOCK_MS && !critical_state()) {
            s_proto = P_NONE;
            s_evt |= EVT_UNLOCK;
            s_scan_slot_ms = now;
        }
        return;
    }

    // ---- scanning --------------------------------------------------------------------
    if ((uint32_t)(now - s_boot_ms) < BOOT_I2C_MS) { port_open(PORT_I2C); return; }
    if ((uint32_t)(now - s_scan_slot_ms) >= SCAN_SLOT_MS) {
        s_scan_slot_ms = now;
        port_open(s_port == PORT_I2C ? PORT_UART : PORT_I2C);
    } else if (s_port == PORT_OFF) {
        port_open(PORT_I2C);
    }
}

/** UMB_5V: only for a Zenith (or while looking for one); never for an Ariadna. */
static bool rocket_supply_wanted(uint32_t now)
{
    if (!s_rocket_pwr || s_umb_trip) return false;
    if (s_proto == P_ARIADNA) return false;
    if (s_proto == P_NONE && (uint32_t)(now - s_boot_ms) < BOOT_I2C_MS) return false;
    return true;
}

// =====================================================================
//  9. STANDALONE LORA (no rocket on the pogo)
// =====================================================================
static void lora_power(bool on)
{
    if (on == s_lora_on) return;
    if (on) {
        exp_write((uint8_t)((s_exp_out | EXP_LORA_EN) & ~EXP_LORA_RST));
        delay(10);
        exp_write((uint8_t)(s_exp_out | EXP_LORA_RST));
        delay(10);
        SPI1.setSCK(PIN_LORA_SCK); SPI1.setTX(PIN_LORA_MOSI); SPI1.setRX(PIN_LORA_MISO);
        LoRa.setSPI(SPI1);
        LoRa.setPins(PIN_LORA_CS, -1, -1);          // DIO0 is on the TCA9534: polled
        if (LoRa.begin(ZL_LORA_FREQ_HZ)) {
            LoRa.setSpreadingFactor(ZL_LORA_SF);
            LoRa.setSignalBandwidth(ZL_LORA_BW_HZ);
            LoRa.setCodingRate4(ZL_LORA_CR4);
            LoRa.setPreambleLength(ZL_LORA_PREAMBLE);
            LoRa.setSyncWord(ZL_LORA_SYNCWORD);
            LoRa.setTxPower(ZL_LORA_TX_DBM);
            LoRa.enableCrc();
            LoRa.idle();
            s_lora_on = true;
            s_lora_ok = true;
            Serial.println("LoRa ON (no rocket on the umbilical)");
            if (s_usb_stream) Serial.printf("$EV,%lu,LoRa ON\n", (unsigned long)millis());
        } else {
            exp_write((uint8_t)(s_exp_out & ~(EXP_LORA_EN | EXP_LORA_RST)));
            s_lora_ok = false;
            Serial.println("LoRa init failed");
            if (s_usb_stream) Serial.printf("$EV,%lu,LoRa init failed\n", (unsigned long)millis());
        }
    } else {
        LoRa.end();
        SPI1.end();
        exp_write((uint8_t)(s_exp_out & ~(EXP_LORA_EN | EXP_LORA_RST)));
        s_lora_on = false;
        Serial.println("LoRa OFF (rocket on the umbilical)");
        if (s_usb_stream) Serial.printf("$EV,%lu,LoRa OFF\n", (unsigned long)millis());
    }
}

static uint8_t rampa_uplink(const zl_uplink_t &u)
{
    if (u.token != zl_ul_token_for(u.cmd)) return ZL_RC_REJ_TOKEN;
    uint8_t rc = ZL_RC_REJ_STATE;
    noInterrupts();
    switch (u.cmd) {
    case ZL_UL_NOP: rc = ZL_RC_OK; break;
    case ZL_UL_ROCKET_PWR: s_rocket_pwr = u.arg != 0; s_umb_trip = false; rc = ZL_RC_OK; break;
    case ZL_UL_ABORT: enter_abort(0); rc = ZL_RC_ABORTED; break;
    case ZL_UL_SAFE:  pyro_off(); s_cd = false; s_latched = 0; s_state = ZR_ST_IDLE; rc = ZL_RC_OK; break;
    case ZL_UL_LOCATE: s_locate = u.arg != 0; rc = ZL_RC_OK; break;
    default: break;                       // launching needs a rocket on the pogo
    }
    interrupts();
    return rc;
}

static void lora_service(uint32_t now)
{
    static uint32_t next_beacon = 0;
    static bool rx = false, ack = false;
    if (!s_lora_on) return;
    if (!s_lora_beacon_now && (now % ZL_WIN_PERIOD_MS) < ZL_WIN_DUR_MS) {
        if (!rx) { LoRa.receive(); rx = true; }
        const int sz = LoRa.parsePacket();
        if (sz > 0) {
            uint8_t b[32]; int n = 0;
            while (LoRa.available() && n < (int)sizeof b) b[n++] = (uint8_t)LoRa.read();
            while (LoRa.available()) LoRa.read();
            zl_uplink_t u;
            if (zl_lora_check(b, (size_t)n, sizeof u, ZL_UL_MAGIC)) {
                memcpy(&u, b, sizeof u);
                if (u.dest == ZL_UL_DEST_RAMPA && u.seq != s_ul_seq) {
                    s_ul_rc = rampa_uplink(u);
                    s_ul_seq = u.seq;
                    s_lora_rx_cnt++;
                    Serial.printf("UPLINK cmd %u arg %ld -> rc %u\n", u.cmd, (long)u.arg, s_ul_rc);
                    if (s_usb_stream) Serial.printf("$EV,%lu,UPLINK cmd %u arg %ld -> rc %u\n", (unsigned long)now, u.cmd, (long)u.arg, s_ul_rc);
                    ack = true;
                }
            }
        }
        return;
    }
    if (rx) { LoRa.idle(); rx = false; }
    int fin = 0;
    if (!s_lora_beacon_now) {
        if (zl_next_window_ms(now, -1, &fin) < ZL_WIN_TX_GUARD_MS) return;
        if (!ack && (int32_t)(now - next_beacon) < 0) return;
    }
    zl_rampa_beacon_t b = {};
    b.state = s_state; b.flags = s_flags; b.t_ms = now;
    b.vbat_mv = s_vbat_mv; b.umb_ma = s_umb_ma;
    b.cont_mv[0] = s_cont_mv[0]; b.cont_mv[1] = s_cont_mv[1]; b.cont_mv[2] = s_cont_mv[2];
    b.next_rx_in_ms = (uint16_t)zl_next_window_ms(millis(), -1, &fin);
    b.faults = (uint8_t)(s_faults | s_latched);
    b.last_ul_seq = s_ul_seq; b.last_ul_rc = s_ul_rc;
    b.fw = (uint8_t)(ZENITH_FW_VERSION & 0xFF);
    zl_lora_seal(&b, sizeof b, ZL_BEACON_MAGIC);
    if (LoRa.beginPacket()) { LoRa.write((const uint8_t *)&b, sizeof b); LoRa.endPacket(false); s_lora_tx_cnt++; }
    ack = false;
    s_lora_beacon_now = false;
    next_beacon = now + 1000;
}

// =====================================================================
//  10. ANNUNCIATORS & USB STANDALONE DIAGNOSTICS
// =====================================================================
static void diag_exit(bool stop_pyro)
{
    s_diag = false;
    s_diag_arm = false;
    s_diag_fire_ch = 0;
    s_diag_fire_end = 0;
    s_diag_led = -1;
    s_diag_rgb = -1;
    s_diag_lora = -1;
    if (stop_pyro) pyro_off();
}

static void outputs_update(uint32_t now)
{
    uint8_t out = (uint8_t)(s_exp_out & (EXP_LORA_EN | EXP_LORA_RST));
    if (rocket_supply_wanted(now)) out |= EXP_ROCKET_EN;
    uint32_t c;
    const int32_t rem = (int32_t)(s_t0_ms - now);
    switch (s_state) {
    case ZR_ST_FIRED:     out |= EXP_FIRE_LED | EXP_BUZZER; c = 0xFF0000; break;
    case ZR_ST_COMMITTED: out |= EXP_FIRE_LED | EXP_BUZZER; c = (now % 100u) < 50u ? 0xFF0000 : 0; break;
    case ZR_ST_COUNTDOWN: {
        const uint32_t per = rem < 10000 ? 250u : 1000u;
        if ((now % per) < 80u) out |= EXP_FIRE_LED | EXP_BUZZER;
        c = (now % 250u) < 125u ? 0xFFA000 : 0; break; }
    case ZR_ST_ABORT:     if ((now % 500u) < 250u) out |= EXP_FIRE_LED; c = (now % 250u) < 125u ? 0xFF00FF : 0; break;
    case ZR_ST_READY:     c = 0x00FF00; break;
    case ZR_ST_FAULT:     c = (now % 1000u) < 500u ? 0xFFFF00 : 0; break;
    default:
        // blue = Zenith, cyan = Daidalos, blinking blue = looking for a rocket
        c = s_proto == P_ZENITH ? 0x0000FF : s_proto == P_ARIADNA ? 0x00A0FF
                                           : ((now % 2000u) < 1000u ? 0x0000FF : 0x000020);
        break;
    }
    if (s_locate && (now % 2000u) < 500u) out |= EXP_BUZZER;
    if (s_diag_buzz_end) {
        if ((int32_t)(now - s_diag_buzz_end) < 0) out |= EXP_BUZZER;
        else s_diag_buzz_end = 0;
    }
    if (s_diag) {
        if (s_diag_led == 1) out |= EXP_FIRE_LED;
        else if (s_diag_led == 0) out &= (uint8_t)~EXP_FIRE_LED;
        static const uint32_t RGB_MAP[8] = {
            0x000000, 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0x00FFFF, 0xFF00FF, 0xFFFFFF
        };
        if (s_diag_rgb >= 0 && s_diag_rgb < 8) c = RGB_MAP[s_diag_rgb];
    }
    exp_write(out);
    static uint32_t c_last = 0xFFFFFFFF;
    if (c != c_last) { s_rgb.setPixelColor(0, s_rgb.Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF)); s_rgb.show(); c_last = c; }
}

static void print_events(void)
{
    noInterrupts();
    const uint32_t e = s_evt;
    s_evt = 0;
    interrupts();
    if (!e) return;
    const unsigned long t = (unsigned long)millis();
    if (e & EVT_LOCK_Z)   { Serial.println("ROCKET: Zenith (UART 115200)"); if (s_usb_stream) Serial.printf("$EV,%lu,ROCKET: Zenith (UART 115200)\n", t); }
    if (e & EVT_LOCK_A)   { Serial.println("ROCKET: Daidalos / Ariadna V2 (I2C slave 0x2B)"); if (s_usb_stream) Serial.printf("$EV,%lu,ROCKET: Daidalos / Ariadna V2 (I2C 0x2B)\n", t); }
    if (e & EVT_UNLOCK)   { Serial.println("ROCKET: none (scanning I2C / UART)"); if (s_usb_stream) Serial.printf("$EV,%lu,ROCKET: none (scanning)\n", t); }
    if (e & EVT_ABORT)    { Serial.printf("ABORT faults 0x%02X\n", s_latched | s_faults); if (s_usb_stream) Serial.printf("$EV,%lu,ABORT faults 0x%02X\n", t, s_latched | s_faults); }
    if (e & EVT_COMMIT)   { Serial.printf("COMMITTED, T-%ld ms\n", (long)(int32_t)(s_t0_ms - millis())); if (s_usb_stream) Serial.printf("$EV,%lu,COMMITTED T-%ld ms\n", t, (long)(int32_t)(s_t0_ms - millis())); }
    if (e & EVT_DETACH)   { Serial.println("Ariadna umbilical detached (GP0/GP1 Hi-Z)"); if (s_usb_stream) Serial.printf("$EV,%lu,Ariadna umbilical detached\n", t); }
    if (e & EVT_FIRE)     { Serial.printf("FIRE channel %u for %u ms\n", s_pyro_ch, s_fire_ms); if (s_usb_stream) Serial.printf("$EV,%lu,FIRE ch %u (%u ms)\n", t, s_pyro_ch, s_fire_ms); }
    if (e & EVT_UMB_TRIP) { Serial.printf("UMB_5V overcurrent %u mA -> rocket supply OFF\n", s_umb_ma); if (s_usb_stream) Serial.printf("$EV,%lu,UMB_5V overcurrent %u mA\n", t, s_umb_ma); }
}

static void usb_send_status(uint32_t now)
{
    if (Serial.availableForWrite() < 160) return;
    const uint8_t hw_mask = (uint8_t)((s_exp_ok ? 1u : 0u) |
                                      (s_ina_ok ? 2u : 0u) |
                                      (s_ina_umb_ok ? 4u : 0u) |
                                      (s_ads_ok ? 8u : 0u) |
                                      (s_lora_ok ? 16u : 0u));
    const uint8_t fire_mask = (uint8_t)((digitalRead(PIN_FIRE[0]) ? 1u : 0u) |
                                        (digitalRead(PIN_FIRE[1]) ? 2u : 0u) |
                                        (digitalRead(PIN_FIRE[2]) ? 4u : 0u));
    const int32_t t_rem = s_cd ? (int32_t)(s_t0_ms - now) : -1;
    const int32_t cmd_age = s_last_cmd_ms ? (int32_t)(now - s_last_cmd_ms) : -1;
    Serial.printf("$S,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%ld,%u,%u,%u,%ld,%u,%u,%lu,%lu\n",
                  (unsigned long)now, (unsigned)s_proto, (unsigned)s_port, (unsigned)s_state,
                  (unsigned)s_flags, (unsigned)(s_faults | s_latched), (unsigned)hw_mask,
                  (unsigned)s_vbat_mv, (unsigned)s_ibat_ma, (unsigned)s_umb_mv, (unsigned)s_umb_ma,
                  (unsigned)s_v5_mv, (unsigned)s_cont_mv[0], (unsigned)s_cont_mv[1], (unsigned)s_cont_mv[2],
                  s_key ? 1 : 0, s_btn ? 1 : 0, digitalRead(PIN_PYRO_ARM) ? 1 : 0, (unsigned)fire_mask,
                  s_lora_on ? 1 : 0, s_rocket_pwr ? 1 : 0, s_umb_trip ? 1 : 0, s_locate ? 1 : 0,
                  s_diag ? 1 : 0, s_diag_arm ? 1 : 0, (long)t_rem, (unsigned)s_pyro_ch, (unsigned)s_fire_ms,
                  (unsigned)s_last_seq, (long)cmd_age, (unsigned)s_exp_in, (unsigned)s_exp_out,
                  (unsigned long)s_lora_tx_cnt, (unsigned long)s_lora_rx_cnt);
}

static bool tok_eq(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = (*a >= 'a' && *a <= 'z') ? (char)(*a - 32) : *a;
        char cb = (*b >= 'a' && *b <= 'z') ? (char)(*b - 32) : *b;
        if (ca != cb) return false;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static void usb_handle_cmd(char *line, uint32_t now)
{
    char *tok[5] = {};
    int nt = 0;
    char *p = line;
    while (*p && nt < 5) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[nt++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = 0;
    }
    if (nt == 0) return;

    if (tok_eq(tok[0], "HELLO")) {
        s_usb_stream = true;
        Serial.printf("$ID,Rampa PCB1,%u,Zenith+Daidalos\n", (unsigned)ZENITH_FW_VERSION);
        usb_send_status(now);
        return;
    }
    if (tok_eq(tok[0], "STATUS") || tok_eq(tok[0], "PING")) {
        usb_send_status(now);
        return;
    }
    if (tok_eq(tok[0], "HELP")) {
        Serial.println("#OK CMDS: HELLO STATUS STREAM DIAG ARM FIRE ROCKETPWR LOCATE BUZZ LED RGB LORA I2C ABORT SAFE");
        return;
    }
    if (tok_eq(tok[0], "STREAM") && nt >= 2) {
        s_usb_stream = atoi(tok[1]) != 0;
        Serial.printf("#OK STREAM %d\n", s_usb_stream ? 1 : 0);
        return;
    }
    if (tok_eq(tok[0], "ABORT")) {
        noInterrupts();
        diag_exit(true);
        enter_abort(0);
        interrupts();
        Serial.println("#OK ABORT");
        return;
    }
    if (tok_eq(tok[0], "SAFE")) {
        noInterrupts();
        diag_exit(true);
        pyro_off();
        s_cd = false;
        s_latched = 0;
        if (s_state != ZR_ST_FIRED || (uint32_t)(now - s_fired_ms) > 2000u) s_state = ZR_ST_IDLE;
        interrupts();
        Serial.println("#OK SAFE");
        if (s_usb_stream) Serial.printf("$EV,%lu,USB SAFE -> IDLE\n", (unsigned long)now);
        return;
    }
    if (tok_eq(tok[0], "ROCKETPWR") && nt >= 2) {
        s_rocket_pwr = atoi(tok[1]) != 0;
        s_umb_trip = false;
        Serial.printf("#OK ROCKETPWR %d\n", s_rocket_pwr ? 1 : 0);
        if (s_usb_stream) Serial.printf("$EV,%lu,USB ROCKETPWR %d\n", (unsigned long)now, s_rocket_pwr ? 1 : 0);
        return;
    }
    if (tok_eq(tok[0], "LOCATE") && nt >= 2) {
        s_locate = atoi(tok[1]) != 0;
        Serial.printf("#OK LOCATE %d\n", s_locate ? 1 : 0);
        return;
    }
    if (tok_eq(tok[0], "BUZZ")) {
        long ms = nt >= 2 ? atol(tok[1]) : 300;
        if (ms < 50) ms = 50;
        if (ms > 5000) ms = 5000;
        s_diag_buzz_end = now + (uint32_t)ms;
        Serial.printf("#OK BUZZ %ld\n", ms);
        return;
    }
    if (tok_eq(tok[0], "I2C") && nt >= 2 && tok_eq(tok[1], "SCAN")) {
        if (critical_state()) { Serial.println("#ERR I2C SCAN blocked in countdown/fire"); return; }
        char out[128];
        int pos = snprintf(out, sizeof out, "$I2C");
        int found = 0;
        for (uint8_t a = 0x08; a <= 0x77; a++) {
            watchdog_update();
            Wire1.beginTransmission(a);
            if (Wire1.endTransmission() == 0) {
                found++;
                if (pos < (int)sizeof(out) - 6) pos += snprintf(out + pos, sizeof(out) - (size_t)pos, ",0x%02X", a);
            }
        }
        Serial.println(out);
        Serial.printf("#OK I2C SCAN %d\n", found);
        return;
    }
    if (tok_eq(tok[0], "DIAG") && nt >= 2) {
        const bool on = atoi(tok[1]) != 0;
        if (!on) {
            noInterrupts(); diag_exit(true); interrupts();
            Serial.println("#OK DIAG 0");
            if (s_usb_stream) Serial.printf("$EV,%lu,DIAG mode OFF\n", (unsigned long)now);
            return;
        }
        if (critical_state() || s_proto != P_NONE) {
            Serial.println("#ERR DIAG requires standalone IDLE/FAULT/ABORT (no rocket)");
            return;
        }
        s_diag = true;
        Serial.println("#OK DIAG 1");
        if (s_usb_stream) Serial.printf("$EV,%lu,DIAG mode ON\n", (unsigned long)now);
        return;
    }
    if (tok_eq(tok[0], "ARM") && nt >= 2) {
        const bool on = atoi(tok[1]) != 0;
        if (!on) {
            noInterrupts();
            s_diag_arm = false; s_diag_fire_ch = 0; s_diag_fire_end = 0;
            if (s_diag) pyro_off();
            interrupts();
            Serial.println("#OK ARM 0");
            if (s_usb_stream) Serial.printf("$EV,%lu,DIAG pyro rail OFF\n", (unsigned long)now);
            return;
        }
        if (!s_diag) { Serial.println("#ERR ARM requires DIAG 1"); return; }
        if (!s_key)  { Serial.println("#ERR ARM key switch J4 is OFF"); return; }
        if (s_btn)   { Serial.println("#ERR ARM abort button SW1 pressed"); return; }
        if (s_vbat_mv < VBAT_MIN_MV) { Serial.println("#ERR ARM low battery"); return; }
        s_diag_arm = true;
        Serial.println("#OK ARM 1");
        if (s_usb_stream) Serial.printf("$EV,%lu,DIAG pyro rail ARMED\n", (unsigned long)now);
        return;
    }
    if (tok_eq(tok[0], "FIRE") && nt >= 3) {
        const int ch = atoi(tok[1]);
        long ms = 300;
        bool confirmed = false;
        if (nt == 3 && tok_eq(tok[2], "CONFIRM")) {
            confirmed = true;
        } else if (nt >= 4 && tok_eq(tok[3], "CONFIRM")) {
            ms = atol(tok[2]);
            confirmed = true;
        }
        if (!confirmed) { Serial.println("#ERR FIRE syntax: FIRE <1..3> [ms] CONFIRM"); return; }
        if (ch < 1 || ch > 3) { Serial.println("#ERR FIRE channel must be 1..3"); return; }
        if (!s_diag || !s_diag_arm) { Serial.println("#ERR FIRE requires DIAG 1 and ARM 1"); return; }
        if (!s_key) { Serial.println("#ERR FIRE key switch J4 is OFF"); return; }
        if (s_btn)  { Serial.println("#ERR FIRE abort button SW1 pressed"); return; }
        if (s_vbat_mv < VBAT_MIN_MV) { Serial.println("#ERR FIRE low battery"); return; }
        if (ms < 50) ms = 50;
        if (ms > 2000) ms = 2000;
        noInterrupts();
        for (int i = 0; i < 3; i++) digitalWrite(PIN_FIRE[i], LOW);
        digitalWrite(PIN_PYRO_ARM, HIGH);
        digitalWrite(PIN_FIRE[ch - 1], HIGH);
        s_diag_fire_ch = (uint8_t)ch;
        s_diag_fire_end = now + (uint32_t)ms;
        interrupts();
        Serial.printf("#OK FIRE %d %ld\n", ch, ms);
        if (s_usb_stream) Serial.printf("$EV,%lu,DIAG FIRE ch %d (%ld ms)\n", (unsigned long)now, ch, ms);
        return;
    }
    if (tok_eq(tok[0], "LED") && nt >= 2) {
        if (!s_diag) { Serial.println("#ERR LED requires DIAG 1"); return; }
        if (tok_eq(tok[1], "AUTO")) s_diag_led = -1;
        else s_diag_led = atoi(tok[1]) != 0 ? 1 : 0;
        Serial.printf("#OK LED %s\n", tok[1]);
        return;
    }
    if (tok_eq(tok[0], "RGB") && nt >= 2) {
        if (!s_diag) { Serial.println("#ERR RGB requires DIAG 1"); return; }
        if (tok_eq(tok[1], "AUTO")) s_diag_rgb = -1;
        else if (tok_eq(tok[1], "OFF")) s_diag_rgb = 0;
        else if (tok_eq(tok[1], "R")) s_diag_rgb = 1;
        else if (tok_eq(tok[1], "G")) s_diag_rgb = 2;
        else if (tok_eq(tok[1], "B")) s_diag_rgb = 3;
        else if (tok_eq(tok[1], "Y")) s_diag_rgb = 4;
        else if (tok_eq(tok[1], "C")) s_diag_rgb = 5;
        else if (tok_eq(tok[1], "M")) s_diag_rgb = 6;
        else if (tok_eq(tok[1], "W")) s_diag_rgb = 7;
        else { Serial.println("#ERR RGB use AUTO|OFF|R|G|B|Y|C|M|W"); return; }
        Serial.printf("#OK RGB %s\n", tok[1]);
        return;
    }
    if (tok_eq(tok[0], "LORA") && nt >= 2) {
        if (!s_diag) { Serial.println("#ERR LORA requires DIAG 1"); return; }
        if (tok_eq(tok[1], "AUTO")) s_diag_lora = -1;
        else if (tok_eq(tok[1], "OFF")) { s_diag_lora = 0; lora_power(false); }
        else if (tok_eq(tok[1], "ON")) { s_diag_lora = 1; lora_power(true); }
        else if (tok_eq(tok[1], "BEACON")) {
            s_diag_lora = 1;
            if (!s_lora_on) lora_power(true);
            if (!s_lora_on) { Serial.println("#ERR LORA init failed"); return; }
            s_lora_beacon_now = true;
        } else { Serial.println("#ERR LORA use AUTO|ON|OFF|BEACON"); return; }
        Serial.printf("#OK LORA %s\n", tok[1]);
        return;
    }

    Serial.printf("#ERR unknown command: %s\n", tok[0]);
}

static void usb_poll(uint32_t now)
{
    static char buf[96];
    static uint8_t len = 0;
    int budget = 128;
    while (Serial.available() && budget-- > 0) {
        const int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\r' || ch == '\n') {
            if (len > 0) {
                buf[len] = 0;
                len = 0;
                usb_handle_cmd(buf, now);
            }
        } else if ((size_t)(len + 1u) < sizeof buf) {
            buf[len++] = (char)ch;
        }
    }
}

// =====================================================================
//  11. SETUP / LOOP
// =====================================================================
void setup()
{
    pinMode(PIN_PYRO_ARM, OUTPUT); digitalWrite(PIN_PYRO_ARM, LOW);
    for (int i = 0; i < 3; i++) { pinMode(PIN_FIRE[i], OUTPUT); digitalWrite(PIN_FIRE[i], LOW); }
    pinMode(PIN_UMB_0, INPUT); pinMode(PIN_UMB_1, INPUT);

    Serial.begin(115200);
    s_rgb.begin(); s_rgb.setBrightness(80); s_rgb.setPixelColor(0, s_rgb.Color(0, 0, 255)); s_rgb.show();

    Wire1.setSDA(PIN_INT_SDA); Wire1.setSCL(PIN_INT_SCL);
    Wire1.begin(); Wire1.setClock(400000); Wire1.setTimeout(5);
    exp_begin();
    i2c_w16(ADDR_INA_BAT, 0x00, 0x4727); i2c_w16(ADDR_INA_BAT, 0x05, 1024);
    i2c_w16(ADDR_INA_UMB, 0x00, 0x4727); i2c_w16(ADDR_INA_UMB, 0x05, 1024);   // 1 mA/LSB on 5 mOhm

    sensors_update(millis());
    compute_flags();
    s_boot_ms = millis();
    s_scan_slot_ms = s_boot_ms;
    port_open(PORT_I2C);                 // listen for an Ariadna first, rocket supply OFF
    watchdog_enable(1000, true);
}

void loop()
{
    const uint32_t now = millis();
    watchdog_update();

    sensors_update(now);
    noInterrupts();
    compute_flags();
    interrupts();

    if (s_port == PORT_UART) zenith_poll();
    umbilical_service(now);
    usb_poll(now);

    // ---- supervision (masked against the I2C slave interrupt) -------------------
    noInterrupts();
    if (s_btn && (s_state == ZR_ST_READY || s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_COMMITTED || s_diag)) {
        diag_exit(true);
        enter_abort(ZL_RMPF_BTN_ABORT);
    }
    if (s_diag) {
        if (s_proto != P_NONE || s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_COMMITTED || s_state == ZR_ST_FIRED) {
            diag_exit(s_state != ZR_ST_FIRED && s_state != ZR_ST_COMMITTED);
        } else {
            if (s_diag_arm && (!s_key || s_vbat_mv < VBAT_MIN_MV)) {
                s_diag_arm = false;
                s_diag_fire_ch = 0;
                s_diag_fire_end = 0;
                pyro_off();
            }
            if (s_diag_fire_end && (int32_t)(now - s_diag_fire_end) >= 0) {
                for (int i = 0; i < 3; i++) digitalWrite(PIN_FIRE[i], LOW);
                s_diag_fire_ch = 0;
                s_diag_fire_end = 0;
            }
            digitalWrite(PIN_PYRO_ARM, s_diag_arm ? HIGH : LOW);
            if (!s_diag_fire_end) {
                for (int i = 0; i < 3; i++) digitalWrite(PIN_FIRE[i], LOW);
            }
        }
    }
    if (s_state == ZR_ST_COUNTDOWN) {
        if ((uint32_t)(now - s_last_cmd_ms) > LINK_TIMEOUT_MS) enter_abort(ZL_RMPF_TIMEOUT);
        else if (!s_rocket_go) enter_abort(ZL_RMPF_ROCKET);
        else if (s_faults) enter_abort(s_faults);
        else if ((int32_t)(s_t0_ms - now) < -100) enter_abort(ZL_RMPF_CLOCK);   // no COMMIT came
    }
    if (s_state == ZR_ST_COMMITTED && (int32_t)(now - s_t0_ms) >= 0) {
        // last-moment interlocks: key, button, battery, igniter
        if (s_key && !s_btn && s_vbat_mv >= VBAT_MIN_MV && !(s_faults & ZL_RMPF_NO_CONT)) {
            digitalWrite(PIN_PYRO_ARM, HIGH);
            digitalWrite(PIN_FIRE[s_pyro_ch - 1], HIGH);
            s_fired_ms = now;
            s_fire_end_ms = now + s_fire_ms;
            s_cd = false;
            s_state = ZR_ST_FIRED;
            s_evt |= EVT_FIRE;
        } else {
            enter_abort(s_faults);
        }
    }
    if (s_state == ZR_ST_FIRED) {
        if (s_fire_end_ms && (int32_t)(now - s_fire_end_ms) >= 0) { pyro_off(); s_fire_end_ms = 0; }
        if (!s_fire_end_ms && (uint32_t)(now - s_fired_ms) > POST_FIRE_MS) s_state = ZR_ST_IDLE;
    }
    if (!s_diag && s_state != ZR_ST_FIRED && s_state != ZR_ST_COMMITTED && digitalRead(PIN_PYRO_ARM)) pyro_off();
    if (s_port == PORT_I2C) ariadna_refresh_tx(now);
    interrupts();

    // ---- rocket supply protection ------------------------------------------------
    if ((s_exp_out & EXP_ROCKET_EN) && s_umb_ma > UMB_OVERCURRENT) { s_umb_trip = true; s_evt |= EVT_UMB_TRIP; }

    // ---- radio ownership ----------------------------------------------------------
    static uint32_t no_rocket_since = 0;
    const bool rocket = s_proto != P_NONE || s_detached;
    if (rocket) { no_rocket_since = 0; if (s_lora_on) lora_power(false); }
    else if (s_diag && s_diag_lora >= 0) {
        lora_power(s_diag_lora == 1);
    } else {
        if (!no_rocket_since) no_rocket_since = now ? now : 1;
        // after a launch the rocket owns the 433 MHz channel: stay silent
        const bool quiet = s_state != ZR_ST_COMMITTED && s_state != ZR_ST_COUNTDOWN &&
                           !(s_fired_ms && (uint32_t)(now - s_fired_ms) < RADIO_QUIET_AFTER_FIRE_MS);
        if (!s_lora_on && quiet && (uint32_t)(now - no_rocket_since) > 2000u) lora_power(true);
    }
    lora_service(now);

    outputs_update(now);
    print_events();

    static uint32_t t_usb = 0;
    if (s_usb_stream && (int32_t)(now - t_usb) >= 0) {
        t_usb = now + 100;
        usb_send_status(now);
    }

    static uint32_t t_dbg = 0;
    if ((int32_t)(now - t_dbg) >= 0) {
        t_dbg = now + 1000;
        static const char *PN[3] = { "scan", "ZENITH", "ARIADNA" };
        Serial.printf("RAMPA %s port %u st %u flags 0x%02X faults 0x%02X vbat %u umb %u mA cont %u/%u/%u key %d btn %d lora %d\n",
                      PN[s_proto], s_port, s_state, s_flags, s_faults | s_latched, s_vbat_mv, s_umb_ma,
                      s_cont_mv[0], s_cont_mv[1], s_cont_mv[2], s_key, s_btn, s_lora_on);
    }
}
