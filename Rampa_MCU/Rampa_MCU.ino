// =====================================================================
//  RAMPA PCB1 — launch pad controller for the ZENITH rocket
//
//  Target: Raspberry Pi Pico 2 (RP2350, U7), arduino-pico core
//          (rp2040:rp2040:rpipico2). Needs: LoRa (sandeepmistry), Adafruit NeoPixel.
//  Hardware and pinout are exactly the Daidalos/Ariadna Rampa PCB1
//  (RampaNetlist_PCB1_2026-11-19.enet) — see ../PINOUT.md.
//
//  ---------------------------------------------------------------------
//  WHAT IT DOES
//  ---------------------------------------------------------------------
//  * Powers the rocket on the pad through the pogo umbilical (UMB_5V,
//    TCA9534 P1 ROCKET_EN) and talks to it over the same pogo pins as a
//    UART (115200 Bd):
//        J3.4  <- R21 1k  <- GP0 (UART0 TX)  -> rocket RX (Zenith J1.4)
//        J3.3  -> R22 100R-> GP1 (UART0 RX)  <- rocket TX (Zenith J1.3)
//    The ROCKET is the master: it sends a command frame every 50 ms and the
//    rampa answers each one with its status (zenith_link.h, ZL_PKT_RMP_*).
//  * While the rocket keeps talking (< 1.5 s), the rampa's own LoRa radio is
//    powered OFF (TCA9534 P4 -> U12) and everything goes through the rocket.
//    With NO rocket on the pogo the rampa switches its radio on, sends a
//    1 Hz beacon to the ground station and listens in the same 10 s TDMA
//    windows as the rocket would (rocket power on/off, abort/safe, locate).
//  * Countdown: the rocket sends ms-to-T-0 in every frame; the rampa locks
//    its own T-0 to it, keeps checking the rocket's GO flag and its own
//    interlocks (key, igniter continuity, battery, abort button), accepts a
//    COMMIT in the last 300 ms and fires the motor igniter at T-0 —
//    only if everything is still OK at that very moment.
//  * Abort sources: rocket ABORT, rocket GO flag dropped, rocket silent for
//    1.5 s during the countdown, local abort button, interlock fault.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/watchdog.h"
#include "zenith_link.h"

// =====================================================================
//  1. PINOUT (RampaNetlist_PCB1_2026-11-19.enet)
// =====================================================================
static const uint8_t PIN_UMB_TX     = 0;   // U7.1  -> R21 1k   -> J3.4
static const uint8_t PIN_UMB_RX     = 1;   // U7.2  <- R22 100R <- J3.3 (R23 100k pull-up)
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
//  2. SAFETY CONSTANTS
// =====================================================================
static const uint16_t VBAT_MIN_MV      = 6600;  // 2S
static const uint16_t CONT_MIN_MV      = 350;   // ADS1115 CONTx threshold
static const uint32_t POST_FIRE_MS     = 8000;  // FIRED -> IDLE after this if nobody says otherwise
static const bool     ROCKET_PWR_BOOT  = true;  // power the rocket over the pogo at boot
static const uint32_t ROCKET_LINK_MS   = ZL_RMP_TIMEOUT_MS;
static const uint32_t RADIO_QUIET_AFTER_FIRE_MS = 30UL * 60UL * 1000UL;  // rocket is flying: keep the band free

// =====================================================================
//  3. STATE
// =====================================================================
static Adafruit_NeoPixel s_rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
static zl_rx_t  s_rx;

static uint8_t  s_state = ZR_ST_IDLE;
static uint8_t  s_faults = 0;          // live interlock faults
static uint8_t  s_latched = 0;         // faults that caused an ABORT (kept until SAFE)
static uint8_t  s_flags = 0;
static uint8_t  s_last_seq = 0;
static uint8_t  s_pyro_ch = 1;
static uint16_t s_fire_ms = 1000;

static uint8_t  s_exp_out = 0;
static bool     s_exp_ok = false, s_ina_ok = false, s_ads_ok = false;
static uint16_t s_vbat_mv = 0, s_umb_ma = 0;
static uint16_t s_cont_mv[3] = { 0, 0, 0 };
static bool     s_key = false, s_btn = false;
static bool     s_rocket_pwr = ROCKET_PWR_BOOT;

static uint32_t s_last_cmd_ms = 0;     // last valid frame from the rocket
static bool     s_rocket_present = false;
static uint8_t  s_rocket_flags = 0;

static bool     s_cd = false;
static uint32_t s_t0_ms = 0;
static uint32_t s_fire_end_ms = 0, s_fired_ms = 0;

static bool     s_lora_on = false;
static uint8_t  s_ul_seq = 0, s_ul_rc = ZL_RC_NONE;
static bool     s_locate = false;

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
    s_exp_out = s_rocket_pwr ? EXP_ROCKET_EN : 0;              // LoRa off, buzzer off, light off
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
            ch = (uint8_t)((ch + 1u) & 3u);
            ads_start(ch);
        }
    }
    if ((int32_t)(now - next) < 0) return;
    next = now + 20;
    uint8_t in = 0;
    if (i2c_r8(ADDR_TCA, 0x00, &in)) {
        s_key = (in & EXP_KEY) != 0;
        s_btn = (in & EXP_BTN) == 0;
    } else { s_key = false; s_exp_ok = false; }
    uint16_t v = 0, c = 0;
    s_ina_ok = i2c_r16(ADDR_INA_BAT, 0x02, &v);
    if (s_ina_ok) s_vbat_mv = (uint16_t)(((uint32_t)v * 125u) / 100u);
    if (i2c_r16(ADDR_INA_UMB, 0x04, &c)) s_umb_ma = (int16_t)c < 0 ? 0 : c;
}

// =====================================================================
//  5. PYROS
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
    if (s_rocket_pwr) f |= ZL_RMP_ROCKET_PWR;

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
    Serial.printf("ABORT faults 0x%02X\n", s_latched | s_faults);
}

// =====================================================================
//  6. UMBILICAL (rocket is the master)
// =====================================================================
static void send_status(uint32_t now)
{
    zl_rmp_status_t st = {};
    st.t_rem_ms = s_cd ? (int32_t)(s_t0_ms - now) : -1;
    st.vbat_mv = s_vbat_mv;
    st.umb_ma = s_umb_ma;
    memcpy(st.cont_mv, s_cont_mv, sizeof st.cont_mv);
    st.state = s_state;
    st.flags = s_flags;
    st.faults = (uint8_t)(s_faults | s_latched);
    st.last_seq = s_last_seq;
    st.fw = (uint8_t)(ZENITH_FW_VERSION & 0xFF);
    uint8_t f[ZL_FRAME_MAX];
    const uint16_t n = zl_encode(f, ZL_PKT_RMP_STATUS, &st, sizeof st);
    if (Serial1.availableForWrite() >= (int)n) Serial1.write(f, n);
}

static void handle_rocket(const zl_rmp_cmd_t &c, uint32_t rx_ms)
{
    s_last_seq = c.seq;
    s_last_cmd_ms = rx_ms ? rx_ms : 1;
    s_rocket_flags = c.rocket_flags;
    if (c.pyro_ch >= 1 && c.pyro_ch <= 3 && s_state != ZR_ST_COMMITTED && s_state != ZR_ST_FIRED) s_pyro_ch = c.pyro_ch;
    if (c.fire_ms >= 50 && c.fire_ms <= 5000) s_fire_ms = c.fire_ms;
    compute_flags();
    const bool go = (c.rocket_flags & ZL_RK_GO) != 0;
    // frame travelled ~1.7 ms (20 B @ 115200) after the rocket stamped t_rem
    const int32_t t_rem = c.t_rem_ms > 0 ? c.t_rem_ms - 2 : c.t_rem_ms;

    switch (c.cmd) {
    case ZR_CMD_ABORT:
        if (s_state != ZR_ST_FIRED) enter_abort(0);
        break;
    case ZR_CMD_SAFE:
        pyro_off(); s_cd = false; s_latched = 0;
        if (s_state != ZR_ST_FIRED || (uint32_t)(rx_ms - s_fired_ms) > 2000u) s_state = ZR_ST_IDLE;
        break;
    case ZR_CMD_PING:
        if (s_state == ZR_ST_ABORT) break;                 // abort is latched until SAFE
        if (s_state == ZR_ST_FIRED && s_fire_end_ms) break;
        pyro_off(); s_cd = false;
        s_state = s_faults ? ZR_ST_FAULT : ZR_ST_IDLE;
        if (s_faults == ZL_RMPF_NO_CONT || s_faults == ZL_RMPF_NO_KEY || s_faults == (ZL_RMPF_NO_CONT | ZL_RMPF_NO_KEY))
            s_state = ZR_ST_IDLE;                          // normal while the pad is being set up
        break;
    case ZR_CMD_PREPARE:
        if (s_state == ZR_ST_ABORT || (s_state == ZR_ST_FIRED && s_fire_end_ms)) break;
        pyro_off(); s_cd = false;
        s_state = (s_faults == 0 && go) ? ZR_ST_READY : ZR_ST_FAULT;
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
        if (s_state == ZR_ST_COMMITTED || s_state == ZR_ST_FIRED) break;   // repeated commits
        if (s_state != ZR_ST_COUNTDOWN) { enter_abort(ZL_RMPF_CLOCK); break; }
        if (!go) { enter_abort(ZL_RMPF_ROCKET); break; }
        if (s_faults) { enter_abort(s_faults); break; }
        if (t_rem < 5 || t_rem > (int32_t)ZL_RMP_COMMIT_MS + 50) { enter_abort(ZL_RMPF_CLOCK); break; }
        s_t0_ms = rx_ms + (uint32_t)t_rem;
        s_state = ZR_ST_COMMITTED;
        Serial.printf("COMMITTED, T-%ld ms\n", (long)t_rem);
        break;
    default: break;
    }
}

static void umb_poll(uint32_t now)
{
    int budget = 256;
    while (Serial1.available() && budget-- > 0) {
        const uint8_t *pl; uint8_t n;
        if (zl_rx_byte(&s_rx, (uint8_t)Serial1.read(), &pl, &n) == ZL_PKT_RMP_CMD) {
            zl_rmp_cmd_t c;
            if (zl_take(&c, sizeof c, pl, n)) {
                handle_rocket(c, millis());
                send_status(millis());
            }
        }
    }
    s_rocket_present = s_last_cmd_ms && (uint32_t)(now - s_last_cmd_ms) < ROCKET_LINK_MS;
}

// =====================================================================
//  7. STANDALONE LORA (no rocket on the pogo)
// =====================================================================
static void lora_power(bool on)
{
    if (on == s_lora_on) return;
    if (on) {
        exp_write((uint8_t)((s_exp_out | EXP_LORA_EN) & ~EXP_LORA_RST));   // power on, held in reset
        delay(10);
        exp_write((uint8_t)(s_exp_out | EXP_LORA_RST));                      // release reset
        delay(10);
        SPI1.setSCK(PIN_LORA_SCK); SPI1.setTX(PIN_LORA_MOSI); SPI1.setRX(PIN_LORA_MISO);
        LoRa.setSPI(SPI1);
        LoRa.setPins(PIN_LORA_CS, -1, -1);          // DIO0 is on the TCA9534: we poll instead
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
            Serial.println("LoRa ON (no rocket on the umbilical)");
        } else {
            exp_write((uint8_t)(s_exp_out & ~(EXP_LORA_EN | EXP_LORA_RST)));
            Serial.println("LoRa init failed");
        }
    } else {
        LoRa.end();
        SPI1.end();
        exp_write((uint8_t)(s_exp_out & ~(EXP_LORA_EN | EXP_LORA_RST)));
        s_lora_on = false;
        Serial.println("LoRa OFF (rocket talks for us)");
    }
}

static uint8_t rampa_uplink(const zl_uplink_t &u)
{
    if (u.token != zl_ul_token_for(u.cmd)) return ZL_RC_REJ_TOKEN;
    switch (u.cmd) {
    case ZL_UL_NOP: return ZL_RC_OK;
    case ZL_UL_ROCKET_PWR:
        s_rocket_pwr = u.arg != 0;
        exp_write(s_rocket_pwr ? (uint8_t)(s_exp_out | EXP_ROCKET_EN) : (uint8_t)(s_exp_out & ~EXP_ROCKET_EN));
        return ZL_RC_OK;
    case ZL_UL_ABORT: enter_abort(0); return ZL_RC_ABORTED;
    case ZL_UL_SAFE:  pyro_off(); s_cd = false; s_latched = 0; s_state = ZR_ST_IDLE; return ZL_RC_OK;
    case ZL_UL_LOCATE: s_locate = u.arg != 0; return ZL_RC_OK;
    default: return ZL_RC_REJ_STATE;      // launching needs a rocket on the pogo
    }
}

static void lora_service(uint32_t now)
{
    static uint32_t next_beacon = 0;
    static bool rx = false, ack = false;
    if (!s_lora_on) return;
    const bool in_win = (now % ZL_WIN_PERIOD_MS) < ZL_WIN_DUR_MS;
    if (in_win) {
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
                    Serial.printf("UPLINK cmd %u arg %ld -> rc %u\n", u.cmd, (long)u.arg, s_ul_rc);
                    ack = true;
                }
            }
        }
        return;
    }
    if (rx) { LoRa.idle(); rx = false; }
    int fin = 0;
    const uint32_t to_win = zl_next_window_ms(now, -1, &fin);
    if (to_win < ZL_WIN_TX_GUARD_MS) return;
    if (!ack && (int32_t)(now - next_beacon) < 0) return;
    zl_rampa_beacon_t b = {};
    b.state = s_state; b.flags = s_flags; b.t_ms = now;
    b.vbat_mv = s_vbat_mv; b.umb_ma = s_umb_ma;
    memcpy(b.cont_mv, s_cont_mv, sizeof b.cont_mv);
    b.next_rx_in_ms = (uint16_t)zl_next_window_ms(millis(), -1, &fin);
    b.faults = (uint8_t)(s_faults | s_latched);
    b.last_ul_seq = s_ul_seq; b.last_ul_rc = s_ul_rc;
    b.fw = (uint8_t)(ZENITH_FW_VERSION & 0xFF);
    zl_lora_seal(&b, sizeof b, ZL_BEACON_MAGIC);
    if (LoRa.beginPacket()) { LoRa.write((const uint8_t *)&b, sizeof b); LoRa.endPacket(false); }
    ack = false;
    next_beacon = now + 1000;
}

// =====================================================================
//  8. ANNUNCIATORS
// =====================================================================
static void outputs_update(uint32_t now)
{
    uint8_t out = (uint8_t)(s_exp_out & (EXP_LORA_EN | EXP_LORA_RST));
    if (s_rocket_pwr) out |= EXP_ROCKET_EN;
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
    default:              c = s_rocket_present ? 0x0000FF : ((now % 2000u) < 1000u ? 0x0000FF : 0x000020); break;
    }
    if (s_locate && (now % 2000u) < 500u) out |= EXP_BUZZER;
    exp_write(out);
    static uint32_t c_last = 0xFFFFFFFF;
    if (c != c_last) { s_rgb.setPixelColor(0, s_rgb.Color((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF)); s_rgb.show(); c_last = c; }
}

// =====================================================================
//  9. SETUP / LOOP
// =====================================================================
void setup()
{
    pinMode(PIN_PYRO_ARM, OUTPUT); digitalWrite(PIN_PYRO_ARM, LOW);
    for (int i = 0; i < 3; i++) { pinMode(PIN_FIRE[i], OUTPUT); digitalWrite(PIN_FIRE[i], LOW); }

    Serial.begin(115200);
    s_rgb.begin(); s_rgb.setBrightness(80); s_rgb.setPixelColor(0, s_rgb.Color(0, 0, 255)); s_rgb.show();

    Wire1.setSDA(PIN_INT_SDA); Wire1.setSCL(PIN_INT_SCL);
    Wire1.begin(); Wire1.setClock(400000); Wire1.setTimeout(5);
    exp_begin();
    i2c_w16(ADDR_INA_BAT, 0x00, 0x4727); i2c_w16(ADDR_INA_BAT, 0x05, 1024);
    i2c_w16(ADDR_INA_UMB, 0x00, 0x4727); i2c_w16(ADDR_INA_UMB, 0x05, 1024);   // 1 mA/LSB on 5 mOhm

    Serial1.setTX(PIN_UMB_TX); Serial1.setRX(PIN_UMB_RX);
    Serial1.setFIFOSize(512);
    Serial1.begin(ZL_UMB_BAUD);
    zl_rx_init(&s_rx);

    watchdog_enable(1000, true);
}

void loop()
{
    const uint32_t now = millis();
    watchdog_update();

    sensors_update(now);
    compute_flags();
    umb_poll(now);

    // ---- local abort button always wins -------------------------------------------
    if (s_btn && (s_state == ZR_ST_READY || s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_COMMITTED))
        enter_abort(ZL_RMPF_BTN_ABORT);

    // ---- countdown supervision -----------------------------------------------------------
    if (s_state == ZR_ST_COUNTDOWN) {
        if ((uint32_t)(now - s_last_cmd_ms) > ZL_RMP_TIMEOUT_MS) enter_abort(ZL_RMPF_TIMEOUT);
        else if (!(s_rocket_flags & ZL_RK_GO)) enter_abort(ZL_RMPF_ROCKET);
        else if (s_faults) enter_abort(s_faults);
        else if ((int32_t)(s_t0_ms - now) < -100) enter_abort(ZL_RMPF_CLOCK);   // no COMMIT came
    }
    if (s_state == ZR_ST_COMMITTED && (int32_t)(now - s_t0_ms) >= 0) {
        // last-moment interlocks: key, button, battery, igniter
        compute_flags();
        if (s_key && !s_btn && s_vbat_mv >= VBAT_MIN_MV && !(s_faults & ZL_RMPF_NO_CONT)) {
            digitalWrite(PIN_PYRO_ARM, HIGH);
            digitalWrite(PIN_FIRE[s_pyro_ch - 1], HIGH);
            s_fired_ms = now;
            s_fire_end_ms = now + s_fire_ms;
            s_cd = false;
            s_state = ZR_ST_FIRED;
            Serial.printf("FIRE channel %u for %u ms\n", s_pyro_ch, s_fire_ms);
        } else {
            enter_abort(s_faults);
        }
    }
    if (s_state == ZR_ST_FIRED) {
        if (s_fire_end_ms && (int32_t)(now - s_fire_end_ms) >= 0) { pyro_off(); s_fire_end_ms = 0; }
        if (!s_fire_end_ms && (uint32_t)(now - s_fired_ms) > POST_FIRE_MS) s_state = ZR_ST_IDLE;
    }
    if (s_state != ZR_ST_FIRED && s_state != ZR_ST_COMMITTED) {
        // nothing may stay energised outside a firing pulse
        if (digitalRead(PIN_PYRO_ARM)) pyro_off();
    }

    // ---- radio ownership --------------------------------------------------------------------
    static uint32_t no_rocket_since = 0;
    if (s_rocket_present) { no_rocket_since = 0; if (s_lora_on) lora_power(false); }
    else {
        if (!no_rocket_since) no_rocket_since = now ? now : 1;
        // never switch the radio on around a launch: the rocket just left
        // After a launch the rocket owns the 433 MHz channel (its telemetry,
        // its RX windows) for the whole flight and recovery: stay silent.
        const bool quiet = s_state != ZR_ST_COMMITTED && s_state != ZR_ST_COUNTDOWN &&
                           !(s_fired_ms && (uint32_t)(now - s_fired_ms) < RADIO_QUIET_AFTER_FIRE_MS);
        if (!s_lora_on && quiet && (uint32_t)(now - no_rocket_since) > 2000u) lora_power(true);
    }
    lora_service(now);

    outputs_update(now);

    // ---- console status ----------------------------------------------------------------------
    static uint32_t t_dbg = 0;
    if ((int32_t)(now - t_dbg) >= 0) {
        t_dbg = now + 1000;
        Serial.printf("RAMPA st %u flags 0x%02X faults 0x%02X vbat %u umb %u mA cont %u/%u/%u key %d btn %d rocket %d lora %d\n",
                      s_state, s_flags, s_faults | s_latched, s_vbat_mv, s_umb_ma, s_cont_mv[0], s_cont_mv[1], s_cont_mv[2],
                      s_key, s_btn, s_rocket_present, s_lora_on);
    }
}
