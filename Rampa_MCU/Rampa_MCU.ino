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
//
//  ---------------------------------------------------------------------
//  THIRD USE: STATIC-FIRE THRUST STAND (no rocket at all)   stand_link.h
//  ---------------------------------------------------------------------
//  With nothing on the pogo the rampa listens on its own LoRa. The first
//  valid STAND packet from the ground station (mode TEST MOTORU) switches
//  it into stand mode: the umbilical is closed, the rampa streams thrust,
//  chamber pressure and two temperatures, and on command
//      ARM (key, igniter, battery, sensors, flash checked; record slot erased)
//      FIRE n s (countdown on the rampa, recording from T-3 s, igniter at T-0,
//                recording until T-0 + rec_s, results + 100 Hz curve to the GS)
//  Abort: ground station, local button SW1, key, igniter/battery/sensor loss,
//  or no packet from the station for 2.5 s during the countdown.
//  Core 1 (thrust_stand.h) samples the ADS1220 / transducer and records to
//  the W25Q128 at the full ADC rate; it never touches the pyro pins.
//  USB: text protocol for the diagnostic GUI (gui/rampa_diag.py), docs/USB_PROTOCOL.md.
//  A motor is never fired from USB — only from the ground station.
//
//  ---------------------------------------------------------------------
//  REMOTE ARMING SWITCH (servo, optional: SET arm_servo 1..4)
//  ---------------------------------------------------------------------
//  The pyro supply goes through TWO switches in series on J4: the key the
//  last person closes before leaving the pad, and a switch turned by a servo
//  (J12) once everybody is safe. The rampa turns the servo to arm_us only
//  when an arm command comes AND every other interlock is fine:
//      rocket on the pogo  PREPARE (Zenith READY / Ariadna "launch soon")
//      thrust stand        ARM from the ground station (LoRa)
//  and needs the key sense (after both switches) to close within
//  arm_settle_ms, else it turns back and reports the key fault. It returns
//  the servo to safe_us on SAFE / DISARM / ABORT / PING, right after the
//  igniter pulse, on link loss, and whenever no arm command keeps it armed.
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include "hardware/watchdog.h"
#include "zenith_link.h"
#include "ariadna_rampa_link.h"
#include "stand_link.h"
#include "thrust_stand.h"

#define RAMPA_FW_VERSION  300          // 3.00: + thrust stand

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
static const uint8_t PIN_SERVO[4]   = { 27, 28, 14, 22 };   // J12.1 / J12.4 / J12.7 / J12.10 (servo power = VBAT!)

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

static uint8_t  s_exp_out = 0;
static bool     s_exp_ok = false, s_ina_ok = false, s_ads_ok = false;
static volatile uint16_t s_vbat_mv = 0, s_umb_ma = 0;
static volatile uint16_t s_cont_mv[3] = { 0, 0, 0 };
static volatile bool     s_key = false, s_btn = false;
static volatile uint16_t s_v5_mv = 0;              // sensor supply 5V_SENS (ADS1115 AIN3 x2)
static bool     s_rocket_pwr = true;               // user wish (GS command); actual output also needs proto
static bool     s_umb_trip = false;

static bool     s_detached = false;
static uint32_t s_detach_ms = 0;
static uint32_t s_scan_slot_ms = 0;
static uint32_t s_boot_ms = 0;

static bool     s_lora_on = false;
static uint8_t  s_ul_seq = 0, s_ul_rc = ZL_RC_NONE;
static bool     s_ul_ack = false;               // answer a Zenith uplink in the next beacon
static bool     s_locate = false;

static uint8_t  s_i2c_tx[ARIADNA_RAMPA_STAT_SIZE];  // Ariadna status served from the ISR

#define EVT_ABORT     (1u << 0)
#define EVT_COMMIT    (1u << 1)
#define EVT_FIRE      (1u << 2)
#define EVT_LOCK_Z    (1u << 3)
#define EVT_LOCK_A    (1u << 4)
#define EVT_UNLOCK    (1u << 5)
#define EVT_DETACH    (1u << 6)
#define EVT_UMB_TRIP  (1u << 7)
#define EVT_STAND_IN  (1u << 8)
#define EVT_STAND_OUT (1u << 9)
#define EVT_SL_ARMED  (1u << 10)
#define EVT_SL_CD     (1u << 11)
#define EVT_SL_FIRE   (1u << 12)
#define EVT_SL_ABORT  (1u << 13)
#define EVT_SL_DONE   (1u << 14)
#define EVT_ARM_OK    (1u << 15)
#define EVT_ARM_FAIL  (1u << 16)

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
            else { s_v5_mv = (uint16_t)(((uint32_t)sr >> 3) * 2u); ts_v5_mv = s_v5_mv; }   // 5V_SENS / 2
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

// =====================================================================
//  4b. SERVOS (J12) AND THE REMOTE ARMING SWITCH
// =====================================================================
enum : uint8_t { AS_SAFE = 0, AS_ARMING = 1, AS_ARMED = 2, AS_FAIL = 3, AS_TEST = 4 };
static Servo    s_servo[4];
static uint16_t s_servo_us[4] = { 0, 0, 0, 0 };     // 0 = not driven
static volatile uint8_t  s_as = AS_SAFE;            // arming switch state
static volatile uint8_t  s_as_req = 0;              // 1 = arm, 2 = safe (also from the I2C ISR)
static volatile uint32_t s_as_hold_ms = 0;          // last rocket command that keeps it armed
static bool     s_as_fail_latch = false;            // a failed arming is not retried until released
static uint32_t s_as_ms = 0, s_as_test_end = 0;
static uint16_t s_as_close_ms = 0;                  // how long the key took to close
static uint8_t  s_armtest = 0;                      // USB ARMTEST: 1 running, 2 ok, 3 failed
static uint8_t  s_as_cfg_servo = 0xFF;

static bool arm_servo_on(void) { return ts_cfg.arm_servo >= 1 && ts_cfg.arm_servo <= 4; }
static void servo_write(uint8_t i, uint16_t us)
{
    if (i > 3) return;
    if (!s_servo[i].attached()) s_servo[i].attach(PIN_SERVO[i], 500, 2500);
    s_servo[i].writeMicroseconds(us);
    s_servo_us[i] = us;
}
static void servo_off(uint8_t i) { if (i <= 3) { if (s_servo[i].attached()) s_servo[i].detach(); s_servo_us[i] = 0; } }

static void arm_safe(uint32_t now)
{
    if (arm_servo_on()) servo_write((uint8_t)(ts_cfg.arm_servo - 1), ts_cfg.safe_us);
    s_as = AS_SAFE; s_as_ms = now;
}
static void arm_go(uint32_t now)
{
    if (!arm_servo_on() || s_as == AS_ARMING || s_as == AS_ARMED || s_as_fail_latch) return;
    servo_write((uint8_t)(ts_cfg.arm_servo - 1), ts_cfg.arm_us);
    s_as = AS_ARMING; s_as_ms = now;
}
/** True while the arming switch may be closed by the servo (or there is no servo). */
static bool arm_pending(void) { return arm_servo_on() && s_as == AS_ARMING; }

static void arm_service(uint32_t now);   // after the stand / rocket state (needs both)

static void enter_abort(uint8_t why)
{
    s_as_req = 2;
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
        s_as_req = 2;
        if (ariadna) {                                   // RAMPA_CMD_DETACH
            pyro_off(); s_cd = false; s_state = ZR_ST_IDLE; s_detach_req = true;
        } else {                                         // ZR_CMD_SAFE
            pyro_off(); s_cd = false; s_latched = 0;
            if (s_state != ZR_ST_FIRED || (uint32_t)(rx_ms - s_fired_ms) > 2000u) s_state = ZR_ST_IDLE;
        }
        break;

    case ZR_CMD_PING:
        s_as_req = 2;                                    // the rocket is not prepared: switch open
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
        if (arm_servo_on() && s_as != AS_FAIL && !s_as_fail_latch) {
            // remote arming switch: close it only if everything else is GO, READY once the key senses it
            if ((s_faults & ~ZL_RMPF_NO_KEY) == 0 && go) {
                s_as_hold_ms = rx_ms ? rx_ms : 1;
                s_as_req = 1;
                if (!s_key) { s_state = ZR_ST_IDLE; break; }
            } else s_as_req = 2;
        }
        if (s_faults == 0 && go) { s_latched &= (uint8_t)~ZL_RMPF_ROCKET; s_state = ZR_ST_READY; }
        else { if (!go) s_latched |= ZL_RMPF_ROCKET; s_state = ZR_ST_FAULT; }
        break;

    case ZR_CMD_COUNTDOWN:
        if (s_state == ZR_ST_ABORT || s_state == ZR_ST_FIRED || s_state == ZR_ST_COMMITTED) break;
        if (t_rem <= 0 || t_rem > (int32_t)ZL_CD_MAX_MS) { enter_abort(ZL_RMPF_CLOCK); break; }
        if (!go) { enter_abort(ZL_RMPF_ROCKET); break; }
        if (s_faults) { enter_abort(s_faults); break; }
        s_as_hold_ms = rx_ms ? rx_ms : 1;
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
            s_as_req = 2;
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
static bool s_stand_flag(void);
static bool rocket_supply_wanted(uint32_t now)
{
    if (!s_rocket_pwr || s_umb_trip) return false;
    if (s_stand_flag()) return false;                // thrust stand: nothing on the pogo
    if (s_proto == P_ARIADNA) return false;
    if (s_proto == P_NONE && (uint32_t)(now - s_boot_ms) < BOOT_I2C_MS) return false;
    return true;
}

static bool s_stand_flag(void);
// =====================================================================
//  9. STAND MODE (thrust stand, no rocket) — state machine on core 0
// =====================================================================
static bool     s_stand = false;                 // stand mode active (umbilical closed)
static bool s_stand_flag(void) { return s_stand; }
static uint8_t  s_sl = SL_ST_SAFE;               // sl_state_t
static uint8_t  s_sl_latched = 0;                // SL_FLT_* that caused the last ABORT / refusal
static uint32_t s_sl_rx_ms = 0;                  // last valid packet from the station
static uint32_t s_sl_usb_ms = 0;                 // last stand command over USB
static uint8_t  s_sl_seq = 0, s_sl_rc = SL_RC_NONE;
static uint32_t s_sl_t0 = 0, s_sl_fire_end = 0, s_sl_rec_end = 0, s_sl_state_ms = 0;
static uint32_t s_sl_rec_go_at = 0;
static bool     s_sl_rec_go = false, s_sl_finishing = false;
static uint8_t  s_sl_ch = 1;
static uint16_t s_sl_fire_ms = 1000;
static uint32_t s_sl_cold_s = 0;
static bool     s_sl_buzzer = false, s_sl_hang = false, s_sl_fired = false;
static int32_t  s_sl_curve_off = -1;             // CURVE answer due
static int16_t  s_bu_next = -1, s_bu_last = -1;  // BURN burst: next block to send, last one
static uint32_t s_bu_at = 0;                     // not before (listening gap)
static uint8_t  s_bu_sent = 0;                   // blocks since the last TLM
static uint8_t  s_sl_res_tx = 0;                 // RES copies still to send
static uint32_t s_c1_alive = 0, s_c1_alive_ms = 0;
static bool     s_c1_ok = true;

#define SL_ARM_FAULTS  (SL_FLT_VBAT | SL_FLT_KEY | SL_FLT_CONT | SL_FLT_ADS | SL_FLT_FLASH | SL_FLT_BTN | SL_FLT_SYS)

static uint8_t sl_faults_live(void)
{
    uint8_t f = 0;
    if (s_vbat_mv < VBAT_MIN_MV) f |= SL_FLT_VBAT;
    if (!s_key) f |= SL_FLT_KEY;
    const uint8_t ch = (s_sl_ch >= 1 && s_sl_ch <= 3) ? s_sl_ch : 1;
    if (s_cont_mv[ch - 1] < CONT_MIN_MV) f |= SL_FLT_CONT;
    if (!ts_ads_ok) f |= SL_FLT_ADS;
    if (!ts_flash_ok) f |= SL_FLT_FLASH;
    if (s_btn) f |= SL_FLT_BTN;
    if (!s_c1_ok || !s_exp_ok || !s_ina_ok || !s_ads_ok) f |= SL_FLT_SYS;
    return f;
}

static void sl_set(uint8_t st, uint32_t now) { s_sl = st; s_sl_state_ms = now; ts_stand_state = st; }

static void stand_enter(uint32_t now)
{
    if (s_stand) return;
    if (s_proto != P_NONE || s_detached) return;     // a rocket is on the pogo: never
    s_stand = true;
    port_close();
    sl_set(SL_ST_SAFE, now);
    s_sl_latched = 0;
    s_sl_ch = (ts_cfg.pyro_ch >= 1 && ts_cfg.pyro_ch <= 3) ? ts_cfg.pyro_ch : 1;
    s_sl_fire_ms = ts_cfg.fire_ms;
    s_evt |= EVT_STAND_IN;
}

static void stand_leave(uint32_t now)
{
    if (!s_stand) return;
    pyro_off(); ts_fire_on = false;
    s_stand = false;
    sl_set(SL_ST_SAFE, now);
    s_last_cmd_ms = now; s_scan_slot_ms = now;
    s_evt |= EVT_STAND_OUT;
}

/** Stops/cancels whatever the recorder does (the request is retried by the loop if core 1 is busy). */
static bool s_sl_stop_req = false, s_sl_stop_abort = false;
static void sl_stop_recorder(bool aborted) { s_sl_stop_req = true; s_sl_stop_abort = aborted; }

static void stand_abort(uint8_t why, uint32_t now)
{
    pyro_off(); ts_fire_on = false;
    s_sl_latched |= why;
    if (s_sl == SL_ST_FIRING || s_sl == SL_ST_BURN) {   // motor already lit: keep recording the burn
        if (s_sl == SL_ST_FIRING) sl_set(SL_ST_BURN, now);
        return;
    }
    if (s_sl == SL_ST_ARMING || s_sl == SL_ST_ARMED || s_sl == SL_ST_COUNTDOWN || s_sl == SL_ST_RECORD)
        sl_stop_recorder(true);
    sl_set(SL_ST_ABORT, now);
    s_evt |= EVT_SL_ABORT;
}

/** One stand command (LoRa or USB). Returns sl_rc_t. */
static uint8_t stand_cmd(uint8_t cmd, int32_t arg, uint16_t arg2, uint16_t token, bool radio, uint32_t now)
{
    if (token != sl_token_for(cmd)) return SL_RC_REJ_ARG;
    const bool idle = s_sl == SL_ST_SAFE;
    switch (cmd) {
    case SL_C_PING:
        return SL_RC_OK;

    case SL_C_ABORT:
        stand_abort(0, now);
        return SL_RC_ABORTED;

    case SL_C_DISARM:
        if (s_sl == SL_ST_FIRING || s_sl == SL_ST_BURN || s_sl == SL_ST_RECORD) return SL_RC_REJ_STATE;
        if (s_sl == SL_ST_COUNTDOWN || s_sl == SL_ST_ARMED || s_sl == SL_ST_ARMING) sl_stop_recorder(true);
        pyro_off(); ts_fire_on = false;
        s_sl_latched = 0;
        sl_set(SL_ST_SAFE, now);
        return SL_RC_OK;

    case SL_C_ARM: {
        if (!radio) return SL_RC_REJ_STATE;                       // arming only from the ground station
        if (!idle) return SL_RC_REJ_STATE;
        if (arg < 1 || arg > 3) return SL_RC_REJ_ARG;
        const uint16_t fms = arg2 ? arg2 : ts_cfg.fire_ms;
        if (fms < SL_FIRE_MS_MIN || fms > SL_FIRE_MS_MAX) return SL_RC_REJ_ARG;
        s_sl_ch = (uint8_t)arg; s_sl_fire_ms = fms;
        const uint32_t blocks = ts_blocks_for(ts_cfg.rec_s + 5u);
        uint8_t f = (uint8_t)(sl_faults_live() & SL_ARM_FAULTS);
        if (arm_servo_on()) f &= (uint8_t)~SL_FLT_KEY;           // the servo closes the arming switch now
        if (ts_flash_ok && ts_flash_free() < blocks * TS_BLOCK) f |= SL_FLT_FLASH;
        if (ts_rec != TR_IDLE) f |= SL_FLT_FLASH;
        if (f) { s_sl_latched = f; return SL_RC_REJ_INTLK; }
        if (!ts_request(TQ_REC_START, (int32_t)blocks, 0)) return SL_RC_BUSY;
        s_sl_latched = 0; s_sl_hang = false; s_sl_fired = false;
        s_as_fail_latch = false;
        arm_go(now);
        sl_set(SL_ST_ARMING, now);
        return SL_RC_OK; }

    case SL_C_FIRE: {
        if (!radio) return SL_RC_REJ_STATE;                       // never from USB
        if (s_sl != SL_ST_ARMED) return SL_RC_REJ_STATE;
        if (arg < SL_CD_MIN_S || arg > SL_CD_MAX_S) return SL_RC_REJ_ARG;
        const uint8_t f = (uint8_t)(sl_faults_live() & SL_ARM_FAULTS);
        if (f) { stand_abort(f, now); return SL_RC_REJ_INTLK; }
        s_sl_t0 = now + (uint32_t)arg * 1000u;
        s_sl_rec_go_at = (uint32_t)arg > 3u ? s_sl_t0 - 3000u : now;
        s_sl_rec_go = false;
        sl_set(SL_ST_COUNTDOWN, now);
        s_evt |= EVT_SL_CD;
        return SL_RC_OK; }

    case SL_C_TARE:
        if (!(idle || s_sl == SL_ST_ARMED)) return SL_RC_REJ_STATE;
        if (!ts_ads_ok) return SL_RC_REJ_INTLK;
        return ts_request(TQ_TARE) ? SL_RC_OK : SL_RC_BUSY;

    case SL_C_CAL:
        if (!idle) return SL_RC_REJ_STATE;
        if (arg <= 0) return SL_RC_REJ_ARG;
        return ts_request(TQ_CAL, arg) ? SL_RC_OK : SL_RC_BUSY;

    case SL_C_PZERO:
        if (!idle) return SL_RC_REJ_STATE;
        return ts_request(TQ_PZERO) ? SL_RC_OK : SL_RC_BUSY;

    case SL_C_RECORD: {
        if (!idle) return SL_RC_REJ_STATE;
        if (arg < 1 || arg > SL_REC_MAX_S) return SL_RC_REJ_ARG;
        const uint32_t blocks = ts_blocks_for((uint32_t)arg + 2u);
        if (!ts_flash_ok || ts_flash_free() < blocks * TS_BLOCK || ts_rec != TR_IDLE) { s_sl_latched = SL_FLT_FLASH; return SL_RC_REJ_INTLK; }
        if (!ts_request(TQ_REC_START, (int32_t)blocks, 1)) return SL_RC_BUSY;
        s_sl_cold_s = (uint32_t)arg; s_sl_rec_go = false; s_sl_finishing = false; s_sl_fired = false; s_sl_hang = false;
        s_sl_latched = 0;
        sl_set(SL_ST_RECORD, now);
        return SL_RC_OK; }

    case SL_C_STOP:
        if (s_sl == SL_ST_RECORD || s_sl == SL_ST_BURN) { s_sl_rec_end = now; return SL_RC_OK; }
        return SL_RC_REJ_STATE;

    case SL_C_RESULT:
        if (!ts_res_ready) return SL_RC_REJ_STATE;
        s_sl_res_tx = 1;
        return SL_RC_OK;

    case SL_C_CURVE:
        if (!ts_res_ready || ts_rec == TR_RECORDING) return SL_RC_REJ_STATE;
        if (arg < 0 || arg >= (int32_t)ts_cn) { s_sl_curve_off = arg < 0 ? 0 : arg; return SL_RC_REJ_ARG; }
        s_sl_curve_off = arg;
        return SL_RC_OK;

    case SL_C_BUZZER:
        s_sl_buzzer = arg != 0;
        return SL_RC_OK;

    case SL_C_BURN:
        if (!ts_res_ready || !ts_burn.valid || ts_rec != TR_IDLE || !(idle || s_sl == SL_ST_ABORT)) return SL_RC_REJ_STATE;
        if (arg < 0 || arg >= ts_burn.nblocks) return SL_RC_REJ_ARG;
        s_bu_next = (int16_t)arg;
        s_bu_last = (int16_t)((arg2 == 0 || arg + arg2 > ts_burn.nblocks) ? ts_burn.nblocks - 1 : arg + arg2 - 1);
        s_bu_at = now + 10u;
        s_bu_sent = 0;
        return SL_RC_OK;

    case SL_C_RECTIME: {
        if (!idle) return SL_RC_REJ_STATE;
        if (arg < SL_REC_MIN_S || arg > SL_REC_MAX_S) return SL_RC_REJ_ARG;
        if (!ts_request_done()) return SL_RC_BUSY;
        ts_cfg_new = ts_cfg; ts_cfg_new.rec_s = (uint16_t)arg;
        return ts_request(TQ_SET_CFG) ? SL_RC_OK : SL_RC_BUSY; }

    default:
        return SL_RC_REJ_ARG;
    }
}

/** The stand state machine, every loop. */
static void stand_service(uint32_t now)
{
    // ---- measurement core alive?
    if (ts_alive != s_c1_alive) { s_c1_alive = ts_alive; s_c1_alive_ms = now; s_c1_ok = true; }
    else if ((uint32_t)(now - s_c1_alive_ms) > 1000u) s_c1_ok = false;

    // ---- pending recorder stop (core 1 may be busy with a request)
    if (s_sl_stop_req && ts_request(TQ_REC_STOP, s_sl_stop_abort ? 1 : 0)) s_sl_stop_req = false;

    if (!s_stand) return;
    ts_stand_state = s_sl;
    const uint8_t live = sl_faults_live();
    const uint32_t link_age = s_sl_rx_ms ? (uint32_t)(now - s_sl_rx_ms) : 0xFFFFFFFFu;

    switch (s_sl) {
    case SL_ST_ARMING:
        if (arm_servo_on() && s_as == AS_FAIL) { stand_abort(SL_FLT_KEY, now); break; }
        if (ts_rec == TR_READY && (!arm_servo_on() || s_as == AS_ARMED)) { sl_set(SL_ST_ARMED, now); s_evt |= EVT_SL_ARMED; break; }
        if ((ts_rec == TR_IDLE && ts_request_done() && (uint32_t)(now - s_sl_state_ms) > 300u) ||
            (uint32_t)(now - s_sl_state_ms) > 60000u) { stand_abort(SL_FLT_FLASH, now); break; }
        {
            const uint8_t watch = (uint8_t)(SL_FLT_CONT | SL_FLT_BTN | SL_FLT_VBAT | (arm_pending() ? 0 : SL_FLT_KEY));
            if (live & watch) stand_abort((uint8_t)(live & SL_ARM_FAULTS & (arm_pending() ? ~SL_FLT_KEY : 0xFF)), now);
        }
        break;

    case SL_ST_ARMED:
        if (live & SL_ARM_FAULTS) { stand_abort(live & SL_ARM_FAULTS, now); break; }
        if (link_age > SL_ARMED_LINK_MS) { stand_abort(SL_FLT_LINK, now); break; }
        if ((uint32_t)(now - s_sl_state_ms) > SL_ARMED_MAX_MS) { sl_stop_recorder(false); sl_set(SL_ST_SAFE, now); }
        break;

    case SL_ST_COUNTDOWN: {
        const uint8_t f = (uint8_t)(live & SL_ARM_FAULTS & ~SL_FLT_FLASH);
        if (f) { stand_abort(f, now); break; }
        if (link_age > SL_LINK_TIMEOUT_MS) { stand_abort(SL_FLT_LINK, now); break; }
        if (!s_sl_rec_go && (int32_t)(now - s_sl_rec_go_at) >= 0) {
            if (ts_rec != TR_READY && ts_rec != TR_RECORDING) { stand_abort(SL_FLT_FLASH, now); break; }
            if (ts_rec == TR_RECORDING || ts_request(TQ_REC_GO)) s_sl_rec_go = true;
        }
        if ((int32_t)(now - s_sl_t0) >= 0) {
            // T-0: last look, then fire
            if (ts_rec != TR_RECORDING) { stand_abort(SL_FLT_FLASH, now); break; }
            if (!s_key || s_btn || s_vbat_mv < VBAT_MIN_MV || s_cont_mv[s_sl_ch - 1] < CONT_MIN_MV) {
                stand_abort((uint8_t)(live & SL_ARM_FAULTS), now); break;
            }
            ts_t0_req = true;
            ts_fire_on = true;
            digitalWrite(PIN_PYRO_ARM, HIGH);
            digitalWrite(PIN_FIRE[s_sl_ch - 1], HIGH);
            s_sl_fire_end = now + s_sl_fire_ms;
            s_sl_rec_end = s_sl_t0 + (uint32_t)ts_cfg.rec_s * 1000u;
            s_sl_fired = true; s_sl_finishing = false;
            sl_set(SL_ST_FIRING, now);
            s_evt |= EVT_SL_FIRE;
        }
        break; }

    case SL_ST_FIRING:
        if ((int32_t)(now - s_sl_fire_end) >= 0 || s_btn || !s_key) {
            pyro_off(); ts_fire_on = false;
            sl_set(SL_ST_BURN, now);
        }
        break;

    case SL_ST_BURN:
    case SL_ST_RECORD:
        if (s_sl == SL_ST_RECORD && !s_sl_rec_go) {
            if (ts_rec == TR_READY && ts_request(TQ_REC_GO)) {
                s_sl_rec_go = true; s_sl_t0 = now; s_sl_rec_end = now + s_sl_cold_s * 1000u;
            } else if ((ts_rec == TR_IDLE && ts_request_done() && (uint32_t)(now - s_sl_state_ms) > 300u) ||
                       (uint32_t)(now - s_sl_state_ms) > 60000u) {
                s_sl_latched = SL_FLT_FLASH; sl_set(SL_ST_SAFE, now);
            }
            break;
        }
        if (!s_sl_finishing) {
            if ((int32_t)(now - s_sl_rec_end) >= 0 || ts_rec != TR_RECORDING) { sl_stop_recorder(false); s_sl_finishing = true; }
        } else if (!s_sl_stop_req && ts_request_done() && ts_rec == TR_IDLE) {
            s_sl_finishing = false;
            s_sl_hang = s_sl_fired && !(ts_res.flags & SL_RF_IGNITION);
            s_sl_res_tx = 3;
            sl_set(s_sl_latched && s_sl == SL_ST_BURN ? SL_ST_ABORT : SL_ST_SAFE, now);
            s_evt |= EVT_SL_DONE;
        }
        break;

    case SL_ST_SAFE:
    case SL_ST_ABORT:
    default:
        if (digitalRead(PIN_PYRO_ARM)) { pyro_off(); ts_fire_on = false; }
        // nobody talks to the stand any more: become a launch pad again
        if (s_sl == SL_ST_SAFE && ts_rec == TR_IDLE &&
            link_age > SL_MODE_EXIT_MS && (uint32_t)(now - s_sl_usb_ms) > SL_MODE_EXIT_MS &&
            (uint32_t)(now - s_sl_state_ms) > SL_MODE_EXIT_MS)
            stand_leave(now);
        break;
    }
    if (s_btn && s_sl != SL_ST_SAFE && s_sl != SL_ST_ABORT && s_sl != SL_ST_BURN && s_sl != SL_ST_RECORD)
        stand_abort(SL_FLT_BTN, now);
}

static uint16_t stand_flags(void)
{
    uint16_t f = 0;
    if (s_key) f |= SL_F_KEY;
    if (s_cont_mv[s_sl_ch - 1] >= CONT_MIN_MV) f |= SL_F_CONT;
    if (digitalRead(PIN_PYRO_ARM)) f |= SL_F_ARM_OUT;
    for (int i = 0; i < 3; i++) if (digitalRead(PIN_FIRE[i])) f |= SL_F_FIRE_OUT;
    if (ts_rec == TR_RECORDING) f |= SL_F_REC;
    if (ts_ads_ok) f |= SL_F_ADS_OK;
    if (ts_flash_ok && ts_flash_free() >= ts_blocks_for(ts_cfg.rec_s + 5u) * TS_BLOCK) f |= SL_F_FLASH_OK;
    if (ts_cfg.lc_scale != 0.0f) f |= SL_F_LC_CAL;
    if (ts_res_ready) f |= SL_F_RESULT;
    if (s_sl_hang) f |= SL_F_HANGFIRE;
    if (s_btn) f |= SL_F_BTN;
    if (s_sl_buzzer) f |= SL_F_BUZZER;
    if (ts_tared) f |= SL_F_TARED;
    return f;
}

// ---------------------------------------------------------------- live points (core 1 -> core 0)
#define LV_HIST 128
static ts_live_t s_lv[LV_HIST];
static uint32_t  s_lv_n = 0;                    // points taken so far
static uint32_t  s_lv_r = 0;                    // ring read index (ts_live)

static void live_pull(void)
{
    uint32_t w = ts_live_w;
    __sync_synchronize();
    if (w - s_lv_r > TS_LIVE_RING) s_lv_r = w - TS_LIVE_RING / 2;   // fell behind: skip
    while (s_lv_r != w) {
        s_lv[s_lv_n % LV_HIST] = ts_live[s_lv_r % TS_LIVE_RING];
        s_lv_r++; s_lv_n++;
    }
}
static const ts_live_t *live_last(void) { return s_lv_n ? &s_lv[(s_lv_n - 1) % LV_HIST] : nullptr; }

/** Stand telemetry from the live points received since the previous TLM. */
static uint32_t s_tlm_from = 0;                 // first live point not yet sent
static void build_tlm(sl_tlm_t &t, uint32_t now)
{
    memset(&t, 0, sizeof t);
    t.state = s_sl;
    t.faults = (uint8_t)(s_sl_latched | sl_faults_live());
    t.flags = stand_flags();
    t.ack_seq = s_sl_seq; t.ack_rc = s_sl_rc;
    t.vbat_mv = s_vbat_mv; t.v5_mv = s_v5_mv;
    if (s_sl == SL_ST_COUNTDOWN || s_sl == SL_ST_FIRING || s_sl == SL_ST_BURN || (s_sl == SL_ST_RECORD && s_sl_rec_go))
        t.t_rel_ms = (int32_t)(now - s_sl_t0);
    uint32_t first = s_tlm_from;
    if (s_lv_n - first > LV_HIST) first = s_lv_n - LV_HIST;
    const uint32_t n = s_lv_n - first;
    float fv[SL_SUBS], fpk = -1e30f, ppk = -1e30f, fabsmax = 0;
    for (int i = 0; i < SL_SUBS; i++) fv[i] = NAN;
    if (n) {
        for (int i = 0; i < SL_SUBS; i++) {
            const uint32_t a = first + (uint32_t)i * n / SL_SUBS, b = first + (uint32_t)(i + 1) * n / SL_SUBS;
            double s = 0; uint32_t k = 0;
            for (uint32_t j = a; j < (b > a ? b : a + 1); j++) { const float v = s_lv[j % LV_HIST].f; if (v == v) { s += v; k++; } }
            fv[i] = k ? (float)(s / k) : NAN;
        }
        for (uint32_t j = first; j < s_lv_n; j++) {
            const ts_live_t &L = s_lv[j % LV_HIST];
            if (L.fmax == L.fmax && L.fmax > fpk) fpk = L.fmax;
            if (L.pmax == L.pmax && L.pmax > ppk) ppk = L.pmax;
        }
    }
    s_tlm_from = s_lv_n;
    for (int i = 0; i < SL_SUBS; i++) if (fv[i] == fv[i] && fabsf(fv[i]) > fabsmax) fabsmax = fabsf(fv[i]);
    if (fpk > fabsmax) fabsmax = fpk;
    t.fexp = sl_fexp_for(fabsmax);
    for (int i = 0; i < SL_SUBS; i++) t.f[i] = fv[i] == fv[i] ? sl_fpack(fv[i], t.fexp) : 0;
    t.fpeak = fpk > -1e29f ? sl_fpack(fpk, t.fexp) : 0;
    const ts_live_t *L = live_last();
    t.p_dpsi = L ? sl_sat16(L->p * 10.0f) : SL_T_NONE;
    t.ppeak_dpsi = ppk > -1e29f ? sl_sat16(ppk * 10.0f) : t.p_dpsi;
    t.t1_dc = L ? sl_sat16(L->t1 * 10.0f) : SL_T_NONE;
    t.t2_dc = L ? sl_sat16(L->t2 * 10.0f) : SL_T_NONE;
    if (L && L->t1 == L->t1) t.flags |= SL_F_TH1;
    if (L && L->t2 == L->t2) t.flags |= SL_F_TH2;
    if (L && L->p_v > 0.3f && L->p_v < 4.9f) t.flags |= SL_F_PRESS;
    t.test_no = ts_test_no;
    t.flash_free_pct = (uint8_t)(ts_flash_ok ? (uint64_t)ts_flash_free() * 100u / (TS_FLASH_SIZE - TS_REC_BASE) : 0);
    t.progress = ts_progress;
    t.sps = ts_sps;
    uint8_t cb = 0;
    for (int i = 0; i < 3; i++) if (s_cont_mv[i] >= CONT_MIN_MV) cb |= (uint8_t)(1u << (4 + i));
    t.pyro = (uint8_t)((s_sl_ch & 0x03) | cb | (arm_servo_on() ? 0x04 : 0) | (s_as == AS_ARMED ? 0x08 : 0));
    sl_seal(&t, sizeof t, SL_TLM_MAGIC);
}

static void build_res(sl_res_t &r)
{
    r = ts_res;
    sl_seal(&r, sizeof r, SL_RES_MAGIC);
}

/** Block k of the cropped burn (each point = mean of `div` 100 Hz curve points). */
static void build_burn(sl_burn_t &p, int k)
{
    const ts_burn_t &B = ts_burn;
    memset(&p, 0, sizeof p);
    p.test_no = ts_test_no;
    p.idx = (uint8_t)k; p.n_blocks = B.nblocks; p.fexp = B.fexp; p.rate_hz = B.rate_hz; p.flags = ts_res.flags;
    const int32_t base = ts_ct0 >= 0 ? ts_ct0 : 0;              // time zero of the curve
    p.t_first_ms = (B.crop_a - base) * (1000 / SL_CURVE_HZ);
    p.burn_a_ms = B.burn ? (B.a - base) * (1000 / SL_CURVE_HZ) : SL_T_NOBURN;
    p.burn_b_ms = B.burn ? (B.b + 1 - base) * (1000 / SL_CURVE_HZ) : SL_T_NOBURN;   // end of the last point
    const int first = k * SL_BURN_PTS;
    const int i0 = B.crop_a + first * B.div;
    p.t1_dc = i0 < (int)ts_cn ? ts_ct1[i0] : SL_T_NONE;
    p.t2_dc = i0 < (int)ts_cn ? ts_ct2[i0] : SL_T_NONE;
    uint8_t n = 0;
    for (int j = first; j < B.npts && n < SL_BURN_PTS; j++, n++) {
        double f = 0, pr = 0; int m = 0;
        for (int i = B.crop_a + j * B.div; i < B.crop_a + (j + 1) * B.div && i < (int)ts_cn; i++, m++) { f += ts_cf[i]; pr += ts_cp[i]; }
        p.f[n] = m ? sl_fpack((float)(f / m), B.fexp) : 0;
        p.p[n] = m ? (int16_t)(pr / m + (pr >= 0 ? 0.5 : -0.5)) : 0;
    }
    p.n = n;
    sl_seal(&p, sizeof p, SL_BURN_MAGIC);
}

static bool build_curve(sl_curve_t &c, int32_t off)
{
    memset(&c, 0, sizeof c);
    c.test_no = ts_test_no;
    c.offset = (uint16_t)off;
    c.fexp = ts_res.curve_fexp;
    uint8_t n = 0;
    for (int32_t i = off; i < (int32_t)ts_cn && n < SL_CRV_PTS; i++, n++) {
        c.f[n] = sl_fpack(ts_cf[i], c.fexp);
        c.p[n] = ts_cp[i];
    }
    c.n = n;
    sl_seal(&c, sizeof c, SL_CRV_MAGIC);
    return true;
}

/**
 * Drives the arming servo: requests from the rocket commands / stand, the
 * key-sense check, and the rule that nothing keeps it armed by accident.
 */
static void arm_service(uint32_t now)
{
    if (!ts_ready) return;
    if (ts_cfg.arm_servo != s_as_cfg_servo) {               // boot / reconfigured: park in SAFE
        if (s_as_cfg_servo >= 1 && s_as_cfg_servo <= 4 && s_as_cfg_servo != ts_cfg.arm_servo)
            servo_write((uint8_t)(s_as_cfg_servo - 1), s_servo_us[s_as_cfg_servo - 1] ? s_servo_us[s_as_cfg_servo - 1] : ts_cfg.safe_us);
        s_as_cfg_servo = ts_cfg.arm_servo;
        s_as_fail_latch = false;
        arm_safe(now);
    }
    if (!arm_servo_on()) { s_as = AS_SAFE; s_as_req = 0; return; }
    const uint8_t r = s_as_req;
    if (r) {
        s_as_req = 0;
        if (r == 2) { if (s_as != AS_SAFE || s_servo_us[ts_cfg.arm_servo - 1] != ts_cfg.safe_us) arm_safe(now); s_as_fail_latch = false; }
        else arm_go(now);
    }
    if (s_as == AS_ARMING) {
        if (s_key) { s_as = AS_ARMED; s_as_close_ms = (uint16_t)(now - s_as_ms); s_evt |= EVT_ARM_OK; }
        else if ((uint32_t)(now - s_as_ms) > ts_cfg.arm_settle_ms) {
            servo_write((uint8_t)(ts_cfg.arm_servo - 1), ts_cfg.safe_us);
            s_as = AS_FAIL; s_as_fail_latch = true; s_evt |= EVT_ARM_FAIL;
        }
    }
    if (s_armtest == 1) {
        if (s_as == AS_ARMED) { s_armtest = 2; s_as = AS_TEST; s_as_test_end = now + 3000; }
        else if (s_as == AS_FAIL) { s_armtest = 3; s_as_fail_latch = false; }
    }
    if (s_as == AS_TEST && (int32_t)(now - s_as_test_end) >= 0) arm_safe(now);

    // who may keep the switch closed?
    if (s_as == AS_ARMING || s_as == AS_ARMED) {
        bool hold;
        if (s_armtest == 1) hold = true;
        else if (s_stand) hold = s_sl == SL_ST_ARMING || s_sl == SL_ST_ARMED || s_sl == SL_ST_COUNTDOWN || s_sl == SL_ST_FIRING;
        else hold = s_state == ZR_ST_COMMITTED || (s_state == ZR_ST_FIRED && s_fire_end_ms) ||
                    (s_as_hold_ms && (uint32_t)(now - s_as_hold_ms) < 1500u &&
                     (s_state == ZR_ST_READY || s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_IDLE));
        if (!hold) arm_safe(now);
    }
}

/** Arming switch of a test from USB: allowed only when nothing is armed or counting. */
static bool arm_usb_allowed(void)
{
    if (s_stand && s_sl != SL_ST_SAFE && s_sl != SL_ST_ABORT) return false;
    if (s_state == ZR_ST_READY || s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_COMMITTED || s_state == ZR_ST_FIRED) return false;
    return s_as != AS_ARMING && s_as != AS_ARMED;
}

// =====================================================================
//  10. STANDALONE LORA (no rocket on the pogo): Zenith beacon or stand link
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
            Serial.println("# LoRa ON (no rocket on the umbilical)");
        } else {
            exp_write((uint8_t)(s_exp_out & ~(EXP_LORA_EN | EXP_LORA_RST)));
            Serial.println("# LoRa init failed");
        }
    } else {
        LoRa.end();
        SPI1.end();
        exp_write((uint8_t)(s_exp_out & ~(EXP_LORA_EN | EXP_LORA_RST)));
        s_lora_on = false;
        Serial.println("# LoRa OFF (rocket on the umbilical)");
    }
}

static uint8_t rampa_uplink(const zl_uplink_t &u)
{
    if (u.token != zl_ul_token_for(u.cmd)) return ZL_RC_REJ_TOKEN;
    uint8_t rc = ZL_RC_REJ_STATE;
    const uint32_t now = millis();
    noInterrupts();
    switch (u.cmd) {
    case ZL_UL_NOP: rc = ZL_RC_OK; break;
    case ZL_UL_ROCKET_PWR: s_rocket_pwr = u.arg != 0; s_umb_trip = false; rc = ZL_RC_OK; break;
    case ZL_UL_ABORT:
        enter_abort(0);
        if (s_stand) stand_abort(0, now);
        rc = ZL_RC_ABORTED; break;
    case ZL_UL_SAFE:
        pyro_off(); s_cd = false; s_latched = 0; s_state = ZR_ST_IDLE;
        if (s_stand && (s_sl == SL_ST_ABORT || s_sl == SL_ST_ARMED || s_sl == SL_ST_ARMING)) stand_cmd(SL_C_DISARM, 0, 0, 0, true, now);
        rc = ZL_RC_OK; break;
    case ZL_UL_LOCATE: s_locate = u.arg != 0; rc = ZL_RC_OK; break;
    default: break;                       // launching needs a rocket on the pogo
    }
    interrupts();
    return rc;
}

static bool     s_lora_rx = false;
static uint32_t s_lora_free_at = 0;           // async TX in progress until then
static uint32_t s_sl_next_tlm = 0, s_sl_reply_at = 0;
static bool     s_sl_reply = false;           // answer the station right after its packet
static int      s_lora_rssi = 0;

static void lora_send(const void *p, size_t n)
{
    if (!LoRa.beginPacket()) return;
    LoRa.write((const uint8_t *)p, n);
    LoRa.endPacket(true);                          // async: core 0 keeps running (T-0 timing)
    s_lora_rx = false;
    s_lora_free_at = millis() + zl_lora_airtime_us((uint16_t)n) / 1000u + 3u;
}

static void lora_receive(uint32_t now)
{
    const int sz = LoRa.parsePacket();
    if (sz <= 0) return;
    uint8_t b[80]; int n = 0;
    while (LoRa.available() && n < (int)sizeof b) b[n++] = (uint8_t)LoRa.read();
    while (LoRa.available()) LoRa.read();
    s_lora_rssi = LoRa.packetRssi();

    sl_cmd_pkt_t c;
    if (sl_check(b, (size_t)n, sizeof c, SL_CMD_MAGIC)) {
        memcpy(&c, b, sizeof c);
        stand_enter(now);
        if (!s_stand) return;
        s_sl_rx_ms = now;
        if (c.seq != s_sl_seq) {
            if (c.cmd != SL_C_PING && c.cmd != SL_C_BURN) s_bu_next = -1;   // anything else stops a burst
            s_sl_rc = stand_cmd(c.cmd, c.arg, c.arg2, c.token, true, now);
            s_sl_seq = c.seq;
            if (c.cmd != SL_C_PING) Serial.printf("# STAND cmd %u arg %ld -> rc %u\n", c.cmd, (long)c.arg, s_sl_rc);
        } else if (c.cmd == SL_C_CURVE && s_sl_rc == SL_RC_OK) {
            s_sl_curve_off = c.arg;                    // the station missed the answer: again
        }
        if (s_sl_curve_off >= 0 || s_sl_res_tx) { s_sl_reply = true; s_sl_reply_at = now + 8u; }
        return;
    }
    zl_uplink_t u;
    if (zl_lora_check(b, (size_t)n, sizeof u, ZL_UL_MAGIC)) {
        memcpy(&u, b, sizeof u);
        if (u.dest == ZL_UL_DEST_RAMPA && u.seq != s_ul_seq) {
            s_ul_rc = rampa_uplink(u);
            s_ul_seq = u.seq;
            Serial.printf("# UPLINK cmd %u arg %ld -> rc %u\n", u.cmd, (long)u.arg, s_ul_rc);
            s_ul_ack = true;
        }
    }
}

static void lora_service(uint32_t now)
{
    static uint32_t next_beacon = 0;
    if (!s_lora_on) return;
    if ((int32_t)(now - s_lora_free_at) < 0) return;          // transmitting
    if (!s_lora_rx) { LoRa.receive(); s_lora_rx = true; }     // listen whenever not transmitting
    lora_receive(now);

    if (s_stand) {
        // keep the air quiet in the last 150 ms before T-0 (the station may be sending ABORT)
        if (s_sl == SL_ST_COUNTDOWN && (int32_t)(s_sl_t0 - now) < 150 && (int32_t)(s_sl_t0 - now) > -20) return;
        // BURN burst: blocks back to back, a listening gap after each, a TLM every 8 blocks
        if (s_bu_next >= 0 && (s_sl == SL_ST_SAFE || s_sl == SL_ST_ABORT) && ts_burn.valid) {
            if ((int32_t)(now - s_bu_at) < 0) return;
            if (s_bu_sent >= 8) {
                s_bu_sent = 0;
                sl_tlm_t t; build_tlm(t, now); lora_send(&t, sizeof t);
            } else {
                sl_burn_t p; build_burn(p, s_bu_next); lora_send(&p, sizeof p);
                s_bu_sent++;
                if (++s_bu_next > s_bu_last) s_bu_next = -1;
            }
            s_bu_at = s_lora_free_at + SL_BURN_GAP_MS;
            s_sl_next_tlm = s_bu_at + 300u;          // the regular TLM waits for the burst
            return;
        }
        s_bu_next = -1;
        if (s_sl_reply && (int32_t)(now - s_sl_reply_at) >= 0) {
            s_sl_reply = false;
            if (s_sl_curve_off >= 0) { sl_curve_t c; build_curve(c, s_sl_curve_off); s_sl_curve_off = -1; lora_send(&c, sizeof c); return; }
            if (s_sl_res_tx) { sl_res_t r; build_res(r); s_sl_res_tx--; lora_send(&r, sizeof r); return; }
        }
        if ((int32_t)(now - s_sl_next_tlm) < 0) return;
        const bool fast = s_sl != SL_ST_SAFE && s_sl != SL_ST_ABORT;
        s_sl_next_tlm = now + (fast ? SL_TLM_FAST_MS : SL_TLM_SLOW_MS);
        if (s_sl_res_tx && s_sl == SL_ST_SAFE && (s_sl_next_tlm & 1u)) {   // results interleaved with telemetry
            sl_res_t r; build_res(r); s_sl_res_tx--; lora_send(&r, sizeof r); return;
        }
        sl_tlm_t t; build_tlm(t, now); lora_send(&t, sizeof t);
        return;
    }

    // ---- Zenith beacon (pad alone): quiet inside the RX windows of the 10 s grid
    if ((now % ZL_WIN_PERIOD_MS) < ZL_WIN_DUR_MS) return;
    int fin = 0;
    if (zl_next_window_ms(now, -1, &fin) < ZL_WIN_TX_GUARD_MS) return;
    if (!s_ul_ack && (int32_t)(now - next_beacon) < 0) return;
    zl_rampa_beacon_t b = {};
    b.state = s_state; b.flags = s_flags; b.t_ms = now;
    b.vbat_mv = s_vbat_mv; b.umb_ma = s_umb_ma;
    b.cont_mv[0] = s_cont_mv[0]; b.cont_mv[1] = s_cont_mv[1]; b.cont_mv[2] = s_cont_mv[2];
    b.next_rx_in_ms = (uint16_t)zl_next_window_ms(millis(), -1, &fin);
    b.faults = (uint8_t)(s_faults | s_latched);
    b.last_ul_seq = s_ul_seq; b.last_ul_rc = s_ul_rc;
    b.fw = (uint8_t)(ZENITH_FW_VERSION & 0xFF);
    zl_lora_seal(&b, sizeof b, ZL_BEACON_MAGIC);
    lora_send(&b, sizeof b);
    s_ul_ack = false;
    next_beacon = now + 1000;
}

// =====================================================================
//  11. ANNUNCIATORS
// =====================================================================
static void outputs_update(uint32_t now)
{
    uint8_t out = (uint8_t)(s_exp_out & (EXP_LORA_EN | EXP_LORA_RST));
    if (rocket_supply_wanted(now)) out |= EXP_ROCKET_EN;
    uint32_t c;
    if (s_stand) {
        const int32_t rem = (int32_t)(s_sl_t0 - now);
        switch (s_sl) {
        case SL_ST_ARMING:    c = (now % 200u) < 100u ? 0xFFC000 : 0; break;
        case SL_ST_ARMED:     c = 0x00FF00; if ((now % 2000u) < 60u) out |= EXP_BUZZER; if ((now % 1000u) < 500u) out |= EXP_FIRE_LED; break;
        case SL_ST_COUNTDOWN: {
            const uint32_t per = rem < 5000 ? 250u : 1000u;
            if ((now % per) < 80u) out |= EXP_BUZZER;
            out |= EXP_FIRE_LED;
            c = (now % 250u) < 125u ? 0xFF6000 : 0; break; }
        case SL_ST_FIRING:    out |= EXP_FIRE_LED | EXP_BUZZER; c = 0xFF0000; break;
        case SL_ST_BURN:      out |= EXP_FIRE_LED; c = 0xFF0000; break;
        case SL_ST_RECORD:    c = (now % 500u) < 250u ? 0x00FFFF : 0x004040; break;
        case SL_ST_ABORT:     if ((now % 500u) < 250u) out |= EXP_FIRE_LED; c = (now % 250u) < 125u ? 0xFF00FF : 0; break;
        default:
            // white = thrust stand, safe; fast white/red blink = hang-fire, do not approach
            c = s_sl_hang ? ((now % 300u) < 150u ? 0xFF0000 : 0xFFFFFF) : 0x606060;
            if (s_sl_hang && (now % 3000u) < 100u) out |= EXP_BUZZER;
            break;
        }
        if (s_sl_buzzer && (now % 2000u) < 500u) out |= EXP_BUZZER;
    } else {
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
    }
    if (s_locate && (now % 2000u) < 500u) out |= EXP_BUZZER;
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
    if (e & EVT_LOCK_Z)   Serial.println("# ROCKET: Zenith (UART 115200)");
    if (e & EVT_LOCK_A)   Serial.println("# ROCKET: Daidalos / Ariadna V2 (I2C slave 0x2B)");
    if (e & EVT_UNLOCK)   Serial.println("# ROCKET: none (scanning I2C / UART)");
    if (e & EVT_ABORT)    Serial.printf("# ABORT faults 0x%02X\n", s_latched | s_faults);
    if (e & EVT_COMMIT)   Serial.printf("# COMMITTED, T-%ld ms\n", (long)(int32_t)(s_t0_ms - millis()));
    if (e & EVT_DETACH)   Serial.println("# Ariadna umbilical detached (GP0/GP1 Hi-Z)");
    if (e & EVT_FIRE)     Serial.printf("# FIRE channel %u for %u ms\n", s_pyro_ch, s_fire_ms);
    if (e & EVT_UMB_TRIP) Serial.printf("# UMB_5V overcurrent %u mA -> rocket supply OFF\n", s_umb_ma);
    if (e & EVT_STAND_IN) Serial.println("# STAND: thrust stand mode (umbilical closed)");
    if (e & EVT_STAND_OUT) Serial.println("# STAND: left thrust stand mode (looking for a rocket)");
    if (e & EVT_SL_ARMED) Serial.printf("# STAND: ARMED igniter %u, %u ms, record slot erased\n", s_sl_ch, s_sl_fire_ms);
    if (e & EVT_SL_CD)    Serial.printf("# STAND: COUNTDOWN T-%ld ms\n", (long)(int32_t)(s_sl_t0 - millis()));
    if (e & EVT_SL_FIRE)  Serial.printf("# STAND: FIRE igniter %u for %u ms\n", s_sl_ch, s_sl_fire_ms);
    if (e & EVT_SL_ABORT) Serial.printf("# STAND: ABORT faults 0x%02X\n", s_sl_latched);
    if (e & EVT_ARM_OK)   Serial.printf("# ARM SERVO: arming switch closed (key sensed after %u ms)\n", s_as_close_ms);
    if (e & EVT_ARM_FAIL) Serial.printf("# ARM SERVO: key not sensed within %u ms -> servo back to SAFE (human switch off? servo position?)\n", ts_cfg.arm_settle_ms);
    if (e & EVT_SL_DONE)  Serial.printf("# STAND: record %u closed, peak %.1f, impulse %.2f, burn %.2f s%s\n",
                                        ts_test_no, (double)ts_res.peak_n, (double)ts_res.impulse_ns, (double)ts_res.burn_s,
                                        s_sl_hang ? " -- NO IGNITION SEEN (hang-fire?)" : "");
}

// =====================================================================
//  12. USB PROTOCOL (diagnostic GUI, docs/USB_PROTOCOL.md)
// =====================================================================
static bool s_usb_live = true;
static struct { bool on; uint16_t test; uint32_t addr, end, off; bool req; } s_dump = {};
static bool s_usb_req_wait = false;     // a USB request (tare/cal/cfg) waits for core 1
static char s_usb_req_name[12];

static bool usb_room(int n) { return Serial && Serial.availableForWrite() >= n; }

static void usb_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void usb_line(const char *fmt, ...)
{
    char b[300];
    va_list ap; va_start(ap, fmt); const int n = vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (!usb_room(n + 2)) return;                    // never block the loop on a slow host
    Serial.print(b); Serial.print("\r\n");
}

static void usb_status(uint32_t now)
{
    static const char *PN[3] = { "NONE", "ZENITH", "ARIADNA" };
    const uint32_t link = s_sl_rx_ms ? now - s_sl_rx_ms : 0xFFFFFFFFu;
    usb_line("$S,%s,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%u,%u,%lu,%lu,%u,%u,%u,%ld,%u,%u,%u,%ld,%d,%06lX,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u",
             PN[s_proto], s_state, s_stand ? (int)s_sl : -1, stand_flags(),
             (unsigned)(s_sl_latched | sl_faults_live()), s_vbat_mv, s_umb_ma, s_v5_mv,
             s_cont_mv[0], s_cont_mv[1], s_cont_mv[2], (unsigned)(s_faults | s_latched),
             s_key, s_btn, s_lora_on, s_stand, ts_ads_ok, ts_flash_ok, ts_sps, ts_test_no,
             (unsigned long)(ts_flash_free() / 1024u), (unsigned long)((TS_FLASH_SIZE - TS_REC_BASE) / 1024u),
             ts_ndir, ts_rec, ts_progress,
             (long)((s_stand && (s_sl == SL_ST_COUNTDOWN || s_sl == SL_ST_FIRING || s_sl == SL_ST_BURN || s_sl == SL_ST_RECORD)) ? (int32_t)(now - s_sl_t0) : 0),
             s_sl_ch, s_sl_fire_ms, ts_cfg.rec_s, (long)(link > 99999999u ? -1 : (int32_t)link), s_c1_ok,
             (unsigned long)ts_jedec, s_lora_rssi, (unsigned)(ts_dropped & 0xFFFF), RAMPA_FW_VERSION,
             s_servo_us[0], s_servo_us[1], s_servo_us[2], s_servo_us[3], ts_cfg.arm_servo, s_as, s_as_close_ms);
}

static void usb_cfg(void)
{
    const ts_cfg_t &c = ts_cfg;
    usb_line("$CFG,lc_offset,%ld", (long)c.lc_offset);
    usb_line("$CFG,lc_scale,%.9g", (double)c.lc_scale);
    usb_line("$CFG,p_zero,%.5f", (double)c.p_zero);
    usb_line("$CFG,p_span,%.5f", (double)c.p_span);
    usb_line("$CFG,p_full_psi,%.1f", (double)c.p_full_psi);
    usb_line("$CFG,adc_vref,%.4f", (double)c.adc_vref);
    usb_line("$CFG,p_div,%.4f", (double)c.p_div);
    usb_line("$CFG,th_r0,%.1f", (double)c.th_r0);
    usb_line("$CFG,th_beta,%.1f", (double)c.th_beta);
    usb_line("$CFG,th_t0_c,%.2f", (double)c.th_t0_c);
    usb_line("$CFG,th_rfix,%.1f", (double)c.th_rfix);
    usb_line("$CFG,ign_thr_n,%.2f", (double)c.ign_thr_n);
    usb_line("$CFG,fire_ms,%u", c.fire_ms);
    usb_line("$CFG,rec_s,%u", c.rec_s);
    usb_line("$CFG,pyro_ch,%u", c.pyro_ch);
    usb_line("$CFG,rate_2k,%u", c.rate_2k);
    usb_line("$CFG,cd_s,%u", c.cd_s);
    usb_line("$CFG,arm_servo,%u", c.arm_servo);
    usb_line("$CFG,arm_us,%u", c.arm_us);
    usb_line("$CFG,safe_us,%u", c.safe_us);
    usb_line("$CFG,arm_settle_ms,%u", c.arm_settle_ms);
    usb_line("#OK CFG");
}

static void usb_result(void)
{
    const sl_res_t &r = ts_res;
    usb_line("$R,%u,%u,%u,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%u,%u,%lu,%u,%d,%d",
             r.test_no, r.flags, r.pyro_ch, (double)r.peak_n, (double)r.impulse_ns, (double)r.burn_s, (double)r.avg_n,
             (double)r.t_peak_s, (double)r.ign_delay_s, (double)r.pmax_psi, r.t1max_dc * 0.1, r.t2max_dc * 0.1,
             r.curve_n, r.curve_hz, (unsigned long)r.n_frames, r.sps, r.t0_idx, s_sl_hang);
}

static bool cfg_set(ts_cfg_t &c, const char *k, const char *v)
{
    const float f = (float)atof(v);
    const long i = atol(v);
    if (!strcmp(k, "lc_offset")) c.lc_offset = (int32_t)i;
    else if (!strcmp(k, "lc_scale")) c.lc_scale = f;
    else if (!strcmp(k, "p_zero") && f > 0.0f && f < 0.5f) c.p_zero = f;
    else if (!strcmp(k, "p_span") && f > 0.05f && f <= 1.0f) c.p_span = f;
    else if (!strcmp(k, "p_full_psi") && f > 0.0f) c.p_full_psi = f;
    else if (!strcmp(k, "adc_vref") && f > 2.5f && f < 3.6f) c.adc_vref = f;
    else if (!strcmp(k, "p_div") && f >= 1.0f && f < 10.0f) c.p_div = f;
    else if (!strcmp(k, "th_r0") && f > 10.0f) c.th_r0 = f;
    else if (!strcmp(k, "th_beta") && f > 100.0f) c.th_beta = f;
    else if (!strcmp(k, "th_t0_c") && f > -50.0f && f < 150.0f) c.th_t0_c = f;
    else if (!strcmp(k, "th_rfix") && f > 10.0f) c.th_rfix = f;
    else if (!strcmp(k, "ign_thr_n") && f >= 0.1f) c.ign_thr_n = f;
    else if (!strcmp(k, "fire_ms") && i >= SL_FIRE_MS_MIN && i <= SL_FIRE_MS_MAX) c.fire_ms = (uint16_t)i;
    else if (!strcmp(k, "rec_s") && i >= SL_REC_MIN_S && i <= SL_REC_MAX_S) c.rec_s = (uint16_t)i;
    else if (!strcmp(k, "pyro_ch") && i >= 1 && i <= 3) c.pyro_ch = (uint8_t)i;
    else if (!strcmp(k, "rate_2k") && (i == 0 || i == 1)) c.rate_2k = (uint8_t)i;
    else if (!strcmp(k, "cd_s") && i >= SL_CD_MIN_S && i <= SL_CD_MAX_S) c.cd_s = (uint16_t)i;
    else if (!strcmp(k, "arm_servo") && i >= 0 && i <= 4) c.arm_servo = (uint8_t)i;
    else if (!strcmp(k, "arm_us") && ts_servo_us_ok((uint16_t)i)) c.arm_us = (uint16_t)i;
    else if (!strcmp(k, "safe_us") && ts_servo_us_ok((uint16_t)i)) c.safe_us = (uint16_t)i;
    else if (!strcmp(k, "arm_settle_ms") && i >= 200 && i <= 10000) c.arm_settle_ms = (uint16_t)i;
    else return false;
    return true;
}

static void usb_request(uint8_t q, int32_t arg, const char *name)
{
    if (!ts_request(q, arg)) { usb_line("#ERR busy (measurement core)"); return; }
    s_usb_req_wait = true;
    strncpy(s_usb_req_name, name, sizeof s_usb_req_name - 1);
    s_usb_req_name[sizeof s_usb_req_name - 1] = 0;
}

static bool stand_safe_for_usb(void) { return !s_stand || s_sl == SL_ST_SAFE || s_sl == SL_ST_ABORT; }

static void usb_stand(uint8_t cmd, int32_t arg, const char *what, uint32_t now)
{
    if (s_proto != P_NONE || s_detached) { usb_line("#ERR a rocket is on the pogo"); return; }
    stand_enter(now);
    s_sl_usb_ms = now;
    const uint8_t rc = stand_cmd(cmd, arg, 0, sl_token_for(cmd), false, now);
    static const char *RC[7] = { "-", "OK", "wrong state", "interlock", "bad argument", "aborted", "busy" };
    if (rc == SL_RC_OK || rc == SL_RC_ABORTED) usb_line("#OK %s", what);
    else usb_line("#ERR %s: %s (faults 0x%02X)", what, RC[rc < 7 ? rc : 0], s_sl_latched);
}

static void usb_command(char *l, uint32_t now)
{
    char *a0 = strtok(l, " "), *a1 = strtok(nullptr, " "), *a2 = strtok(nullptr, " ");
    if (!a0) return;
    for (char *p = a0; *p; p++) if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 32);
    if (!strcmp(a0, "HELLO")) {
        usb_line("$ID,Rampa,%u.%02u,PCB1,%s", RAMPA_FW_VERSION / 100, RAMPA_FW_VERSION % 100, ts_flash_ok ? "W25Q128" : "noflash");
        usb_cfg();
        usb_status(now);
        if (ts_res_ready) usb_result();
        return;
    }
    if (!strcmp(a0, "STATUS")) { usb_status(now); return; }
    if (!strcmp(a0, "LIVE")) { s_usb_live = !a1 || atoi(a1); usb_line("#OK LIVE %d", s_usb_live); return; }
    if (!strcmp(a0, "CFG")) { usb_cfg(); return; }
    if (!strcmp(a0, "RESULT")) { if (ts_res_ready) usb_result(); else usb_line("#ERR no result yet"); return; }
    if (!strcmp(a0, "ABORT")) { if (s_stand) stand_abort(0, now); usb_line("#OK ABORT"); return; }
    if (!strcmp(a0, "FIRE") || !strcmp(a0, "ARM")) { usb_line("#ERR %s only from the ground station (LoRa, safe distance)", a0); return; }
    if (!strcmp(a0, "DISARM") || !strcmp(a0, "SAFE")) { usb_stand(SL_C_DISARM, 0, "DISARM", now); return; }
    if (!strcmp(a0, "BUZZER")) { s_sl_buzzer = a1 && atoi(a1); usb_line("#OK BUZZER %d", s_sl_buzzer); return; }
    if (!strcmp(a0, "STAND")) {
        if (a1 && atoi(a1)) usb_stand(SL_C_PING, 0, "STAND 1", now);
        else if (stand_safe_for_usb() && s_sl != SL_ST_ABORT) { stand_leave(now); usb_line("#OK STAND 0"); }
        else usb_line("#ERR STAND 0: not safe");
        return;
    }
    if (!strcmp(a0, "SERVO")) {
        // SERVO <1-4> <500-2500 us> | SERVO <1-4> OFF
        const long n = a1 ? atol(a1) : 0;
        if (n < 1 || n > 4 || !a2) { usb_line("#ERR SERVO <1-4> <500-2500 us | OFF>"); return; }
        const bool is_arm = arm_servo_on() && n == ts_cfg.arm_servo;
        if (is_arm && !arm_usb_allowed()) { usb_line("#ERR SERVO %ld is the arming switch and the rampa is armed / counting", n); return; }
        for (char *p = a2; *p; p++) if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 32);
        if (!strcmp(a2, "OFF")) {
            if (is_arm) { usb_line("#ERR SERVO %ld: the arming servo stays driven (SAFE position)", n); return; }
            servo_off((uint8_t)(n - 1)); usb_line("#OK SERVO %ld OFF", n); return;
        }
        const long us = atol(a2);
        if (!ts_servo_us_ok((uint16_t)us)) { usb_line("#ERR SERVO pulse 500-2500 us"); return; }
        servo_write((uint8_t)(n - 1), (uint16_t)us);
        if (is_arm && us != ts_cfg.safe_us) { s_as = AS_TEST; s_as_test_end = now + 10000; }   // back to SAFE in 10 s
        usb_line("#OK SERVO %ld %ld%s", n, us, is_arm && us != ts_cfg.safe_us ? " (arming servo: back to SAFE in 10 s)" : "");
        return;
    }
    if (!strcmp(a0, "ARMTEST")) {
        if (!arm_servo_on()) { usb_line("#ERR ARMTEST: no arming servo configured (SET arm_servo 1-4)"); return; }
        if (!arm_usb_allowed() || s_proto != P_NONE) { usb_line("#ERR ARMTEST: not now (rocket on the pogo / armed / counting)"); return; }
        s_as_fail_latch = false;
        if (s_as == AS_TEST) arm_safe(now);
        s_armtest = 1;
        arm_go(now);
        usb_line("#OK ARMTEST started (servo to arm_us, waiting for the key sense)");
        return;
    }
    if (!strcmp(a0, "REC")) { usb_stand(SL_C_RECORD, a1 ? atol(a1) : 10, "REC", now); return; }
    if (!strcmp(a0, "STOP")) { usb_stand(SL_C_STOP, 0, "STOP", now); return; }
    // ---- calibration (only when nothing is armed)
    const bool cal_ok = stand_safe_for_usb() && s_state != ZR_ST_COUNTDOWN && s_state != ZR_ST_COMMITTED && ts_rec == TR_IDLE;
    if (!strcmp(a0, "TARE") || !strcmp(a0, "CAL") || !strcmp(a0, "PZERO") || !strcmp(a0, "SET") ||
        !strcmp(a0, "LCRATED") || !strcmp(a0, "ERASEALL") || !strcmp(a0, "DUMP") || !strcmp(a0, "DEFAULTS")) {
        if (!cal_ok) { usb_line("#ERR %s: not now (armed / counting / recording)", a0); return; }
        if (s_dump.on) { usb_line("#ERR dump in progress"); return; }
    }
    if (!strcmp(a0, "TARE")) { usb_request(TQ_TARE, 0, "TARE"); return; }
    if (!strcmp(a0, "CAL")) {
        const double n = a1 ? atof(a1) : 0;
        if (n <= 0) { usb_line("#ERR CAL <known force N>"); return; }
        usb_request(TQ_CAL, (int32_t)lround(n * 1000.0), "CAL"); return;
    }
    if (!strcmp(a0, "PZERO")) { usb_request(TQ_PZERO, 0, "PZERO"); return; }
    if (!strcmp(a0, "SET") || !strcmp(a0, "LCRATED") || !strcmp(a0, "DEFAULTS")) {
        if (!ts_request_done()) { usb_line("#ERR busy"); return; }
        ts_cfg_new = ts_cfg;
        if (!strcmp(a0, "DEFAULTS")) {
            ts_cfg_defaults(ts_cfg_new);
            ts_cfg_new.lc_offset = ts_cfg.lc_offset; ts_cfg_new.lc_scale = ts_cfg.lc_scale;   // keep the calibration
            ts_cfg_new.arm_servo = ts_cfg.arm_servo; ts_cfg_new.arm_us = ts_cfg.arm_us;       // and the arming servo
            ts_cfg_new.safe_us = ts_cfg.safe_us; ts_cfg_new.arm_settle_ms = ts_cfg.arm_settle_ms;
        } else if (!strcmp(a0, "LCRATED")) {
            // LCRATED <capacity kg> <sensitivity mV/V>: theoretical scale without a weight
            const double kg = a1 ? atof(a1) : 0, mvv = a2 ? atof(a2) : 0;
            if (kg <= 0 || mvv <= 0) { usb_line("#ERR LCRATED <capacity kg> <mV/V>"); return; }
            ts_cfg_new.lc_scale = (float)(kg * 9.80665 / (mvv * 1e-3 * 128.0 * 8388608.0));
        } else if (!a1 || !a2 || !cfg_set(ts_cfg_new, a1, a2)) { usb_line("#ERR SET <key> <value> (see CFG; value out of range?)"); return; }
        usb_request(TQ_SET_CFG, 0, a0);
        return;
    }
    if (!strcmp(a0, "LIST")) {
        for (uint8_t i = 0; i < ts_ndir; i++) {
            const ts_dir_t &d = ts_dir[i];
            usb_line("$REC,%u,%lu,%lu,%u,%u,%u,%lu,%.3f,%.4f,%.3f", d.test_no, (unsigned long)d.addr, (unsigned long)d.blocks,
                     d.sps, d.closed, d.flags, (unsigned long)d.n_frames, (double)d.peak_n, (double)d.impulse_ns, (double)d.burn_s);
        }
        usb_line("#OK LIST %u", ts_ndir);
        return;
    }
    if (!strcmp(a0, "DUMP")) {
        const long t = a1 ? atol(a1) : -1;
        for (uint8_t i = 0; i < ts_ndir; i++) if (ts_dir[i].test_no == t) {
            const ts_dir_t &d = ts_dir[i];
            uint32_t len = d.blocks * TS_BLOCK;
            if (d.closed) len = TS_FRAMES_OFF + d.n_frames * TS_FRAME_SIZE;
            len = (len + TS_PAGE - 1) / TS_PAGE * TS_PAGE;
            s_dump.on = true; s_dump.test = d.test_no; s_dump.addr = d.addr; s_dump.end = d.addr + len; s_dump.off = 0; s_dump.req = false;
            usb_line("$DBEGIN,%u,%lu,%lu", d.test_no, (unsigned long)d.addr, (unsigned long)len);
            return;
        }
        usb_line("#ERR DUMP <test number> (see LIST)");
        return;
    }
    if (!strcmp(a0, "ERASEALL")) {
        if (!a1 || strcmp(a1, "YES")) { usb_line("#ERR ERASEALL YES  (deletes every record)"); return; }
        usb_request(TQ_ERASE_ALL, 0, "ERASEALL");
        return;
    }
    usb_line("#ERR HELLO STATUS LIVE CFG SET LCRATED DEFAULTS TARE CAL PZERO REC STOP LIST DUMP RESULT ERASEALL BUZZER STAND DISARM ABORT SERVO ARMTEST");
}

static void usb_service(uint32_t now)
{
    static char line[96]; static uint8_t n = 0;
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (n) { line[n] = 0; usb_command(line, now); n = 0; } }
        else if (n < sizeof(line) - 1) line[n++] = c;
    }
    if (s_armtest == 2) { s_armtest = 0; usb_line("#OK ARMTEST key closed after %u ms, servo back to SAFE in 3 s", s_as_close_ms); }
    if (s_armtest == 3) { s_armtest = 0; usb_line("#ERR ARMTEST key not sensed within %u ms (human switch off? arm_us wrong?) - servo back to SAFE", ts_cfg.arm_settle_ms); }
    // finished USB request
    if (s_usb_req_wait && ts_request_done()) {
        s_usb_req_wait = false;
        if (ts_ack_res == TQR_OK) { usb_line("#OK %s", s_usb_req_name); usb_cfg(); }
        else usb_line("#ERR %s failed (load cell / weight / transducer / flash?)", s_usb_req_name);
    }
    // record download: one page per loop
    if (s_dump.on) {
        if (s_dump.req) {
            if (ts_request_done() && usb_room(2 * TS_PAGE + 24)) {
                s_dump.req = false;
                if (ts_ack_res != TQR_OK) { usb_line("#ERR DUMP read failed"); s_dump.on = false; }
                else {
                    static char hex[2 * TS_PAGE + 1];
                    static const char *H = "0123456789ABCDEF";
                    for (unsigned i = 0; i < TS_PAGE; i++) { hex[2 * i] = H[ts_rd_buf[i] >> 4]; hex[2 * i + 1] = H[ts_rd_buf[i] & 15]; }
                    hex[2 * TS_PAGE] = 0;
                    Serial.printf("$D,%lu,%s\r\n", (unsigned long)s_dump.off, hex);
                    s_dump.off += TS_PAGE;
                    if (s_dump.addr + s_dump.off >= s_dump.end) { usb_line("$DEND,%u,%lu", s_dump.test, (unsigned long)s_dump.off); s_dump.on = false; }
                }
            }
        } else if (ts_request(TQ_READ, (int32_t)(s_dump.addr + s_dump.off), TS_PAGE)) {
            s_dump.req = true;
        }
    }
    // live + status
    live_pull();
    static uint32_t live_sent = 0;
    if (s_lv_n - live_sent > LV_HIST) live_sent = s_lv_n - LV_HIST;
    while (s_lv_n - live_sent >= 2) {                  // 50 Hz: mean of two 100 Hz points
        const ts_live_t &a = s_lv[live_sent % LV_HIST], &b = s_lv[(live_sent + 1) % LV_HIST];
        live_sent += 2;
        if (s_usb_live && !s_dump.on)
            usb_line("$M,%lu,%.3f,%.3f,%.2f,%.4f,%.2f,%.2f,%.3f", (unsigned long)b.t_ms, (double)((a.f + b.f) * 0.5f),
                     (double)(a.fmax > b.fmax ? a.fmax : b.fmax), (double)((a.p + b.p) * 0.5f), (double)((a.p_v + b.p_v) * 0.5f),
                     (double)b.t1, (double)b.t2, (double)((a.uvv + b.uvv) * 0.5f));
    }
    static uint32_t t_s = 0;
    if ((int32_t)(now - t_s) >= 0 && !s_dump.on) { t_s = now + 200; usb_status(now); }
    static bool res_was = false;
    if (ts_res_ready && !res_was) usb_result();
    res_was = ts_res_ready;
}

// =====================================================================
//  13. SETUP / LOOP (core 0)  ·  SETUP1 / LOOP1 (core 1, thrust_stand.h)
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
    s_c1_alive_ms = s_boot_ms;
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

    if (!s_stand) {
        if (s_port == PORT_UART) zenith_poll();
        umbilical_service(now);
    }

    // ---- supervision (masked against the I2C slave interrupt) -------------------
    noInterrupts();
    if (s_btn && (s_state == ZR_ST_READY || s_state == ZR_ST_COUNTDOWN || s_state == ZR_ST_COMMITTED))
        enter_abort(ZL_RMPF_BTN_ABORT);
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
        if (s_fire_end_ms && (int32_t)(now - s_fire_end_ms) >= 0) { pyro_off(); s_fire_end_ms = 0; s_as_req = 2; }
        if (!s_fire_end_ms && (uint32_t)(now - s_fired_ms) > POST_FIRE_MS) s_state = ZR_ST_IDLE;
    }
    // the pyro outputs belong to exactly one owner: a rocket launch or the thrust stand
    const bool stand_pyro = s_stand && (s_sl == SL_ST_FIRING);
    if (!stand_pyro && s_state != ZR_ST_FIRED && s_state != ZR_ST_COMMITTED && digitalRead(PIN_PYRO_ARM)) pyro_off();
    if (s_port == PORT_I2C) ariadna_refresh_tx(now);
    interrupts();

    stand_service(now);
    arm_service(now);

    // ---- rocket supply protection ------------------------------------------------
    if ((s_exp_out & EXP_ROCKET_EN) && s_umb_ma > UMB_OVERCURRENT) { s_umb_trip = true; s_evt |= EVT_UMB_TRIP; }

    // ---- radio ownership ----------------------------------------------------------
    static uint32_t no_rocket_since = 0;
    const bool rocket = s_proto != P_NONE || s_detached;
    if (rocket) { no_rocket_since = 0; if (s_lora_on) lora_power(false); }
    else {
        if (!no_rocket_since) no_rocket_since = now ? now : 1;
        // after a launch the rocket owns the 433 MHz channel: stay silent
        const bool may_talk = s_state != ZR_ST_COMMITTED && s_state != ZR_ST_COUNTDOWN &&
                              !(s_fired_ms && (uint32_t)(now - s_fired_ms) < RADIO_QUIET_AFTER_FIRE_MS);
        if (!s_lora_on && may_talk && (uint32_t)(now - no_rocket_since) > 2000u) lora_power(true);
    }
    lora_service(now);

    outputs_update(now);
    print_events();
    usb_service(now);
}

// ---- core 1: measurement and recording only (never the pyro pins)
void setup1() { ts_core1_setup(); }
void loop1()  { ts_core1_loop(); }
