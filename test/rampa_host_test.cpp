// Host test: the REAL Rampa_MCU.ino against a simulated Daidalos (Ariadna V2,
// I2C master @ 0x2B) and a simulated Zenith (UART), with simulated TCA9534 /
// INA226 / ADS1115 on the internal bus. Arduino API comes from test/stubs.
//
//   make -C test
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "LoRa.h"

uint32_t g_ms = 0;
int g_pin[32];
StubPrint Serial;
StubUart Serial1;
StubUart Serial2;
TwoWire Wire(false), Wire1(true);
StubSPI SPI1;
StubLoRa LoRa;

// ---------------------------------------------------------------- simulated pad hardware
static uint8_t g_tca_out = 0;
static bool g_key = true, g_btn = false;
static uint16_t g_vbat_mv = 8000, g_cont_mv = 1200;

uint8_t TwoWire::endTransmission(bool)
{
    if (!internal) return 0;
    const bool present = (cur == 0x20 || cur == 0x40 || cur == 0x41 || cur == 0x48);
    if (wbuf.empty()) return present ? 0 : 2;
    reg = wbuf[0];
    if (cur == 0x20 && reg == 0x01 && wbuf.size() >= 2) g_tca_out = wbuf[1];
    return present ? 0 : 2;
}
int TwoWire::requestFrom(int a, int n)
{
    rbuf.clear();
    uint16_t v = 0;
    if (a == 0x20 && reg == 0x00) { uint8_t in = (uint8_t)((g_key ? 0x04 : 0) | (g_btn ? 0 : 0x80)); rbuf.push_back(in); return 1; }
    if (a == 0x40 && reg == 0x02) v = (uint16_t)(g_vbat_mv * 100u / 125u);
    if (a == 0x40 && reg == 0x04) v = 240;
    if (a == 0x41 && reg == 0x02) v = (g_tca_out & 0x02) ? (uint16_t)(5000u * 100u / 125u) : 0;
    if (a == 0x41 && reg == 0x04) v = (g_tca_out & 0x02) ? 180 : 0;
    if (a == 0x48 && reg == 0x01) v = 0x8000;
    if (a == 0x48 && reg == 0x00) v = (uint16_t)(g_cont_mv * 8u);
    rbuf.push_back((uint8_t)(v >> 8)); rbuf.push_back((uint8_t)v);
    (void)n;
    return 2;
}

#include "../Rampa_MCU/Rampa_MCU.ino"

static int g_fail = 0;
#define CHECK(c, ...) do { printf((c) ? "   ok:   " : "   FAIL: "); printf(__VA_ARGS__); printf("\n"); if (!(c)) g_fail++; } while (0)

static void run_ms(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { g_ms++; loop(); } }

static void reset_rampa()
{
    memset(g_pin, 0, sizeof g_pin);
    g_ms += 100000; g_tca_out = 0; g_key = true; g_btn = false; g_vbat_mv = 8000; g_cont_mv = 1200;
    s_proto = P_NONE; s_conn = CONN_NONE; s_port = PORT_OFF; s_i2c_conn = CONN_NONE;
    s_uart1_on = false; s_uart2_on = false;
    s_state = ZR_ST_IDLE; s_latched = 0; s_cd = false; s_detached = false;
    s_fired_ms = 0; s_fire_end_ms = 0; s_detach_req = false; s_last_cmd_ms = 0; s_lora_on = false; s_lora_ok = false;
    s_lora_tx_cnt = 0; s_lora_rx_cnt = 0; s_lora_beacon_now = false; s_usb_stream = false; s_locate = false;
    s_rocket_pwr = true; s_umb_trip = false; s_diag_buzz_end = 0; diag_exit(true);
    Serial.rx.clear(); Serial.tx.clear();
    Wire.slave_on = false; Wire.sda = -1; Wire.scl = -1;
    Serial1.on = false; Serial1.rx.clear(); Serial1.tx.clear();
    Serial2.on = false; Serial2.rx.clear(); Serial2.tx.clear();
    setup();
}

// ---------------------------------------------------------------- Ariadna (Daidalos)
static uint8_t a_seq = 0;
/** One Ariadna Navigation-MCU transaction on a specific SDA pin (0 = J3, 8 = J14). Returns 0 = NACK. */
static int ariadna_xfer_on(int want_sda, uint8_t cmd, int32_t t_rem, ariadna_rampa_status_t *out)
{
    if (!Wire.slave_on || Wire.sda != want_sda) return 0; // nobody ACKs 0x2B on this connector
    ariadna_rampa_cmd_t c; memset(&c, 0, sizeof c);
    c.cmd = cmd; c.seq = ++a_seq; c.t_rem_ms = t_rem; c.utc_ms = 0xFFFFFFFFu; c.pyro_ch = 2;
    c.rocket_flags = ARIADNA_RK_POST_OK | ARIADNA_RK_NAV_VALID | ARIADNA_RK_POS_OK | ARIADNA_RK_NAV_OK |
                     ARIADNA_RK_NO_ABORT | ARIADNA_RK_ARMED | ARIADNA_RK_RBF_OUT;
    ariadna_rampa_cmd_seal(&c);
    Wire.srx.assign((uint8_t *)&c, (uint8_t *)&c + sizeof c);
    Wire.on_rx((int)sizeof c);
    Wire.stx.clear();
    Wire.on_req();
    if (Wire.stx.size() != ARIADNA_RAMPA_STAT_SIZE) return 0;
    return ariadna_rampa_status_decode(Wire.stx.data(), Wire.stx.size(), out) == ARIADNA_RAMPA_OK ? 1 : 0;
}

static int ariadna_xfer(uint8_t cmd, int32_t t_rem, ariadna_rampa_status_t *out)
{
    return ariadna_xfer_on(PIN_UMB2_0, cmd, t_rem, out);
}

static void test_ariadna_on(const char *label, int sda_pin, uint8_t expect_conn)
{
    printf("== Daidalos / Ariadna V2 on %s (I2C slave 0x2B, GP%d/GP%d)\n", label, sda_pin, sda_pin + 1);
    reset_rampa();
    ariadna_rampa_status_t st{};
    int ok = 0;
    for (int i = 0; i < 40; i++) { ok = ariadna_xfer_on(sda_pin, RAMPA_CMD_PING, -1, &st); run_ms(50); }   // 2 s of 20 Hz pings
    CHECK(ok && s_proto == P_ARIADNA && s_conn == expect_conn, "recognised as Ariadna on %s", label);
    CHECK(!(g_tca_out & 0x02), "UMB_5V stays OFF for an Ariadna on %s", label);
    CHECK(st.state == RAMPA_STATE_IDLE && (st.flags & ARIADNA_RMP_LORA_OFF), "IDLE, LoRa off (state %u flags %02X)", st.state, st.flags);
    for (int i = 0; i < 10; i++) { ariadna_xfer_on(sda_pin, RAMPA_CMD_PREPARE, -1, &st); run_ms(50); }
    CHECK(st.state == RAMPA_STATE_READY && (st.flags & ARIADNA_RMP_READY), "PREPARE -> READY");
    const uint32_t t0 = g_ms + 10000;
    bool committed_same_xfer = false;
    while ((int32_t)(t0 - g_ms) > 100) { ariadna_xfer_on(sda_pin, RAMPA_CMD_COUNTDOWN, (int32_t)(t0 - g_ms), &st); run_ms(50); }
    CHECK(st.state == RAMPA_STATE_COUNTDOWN, "COUNTDOWN running (T-%ld)", (long)st.t_rem_ms);
    // Ariadna's T-100 ms commit: the answer must be COMMITTED in the same transaction
    ariadna_xfer_on(sda_pin, RAMPA_CMD_COMMIT, (int32_t)(t0 - g_ms), &st);
    committed_same_xfer = st.state == RAMPA_STATE_COMMITTED && (st.flags & ARIADNA_RMP_DETACHING);
    CHECK(committed_same_xfer, "COMMITTED returned in the same I2C transaction (state %u)", st.state);
    run_ms(5);
    CHECK(!Wire.slave_on && s_detached, "umbilical %s detached (Hi-Z) before T-0", label);
    bool fired = false;
    while ((int32_t)(g_ms - t0) < 50) { run_ms(1); if (g_pin[17] && g_pin[19]) fired = true; }
    CHECK(fired, "motor igniter (channel 2 from the command) fired at T-0");
    run_ms(1200);
    CHECK(!g_pin[17] && !g_pin[19], "firing pulse ended (1 s)");
    run_ms(4500);
    CHECK(Wire.slave_on && Wire.sda == sda_pin, "umbilical %s re-attached after the launch window", label);
    // abort path: button
    for (int i = 0; i < 10; i++) { ariadna_xfer_on(sda_pin, RAMPA_CMD_PREPARE, -1, &st); run_ms(50); }
    const uint32_t t1 = g_ms + 15000;
    for (int i = 0; i < 20; i++) { ariadna_xfer_on(sda_pin, RAMPA_CMD_COUNTDOWN, (int32_t)(t1 - g_ms), &st); run_ms(50); }
    g_btn = true; run_ms(60); g_btn = false;
    ariadna_xfer_on(sda_pin, RAMPA_CMD_COUNTDOWN, (int32_t)(t1 - g_ms), &st);
    CHECK(st.state == RAMPA_STATE_ABORT && (st.fault_mask & ARIADNA_RMPF_BTN_ABORT), "local abort button -> ABORT");
    run_ms(20000);
    CHECK(!g_pin[16] && !g_pin[17] && !g_pin[18] && !g_pin[19], "nothing fired after the abort");
}

static void test_ariadna()
{
    test_ariadna_on("J14", PIN_UMB2_0, CONN_J14);
    test_ariadna_on("J3",  PIN_UMB1_0, CONN_J3);
}

// ---------------------------------------------------------------- Zenith
static uint8_t z_seq = 0;
static zl_rx_t z_rx;
static zl_rmp_status_t z_st;
static int z_got = 0;

static void zenith_send_on(StubUart &ser, uint8_t cmd, int32_t t_rem, uint8_t flags = ZL_RK_GO | ZL_RK_NAV_OK)
{
    zl_rmp_cmd_t c{}; c.cmd = cmd; c.seq = ++z_seq; c.t_rem_ms = t_rem; c.rocket_flags = flags; c.pyro_ch = 1; c.fire_ms = 800;
    uint8_t f[ZL_FRAME_MAX]; uint16_t n = zl_encode(f, ZL_PKT_RMP_CMD, &c, sizeof c);
    if (ser.on) ser.rx.insert(ser.rx.end(), f, f + n);   // bytes on a closed UART are lost
}
static void zenith_rx_on(StubUart &ser)
{
    for (uint8_t b : ser.tx) {
        const uint8_t *pl; uint8_t n;
        if (zl_rx_byte(&z_rx, b, &pl, &n) == ZL_PKT_RMP_STATUS && zl_take(&z_st, sizeof z_st, pl, n)) z_got++;
    }
    ser.tx.clear();
}
static void zenith_tick_on(StubUart &ser, uint8_t cmd, int32_t t_rem) { zenith_send_on(ser, cmd, t_rem); run_ms(50); zenith_rx_on(ser); }
static void zenith_send(uint8_t cmd, int32_t t_rem, uint8_t flags = ZL_RK_GO | ZL_RK_NAV_OK) { zenith_send_on(Serial1, cmd, t_rem, flags); }
static void zenith_rx() { zenith_rx_on(Serial1); }
static void zenith_tick(uint8_t cmd, int32_t t_rem) { zenith_tick_on(Serial1, cmd, t_rem); }

static void test_zenith_on_j14()
{
    printf("== Zenith on J14 (UART1 Serial2, GP8/GP9)\n");
    reset_rampa(); zl_rx_init(&z_rx); z_got = 0;
    run_ms(1700);
    CHECK(g_tca_out & 0x02, "UMB_5V on after 1.5 s boot listen");
    for (int i = 0; i < 60; i++) zenith_tick_on(Serial2, ZR_CMD_PING, -1);
    CHECK(s_proto == P_ZENITH && s_conn == CONN_J14 && z_got > 10, "recognised as Zenith on J14 (%d answers)", z_got);
    for (int i = 0; i < 10; i++) zenith_tick_on(Serial2, ZR_CMD_PREPARE, -1);
    CHECK(z_st.state == ZR_ST_READY, "J14 PREPARE -> READY");
    const uint32_t t0 = g_ms + 12000;
    while ((int32_t)(t0 - g_ms) > 300) zenith_tick_on(Serial2, ZR_CMD_COUNTDOWN, (int32_t)(t0 - g_ms));
    zenith_tick_on(Serial2, ZR_CMD_COMMIT, (int32_t)(t0 - g_ms));
    CHECK(z_st.state == ZR_ST_COMMITTED, "J14 COMMITTED");
    bool fired = false;
    while ((int32_t)(g_ms - t0) < 100) { run_ms(1); if (g_pin[16] && g_pin[19]) fired = true; }
    CHECK(fired, "J14 igniter fired at T-0");
    run_ms(900);
    CHECK(!g_pin[16] && !g_pin[19], "J14 firing pulse ended");
}

static void test_zenith()
{
    printf("== Zenith on J3 (UART0 Serial1, GP0/GP1)\n");
    reset_rampa(); zl_rx_init(&z_rx); z_got = 0;
    run_ms(1400);
    CHECK(!(g_tca_out & 0x02), "UMB_5V off during the first 1.5 s (looking for an Ariadna)");
    run_ms(300);
    CHECK(g_tca_out & 0x02, "then UMB_5V on (powers a Zenith with its battery off)");
    for (int i = 0; i < 60; i++) zenith_tick(ZR_CMD_PING, -1);   // Zenith boots and pings at 20 Hz
    CHECK(s_proto == P_ZENITH && s_conn == CONN_J3 && z_got > 10, "recognised as Zenith on J3 (%d answers)", z_got);
    CHECK(z_st.state == ZR_ST_IDLE && (z_st.flags & ZL_RMP_ROCKET_PWR), "IDLE, rocket powered");
    for (int i = 0; i < 10; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    CHECK(z_st.state == ZR_ST_READY, "PREPARE -> READY");
    const uint32_t t0 = g_ms + 12000;
    while ((int32_t)(t0 - g_ms) > 300) zenith_tick(ZR_CMD_COUNTDOWN, (int32_t)(t0 - g_ms));
    CHECK(z_st.state == ZR_ST_COUNTDOWN && labs(z_st.t_rem_ms - (int32_t)(t0 - g_ms)) < 60, "countdown locked to the rocket (T-%ld)", (long)z_st.t_rem_ms);
    zenith_tick(ZR_CMD_COMMIT, (int32_t)(t0 - g_ms));
    CHECK(z_st.state == ZR_ST_COMMITTED, "COMMITTED");
    bool fired = false; uint32_t t_fire = 0;
    while ((int32_t)(g_ms - t0) < 100) { run_ms(1); if (g_pin[16] && g_pin[19] && !fired) { fired = true; t_fire = g_ms; } }
    CHECK(fired && labs((long)(int32_t)(t_fire - t0)) <= 5, "igniter fired at T-0 (%+ld ms)", (long)(int32_t)(t_fire - t0));
    run_ms(900);
    CHECK(!g_pin[16] && !g_pin[19], "pulse 800 ms from the rocket ended");
    // the rocket left: no frames -> unlock, and the radio stays silent after a launch
    run_ms(10000);
    CHECK(s_proto == P_NONE && !s_lora_on, "rocket gone: scanning again, LoRa silent after a launch");

    printf("== Zenith: rocket drops GO in the countdown\n");
    reset_rampa(); zl_rx_init(&z_rx);
    run_ms(1700);
    for (int i = 0; i < 40; i++) zenith_tick(ZR_CMD_PING, -1);
    for (int i = 0; i < 5; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    const uint32_t t1 = g_ms + 20000;
    for (int i = 0; i < 20; i++) zenith_tick(ZR_CMD_COUNTDOWN, (int32_t)(t1 - g_ms));
    zenith_send(ZR_CMD_COUNTDOWN, (int32_t)(t1 - g_ms), 0); run_ms(50); zenith_rx();
    CHECK(z_st.state == ZR_ST_ABORT && (z_st.faults & ZL_RMPF_ROCKET), "ABORT (rocket NO-GO)");
    printf("== Zenith: umbilical pulled in the countdown\n");
    zenith_tick(ZR_CMD_SAFE, -1);
    for (int i = 0; i < 5; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    const uint32_t t2 = g_ms + 20000;
    for (int i = 0; i < 20; i++) zenith_tick(ZR_CMD_COUNTDOWN, (int32_t)(t2 - g_ms));
    run_ms(1600);
    CHECK(s_state == ZR_ST_ABORT && (s_latched & ZL_RMPF_TIMEOUT), "ABORT on link timeout");
    run_ms(20000);
    CHECK(!g_pin[16] && !g_pin[19], "nothing fired");

    printf("== no rocket at all\n");
    reset_rampa();
    run_ms(5000);
    CHECK(s_proto == P_NONE && s_lora_on, "standalone: LoRa beacon on");
}

static void usb_cmd(const char *c)
{
    Serial.tx.clear();
    Serial.feed(c);
    Serial.feed("\n");
    run_ms(5);
}

static void test_usb_diag()
{
    printf("== USB standalone diagnostics (no rocket / no ground station)\n");
    reset_rampa();
    run_ms(50);

    usb_cmd("HELLO");
    CHECK(Serial.tx.find("$ID,Rampa PCB1,") != std::string::npos, "HELLO returns $ID");
    CHECK(Serial.tx.find("$S,") != std::string::npos, "HELLO returns immediate $S status");

    usb_cmd("I2C SCAN");
    CHECK(Serial.tx.find("$I2C,0x20,0x40,0x41,0x48") != std::string::npos, "I2C SCAN finds TCA/INA/ADS");

    // Pyro ARM must be refused outside DIAG
    usb_cmd("ARM 1");
    CHECK(Serial.tx.find("#ERR") != std::string::npos && !g_pin[19], "ARM 1 rejected outside DIAG");

    // Enter DIAG and test buzzer, LED, LoRa beacon, ARM and FIRE
    usb_cmd("DIAG 1");
    CHECK(Serial.tx.find("#OK DIAG 1") != std::string::npos && s_diag, "DIAG 1 enabled");

    usb_cmd("BUZZ 100");
    CHECK(Serial.tx.find("#OK BUZZ 100") != std::string::npos && (g_tca_out & EXP_BUZZER), "BUZZ 100 activates buzzer");
    run_ms(120);
    CHECK(!(g_tca_out & EXP_BUZZER), "buzzer turns off after 100 ms");

    usb_cmd("LED 1");
    CHECK((g_tca_out & EXP_FIRE_LED) != 0, "LED 1 activates fire warning light");
    usb_cmd("LED AUTO");

    const uint32_t tx_before = s_lora_tx_cnt;
    usb_cmd("LORA BEACON");
    run_ms(20);
    CHECK(s_lora_on && s_lora_tx_cnt > tx_before, "LORA BEACON sends immediate test beacon");

    // Key off blocks ARM
    g_key = false; run_ms(30);
    usb_cmd("ARM 1");
    CHECK(Serial.tx.find("#ERR") != std::string::npos && !g_pin[19], "ARM 1 rejected when key J4 is OFF");

    // Key on allows ARM and FIRE CONFIRM
    g_key = true; run_ms(30);
    usb_cmd("ARM 1");
    CHECK(Serial.tx.find("#OK ARM 1") != std::string::npos && g_pin[19], "ARM 1 energises pyro rail in DIAG");
    usb_cmd("FIRE 2 150 CONFIRM");
    CHECK(Serial.tx.find("#OK FIRE 2 150") != std::string::npos && g_pin[19] && g_pin[17], "FIRE 2 150 CONFIRM fires CH2");
    run_ms(170);
    CHECK(!g_pin[17] && g_pin[19], "CH2 pulse ends after 150 ms while rail stays armed");

    // Turning key OFF immediately disarms pyro rail
    g_key = false; run_ms(30);
    CHECK(!g_pin[19] && !s_diag_arm, "turning key OFF immediately disarms DIAG rail");

    // Local ABORT button exits DIAG and enters ABORT
    g_key = true; run_ms(30);
    usb_cmd("ARM 1");
    CHECK(g_pin[19], "re-armed in DIAG");
    g_btn = true; run_ms(30); g_btn = false; run_ms(30);
    CHECK(!s_diag && !g_pin[19] && s_state == ZR_ST_ABORT, "local ABORT button exits DIAG and disarms");

    usb_cmd("SAFE");
    CHECK(s_state == ZR_ST_IDLE, "USB SAFE returns to IDLE");

    // Rocket connecting automatically exits DIAG
    usb_cmd("DIAG 1");
    CHECK(s_diag, "DIAG re-entered");
    ariadna_rampa_status_t st{};
    ariadna_xfer(RAMPA_CMD_PING, -1, &st);
    run_ms(5);
    CHECK(!s_diag && s_proto == P_ARIADNA, "rocket connecting automatically exits DIAG");
}

int main()
{
    test_ariadna();
    test_zenith();
    test_zenith_on_j14();
    test_usb_diag();
    printf("\n%s (%d failures)\n", g_fail ? "RAMPA TEST FAILED" : "RAMPA TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}
