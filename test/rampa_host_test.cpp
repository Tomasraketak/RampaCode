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
    if (wbuf.empty()) return 2;
    reg = wbuf[0];
    if (cur == 0x20 && reg == 0x01 && wbuf.size() >= 2) g_tca_out = wbuf[1];
    return (cur == 0x20 || cur == 0x40 || cur == 0x41 || cur == 0x48) ? 0 : 2;
}
int TwoWire::requestFrom(int a, int n)
{
    rbuf.clear();
    uint16_t v = 0;
    if (a == 0x20 && reg == 0x00) { uint8_t in = (uint8_t)((g_key ? 0x04 : 0) | (g_btn ? 0 : 0x80)); rbuf.push_back(in); return 1; }
    if (a == 0x40 && reg == 0x02) v = (uint16_t)(g_vbat_mv * 100u / 125u);
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
    g_ms = 1000; g_tca_out = 0; g_key = true; g_btn = false; g_vbat_mv = 8000; g_cont_mv = 1200;
    s_proto = P_NONE; s_port = PORT_OFF; s_state = ZR_ST_IDLE; s_latched = 0; s_cd = false; s_detached = false;
    s_fired_ms = 0; s_fire_end_ms = 0; s_detach_req = false; s_last_cmd_ms = 0; s_lora_on = false;
    Wire.slave_on = false; Serial1.on = false; Serial1.rx.clear(); Serial1.tx.clear();
    setup();
}

// ---------------------------------------------------------------- Ariadna (Daidalos)
static uint8_t a_seq = 0;
/** One Ariadna Navigation-MCU transaction: write the 16 B command, read 20 B. Returns 0 = NACK. */
static int ariadna_xfer(uint8_t cmd, int32_t t_rem, ariadna_rampa_status_t *out)
{
    if (!Wire.slave_on) return 0;                         // nobody ACKs address 0x2B
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

static void test_ariadna()
{
    printf("== Daidalos / Ariadna V2 on the pogo (I2C slave 0x2B)\n");
    reset_rampa();
    ariadna_rampa_status_t st{};
    int ok = 0;
    for (int i = 0; i < 40; i++) { ok = ariadna_xfer(RAMPA_CMD_PING, -1, &st); run_ms(50); }   // 2 s of 20 Hz pings
    CHECK(ok && s_proto == P_ARIADNA, "recognised as Ariadna");
    CHECK(!(g_tca_out & 0x02), "UMB_5V stays OFF for an Ariadna");
    CHECK(st.state == RAMPA_STATE_IDLE && (st.flags & ARIADNA_RMP_LORA_OFF), "IDLE, LoRa off (state %u flags %02X)", st.state, st.flags);
    for (int i = 0; i < 10; i++) { ariadna_xfer(RAMPA_CMD_PREPARE, -1, &st); run_ms(50); }
    CHECK(st.state == RAMPA_STATE_READY && (st.flags & ARIADNA_RMP_READY), "PREPARE -> READY");
    const uint32_t t0 = g_ms + 10000;
    bool committed_same_xfer = false;
    while ((int32_t)(t0 - g_ms) > 100) { ariadna_xfer(RAMPA_CMD_COUNTDOWN, (int32_t)(t0 - g_ms), &st); run_ms(50); }
    CHECK(st.state == RAMPA_STATE_COUNTDOWN, "COUNTDOWN running (T-%ld)", (long)st.t_rem_ms);
    // Ariadna's T-100 ms commit: the answer must be COMMITTED in the same transaction
    ariadna_xfer(RAMPA_CMD_COMMIT, (int32_t)(t0 - g_ms), &st);
    committed_same_xfer = st.state == RAMPA_STATE_COMMITTED && (st.flags & ARIADNA_RMP_DETACHING);
    CHECK(committed_same_xfer, "COMMITTED returned in the same I2C transaction (state %u)", st.state);
    run_ms(5);
    CHECK(!Wire.slave_on && s_detached, "umbilical detached (Hi-Z) before T-0");
    bool fired = false;
    while ((int32_t)(g_ms - t0) < 50) { run_ms(1); if (g_pin[17] && g_pin[19]) fired = true; }
    CHECK(fired, "motor igniter (channel 2 from the command) fired at T-0");
    run_ms(1200);
    CHECK(!g_pin[17] && !g_pin[19], "firing pulse ended (1 s)");
    run_ms(7000);
    CHECK(Wire.slave_on, "umbilical re-attached after the launch window");
    // abort path: button
    for (int i = 0; i < 10; i++) { ariadna_xfer(RAMPA_CMD_PREPARE, -1, &st); run_ms(50); }
    const uint32_t t1 = g_ms + 15000;
    for (int i = 0; i < 20; i++) { ariadna_xfer(RAMPA_CMD_COUNTDOWN, (int32_t)(t1 - g_ms), &st); run_ms(50); }
    g_btn = true; run_ms(60); g_btn = false;
    ariadna_xfer(RAMPA_CMD_COUNTDOWN, (int32_t)(t1 - g_ms), &st);
    CHECK(st.state == RAMPA_STATE_ABORT && (st.fault_mask & ARIADNA_RMPF_BTN_ABORT), "local abort button -> ABORT");
    run_ms(20000);
    CHECK(!g_pin[16] && !g_pin[17] && !g_pin[18] && !g_pin[19], "nothing fired after the abort");
}

// ---------------------------------------------------------------- Zenith
static uint8_t z_seq = 0;
static zl_rx_t z_rx;
static zl_rmp_status_t z_st;
static int z_got = 0;

static void zenith_send(uint8_t cmd, int32_t t_rem, uint8_t flags = ZL_RK_GO | ZL_RK_NAV_OK)
{
    zl_rmp_cmd_t c{}; c.cmd = cmd; c.seq = ++z_seq; c.t_rem_ms = t_rem; c.rocket_flags = flags; c.pyro_ch = 1; c.fire_ms = 800;
    uint8_t f[ZL_FRAME_MAX]; uint16_t n = zl_encode(f, ZL_PKT_RMP_CMD, &c, sizeof c);
    if (Serial1.on) Serial1.rx.insert(Serial1.rx.end(), f, f + n);   // bytes on a closed UART are lost
}
static void zenith_rx()
{
    for (uint8_t b : Serial1.tx) {
        const uint8_t *pl; uint8_t n;
        if (zl_rx_byte(&z_rx, b, &pl, &n) == ZL_PKT_RMP_STATUS && zl_take(&z_st, sizeof z_st, pl, n)) z_got++;
    }
    Serial1.tx.clear();
}
static void zenith_tick(uint8_t cmd, int32_t t_rem) { zenith_send(cmd, t_rem); run_ms(50); zenith_rx(); }

static void test_zenith()
{
    printf("== Zenith on the pogo (UART 115200)\n");
    reset_rampa(); zl_rx_init(&z_rx); z_got = 0;
    run_ms(1400);
    CHECK(!(g_tca_out & 0x02), "UMB_5V off during the first 1.5 s (looking for an Ariadna)");
    run_ms(300);
    CHECK(g_tca_out & 0x02, "then UMB_5V on (powers a Zenith with its battery off)");
    for (int i = 0; i < 60; i++) zenith_tick(ZR_CMD_PING, -1);   // Zenith boots and pings at 20 Hz
    CHECK(s_proto == P_ZENITH && z_got > 10, "recognised as Zenith (%d answers)", z_got);
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

int main()
{
    test_ariadna();
    test_zenith();
    printf("\n%s (%d failures)\n", g_fail ? "RAMPA TEST FAILED" : "RAMPA TEST PASSED", g_fail);
    return g_fail ? 1 : 0;
}
