// Thrust-stand mode of the real Rampa_MCU.ino: the test plays the ground
// station (LoRa STAND packets), the operator (USB lines) and the motor.
#pragma once

static uint8_t gs_seq = 0;

static void gs_send(uint8_t cmd, int32_t arg = 0, uint16_t arg2 = 0, bool same_seq = false)
{
    sl_cmd_pkt_t c{};
    if (!same_seq) gs_seq = (uint8_t)(gs_seq % 255 + 1);
    c.seq = gs_seq; c.cmd = cmd; c.arg = arg; c.arg2 = arg2; c.token = sl_token_for(cmd);
    sl_seal(&c, sizeof c, SL_CMD_MAGIC);
    LoRa.rx.push_back(std::vector<uint8_t>((uint8_t *)&c, (uint8_t *)&c + sizeof c));
}

static bool last_tlm(sl_tlm_t &t)
{
    for (auto it = LoRa.tx.rbegin(); it != LoRa.tx.rend(); ++it)
        if (sl_check(it->b.data(), it->b.size(), sizeof t, SL_TLM_MAGIC)) { memcpy(&t, it->b.data(), sizeof t); return true; }
    return false;
}
template <typename T> static bool last_pkt(T &p, uint8_t magic)
{
    for (auto it = LoRa.tx.rbegin(); it != LoRa.tx.rend(); ++it)
        if (sl_check(it->b.data(), it->b.size(), sizeof p, magic)) { memcpy(&p, it->b.data(), sizeof p); return true; }
    return false;
}

/** The ground station's habit: a PING after the rampa's telemetry, every 400 ms. */
static void run_link(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i += 100) { if (i % 400 == 0) gs_send(SL_C_PING); run_ms(100); }
}

/** Sends a command and runs until the telemetry acknowledges it; returns the rc. */
static uint8_t gs_cmd(uint8_t cmd, int32_t arg = 0, uint16_t arg2 = 0)
{
    gs_send(cmd, arg, arg2);
    for (int i = 0; i < 30; i++) {
        run_ms(100);
        sl_tlm_t t;
        if (last_tlm(t) && t.ack_seq == gs_seq && t.ack_rc) return t.ack_rc;
        if (i % 4 == 3) gs_send(cmd, arg, arg2, true);   // retry, same seq
    }
    return 0;
}

static std::string usb(const char *line, uint32_t run = 50)
{
    Serial.out.clear();
    Serial.host(line);
    run_ms(run);
    return Serial.out;
}
static bool has_line(const std::string &out, const char *prefix)
{
    return out.find(std::string("\n") + prefix) != std::string::npos || out.compare(0, strlen(prefix), prefix) == 0;
}

static bool fired_any(void) { return g_pin[16] || g_pin[17] || g_pin[18] || g_pin[19]; }

static void test_stand()
{
    printf("== thrust stand: boot, sensors\n");
    reset_rampa();
    run_ms(3000);
    CHECK(ts_ready && ts_flash_ok && ts_jedec == TS_JEDEC_W25Q128, "W25Q128 found (JEDEC %06lX)", (unsigned long)ts_jedec);
    CHECK(ts_ads_ok && ts_sps > 1900 && ts_sps <= 2000, "ADS1220 converting, %u load-cell samples/s", ts_sps);
    CHECK(s_lora_on && !s_stand, "no rocket: LoRa on, still a launch pad");
    const std::string hello = usb("HELLO");
    CHECK(has_line(hello, "$ID,Rampa,3.00") && hello.find("$CFG,rec_s,") != std::string::npos && has_line(hello, "$S,"), "USB HELLO -> $ID, $CFG, $S");

    printf("== stand mode over LoRa\n");
    gs_send(SL_C_PING);
    run_ms(1200);
    sl_tlm_t t{};
    CHECK(s_stand && s_port == PORT_OFF, "PING from the station -> stand mode, umbilical closed");
    CHECK(last_tlm(t) && t.state == SL_ST_SAFE, "stand telemetry on the air (state %u)", t.state);
    CHECK((t.flags & SL_F_ADS_OK) && (t.flags & SL_F_FLASH_OK) && (t.flags & SL_F_TH1) && (t.flags & SL_F_TH2) && (t.flags & SL_F_PRESS),
          "flags: ADS, flash, both thermistors, transducer (0x%04X)", t.flags);
    CHECK(fabs(t.t1_dc / 10.0 - 25.0) < 0.5 && fabs(t.t2_dc / 10.0 - 40.0) < 0.5, "thermistors %.1f / %.1f degC", t.t1_dc / 10.0, t.t2_dc / 10.0);
    CHECK(abs(t.p_dpsi) < 30, "pressure ~0 psi (%.1f)", t.p_dpsi / 10.0);

    printf("== tare + calibration (USB) and pressure\n");
    std::string o = usb("TARE", 800);
    CHECK(o.find("#OK TARE") != std::string::npos && ts_tared && labs(ts_cfg.lc_offset - (long)SIM_LC_OFFSET) < 10, "TARE: offset %ld", (long)ts_cfg.lc_offset);
    g_force_n = 98.0665;                                   // 10 kg on the load cell
    run_ms(200);
    o = usb("CAL 98.0665", 800);
    CHECK(o.find("#OK CAL") != std::string::npos && fabs(ts_cfg.lc_scale - 1.0 / SIM_CODES_PER_N) < 1e-5, "CAL with 10 kg: scale %.6g N/code", ts_cfg.lc_scale);
    g_psi = 250;
    run_link(600);
    last_tlm(t);
    CHECK(fabs(sl_funpack(t.f[SL_SUBS - 1], t.fexp) - 98.07) < 0.5 && (t.flags & SL_F_LC_CAL), "force over LoRa %.2f N", sl_funpack(t.f[SL_SUBS - 1], t.fexp));
    CHECK(fabs(t.p_dpsi / 10.0 - 250) < 3, "pressure over LoRa %.1f psi", t.p_dpsi / 10.0);
    g_force_n = 0; g_psi = 0;
    o = usb("SET rec_s 4", 200);
    CHECK(ts_cfg.rec_s == 4 && o.find("#OK SET") != std::string::npos, "SET rec_s 4 stored");
    o = usb("FIRE 5");
    CHECK(o.find("#ERR FIRE only from the ground station") != std::string::npos, "FIRE over USB refused");

    printf("== ARM interlocks\n");
    g_key = false;
    run_link(300);
    { uint8_t rc = gs_cmd(SL_C_ARM, 1, 1000); CHECK(rc == SL_RC_REJ_INTLK && s_sl == SL_ST_SAFE, "ARM with the key off -> refused (interlock) rc %u st %u", rc, s_sl); }
    g_key = true; g_cont_mv = 100;
    run_link(300);
    CHECK(gs_cmd(SL_C_ARM, 1, 1000) == SL_RC_REJ_INTLK, "ARM without igniter continuity -> refused");
    g_cont_mv = 1200;
    run_link(300);
    CHECK(gs_cmd(SL_C_FIRE, 5) == SL_RC_REJ_STATE, "FIRE when not armed -> refused");

    printf("== ARM, FIRE, burn, results\n");
    CHECK(gs_cmd(SL_C_ARM, 1, 800) == SL_RC_OK, "ARM igniter 1, 800 ms accepted");
    run_link(1500);
    CHECK(s_sl == SL_ST_ARMED && ts_rec == TR_READY && !fired_any(), "ARMED (slot erased, %u blocks), pyro off", fl.erases);
    const uint16_t test_before = ts_test_no;
    CHECK(gs_cmd(SL_C_FIRE, 5) == SL_RC_OK && s_sl == SL_ST_COUNTDOWN, "FIRE 5 s -> COUNTDOWN");
    const uint32_t t0 = s_sl_t0;
    bool early = false, fired = false; uint32_t t_fire = 0;
    uint32_t k = 0;
    while ((int32_t)(g_ms - t0) < 30) {
        if (k++ % 400 == 0) gs_send(SL_C_PING);
        run_ms(1);
        if (!fired && g_pin[16] && g_pin[19]) { fired = true; t_fire = g_ms; }
        if (fired_any() && (int32_t)(g_ms - t0) < 0) early = true;
    }
    CHECK(!early, "nothing on before T-0");
    CHECK(fired && labs((long)(int32_t)(t_fire - t0)) <= 2, "igniter 1 fired at T-0 (%+ld ms)", (long)(int32_t)(t_fire - t0));
    CHECK(ts_rec == TR_RECORDING && ts_test_no == test_before + 1, "recording record #%u", ts_test_no);
    // the motor: 150 ms ignition delay, 100 N for 2 s, chamber 300 psi, case warms up
    for (uint32_t ms = 0; ms < 4600; ms++) {
        const uint32_t dt = g_ms - t0;
        g_force_n = (dt > 150 && dt < 2150) ? 100.0 : 0.0;
        g_psi = (dt > 150 && dt < 2150) ? 300.0 : 0.0;
        g_t1_c = 25.0 + (dt > 150 ? (dt < 3000 ? (dt - 150) * 0.01 : 28.5) : 0);
        if (ms % 400 == 0) gs_send(SL_C_PING);
        run_ms(1);
        if (dt == 900) CHECK(!fired_any(), "800 ms igniter pulse ended");
    }
    run_link(1500);
    CHECK(s_sl == SL_ST_SAFE && ts_res_ready && ts_rec == TR_IDLE, "record closed, back to SAFE (state %u)", s_sl);
    const sl_res_t &r = ts_res;
    CHECK((r.flags & SL_RF_FIRED) && (r.flags & SL_RF_IGNITION) && (r.flags & SL_RF_SAVED), "result: fired, ignition, saved (0x%02X)", r.flags);
    CHECK(fabs(r.peak_n - 100.0) < 1.0, "peak %.2f N", r.peak_n);
    CHECK(fabs(r.impulse_ns - 200.0) < 3.0, "impulse %.2f N.s (expected ~200)", r.impulse_ns);
    CHECK(fabs(r.burn_s - 2.0) < 0.05, "burn time %.2f s", r.burn_s);
    CHECK(fabs(r.ign_delay_s - 0.15) < 0.03, "ignition delay %.2f s", r.ign_delay_s);
    CHECK(fabs(r.pmax_psi - 300.0) < 3.0, "max pressure %.1f psi", r.pmax_psi);
    CHECK(r.t1max_dc > 450, "max temperature T1 %.1f degC", r.t1max_dc / 10.0);
    CHECK(r.n_frames > 2000u * 6u && ts_dropped == 0, "%lu frames on flash at full rate, none dropped", (unsigned long)r.n_frames);
    sl_res_t rp{};
    CHECK(last_pkt(rp, SL_RES_MAGIC) && rp.test_no == ts_test_no && fabs(rp.impulse_ns - r.impulse_ns) < 1e-3, "RES packet sent to the station");
    CHECK(!s_sl_hang, "no hang-fire");

    printf("== curve download over LoRa\n");
    gs_send(SL_C_CURVE, 120);
    run_ms(150);
    sl_curve_t c{};
    CHECK(last_pkt(c, SL_CRV_MAGIC) && c.offset == 120 && c.n == SL_CRV_PTS, "CURVE 120 -> 16 points");
    CHECK(fabs(sl_funpack(c.f[10], c.fexp) - 100.0) < 1.0 && fabs(c.p[10] / 10.0 - 300) < 3, "curve point = 100 N / 300 psi (T0 at %d)", r.t0_idx);

    printf("== cropped burn over LoRa (BURN, selective repeat)\n");
    {
        LoRa.tx.clear();
        gs_send(SL_C_BURN, 0, 0);
        run_ms(2500);
        std::vector<sl_burn_t> got;
        uint32_t t_first = 0, t_last = 0;
        for (auto &k : LoRa.tx) {
            sl_burn_t b;
            if (sl_check(k.b.data(), k.b.size(), sizeof b, SL_BURN_MAGIC)) {
                memcpy(&b, k.b.data(), sizeof b); got.push_back(b);
                if (!t_first) t_first = k.t_ms;
                t_last = k.t_ms;
            }
        }
        CHECK(!got.empty() && got[0].n_blocks == got.size(), "burst: %zu of %u blocks", got.size(), got.empty() ? 0 : got[0].n_blocks);
        if (!got.empty()) {
            const sl_burn_t &b0 = got[0];
            CHECK(b0.rate_hz == 50 && abs(b0.burn_a_ms - 150) <= 10 && abs(b0.burn_b_ms - 2150) <= 20,
                  "burn detected %ld .. %ld ms after T-0 (motor 150 .. 2150), 50 Hz", (long)b0.burn_a_ms, (long)b0.burn_b_ms);
            CHECK(abs(b0.t_first_ms - (b0.burn_a_ms - SL_BURN_PRE_MS)) <= 10, "crop starts %ld ms (burn start - 250 ms)", (long)b0.t_first_ms);
            int total = 0; double fmid = 0, pmid = 0, fpre = 99;
            for (auto &b : got) for (int i = 0; i < b.n; i++) {
                const int j = b.idx * SL_BURN_PTS + i;
                const double tm = b0.t_first_ms + j * 1000.0 / b0.rate_hz;
                if (fabs(tm - 1000) < 10) { fmid = sl_funpack(b.f[i], b.fexp); pmid = b.p[i] / 10.0; }
                if (fabs(tm - 0) < 10) fpre = sl_funpack(b.f[i], b.fexp);
                total++;
            }
            const double t_end = b0.t_first_ms + total * 20.0;
            CHECK(fabs(t_end - (b0.burn_b_ms + SL_BURN_POST_MS)) <= 30, "crop ends %.0f ms (burn end + 500 ms), %d points", t_end, total);
            CHECK(fabs(fmid - 100) < 1.0 && fabs(pmid - 300) < 3 && fabs(fpre) < 1.0, "values: T+1 s %.1f N %.0f psi, T-0 %.1f N", fmid, pmid, fpre);
            CHECK(t_last - t_first < 1500, "whole burst on the air in %u ms", t_last - t_first);
            CHECK(got.back().t1_dc > 300, "block temperatures (T1 %.1f degC)", got.back().t1_dc / 10.0);
            // a block lost: ask for it alone
            LoRa.tx.clear();
            gs_send(SL_C_BURN, 2, 1);
            run_ms(600);
            int n2 = 0, other = 0;
            for (auto &k : LoRa.tx) { sl_burn_t b; if (sl_check(k.b.data(), k.b.size(), sizeof b, SL_BURN_MAGIC)) { memcpy(&b, k.b.data(), sizeof b); if (b.idx == 2) n2++; else other++; } }
            CHECK(n2 == 1 && other == 0, "BURN 2 1 -> only block 2 again");
            sl_tlm_t t2{};
            CHECK(last_tlm(t2) && t2.ack_seq == gs_seq && t2.ack_rc == SL_RC_OK, "BURN acknowledged in the telemetry");
            CHECK(gs_cmd(SL_C_BURN, 99, 1) == SL_RC_REJ_ARG, "BURN past the last block -> rejected");
        }
    }

    printf("== flash record via USB\n");
    o = usb("LIST", 50);
    char want[32]; snprintf(want, sizeof want, "$REC,%u,", ts_test_no);
    CHECK(o.find(want) != std::string::npos, "LIST shows record %u", ts_test_no);
    snprintf(want, sizeof want, "DUMP %u", ts_test_no);
    Serial.out.clear(); Serial.host(want);
    for (int i = 0; i < 2000 && Serial.out.find("$DEND") == std::string::npos; i++) run_ms(1);
    o = Serial.out;
    std::vector<uint8_t> img;
    size_t pos = 0;
    while ((pos = o.find("$D,", pos)) != std::string::npos) {
        const size_t c1 = o.find(',', pos + 3), e = o.find('\r', c1);
        for (size_t j = c1 + 1; j + 1 < e; j += 2) img.push_back((uint8_t)strtol(o.substr(j, 2).c_str(), nullptr, 16));
        pos = e;
    }
    ts_hdr_t h{}; ts_end_t en{};
    if (img.size() < 256) img.resize(256, 0);
    memcpy(&h, img.data(), sizeof h); memcpy(&en, img.data() + 128, sizeof en);
    if (img.size() < TS_FRAMES_OFF + (size_t)en.n_frames * 16) en.n_frames = 0;
    CHECK(img.size() > 100000 && h.magic == TS_HDR_MAGIC && en.magic == TS_END_MAGIC && en.n_frames == r.n_frames, "DUMP: header + footer + %zu bytes", img.size());
    if (FILE *df = fopen("build/dump_test.bin", "wb")) { fwrite(img.data(), 1, img.size(), df); fclose(df); }   // for gui/rampa_record.py
    uint32_t good = 0, fire_frames = 0; int32_t lc_max = 0;
    for (uint32_t i = 0; i < en.n_frames; i++) {
        ts_frame_t fr; memcpy(&fr, img.data() + TS_FRAMES_OFF + i * 16, 16);
        if (ts_crc8((uint8_t *)&fr, 15) == fr.crc8) good++;
        if (fr.flags & TS_FR_FIRE) fire_frames++;
        if (fr.lc > lc_max) lc_max = fr.lc;
    }
    CHECK(good == en.n_frames, "every frame CRC ok (%u / %lu)", good, (unsigned long)en.n_frames);
    CHECK(fire_frames > 1500 && fire_frames < 1700, "igniter-on frames = 800 ms at 2 kHz (%u)", fire_frames);
    CHECK(fabs(ts_force(h.cfg, lc_max) - 100.0) < 1.0, "offline conversion with the header calibration: %.2f N", ts_force(h.cfg, lc_max));

    printf("== abort paths\n");
    // link loss in the countdown
    CHECK(gs_cmd(SL_C_ARM, 2, 1000) == SL_RC_OK, "ARM igniter 2");
    run_link(1500);
    CHECK(gs_cmd(SL_C_FIRE, 10) == SL_RC_OK, "FIRE 10 s");
    run_ms(3000);                                           // station silent
    CHECK(s_sl == SL_ST_ABORT && (s_sl_latched & SL_FLT_LINK) && !fired_any(), "no station packet for 2.5 s -> ABORT (faults 0x%02X)", s_sl_latched);
    run_link(9000);
    CHECK(!fired_any() && s_sl == SL_ST_ABORT, "nothing fired, ABORT latched");
    CHECK(gs_cmd(SL_C_ARM, 1) == SL_RC_REJ_STATE, "ARM refused until DISARM");
    CHECK(gs_cmd(SL_C_DISARM) == SL_RC_OK && s_sl == SL_ST_SAFE, "DISARM -> SAFE");
    // local button
    CHECK(gs_cmd(SL_C_ARM, 1) == SL_RC_OK, "ARM again");
    run_link(1500);
    gs_cmd(SL_C_FIRE, 8);
    run_link(1000);
    g_btn = true; run_ms(60); g_btn = false;
    CHECK(s_sl == SL_ST_ABORT && (s_sl_latched & SL_FLT_BTN), "local button -> ABORT");
    run_link(8000);
    CHECK(!fired_any(), "nothing fired");
    gs_cmd(SL_C_DISARM);
    // ABORT from the station, key pulled
    gs_cmd(SL_C_ARM, 1); run_link(1500);
    gs_cmd(SL_C_FIRE, 6); run_link(1000);
    CHECK(gs_cmd(SL_C_ABORT) == SL_RC_ABORTED && s_sl == SL_ST_ABORT, "ABORT from the station");
    run_link(6000);
    CHECK(!fired_any(), "nothing fired");
    gs_cmd(SL_C_DISARM);
    gs_cmd(SL_C_ARM, 1); run_link(1500);
    gs_cmd(SL_C_FIRE, 6); run_link(1000);
    g_key = false; run_ms(100); g_key = true;
    CHECK(s_sl == SL_ST_ABORT && (s_sl_latched & SL_FLT_KEY), "key switched off in the countdown -> ABORT");
    run_link(6000);
    CHECK(!fired_any(), "nothing fired");
    gs_cmd(SL_C_DISARM);

    printf("== hang-fire and cold recording\n");
    gs_cmd(SL_C_ARM, 1, 500); run_link(1500);
    gs_cmd(SL_C_FIRE, 3);
    run_link(9000);                                         // igniter fires, the motor does nothing
    CHECK(s_sl == SL_ST_SAFE && ts_res_ready && s_sl_hang && (stand_flags() & SL_F_HANGFIRE), "no thrust after firing -> HANG-FIRE warning");
    CHECK(gs_cmd(SL_C_RECORD, 2) == SL_RC_OK && s_sl == SL_ST_RECORD, "cold RECORD 2 s");
    g_force_n = 20;
    run_link(3500);
    g_force_n = 0;
    CHECK(s_sl == SL_ST_SAFE && (ts_res.flags & SL_RF_COLD) && fabs(ts_res.peak_n - 20) < 0.5 && !fired_any(),
          "cold record closed, peak %.1f N, pyro never on", ts_res.peak_n);
    CHECK(!s_sl_hang, "a cold record clears the hang-fire warning");

    printf("== sensors missing\n");
    g_ads_present = false;
    run_link(1200);
    CHECK(!ts_ads_ok && gs_cmd(SL_C_ARM, 1) == SL_RC_REJ_INTLK && (s_sl_latched & SL_FLT_ADS), "ADS1220 gone -> ARM refused");
    g_ads_present = true;
    run_link(2500);
    CHECK(ts_ads_ok, "ADS1220 back by itself");

    printf("== back to a launch pad\n");
    run_ms(61000);
    CHECK(!s_stand && s_port != PORT_OFF, "60 s without the station -> stand mode left, scanning the pogo");
    CHECK(!fired_any(), "pyro off");
}
