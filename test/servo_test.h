// Remote arming switch: a servo (J12) closes the second switch in series with the key.
#pragma once

static void test_servo()
{
    printf("== arming servo: configuration over USB, stored on flash\n");
    reset_rampa();
    run_ms(2500);
    std::string o = usb("SET arm_servo 1", 300);
    CHECK(o.find("#OK SET") != std::string::npos && ts_cfg.arm_servo == 1 && o.find("$CFG,arm_servo,1") != std::string::npos, "SET arm_servo 1");
    run_ms(50);
    CHECK(g_servo_us[27] == 1000, "arming servo (GP27) parked in SAFE (%d us)", g_servo_us[27]);
    reset_rampa();                                        // power cycle: the configuration comes back from flash
    g_servo_sim = true;
    run_ms(2500);
    CHECK(ts_cfg.arm_servo == 1 && g_servo_us[27] == 1000 && !s_key, "after reboot: servo 1, SAFE, arming switch open");

    printf("== arming servo: thrust stand\n");
    gs_send(SL_C_PING); run_link(1200);
    CHECK(s_stand, "stand mode");
    sl_tlm_t t{};
    last_tlm(t);
    CHECK((t.pyro & 0x04) && !(t.pyro & 0x08), "TLM: arming servo configured, not armed (pyro 0x%02X)", t.pyro);
    CHECK(gs_cmd(SL_C_ARM, 1, 800) == SL_RC_OK, "ARM accepted although the key sense is open (the servo will close it)");
    CHECK(g_servo_us[27] == 2000, "servo turned to arm_us");
    run_link(1500);
    last_tlm(t);
    CHECK(s_sl == SL_ST_ARMED && s_key && s_as == AS_ARMED && (t.pyro & 0x08), "key sensed (%u ms) -> ARMED", s_as_close_ms);
    gs_cmd(SL_C_FIRE, 3);
    bool fired = false;
    for (int i = 0; i < 4300; i++) { if (i % 400 == 0) gs_send(SL_C_PING); run_ms(1); if (g_pin[16] && g_pin[19]) fired = true; }
    CHECK(fired, "igniter fired at T-0");
    CHECK(g_servo_us[27] == 1000, "right after the igniter pulse the servo opens the arming switch");
    run_link(500);
    CHECK(!s_key, "pyro supply disconnected while the record continues");
    run_link(6000);
    CHECK(s_sl == SL_ST_SAFE && ts_res_ready, "record closed");
    CHECK(gs_cmd(SL_C_ARM, 1) == SL_RC_OK, "ARM again");
    run_link(1500);
    CHECK(gs_cmd(SL_C_DISARM) == SL_RC_OK && g_servo_us[27] == 1000, "DISARM -> servo SAFE");
    gs_cmd(SL_C_ARM, 1); run_link(1500);
    gs_cmd(SL_C_FIRE, 10); run_link(1000);
    CHECK(gs_cmd(SL_C_ABORT) == SL_RC_ABORTED, "ABORT in the countdown");
    run_link(200);
    CHECK(g_servo_us[27] == 1000 && !fired_any(), "ABORT -> servo SAFE, nothing fired");
    gs_cmd(SL_C_DISARM);
    g_human = false;                                      // somebody forgot the key
    CHECK(gs_cmd(SL_C_ARM, 1) == SL_RC_OK, "ARM with the human switch open");
    run_link(2500);
    CHECK(s_sl == SL_ST_ABORT && (s_sl_latched & SL_FLT_KEY) && g_servo_us[27] == 1000, "key never sensed -> ABORT (key), servo back to SAFE");
    g_human = true;
    gs_cmd(SL_C_DISARM);

    printf("== arming servo over USB\n");
    o = usb("SERVO 2 1500");
    CHECK(o.find("#OK SERVO 2 1500") != std::string::npos && g_servo_us[28] == 1500, "SERVO 2 1500 (GP28)");
    o = usb("SERVO 2 OFF");
    CHECK(g_servo_us[28] == 0, "SERVO 2 OFF");
    o = usb("SERVO 1 2000", 600);
    CHECK(g_servo_us[27] == 2000 && s_key, "SERVO 1 2000: position test of the arming servo");
    run_ms(10000);
    CHECK(g_servo_us[27] == 1000, "...back to SAFE by itself after 10 s");
    CHECK(usb("SERVO 1 OFF").find("#ERR") != std::string::npos, "the arming servo cannot be released");
    o = usb("ARMTEST", 1000);
    CHECK(o.find("#OK ARMTEST key closed after") != std::string::npos, "ARMTEST: key closed");
    run_ms(3500);
    CHECK(g_servo_us[27] == 1000, "ARMTEST ends in SAFE");
    g_human = false;
    o = usb("ARMTEST", 2000);
    CHECK(o.find("#ERR ARMTEST key not sensed") != std::string::npos && g_servo_us[27] == 1000, "ARMTEST with the human switch open -> error, SAFE");
    g_human = true;
    usb("STAND 0", 100);

    printf("== arming servo: Zenith on the pogo\n");
    reset_rampa(); zl_rx_init(&z_rx);
    g_servo_sim = true;
    run_ms(1700);
    for (int i = 0; i < 40; i++) zenith_tick(ZR_CMD_PING, -1);
    CHECK(s_proto == P_ZENITH && g_servo_us[27] == 1000 && z_st.state == ZR_ST_IDLE, "Zenith IDLE, servo SAFE");
    g_cont_mv = 100;                                      // igniter missing: never arm
    for (int i = 0; i < 20; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    CHECK(g_servo_us[27] == 1000 && z_st.state == ZR_ST_FAULT, "PREPARE without igniter -> FAULT, servo stays SAFE");
    g_cont_mv = 1200;
    for (int i = 0; i < 5; i++) zenith_tick(ZR_CMD_PING, -1);
    zenith_tick(ZR_CMD_PREPARE, -1);
    CHECK(g_servo_us[27] == 2000 && z_st.state == ZR_ST_IDLE, "PREPARE -> servo arms, rampa waits for the key (IDLE)");
    for (int i = 0; i < 20; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    CHECK(z_st.state == ZR_ST_READY && (z_st.flags & ZL_RMP_KEY), "key sensed -> READY");
    const uint32_t t0 = g_ms + 12000;
    while ((int32_t)(t0 - g_ms) > 300) zenith_tick(ZR_CMD_COUNTDOWN, (int32_t)(t0 - g_ms));
    zenith_tick(ZR_CMD_COMMIT, (int32_t)(t0 - g_ms));
    CHECK(z_st.state == ZR_ST_COMMITTED && g_servo_us[27] == 2000, "COMMITTED, servo armed");
    bool fz = false;
    while ((int32_t)(g_ms - t0) < 1200) { run_ms(1); if (g_pin[16] && g_pin[19]) fz = true; }
    CHECK(fz && g_servo_us[27] == 1000, "fired at T-0, servo SAFE after the pulse");
    printf("== arming servo: Zenith SAFE / rocket gone\n");
    reset_rampa(); zl_rx_init(&z_rx); g_servo_sim = true;
    run_ms(1700);
    for (int i = 0; i < 40; i++) zenith_tick(ZR_CMD_PING, -1);
    for (int i = 0; i < 20; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    CHECK(z_st.state == ZR_ST_READY && g_servo_us[27] == 2000, "READY, armed");
    zenith_tick(ZR_CMD_SAFE, -1); zenith_tick(ZR_CMD_PING, -1);
    CHECK(g_servo_us[27] == 1000, "SAFE / PING -> servo SAFE");
    for (int i = 0; i < 20; i++) zenith_tick(ZR_CMD_PREPARE, -1);
    run_ms(2000);                                         // the rocket stops talking
    CHECK(g_servo_us[27] == 1000, "no rocket command for 1.5 s -> servo SAFE");

    printf("== configuration v1 (before the servo) is migrated, calibration kept\n");
    {
        uint8_t v1[68]; ts_cfg_t d; ts_cfg_defaults(d);
        memcpy(v1, &d, 64);
        v1[4] = 1; v1[5] = 0;                             // ver 1
        v1[6] = 50; v1[7] = 0;                            // seq
        const float sc = 0.0123f; memcpy(v1 + 12, &sc, 4);   // lc_scale
        v1[64] = 0; v1[65] = 0;                           // pad
        const uint16_t c = sl_crc16(v1, 66); v1[66] = (uint8_t)c; v1[67] = (uint8_t)(c >> 8);
        memset(&g_flash[0], 0xFF, 0x2000);
        memcpy(&g_flash[0], v1, 68);
        reset_rampa();
        run_ms(200);
        CHECK(ts_cfg.ver == 2 && fabsf(ts_cfg.lc_scale - 0.0123f) < 1e-7 && ts_cfg.arm_servo == 0 && ts_cfg.safe_us == 1000,
              "v1 -> v2: lc_scale %.4f kept, servo defaults", ts_cfg.lc_scale);
    }
}
