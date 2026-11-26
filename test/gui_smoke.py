#!/usr/bin/env python3
"""Smoke test for Rampa standalone diagnostics (protocol, simulator, and optional PyQt6 GUI).

Always runs pure-Python tests (syntax check of all gui/*.py, $S & legacy parser,
Go/No-Go checks, and RampaSimCore command/safety logic). If PyQt6 is installed,
also runs an offscreen GUI smoke test matching Zenith's test/gui_smoke.py.

    python3 test/gui_smoke.py
"""
from __future__ import annotations

import os
import py_compile
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GUI_DIR = os.path.join(HERE, "..", "gui")
sys.path.insert(0, GUI_DIR)

import rampa_proto as rp  # noqa: E402

fails = 0


def check(cond: bool, msg: str) -> None:
    global fails
    print(("   ok:   " if cond else "   FAIL: ") + msg)
    if not cond:
        fails += 1


print("== Python syntax check (gui/*.py)")
for fn in sorted(os.listdir(GUI_DIR)):
    if fn.endswith(".py"):
        path = os.path.join(GUI_DIR, fn)
        py_compile.compile(path, doraise=True)
        check(True, f"compiled gui/{fn}")

print("== Protocol parser & legacy 1 Hz compatibility")
sim = rp.RampaSimCore()
replies = sim.handle_command("HELLO")
check(any(r.startswith("$ID,Rampa PCB1,") for r in replies), "HELLO returns $ID")
s_lines = [r for r in replies if r.startswith("$S,")]
check(len(s_lines) == 1, "HELLO returns immediate $S")
parsed = rp.parse_status_line(s_lines[0])
check(parsed is not None and parsed["vbat_mv"] > 7000 and parsed["legacy"] == 0, "$S parsed cleanly")
check(parsed is not None and parsed["c1_mv"] > rp.CONT_MIN_MV, "CH1 continuity present in demo")

leg_line = "RAMPA scan port 2 st 1 flags 0x45 faults 0x00 vbat 7920 umb 140 mA cont 1200/0/0 key 1 btn 0 lora 1"
leg = rp.parse_legacy_line(leg_line)
check(leg is not None and leg["legacy"] == 1 and leg["vbat_mv"] == 7920 and leg["c1_mv"] == 1200, "legacy 1 Hz line parsed")

i2c_rep = sim.handle_command("I2C SCAN")
addrs = rp.parse_i2c_scan_line(i2c_rep[0])
check(addrs == [0x20, 0x40, 0x41, 0x48], f"I2C SCAN parsed: {addrs}")

print("== Simulator DIAG & interlock commands")
check(any("#ERR" in r for r in sim.handle_command("ARM 1")), "ARM 1 rejected before DIAG 1")
check(any("#OK DIAG 1" in r for r in sim.handle_command("DIAG 1")), "DIAG 1 accepted")
check(any("#OK ARM 1" in r for r in sim.handle_command("ARM 1")), "ARM 1 accepted in DIAG")
check(any("#OK FIRE 2 250" in r for r in sim.handle_command("FIRE 2 250 CONFIRM")), "FIRE 2 250 CONFIRM accepted")
st_firing = rp.parse_status_line(sim.status_line())
check(st_firing is not None and (st_firing["fire_mask"] & 2) != 0, "CH2 fire_mask bit active during pulse")
check(any("#OK ABORT" in r for r in sim.handle_command("ABORT")), "ABORT accepted")
st_abort = rp.parse_status_line(sim.status_line())
check(st_abort is not None and st_abort["state"] == 6 and not st_abort["diag"] and not st_abort["pyro_arm"], "ABORT exits DIAG and disarms")
check(any("#OK SAFE" in r for r in sim.handle_command("SAFE")), "SAFE accepted")
st_safe = rp.parse_status_line(sim.status_line())
check(st_safe is not None and st_safe["state"] == 1, "SAFE returns to IDLE")

checks = rp.evaluate_checks(st_safe)
check(any(lvl == "ok" for lvl, _ in checks), "Go/No-Go checklist produces items")
bad_state = dict(st_safe, vbat_mv=6000, btn=1, key=0, hw_mask=0x00, faults=0x63)
bad_checks = rp.evaluate_checks(bad_state)
check(sum(1 for lvl, _ in bad_checks if lvl == "bad") >= 3, "Go/No-Go flags low VBAT, ABORT btn, and missing I2C HW")

try:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt6 import QtWidgets
    import voltino_style as vs
    import rampa_diag as rd
except ImportError:
    print("== PyQt6 not installed in this environment -> skipped offscreen Qt widget pass")
else:
    print("== Offscreen PyQt6 GUI smoke test (demo mode)")
    app = QtWidgets.QApplication(sys.argv)
    vs.setup_app(app, "VoltinoLabs.Rampa.Diagnostika")
    QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.StandardButton.Yes)

    def spin(sec: float) -> None:
        end = time.time() + sec
        while time.time() < end:
            app.processEvents()
            time.sleep(0.01)

    w = rd.RampaDiagWindow(demo=True)
    w.show()
    spin(0.6)
    check(bool(w.s) and w.s.get("state") == 1, "GUI connected to demo and received IDLE status")
    check("Online" in w.chip_board.text(), "header board chip shows Online")
    w._cmd("I2C SCAN")
    spin(0.2)
    check(w.i2c_addrs == [0x20, 0x40, 0x41, 0x48], "GUI parsed I2C SCAN result")
    w.diag_btn.setChecked(True)
    w.toggle_diag()
    spin(0.3)
    check(bool(w.s.get("diag")), "GUI entered DIAG mode")
    w.arm_clicked()
    spin(0.3)
    check(bool(w.s.get("diag_arm")), "GUI armed pyro rail")
    w.fire_channel(1)
    spin(0.15)
    check(bool(w.s.get("fire_mask", 0) & 1), "GUI fired CH1 pulse")
    w.disarm()
    spin(0.3)
    check(not w.s.get("diag_arm"), "GUI disarmed pyro rail")
    w.close()

print("\n%s (%d failures)" % ("GUI SMOKE PASSED" if not fails else "GUI SMOKE FAILED", fails))
sys.exit(1 if fails else 0)
