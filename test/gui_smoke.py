#!/usr/bin/env python3
"""Offscreen smoke test of the rampa diagnostic GUI against the simulated rampa,
plus the record decoder against a real firmware dump (made by `make -C test`).

    QT_QPA_PLATFORM=offscreen python3 test/gui_smoke.py [screenshot_dir]
"""
import os
import sys
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["RAMPA_DATA"] = tempfile.mkdtemp()
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gui"))

from PyQt6 import QtWidgets  # noqa: E402
import voltino_style as vs  # noqa: E402
import rampa_record as rr  # noqa: E402
import rampa_diag as rd  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else None
fails = 0


def check(cond, msg):
    global fails
    print(("   ok:   " if cond else "   FAIL: ") + msg)
    if not cond:
        fails += 1


def wait_for(app, cond, timeout):
    end = time.time() + timeout
    while time.time() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.004)
    return False


print("== record decoder vs. the firmware's own dump")
dump = os.path.join(HERE, "build", "dump_test.bin")
if os.path.exists(dump):
    rec = rr.decode(open(dump, "rb").read())
    st = rr.stats(rec)
    check(rec.closed and rec.sps == 2000 and rec.calibrated, f"record #{rec.test_no}: closed, 2000 S/s, calibrated")
    check(abs(st["peak"] - 100) < 1 and abs(st["impulse"] - 200) < 3 and abs(st["burn"] - 2.0) < 0.05,
          f"full-rate stats: peak {st['peak']:.2f} N, impulse {st['impulse']:.2f} N.s, burn {st['burn']:.3f} s")
    check(abs(st["pmax"] - 300) < 3 and abs(st["ign_delay"] - 0.15) < 0.01, f"pmax {st['pmax']:.1f} psi, delay {st['ign_delay']:.3f} s")
else:
    print("   skip: run `make -C test` first for build/dump_test.bin")
check(rr.motor_class(440, 176) == "I176", "motor class I176")

app = QtWidgets.QApplication(sys.argv)
app.setStyleSheet(vs.QSS)
QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.StandardButton.Yes)

print("== diagnostic window against the simulated rampa")
w = rd.RampaDiag(None, True)
w.resize(1536, 900)
w.show()
check(wait_for(app, lambda: bool(w.s) and bool(w.m), 3), "$S and $M parsed")
check(wait_for(app, lambda: "lc_scale" in w.cfg, 2), "configuration read ($CFG)")
check(w.cfg_tab.rowCount() == 21, f"parameter table has 21 rows ({w.cfg_tab.rowCount()})")
check(wait_for(app, lambda: w.rec_tab.rowCount() == 1, 3), "LIST: 1 record")
check(wait_for(app, lambda: "ADS1220: 2000" in w.chips["ads"].text(), 2), "chip " + w.chips["ads"].text())

w.reader.send("LOAD 0")
w.tare()
check(wait_for(app, lambda: "#OK TARE" in w.log.toPlainText(), 2), "TARE")
w.reader.send("LOAD 49.03325")                # 5 kg on the load cell
w.kg_sp.setValue(5.0)
w.send(f"CAL {5.0 * 9.80665:.4f}")
check(wait_for(app, lambda: rd.to_f(w.cfg.get("lc_scale"), 0) > 0, 2), f"CAL with 5 kg -> scale {w.cfg.get('lc_scale')}")
check(wait_for(app, lambda: abs(rd.to_f(w.m.get("f")) - 49.03) < 0.5, 2), f"force now {w.m.get('f')} N")

w.p_full.setValue(1000)
w.write_pressure()
check(wait_for(app, lambda: w.cfg.get("p_full_psi") in ("1000.0", "1000"), 3), "pressure range written (3 SETs in a row)")

w.send("FIRE 5")
check(wait_for(app, lambda: "only from the ground station" in w.log.toPlainText(), 2), "FIRE over USB refused by the rampa")

print("== record download at full rate")
w.rec_tab.selectRow(0)
w.download()
check(wait_for(app, lambda: w.rec is not None, 20), "record downloaded and decoded")
if w.rec:
    st = rr.stats(w.rec)
    check(len(w.rec.t) == 16000 and abs(st["peak"] - 230) < 3, f"{len(w.rec.t)} frames, peak {st['peak']:.1f} N")
    check("třída" in w.rec_info.text() and "I" in w.rec_info.text(), "statistics and motor class shown")
files = os.listdir(os.environ["RAMPA_DATA"])
check(any(f.endswith(".csv") for f in files) and any(f.endswith(".bin") for f in files), f"CSV + BIN saved ({files})")
if OUT:
    w.tabs.setCurrentIndex(4)
    wait_for(app, lambda: False, 0.3)
    w.grab().save(os.path.join(OUT, "rampa_records.png"))
    for i, n in ((0, "overview"), (1, "measure"), (2, "calib"), (3, "servos")):
        w.tabs.setCurrentIndex(i)
        wait_for(app, lambda: False, 0.3)
        w.grab().save(os.path.join(OUT, f"rampa_{n}.png"))

print("== servos and the arming switch")
check(wait_for(app, lambda: "J12" in w.sv_rows[0][1].text() or "drží" in w.sv_rows[0][1].text(), 2), "servo 1 row: " + w.sv_rows[0][1].text())
check(w.as_cb.currentIndex() == 1, "arming servo 1 read from $CFG")
w._servo(1, 1300)
check(wait_for(app, lambda: rd.to_i(w.s.get("sv2")) == 1300, 2), "servo 2 moved to 1300 us")
w.send("SERVO 2 OFF")
check(wait_for(app, lambda: rd.to_i(w.s.get("sv2")) == 0, 2), "servo 2 released")
w.send("ARMTEST")
check(wait_for(app, lambda: "#OK ARMTEST key closed" in w.log.toPlainText(), 3), "ARMTEST ok")
check(wait_for(app, lambda: "SEPNUT" in w.as_vals["key"].text(), 2), "key shown closed during the test")
w.as_settle.setValue(2000)
w.write_arm_servo()
check(wait_for(app, lambda: w.cfg.get("arm_settle_ms") == "2000" and not w._q, 3), "arming servo settings written")

print("== cold recording")
w.send("REC 1")
check(wait_for(app, lambda: bool(w.res), 4), "result of the cold recording")
check(wait_for(app, lambda: w.rec_tab.rowCount() == 2, 3), "second record listed")
w.close()
print("\n%s (%d failures)" % ("RAMPA GUI SMOKE PASSED" if not fails else "RAMPA GUI SMOKE FAILED", fails))
sys.exit(1 if fails else 0)
