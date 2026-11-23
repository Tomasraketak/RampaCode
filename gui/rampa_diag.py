#!/usr/bin/env python3
"""
Rampa — DIAGNOSTIC CONSOLE (Rampa PCB1 on a USB cable)

Plug the rampa's Raspberry Pi Pico 2 into the laptop. The window shows the
whole pad: power, key, igniter continuity, the rocket on the pogo (Zenith /
Daidalos) or the thrust-stand state, LoRa, and the measurement core (ADS1220,
W25Q128); live thrust, chamber pressure and temperatures; calibration of the
load cell, the pressure transducer and the thermistors; and the records of
motor tests on the rampa's flash — download at the full rate (2000 S/s),
statistics, CSV, plot.

Same design system and layout idea as the Daidalos console (VoltinoLabs:
dark ground, amber accent, cards with a kicker, outlined chips).

A motor is NEVER fired from here: ARM / FIRE exist only on the ground
station (LoRa, safe distance). Over USB the rampa refuses them.

    pip install -r requirements.txt
    python rampa_diag.py                 # pick the port in the window
    python rampa_diag.py --port COM5
    python rampa_diag.py --demo          # no hardware: a simulated rampa

Protocol: docs/USB_PROTOCOL.md.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import struct
import sys
import time
from collections import deque

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import voltino_style as vs  # noqa: E402
from voltino_style import Chip, Dot, Card, C, cz, label, button  # noqa: E402
import rampa_record as rr  # noqa: E402

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

APP = "Rampa · Diagnostika"
S_FIELDS = ["proto", "rstate", "sstate", "sflags", "sfaults", "vbat", "umb_ma", "v5", "c1", "c2", "c3", "rfaults",
            "key", "btn", "lora", "stand", "ads", "flash", "sps", "test_no", "free_kb", "total_kb", "ndir", "rec",
            "progress", "t_rel", "ch", "fire_ms", "rec_s", "link_ms", "c1ok", "jedec", "rssi", "dropped", "fw",
            "sv1", "sv2", "sv3", "sv4", "arm_servo", "as", "as_ms"]
AS_NAMES = {0: "SAFE – spínač rozepnut", 1: "ARMING – servo se točí, čeká na klíč", 2: "ARMED – spínač sepnut",
            3: "SELHALO – klíč nesepnul, servo zpět", 4: "ZKOUŠKA polohy"}
SERVO_PINS = ["J12.1 · GP27", "J12.4 · GP28", "J12.7 · GP14", "J12.10 · GP22"]
M_FIELDS = ["t_ms", "f", "fmax", "p", "pv", "t1", "t2", "uvv"]
R_FIELDS = ["test_no", "flags", "ch", "peak", "impulse", "burn", "avg", "t_peak", "ign_delay", "pmax", "t1max",
            "t2max", "curve_n", "curve_hz", "n_frames", "sps", "t0_idx", "hang"]
ROCKET_STATES = ["OFFLINE", "IDLE", "READY", "COUNTDOWN", "COMMITTED", "FIRED", "ABORT", "FAULT"]
STAND_CZ = {0: "SAFE · zajištěno", 1: "ARMING · maže flash", 2: "ARMED · odjištěno", 3: "COUNTDOWN · odpočet",
            4: "FIRING · zážeh", 5: "BURN · záznam", 6: "ABORT · přerušeno", 7: "RECORD · zkušební záznam"}
SL_FAULTS = ["nízká baterie", "klíč vypnut", "rozbuška bez kontinuity", "ztráta spojení", "ADS1220", "flash",
             "tlačítko ABORT", "měřicí jádro / I2C"]
ROCKET_FAULTS = ["nízká baterie", "klíč vypnut", "rozbuška bez kontinuity", "timeout rakety", "raketa NO-GO",
                 "HW rampy", "tlačítko ABORT", "chyba časování"]
CONT_MIN_MV = 350
REC_STATES = ["nečinný", "maže flash", "připraven", "ZAZNAMENÁVÁ", "chyba"]
CFG_HELP = {
    "lc_offset": "TARE: kód ADS1220 bez zatížení",
    "lc_scale": "N na kód ADS1220 (0 = nekalibrováno, síla v µV/V)",
    "p_zero": "výstup čidla / napájení při 0 psi (0,5 V z 5 V = 0,100)",
    "p_span": "rozsah výstupu / napájení (4,0 V z 5 V = 0,800)",
    "p_full_psi": "tlak při plném výstupu (4,5 V)",
    "adc_vref": "reference ADC Pica (3,30 V)",
    "p_div": "dělič R46/R47 = (4k7 + 10k) / 10k",
    "th_r0": "odpor NTC při T0 [Ω]", "th_beta": "konstanta B termistoru", "th_t0_c": "T0 [°C]",
    "th_rfix": "pevný odpor k zemi R48/R49 [Ω]",
    "ign_thr_n": "minimální tah, který se počítá jako hoření",
    "fire_ms": "výchozí délka pulzu do rozbušky [ms]", "rec_s": "záznam po T-0 [s]",
    "pyro_ch": "výchozí rozbuška 1–3", "rate_2k": "1 = 2000 S/s (turbo), 0 = 1000 S/s",
    "cd_s": "výchozí odpočet [s] (nápověda pro stanici)",
    "arm_servo": "servo bezpečnostního spínače 1–4, 0 = jen klíč",
    "arm_us": "puls serva: spínač SEPNUT [µs]", "safe_us": "puls serva: spínač ROZEPNUT [µs]",
    "arm_settle_ms": "do kdy po natočení musí klíč sepnout [ms]",
}


def to_f(s, d=float("nan")):
    try:
        return float(s)
    except (TypeError, ValueError):
        return d


def to_i(s, d=0):
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return d


def bits(v: int, names: list[str]) -> list[str]:
    return [n for i, n in enumerate(names) if v & (1 << i)]


def f_or(v, fmt: str, dash: str = "—") -> str:
    return dash if v is None or v != v else fmt.format(v).replace(".", ",").replace("-", "−")


# =====================================================================
#  Serial
# =====================================================================
class SerialReader(QtCore.QThread):
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self, port: str):
        super().__init__()
        self.port = port
        self._run = True
        self._tx: list[str] = []
        self._lock = QtCore.QMutex()

    def send(self, text: str):
        with QtCore.QMutexLocker(self._lock):
            self._tx.append(text)

    def stop(self):
        self._run = False
        self.wait(1500)

    def run(self):
        if serial is None:
            self.status.emit("chybí pyserial (pip install pyserial)")
            return
        while self._run:
            ser = None
            try:
                ser = serial.Serial(self.port, 115200, timeout=0.03)
                ser.dtr = True
                self.status.emit(f"připojeno {self.port}")
                buf = b""
                while self._run:
                    with QtCore.QMutexLocker(self._lock):
                        tx, self._tx = self._tx, []
                    for t in tx:
                        ser.write((t + "\n").encode())
                    chunk = ser.read(16384)
                    if not chunk:
                        continue
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for raw in lines:
                        s = raw.decode("ascii", errors="ignore").strip()
                        if s:
                            self.line.emit(s)
                    if len(buf) > 65536:
                        buf = b""
            except Exception as ex:                       # unplugged, busy, wrong port
                self.status.emit(f"odpojeno: {ex}")
                for _ in range(20):
                    if not self._run:
                        break
                    time.sleep(0.05)
            finally:
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass


def list_ports() -> list[str]:
    if serial is None:
        return []
    ports = list(serial.tools.list_ports.comports())
    ports.sort(key=lambda p: (0 if p.vid == 0x2E8A else 1, p.device))
    return [p.device for p in ports]


# =====================================================================
#  Demo rampa (speaks the USB protocol of Rampa_MCU.ino)
# =====================================================================
class DemoRampa(QtCore.QObject):
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.t0 = time.time()
        self.cfg = dict(lc_offset=0, lc_scale=0.0, p_zero=0.1, p_span=0.8, p_full_psi=500.0, adc_vref=3.3, p_div=1.47,
                        th_r0=10000.0, th_beta=3950.0, th_t0_c=25.0, th_rfix=10000.0, ign_thr_n=2.0, fire_ms=1000,
                        rec_s=20, pyro_ch=1, rate_2k=1, cd_s=10, arm_servo=1, arm_us=2000, safe_us=1000,
                        arm_settle_ms=1500)
        self.sv = [1000, 0, 0, 0]
        self.as_state, self.as_t, self.as_ms = 0, 0.0, 0
        self.code_off, self.code_per_n = 21345.0, 850.0
        self.load_n = 0.0
        self.stand, self.sstate, self.rec_end = False, 0, 0.0
        self.records: list[bytes] = [self._make_record(1, cold=False)]
        self.dump = None
        self.n = 0
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    def stop(self):
        self.timer.stop()

    def now(self):
        return time.time() - self.t0

    # ---- a synthetic record in the real flash format
    def _make_record(self, test: int, cold: bool) -> bytes:
        cfg = dict(self.cfg)
        cfg["lc_offset"] = int(self.code_off)
        cfg["lc_scale"] = 1.0 / self.code_per_n
        vals = [0x31464352, 2, 3, cfg["lc_offset"], cfg["lc_scale"], cfg["p_zero"], cfg["p_span"], cfg["p_full_psi"],
                cfg["adc_vref"], cfg["p_div"], cfg["th_r0"], cfg["th_beta"], cfg["th_t0_c"], cfg["th_rfix"],
                cfg["ign_thr_n"], cfg["fire_ms"], cfg["rec_s"], cfg["pyro_ch"], cfg["rate_2k"], cfg["cd_s"],
                cfg["arm_servo"], cfg["arm_us"], cfg["safe_us"], cfg["arm_settle_ms"], 0]
        cfgb = rr.CFG.pack(*vals)
        sps, n_frames = 2000, 2000 * 8
        hdr = rr.HDR.pack(rr.HDR_MAGIC, 2, test, 4, sps, int(cold), 1, 1000, 5, 3, 4990) + cfgb
        hdr += b"\xff" * (126 - len(hdr))
        hdr += struct.pack("<H", rr.crc16(hdr))
        frames = bytearray()
        t0f = -1 if cold else 6000
        ntc = lambda tc: int(10000.0 / (10000.0 * math.exp(3950 * (1 / (tc + 273.15) - 1 / 298.15)) + 10000.0) * 65535)
        for i in range(n_frames):
            tb = (i - 6000) / sps
            fN = 0.0
            if not cold and 0.22 < tb < 2.9:
                x = tb - 0.22
                fN = 230 * x / 0.06 if x < 0.06 else 230 - 60 * (x - 0.06) / 2.4 if x < 2.5 else 170 * (1 - (x - 2.5) / 0.18)
                fN = max(0.0, fN)
            fN += random.gauss(0, 0.2)
            psi = fN * 1.7
            code = int(cfg["lc_offset"] + fN * self.code_per_n)
            pv = 5.0 * (0.1 + 0.8 * psi / 500)
            praw = int(pv / 1.47 / 3.3 * 4095 * 16)
            temp = 23 + (max(0.0, tb) * 9 if not cold else 0)
            flags = (1 if (not cold and 0 <= i - 6000 < 2000) else 0) | ((5 if not cold and i >= 6000 else 3) << 4)
            fr = struct.pack("<IiHHHB", int(i * 500), code, min(65535, praw), ntc(temp), ntc(24.0), flags)
            frames += fr + bytes([rr.crc8(fr)])
        sb = rr.END.pack(rr.END_MAGIC, n_frames, t0f, 0x13 | (0x20 if cold else 0) | 0x40, 0, 4990,
                         230.0, 480.0, 2.6, 184.0, 0.28, 0.22, 391.0, 470, 240, 0)
        sb += b"\xff" * (126 - len(sb))
        sb += struct.pack("<H", rr.crc16(sb))
        return hdr + sb + bytes(frames)

    # ---- commands
    def send(self, text: str):
        p = text.split()
        if not p:
            return
        c, a = p[0].upper(), p[1:]
        cfgline = lambda: [self.line.emit(f"$CFG,{k},{v}") for k, v in self.cfg.items()] + [self.line.emit("#OK CFG")]
        if c == "HELLO":
            self.line.emit("$ID,Rampa,3.00,PCB1,W25Q128 (DEMO)")
            cfgline()
        elif c == "CFG":
            cfgline()
        elif c == "TARE":
            self.cfg["lc_offset"] = int(self.code_off + self.load_n * self.code_per_n)
            self.line.emit("#OK TARE")
            cfgline()
        elif c == "CAL" and a:
            d = self.load_n * self.code_per_n
            if abs(d) < 2000:
                self.line.emit("#ERR CAL failed (load cell / weight / transducer / flash?)")
            else:
                self.cfg["lc_scale"] = float(a[0]) / (self.code_off + d - self.cfg["lc_offset"])
                self.line.emit("#OK CAL")
                cfgline()
        elif c == "PZERO":
            self.line.emit("#OK PZERO")
            cfgline()
        elif c == "LCRATED" and len(a) >= 2:
            self.cfg["lc_scale"] = float(a[0]) * 9.80665 / (float(a[1]) * 1e-3 * 128 * 8388608)
            self.line.emit("#OK LCRATED")
            cfgline()
        elif c == "SET" and len(a) >= 2 and a[0] in self.cfg:
            self.cfg[a[0]] = type(self.cfg[a[0]])(float(a[1])) if not isinstance(self.cfg[a[0]], int) else int(float(a[1]))
            self.line.emit("#OK SET")
            cfgline()
        elif c == "DEFAULTS":
            self.line.emit("#OK DEFAULTS")
            cfgline()
        elif c == "LIST":
            for r in self.records:
                h = rr.HDR.unpack_from(r, 0)
                e = rr.END.unpack_from(r, 128)
                self.line.emit(f"$REC,{h[2]},{65536 * (h[2] + 1)},{h[3]},{h[4]},1,{e[3]},{e[1]},{e[6]:.3f},{e[7]:.4f},{e[8]:.3f}")
            self.line.emit(f"#OK LIST {len(self.records)}")
        elif c == "DUMP" and a:
            for r in self.records:
                if rr.HDR.unpack_from(r, 0)[2] == to_i(a[0]):
                    n = (len(r) + 255) // 256 * 256
                    self.dump = [r + b"\xff" * (n - len(r)), 0, to_i(a[0])]
                    self.line.emit(f"$DBEGIN,{a[0]},65536,{n}")
                    return
            self.line.emit("#ERR DUMP <test number> (see LIST)")
        elif c == "REC":
            self.stand, self.sstate, self.rec_end = True, 7, self.now() + to_f(a[0] if a else 5, 5)
            self.line.emit("#OK REC")
        elif c == "STOP":
            self.rec_end = self.now()
            self.line.emit("#OK STOP")
        elif c in ("FIRE", "ARM"):
            self.line.emit(f"#ERR {c} only from the ground station (LoRa, safe distance)")
        elif c == "ERASEALL":
            if a and a[0] == "YES":
                self.records = []
                self.line.emit("#OK ERASEALL")
            else:
                self.line.emit("#ERR ERASEALL YES  (deletes every record)")
        elif c == "SERVO" and len(a) >= 2:
            n = to_i(a[0])
            if not 1 <= n <= 4:
                self.line.emit("#ERR SERVO <1-4> <500-2500 us | OFF>")
            elif a[1].upper() == "OFF":
                if n == self.cfg["arm_servo"]:
                    self.line.emit(f"#ERR SERVO {n}: the arming servo stays driven (SAFE position)")
                else:
                    self.sv[n - 1] = 0
                    self.line.emit(f"#OK SERVO {n} OFF")
            else:
                self.sv[n - 1] = to_i(a[1])
                arm = n == self.cfg["arm_servo"] and self.sv[n - 1] != self.cfg["safe_us"]
                if arm:
                    self.as_state, self.as_t = 4, self.now() + 10
                self.line.emit(f"#OK SERVO {n} {a[1]}" + (" (arming servo: back to SAFE in 10 s)" if arm else ""))
        elif c == "ARMTEST":
            if not self.cfg["arm_servo"]:
                self.line.emit("#ERR ARMTEST: no arming servo configured (SET arm_servo 1-4)")
            else:
                self.sv[self.cfg["arm_servo"] - 1] = self.cfg["arm_us"]
                self.as_state, self.as_t = 1, self.now()
                self.line.emit("#OK ARMTEST started (servo to arm_us, waiting for the key sense)")
        elif c == "LOAD":                                # demo only: put a weight on the load cell
            self.load_n = to_f(a[0] if a else 0, 0)
        else:
            self.line.emit(f"#OK {text}")

    def tick(self):
        self.n += 1
        t = self.now()
        if self.dump:
            img, off, test = self.dump
            for _ in range(24):
                if off >= len(img):
                    self.line.emit(f"$DEND,{test},{off}")
                    self.dump = None
                    break
                self.line.emit(f"$D,{off},{img[off:off + 256].hex().upper()}")
                off += 256
            if self.dump:
                self.dump[1] = off
            return
        if self.sstate == 7 and t >= self.rec_end:
            self.sstate = 0
            k = len(self.records) + 1
            self.records.append(self._make_record(k, cold=True))
            self.line.emit(f"$R,{k},{0x30 | 0x40 | 0x10 | 8},1,{self.load_n:.3f},0,0,0,0,0,0.5,23.0,24.0,500,100,10000,2000,-1,0")
        if self.as_state == 1 and t - self.as_t > 0.6:
            self.as_state, self.as_ms, self.as_t = 4, 600, t + 3
            self.line.emit("#OK ARMTEST key closed after 600 ms, servo back to SAFE in 3 s")
        if self.as_state == 4 and t > self.as_t:
            self.as_state = 0
            if self.cfg["arm_servo"]:
                self.sv[self.cfg["arm_servo"] - 1] = self.cfg["safe_us"]
        key = 1 if (not self.cfg["arm_servo"] or self.sv[self.cfg["arm_servo"] - 1] >= 1800) else 0
        cal = self.cfg["lc_scale"]
        code = self.code_off + self.load_n * self.code_per_n + random.gauss(0, 6)
        d = code - self.cfg["lc_offset"]
        f = d * cal if cal else d * rr.UVV_PER_CODE
        uvv = code * rr.UVV_PER_CODE
        p = 0.4 * math.sin(t * 0.7) + random.gauss(0, 0.3)
        t1 = 23.4 + 0.3 * math.sin(t * 0.05)
        self.line.emit(f"$M,{int(t * 1000)},{f:.3f},{f + 0.1:.3f},{p:.2f},{0.5 + p / 125:.4f},{t1:.2f},nan,{uvv:.3f}")
        if self.n % 10 == 0:
            st = self.sstate if self.stand else -1
            flags = 0x01 | 0x02 | 0x20 | 0x40 | (0x80 if cal else 0) | 0x100 | 0x400 | (0x10 if self.sstate == 7 else 0)
            free = 16320 - 512 * len(self.records)
            flags = (flags & ~1) | key
            self.line.emit(f"$S,NONE,1,{st},{flags},0,8030,0,4992,1210,30,28,0,{key},0,1,{int(self.stand)},1,1,2000,"
                           f"{len(self.records)},{free},16320,{len(self.records)},{3 if self.sstate == 7 else 0},0,"
                           f"0,1,{self.cfg['fire_ms']},{self.cfg['rec_s']},-1,1,EF4018,-58,0,300,"
                           f"{self.sv[0]},{self.sv[1]},{self.sv[2]},{self.sv[3]},{self.cfg['arm_servo']},{self.as_state},{self.as_ms}")


# =====================================================================
#  Widgets
# =====================================================================
class VChart(QtWidgets.QWidget):
    """Rolling chart in the VoltinoLabs colours."""
    COLORS = [C["accent"], "#7cc7ff", C["positive"], "#c79bff"]

    def __init__(self, title: str, series: list[str], span: float = 30.0, unit: str = "", min_span: float = 1.0):
        super().__init__()
        self.title, self.series, self.span, self.unit, self.min_span = title, series, span, unit, min_span
        self.data = {s: deque(maxlen=20000) for s in series}
        self.setMinimumHeight(150)

    def add(self, t: float, **vals):
        for k, v in vals.items():
            if k in self.data and v is not None and v == v and math.isfinite(v):
                self.data[k].append((t, v))

    def clear(self):
        for d in self.data.values():
            d.clear()
        self.update()

    def paintEvent(self, _e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(QtGui.QColor(233, 233, 237, 25), 1))
        p.setBrush(QtGui.QColor(C["bg_raised"]))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 12, 12)
        l, r, t, b = 60, 12, 30, 22
        pw, ph = w - l - r, h - t - b
        f = p.font()
        f.setPixelSize(11)
        f.setBold(True)
        p.setFont(f)
        p.setPen(QtGui.QColor(C["accent_soft"]))
        p.drawText(14, 19, (self.title + (f"  [{self.unit}]" if self.unit else "")).upper())
        x = w - 14
        for i, s in enumerate(reversed(self.series)):
            col = QtGui.QColor(self.COLORS[(len(self.series) - 1 - i) % len(self.COLORS)])
            tw = p.fontMetrics().horizontalAdvance(s)
            p.setPen(col)
            p.drawText(x - tw, 19, s)
            x -= tw + 16
        now = max((d[-1][0] for d in self.data.values() if d), default=0.0)
        t0 = now - self.span
        vals = [v for d in self.data.values() for tt, v in d if tt >= t0]
        if not vals or pw < 20:
            p.setPen(QtGui.QColor(C["subtle"]))
            p.drawText(l + 8, t + ph // 2, "žádná data")
            return
        lo, hi = min(vals), max(vals)
        if hi - lo < self.min_span:
            mid = (hi + lo) / 2
            lo, hi = mid - self.min_span / 2, mid + self.min_span / 2
        pad = (hi - lo) * 0.08
        lo, hi = lo - pad, hi + pad
        f.setBold(False)
        p.setFont(f)
        for i in range(5):
            y = t + ph * i / 4
            p.setPen(QtGui.QPen(QtGui.QColor(233, 233, 237, 22), 1))
            p.drawLine(l, int(y), l + pw, int(y))
            p.setPen(QtGui.QColor(C["subtle"]))
            v = hi - (hi - lo) * i / 4
            p.drawText(4, int(y) + 4, cz(v, 2 if hi - lo < 5 else 1 if hi - lo < 200 else 0))
        for i in range(7):
            p.drawText(int(l + pw * i / 6) - 12, t + ph + 16, f"{-self.span + self.span * i / 6:.0f}s")
        for i, s in enumerate(self.series):
            path = QtGui.QPainterPath()
            first = True
            for tt, v in self.data[s]:
                if tt < t0:
                    continue
                pt = QtCore.QPointF(l + (tt - t0) / self.span * pw, t + ph - (v - lo) / (hi - lo) * ph)
                if first:
                    path.moveTo(pt)
                    first = False
                else:
                    path.lineTo(pt)
            p.setPen(QtGui.QPen(QtGui.QColor(self.COLORS[i % len(self.COLORS)]), 1.6))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)


class RecordPlot(QtWidgets.QWidget):
    """A downloaded record: thrust and pressure vs time (decimated for drawing)."""

    def __init__(self):
        super().__init__()
        self.rec: rr.Record | None = None
        self.st: dict = {}
        self.setMinimumHeight(260)

    def set(self, rec, st):
        self.rec, self.st = rec, st
        self.update()

    def paintEvent(self, _e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(QtGui.QPen(QtGui.QColor(233, 233, 237, 25), 1))
        p.setBrush(QtGui.QColor(C["bg_raised"]))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 12, 12)
        l, r, t, b = 64, 64, 34, 26
        pw, ph = w - l - r, h - t - b
        f = p.font()
        f.setPixelSize(11)
        p.setFont(f)
        rec = self.rec
        if not rec or len(rec.t) < 2 or pw < 30:
            p.setPen(QtGui.QColor(C["subtle"]))
            p.drawText(l, t + ph // 2, "Vyber záznam a stáhni ho (plná frekvence z flash rampy).")
            return
        step = max(1, len(rec.t) // (pw * 2))
        idx = range(0, len(rec.t), step)
        T = [rec.t[i] for i in idx]
        F = [max(rec.f[i:i + step]) if rec.f[i] == rec.f[i] else float("nan") for i in idx]
        P = [rec.p[i] for i in idx]
        t0, t1 = T[0], T[-1]
        fv = [v for v in F if v == v] or [0.0]
        pvv = [v for v in P if v == v] or [0.0]
        flo, fhi = min(0.0, min(fv)), max(1.0, max(fv)) * 1.08
        plo, phi = min(0.0, min(pvv)), max(1.0, max(pvv)) * 1.08
        X = lambda tt: l + (tt - t0) / ((t1 - t0) or 1) * pw
        if self.st.get("ignition"):
            a = self.st["ign_delay"] if rec.t0_index >= 0 else 0
            p.fillRect(QtCore.QRectF(X(a), t, X(a + self.st["burn"]) - X(a), ph), QtGui.QColor(245, 165, 28, 28))
        for i in range(5):
            y = t + ph * i / 4
            p.setPen(QtGui.QPen(QtGui.QColor(233, 233, 237, 22), 1))
            p.drawLine(l, int(y), l + pw, int(y))
            p.setPen(QtGui.QColor(C["accent"]))
            p.drawText(6, int(y) + 4, cz(fhi - (fhi - flo) * i / 4, 0))
            p.setPen(QtGui.QColor("#7cc7ff"))
            p.drawText(l + pw + 6, int(y) + 4, cz(phi - (phi - plo) * i / 4, 0))
        p.setPen(QtGui.QColor(C["subtle"]))
        for i in range(9):
            tt = t0 + (t1 - t0) * i / 8
            p.drawText(int(X(tt)) - 14, t + ph + 18, f"{tt:+.1f}s")
        if t0 < 0 < t1:
            p.setPen(QtGui.QPen(QtGui.QColor(C["danger"]), 1, Qt.PenStyle.DashLine))
            p.drawLine(int(X(0)), t, int(X(0)), t + ph)
            p.drawText(int(X(0)) + 4, t + 12, "T-0")
        for arr, lo, hi, col in ((P, plo, phi, "#7cc7ff"), (F, flo, fhi, C["accent"])):
            path = QtGui.QPainterPath()
            first = True
            for tt, v in zip(T, arr):
                if v != v:
                    continue
                pt = QtCore.QPointF(X(tt), t + ph - (v - lo) / (hi - lo) * ph)
                path.moveTo(pt) if first else path.lineTo(pt)
                first = False
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1.6))
            p.drawPath(path)
        f.setBold(True)
        p.setFont(f)
        p.setPen(QtGui.QColor(C["accent"]))
        p.drawText(l, 20, f"TAH [{rec.unit}]")
        p.setPen(QtGui.QColor("#7cc7ff"))
        p.drawText(l + pw - 70, 20, "TLAK [psi]")


def kv_grid(rows: list[tuple[str, str]]) -> tuple[QtWidgets.QGridLayout, dict]:
    g = QtWidgets.QGridLayout()
    g.setHorizontalSpacing(14)
    g.setVerticalSpacing(5)
    vals = {}
    for i, (key, name) in enumerate(rows):
        g.addWidget(label(name, "muted"), i, 0)
        v = label("—", "value")
        v.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        v.setWordWrap(True)
        g.addWidget(v, i, 1)
        vals[key] = v
    g.setColumnStretch(1, 1)
    return g, vals


# =====================================================================
#  Main window
# =====================================================================
class RampaDiag(QtWidgets.QMainWindow):
    def __init__(self, port: str | None = None, demo: bool = False):
        super().__init__()
        self.setWindowTitle(APP)
        self.resize(1480, 940)
        self.reader = None
        self.s: dict = {}
        self.s_rx = 0.0
        self.m: dict = {}
        self.res: dict = {}
        self.cfg: dict = {}
        self.ident = ""
        self.t0 = time.time()
        self.recs: list[list[str]] = []
        self.dump = None              # [test, bytearray, expected length]
        self.rec: rr.Record | None = None
        self.data_dir = self._data_dir()
        self._q: list[str] = []          # SET commands waiting for the previous #OK
        self._build()
        QtCore.QTimer(self, timeout=self.refresh, interval=150).start()
        if demo:
            self.start_demo()
        elif port:
            self.connect_port(port)

    @staticmethod
    def _data_dir() -> str:
        base = os.environ.get("RAMPA_DATA", "")
        if not base:
            docs = QtCore.QStandardPaths.writableLocation(QtCore.QStandardPaths.StandardLocation.DocumentsLocation) \
                or os.path.expanduser("~")
            base = os.path.join(docs, "Rampa")
        os.makedirs(base, exist_ok=True)
        return base

    # ------------------------------------------------------------------ ui
    def _build(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        v = QtWidgets.QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        head, hl = vs.header("RAMPA PCB1 · STARTOVACÍ RAMPA · TEST MOTORU", "Rampa ", "Console")
        v.addWidget(head)
        self.port_cb = QtWidgets.QComboBox()
        self.port_cb.setMinimumWidth(220)
        self.port_cb.addItems(list_ports())
        b_ref = button("Obnovit", tip="Znovu načíst sériové porty")
        b_ref.clicked.connect(self.refresh_ports)
        self.con_btn = button("Připojit", "primary", "Připojit / odpojit rampu (USB)")
        self.con_btn.clicked.connect(self.toggle_connection)
        b_demo = button("Simulace", tip="Simulovaná rampa – bez hardwaru")
        b_demo.clicked.connect(self.start_demo)
        self.conn_chip = Chip("nepřipojeno")
        for w_ in (self.port_cb, b_ref, self.con_btn, b_demo, self.conn_chip):
            hl.addWidget(w_)

        # chips row
        bar = QtWidgets.QFrame()
        bar.setObjectName("header")
        bl = QtWidgets.QHBoxLayout(bar)
        bl.setContentsMargins(20, 8, 20, 8)
        bl.setSpacing(8)
        self.chips = {}
        for k, txt in (("fw", "Deska: —"), ("mode", "Režim: —"), ("state", "Stav: —"), ("bat", "Baterie: —"),
                       ("key", "Klíč: —"), ("cont", "Rozbušky: —"), ("ads", "ADS1220: —"), ("flash", "Flash: —"),
                       ("lora", "LoRa: —")):
            c = Chip(txt)
            self.chips[k] = c
            bl.addWidget(c)
        bl.addStretch(1)
        self.tare_btn = button("TARE", "primary", "Vynulovat tenzometr (bez zatížení)")
        self.tare_btn.clicked.connect(self.tare)
        self.buz_btn = button("Bzučák", tip="Bzučák rampy zap/vyp")
        self.buz_btn.setCheckable(True)
        self.buz_btn.toggled.connect(lambda on: self.send(f"BUZZER {1 if on else 0}"))
        self.safe_btn = button("Zajistit", tip="Rampa (test motoru) zpět do SAFE, zruší zablokovaný ABORT")
        self.safe_btn.clicked.connect(lambda: self.send("DISARM"))
        self.abort_btn = button("ABORT", "danger", "Okamžitě přeruší odpočet / vypne pyro")
        self.abort_btn.clicked.connect(lambda: self.send("ABORT"))
        for b in (self.tare_btn, self.buz_btn, self.safe_btn, self.abort_btn):
            bl.addWidget(b)
        v.addWidget(bar)

        self.tabs = QtWidgets.QTabWidget()
        v.addWidget(self.tabs, 1)
        self._tab_overview()
        self._tab_measure()
        self._tab_calib()
        self._tab_servos()
        self._tab_records()
        self._tab_console()
        self.status = self.statusBar()
        QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Shift+A"), self).activated.connect(lambda: self.send("ABORT"))

    def _scroll(self, inner: QtWidgets.QWidget, title: str):
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        sa.setWidget(inner)
        self.tabs.addTab(sa, title)

    def _tab_overview(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 20, 20, 20)
        g.setSpacing(16)
        c1 = Card("2S LiPo · INA226 · LP2985", "Napájení")
        gl, self.ov_pow = kv_grid([("vbat", "Baterie"), ("v5", "Napájení čidel (5V_SENS)"), ("umb", "Proud do poga"),
                                   ("key", "Klíč J4"), ("btn", "Tlačítko SW1")])
        c1.body.setLayout(gl)
        c2 = Card("AO4407A · AO3400A · ADS1115", "Pyro výstupy")
        gl, self.ov_pyro = kv_grid([("c1", "Rozbuška 1 (J5)"), ("c2", "Rozbuška 2 (J6)"), ("c3", "Rozbuška 3 (J7)"),
                                    ("sel", "Zvolená / puls"), ("arm", "Spínač (servo)"), ("rfaults", "Blokování raketa"),
                                    ("sfaults", "Blokování test")])
        c2.body.setLayout(gl)
        c3 = Card("POGO J3 · RA-02 433 MHZ", "Raketa a spojení")
        gl, self.ov_link = kv_grid([("proto", "Na pogu"), ("rstate", "Stav rampy (raketa)"), ("stand", "Test motoru"),
                                    ("lora", "LoRa rampy"), ("link", "Pozemní stanice"), ("fw", "Firmware")])
        c3.body.setLayout(gl)
        c4 = Card("RP2350 CORE 1 · ADS1220 · W25Q128", "Měřicí jádro")
        gl, self.ov_meas = kv_grid([("core", "Jádro 1"), ("ads", "ADS1220"), ("flash", "Flash"), ("free", "Volno"),
                                    ("recs", "Záznamů"), ("rec", "Záznamník"), ("drop", "Ztracené vzorky")])
        c4.body.setLayout(gl)
        c5 = Card("POSLEDNÍ TEST", "Výsledek")
        self.ov_res = label("Zatím žádný výsledek.", "muted")
        self.ov_res.setWordWrap(True)
        QtWidgets.QVBoxLayout(c5.body).addWidget(self.ov_res)
        c6 = Card("PROČ NEJDE ZAPÁLIT", "Kontrola před testem")
        self.ov_checks = QtWidgets.QVBoxLayout(c6.body)
        self.ov_check_rows = []
        for _ in range(8):
            row = QtWidgets.QHBoxLayout()
            d = Dot()
            t = label("", "muted")
            row.addWidget(d)
            row.addWidget(t, 1)
            self.ov_checks.addLayout(row)
            self.ov_check_rows.append((d, t))
        note = label("Zážeh jen z pozemní stanice (LoRa, bezpečná vzdálenost). Přes USB rampa ARM ani FIRE nepřijme.", "subtle")
        note.setWordWrap(True)
        self.ov_checks.addWidget(note)
        g.addWidget(c1, 0, 0)
        g.addWidget(c2, 0, 1)
        g.addWidget(c3, 0, 2)
        g.addWidget(c4, 1, 0)
        g.addWidget(c6, 1, 1)
        g.addWidget(c5, 1, 2)
        for i in range(3):
            g.setColumnStretch(i, 1)
        g.setRowStretch(2, 1)
        self._scroll(w, "Přehled")

    def _tab_measure(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.setSpacing(16)
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(12)
        lay.addLayout(left, 1)
        self.big = {}
        for key, kick, title in (("f", "ADS1220 · TENZOMETR", "Tah"), ("p", "0,5–4,5 V · 0–500 PSI", "Tlak v komoře"),
                                 ("t", "NTC · ADS1220 AIN2/3", "Teploty")):
            c = Card(kick, title)
            vl = QtWidgets.QVBoxLayout(c.body)
            big = label("—", "title")
            big.setStyleSheet("font-size: 34px; font-weight: 600; font-family: 'JetBrains Mono', Consolas, monospace;")
            sub = label(" ", "muted")
            vl.addWidget(big)
            vl.addWidget(sub)
            self.big[key] = (big, sub)
            left.addWidget(c)
        rc = Card("ZÁPIS NA FLASH BEZ ZÁŽEHU", "Zkušební záznam")
        rl = QtWidgets.QHBoxLayout(rc.body)
        self.rec_sp = QtWidgets.QSpinBox()
        self.rec_sp.setRange(1, 120)
        self.rec_sp.setValue(10)
        self.rec_sp.setSuffix(" s")
        b_rec = button("Nahrávat", "primary", "Záznam všech čidel na flash při plné frekvenci, pyro zůstává vypnuté")
        b_rec.clicked.connect(lambda: self.send(f"REC {self.rec_sp.value()}"))
        b_stop = button("Stop", tip="Ukončit záznam")
        b_stop.clicked.connect(lambda: self.send("STOP"))
        for x in (self.rec_sp, b_rec, b_stop):
            rl.addWidget(x)
        left.addWidget(rc)
        left.addStretch(1)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(12)
        lay.addLayout(right, 3)
        self.ch_f = VChart("Tah", ["tah"], 30, "N", 2)
        self.ch_p = VChart("Tlak", ["tlak"], 30, "psi", 5)
        self.ch_t = VChart("Teploty", ["T1", "T2"], 120, "°C", 2)
        right.addWidget(self.ch_f, 3)
        right.addWidget(self.ch_p, 2)
        right.addWidget(self.ch_t, 2)
        self._scroll(w, "Měření")

    def _tab_calib(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 20, 20, 20)
        g.setSpacing(16)
        # load cell
        c1 = Card("ADS1220 · PGA 128 · RATIOMETRICKY", "Tenzometr")
        vl = QtWidgets.QVBoxLayout(c1.body)
        gl, self.cal_lc = kv_grid([("raw", "Signál"), ("off", "TARE (offset)"), ("scale", "Měřítko"), ("f", "Síla teď")])
        vl.addLayout(gl)
        vl.addWidget(label("1 · bez zatížení:", "muted"))
        b = button("TARE – vynulovat", "primary")
        b.clicked.connect(self.tare)
        vl.addWidget(b)
        vl.addWidget(label("2 · zatěž známým závažím ve směru tahu:", "muted"))
        row = QtWidgets.QHBoxLayout()
        self.kg_sp = QtWidgets.QDoubleSpinBox()
        self.kg_sp.setRange(0.01, 100000)
        self.kg_sp.setDecimals(3)
        self.kg_sp.setValue(5.0)
        self.kg_sp.setSuffix(" kg")
        b = button("Kalibrovat závažím", "primary")
        b.clicked.connect(lambda: self.send(f"CAL {self.kg_sp.value() * 9.80665:.4f}"))
        row.addWidget(self.kg_sp)
        row.addWidget(b)
        vl.addLayout(row)
        vl.addWidget(label("nebo bez závaží z katalogu snímače (méně přesné):", "muted"))
        row = QtWidgets.QHBoxLayout()
        self.cap_sp = QtWidgets.QDoubleSpinBox()
        self.cap_sp.setRange(0.1, 100000)
        self.cap_sp.setValue(50)
        self.cap_sp.setSuffix(" kg")
        self.sens_sp = QtWidgets.QDoubleSpinBox()
        self.sens_sp.setRange(0.1, 10)
        self.sens_sp.setDecimals(3)
        self.sens_sp.setValue(2.0)
        self.sens_sp.setSuffix(" mV/V")
        b = button("Z katalogu")
        b.clicked.connect(lambda: self.send(f"LCRATED {self.cap_sp.value()} {self.sens_sp.value()}"))
        for x in (self.cap_sp, self.sens_sp, b):
            row.addWidget(x)
        vl.addLayout(row)
        vl.addStretch(1)
        # pressure
        c2 = Card("J9 · GP26 · DĚLIČ 4K7/10K", "Tlakové čidlo")
        vl = QtWidgets.QVBoxLayout(c2.body)
        gl, self.cal_p = kv_grid([("v", "Výstup čidla"), ("psi", "Tlak teď"), ("zero", "Nula"), ("range", "Rozsah")])
        vl.addLayout(gl)
        b = button("Nula tlaku (čidlo na atmosféře)", "primary")
        b.clicked.connect(lambda: self.send("PZERO"))
        vl.addWidget(b)
        f2 = QtWidgets.QFormLayout()
        self.p_full = QtWidgets.QDoubleSpinBox()
        self.p_full.setRange(1, 20000)
        self.p_full.setValue(500)
        self.p_full.setSuffix(" psi")
        self.p_vz = QtWidgets.QDoubleSpinBox()
        self.p_vz.setRange(0, 5)
        self.p_vz.setDecimals(3)
        self.p_vz.setValue(0.5)
        self.p_vz.setSuffix(" V")
        self.p_vf = QtWidgets.QDoubleSpinBox()
        self.p_vf.setRange(0, 5)
        self.p_vf.setDecimals(3)
        self.p_vf.setValue(4.5)
        self.p_vf.setSuffix(" V")
        f2.addRow("Plný rozsah", self.p_full)
        f2.addRow("Výstup při 0 (při 5 V)", self.p_vz)
        f2.addRow("Výstup při plném", self.p_vf)
        vl.addLayout(f2)
        b = button("Zapsat čidlo do rampy")
        b.clicked.connect(self.write_pressure)
        vl.addWidget(b)
        vl.addStretch(1)
        # config table
        c3 = Card("ULOŽENO VE FLASH RAMPY", "Všechny parametry")
        vl = QtWidgets.QVBoxLayout(c3.body)
        self.cfg_tab = QtWidgets.QTableWidget(0, 3)
        self.cfg_tab.setHorizontalHeaderLabels(["Parametr", "Hodnota", "Význam"])
        self.cfg_tab.horizontalHeader().setStretchLastSection(True)
        self.cfg_tab.verticalHeader().setVisible(False)
        self.cfg_tab.setMinimumHeight(420)
        vl.addWidget(self.cfg_tab)
        row = QtWidgets.QHBoxLayout()
        b1 = button("Načíst", tip="CFG")
        b1.clicked.connect(lambda: self.send("CFG"))
        b2 = button("Zapsat změny", "primary", "Pošle změněné hodnoty (SET) – rampa je hned uloží")
        b2.clicked.connect(self.write_cfg)
        b3 = button("Výchozí (kalibrace zůstane)", "danger")
        b3.clicked.connect(lambda: self._confirm("Vrátit parametry na výchozí? Kalibrace tenzometru zůstane.", "DEFAULTS"))
        for x in (b1, b2, b3):
            row.addWidget(x)
        row.addStretch(1)
        vl.addLayout(row)
        g.addWidget(c1, 0, 0)
        g.addWidget(c2, 0, 1)
        g.addWidget(c3, 0, 2, 2, 1)
        g.setColumnStretch(2, 2)
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        self._scroll(w, "Kalibrace")

    def _tab_servos(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 20, 20, 20)
        g.setSpacing(16)
        # --- the arming switch
        c1 = Card("DRUHÝ SPÍNAČ V SÉRII S KLÍČEM J4", "Bezpečnostní spínač (servo)")
        vl = QtWidgets.QVBoxLayout(c1.body)
        info = label("Pyro napájení jde přes dva spínače v sérii: klíč, který sepne poslední člověk u rampy, a spínač "
                     "otočený servem až na povel z dálky – ARM ze stanice (test motoru) nebo PREPARE od rakety na pogu. "
                     "Rampa servo natočí jen když je vše ostatní v pořádku a do nastavené doby musí cítit sepnutý klíč, "
                     "jinak servo vrátí. Zpět do SAFE: SAFE / DISARM / ABORT, hned po pulzu do rozbušky, ztráta spojení.",
                     "muted")
        info.setWordWrap(True)
        vl.addWidget(info)
        warn = label("⚠ Serva na J12 jsou napájená přímo z baterie (2S, až 8,4 V) – použij HV servo (7,4–8,4 V).", "subtle")
        warn.setWordWrap(True)
        warn.setStyleSheet(f"color: {C['accent']};")
        vl.addWidget(warn)
        f = QtWidgets.QFormLayout()
        self.as_cb = QtWidgets.QComboBox()
        self.as_cb.addItems(["vypnuto (jen klíč)"] + [f"Servo {i + 1} ({SERVO_PINS[i]})" for i in range(4)])
        self.as_safe = QtWidgets.QSpinBox()
        self.as_arm = QtWidgets.QSpinBox()
        for sp in (self.as_safe, self.as_arm):
            sp.setRange(500, 2500)
            sp.setSingleStep(10)
            sp.setSuffix(" µs")
        self.as_safe.setValue(1000)
        self.as_arm.setValue(2000)
        self.as_settle = QtWidgets.QSpinBox()
        self.as_settle.setRange(200, 10000)
        self.as_settle.setSingleStep(100)
        self.as_settle.setValue(1500)
        self.as_settle.setSuffix(" ms")
        f.addRow("Servo spínače", self.as_cb)
        f.addRow("Poloha SAFE (rozepnuto)", self.as_safe)
        f.addRow("Poloha ARM (sepnuto)", self.as_arm)
        f.addRow("Klíč musí sepnout do", self.as_settle)
        vl.addLayout(f)
        row = QtWidgets.QHBoxLayout()
        b_w = button("Zapsat do rampy", "primary")
        b_w.clicked.connect(self.write_arm_servo)
        b_s = button("Vyzkoušet SAFE")
        b_s.clicked.connect(lambda: self._as_try(self.as_safe.value()))
        b_a = button("Vyzkoušet ARM (10 s)")
        b_a.clicked.connect(lambda: self._as_try(self.as_arm.value(), arm=True))
        for x in (b_w, b_s, b_a):
            row.addWidget(x)
        vl.addLayout(row)
        b_t = button("ARMTEST – sepne klíč? (rozbuška odpojená!)", "danger",
                     "Servo do ARM, rampa změří, za jak dlouho sepne klíč, a vrátí servo do SAFE")
        b_t.clicked.connect(lambda: self._confirm("ARMTEST: servo sepne pyro napájení (lidský klíč musí být zapnutý).\n\n"
                                                  "ROZBUŠKU MĚJ ODPOJENOU. Pokračovat?", "ARMTEST"))
        vl.addWidget(b_t)
        gl, self.as_vals = kv_grid([("state", "Stav spínače"), ("key", "Klíč (za oběma spínači)"), ("close", "Poslední sepnutí")])
        vl.addLayout(gl)
        vl.addStretch(1)
        g.addWidget(c1, 0, 0, 2, 1)
        # --- the four servos
        self.sv_rows = []
        for i in range(4):
            c = Card(SERVO_PINS[i], f"Servo {i + 1}")
            vl = QtWidgets.QVBoxLayout(c.body)
            sl = QtWidgets.QSlider(Qt.Orientation.Horizontal)
            sl.setRange(500, 2500)
            sl.setValue(1500)
            sp = QtWidgets.QSpinBox()
            sp.setRange(500, 2500)
            sp.setValue(1500)
            sp.setSuffix(" µs")
            sl.valueChanged.connect(sp.setValue)
            sp.valueChanged.connect(sl.setValue)
            sl.sliderReleased.connect(lambda i=i, sp=sp: self._servo(i, sp.value()))
            sp.editingFinished.connect(lambda i=i, sp=sp: self._servo(i, sp.value()))
            row = QtWidgets.QHBoxLayout()
            row.addWidget(sl, 1)
            row.addWidget(sp)
            vl.addLayout(row)
            row = QtWidgets.QHBoxLayout()
            for txt, us in (("1000", 1000), ("1500", 1500), ("2000", 2000)):
                b = button(txt)
                b.clicked.connect(lambda _=False, i=i, us=us, sp=sp: (sp.setValue(us), self._servo(i, us)))
                row.addWidget(b)
            b = button("Uvolnit", tip="Přestat posílat pulzy (servo se nedrží)")
            b.clicked.connect(lambda _=False, i=i: self.send(f"SERVO {i + 1} OFF"))
            row.addWidget(b)
            row.addStretch(1)
            vl.addLayout(row)
            st = label("—", "value")
            vl.addWidget(st)
            self.sv_rows.append((sp, st))
            g.addWidget(c, i // 2, 1 + i % 2)
        g.setColumnStretch(0, 3)
        g.setColumnStretch(1, 2)
        g.setColumnStretch(2, 2)
        g.setRowStretch(2, 1)
        self._scroll(w, "Serva")

    def _servo(self, i: int, us: int):
        arm = to_i(self.cfg.get("arm_servo")) == i + 1
        if arm and us != to_i(self.cfg.get("safe_us"), -1):
            if QtWidgets.QMessageBox.question(self, APP, f"Servo {i + 1} ovládá bezpečnostní spínač – {us} µs může sepnout "
                                              "pyro napájení. Rampa ho za 10 s sama vrátí do SAFE.\n\nPokračovat?") \
                    != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        self.send(f"SERVO {i + 1} {us}")

    def _as_try(self, us: int, arm: bool = False):
        n = self.as_cb.currentIndex()
        if n == 0:
            QtWidgets.QMessageBox.information(self, APP, "Nejdřív vyber servo spínače.")
            return
        if arm and QtWidgets.QMessageBox.question(self, APP, "Poloha ARM může sepnout pyro napájení (rozbušku měj odpojenou). "
                                                  "Za 10 s se servo samo vrátí do SAFE. Pokračovat?") \
                != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.send(f"SERVO {n} {us}")

    def write_arm_servo(self):
        if self.as_arm.value() == self.as_safe.value():
            QtWidgets.QMessageBox.warning(self, APP, "Polohy SAFE a ARM musí být různé.")
            return
        for k, v in (("safe_us", self.as_safe.value()), ("arm_us", self.as_arm.value()),
                     ("arm_settle_ms", self.as_settle.value()), ("arm_servo", self.as_cb.currentIndex())):
            self._queue(f"SET {k} {v}")

    def _tab_records(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(w)
        lay.setContentsMargins(20, 20, 20, 20)
        lay.setSpacing(16)
        c1 = Card("W25Q128 · 16 MB", "Záznamy na flash")
        vl = QtWidgets.QVBoxLayout(c1.body)
        self.rec_tab = QtWidgets.QTableWidget(0, 7)
        self.rec_tab.setHorizontalHeaderLabels(["#", "Druh", "S/s", "Vzorků", "Špička", "Impulz", "Hoření"])
        self.rec_tab.verticalHeader().setVisible(False)
        self.rec_tab.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.rec_tab.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.rec_tab.horizontalHeader().setStretchLastSection(True)
        vl.addWidget(self.rec_tab, 1)
        row = QtWidgets.QHBoxLayout()
        b1 = button("Obnovit", tip="LIST")
        b1.clicked.connect(self.list_records)
        self.dl_btn = button("Stáhnout vybraný", "primary", "Plná frekvence → CSV do Dokumenty/Rampa")
        self.dl_btn.clicked.connect(self.download)
        b3 = button("Smazat vše", "danger")
        b3.clicked.connect(self.erase_all)
        for x in (b1, self.dl_btn, b3):
            row.addWidget(x)
        vl.addLayout(row)
        self.dl_bar = QtWidgets.QProgressBar()
        self.dl_bar.setRange(0, 100)
        self.dl_bar.setValue(0)
        self.dl_bar.setTextVisible(True)
        vl.addWidget(self.dl_bar)
        lay.addWidget(c1, 2)
        c2 = Card("PLNÁ FREKVENCE", "Křivka a statistika")
        vl = QtWidgets.QVBoxLayout(c2.body)
        self.plot = RecordPlot()
        vl.addWidget(self.plot, 1)
        self.rec_info = label("—", "muted")
        self.rec_info.setWordWrap(True)
        self.rec_info.setTextFormat(Qt.TextFormat.RichText)
        vl.addWidget(self.rec_info)
        row = QtWidgets.QHBoxLayout()
        b = button("Otevřít záznam (.bin)…", tip="Načíst dříve stažený binární záznam")
        b.clicked.connect(self.open_bin)
        b2 = button("Složka se záznamy", tip=self.data_dir)
        b2.clicked.connect(lambda: QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(self.data_dir)))
        row.addWidget(b)
        row.addWidget(b2)
        row.addStretch(1)
        vl.addLayout(row)
        lay.addWidget(c2, 3)
        self.tabs.addTab(w, "Záznamy")

    def _tab_console(self):
        w = QtWidgets.QWidget()
        vl = QtWidgets.QVBoxLayout(w)
        vl.setContentsMargins(20, 20, 20, 20)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        vl.addWidget(self.log, 1)
        row = QtWidgets.QHBoxLayout()
        self.cmd_ed = QtWidgets.QLineEdit()
        self.cmd_ed.setPlaceholderText("příkaz (HELLO, STATUS, CFG, SET klíč hodnota, LIST, …)")
        self.cmd_ed.returnPressed.connect(self._send_typed)
        self.show_m = QtWidgets.QCheckBox("zobrazit $M / $S")
        row.addWidget(self.cmd_ed, 1)
        row.addWidget(self.show_m)
        vl.addLayout(row)
        self.tabs.addTab(w, "Konzole")

    # ------------------------------------------------------------------ connection
    def refresh_ports(self):
        cur = self.port_cb.currentText()
        self.port_cb.clear()
        self.port_cb.addItems(list_ports())
        if cur:
            self.port_cb.setCurrentText(cur)

    def toggle_connection(self):
        if self.reader is not None:
            self.stop_reader()
            self.conn_chip.set(None, "odpojeno")
            self.con_btn.setText("Připojit")
            return
        self.connect_port(self.port_cb.currentText())

    def connect_port(self, port: str):
        if not port:
            self.note("žádný port – připoj rampu USB kabelem a stiskni Obnovit")
            return
        self.stop_reader()
        self.reader = SerialReader(port)
        self.reader.line.connect(self.on_line)
        self.reader.status.connect(self._on_status)
        self.reader.start()
        self.con_btn.setText("Odpojit")

    def _on_status(self, s: str):
        self.conn_chip.set(s.startswith("připojeno") or None, s)
        if s.startswith("připojeno"):
            QtCore.QTimer.singleShot(400, lambda: self.send("HELLO"))
            QtCore.QTimer.singleShot(900, self.list_records)

    def start_demo(self):
        self.stop_reader()
        self.reader = DemoRampa()
        self.reader.line.connect(self.on_line)
        self.conn_chip.set(True, "simulace")
        self.con_btn.setText("Odpojit")
        self.send("HELLO")
        QtCore.QTimer.singleShot(300, self.list_records)

    def stop_reader(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None

    def send(self, text: str) -> bool:
        if self.reader is None:
            self.status.showMessage("Rampa není připojená", 4000)
            return False
        self.reader.send(text)
        self.note("→ " + text)
        return True

    def _send_typed(self):
        t = self.cmd_ed.text().strip()
        if t:
            self.send(t)
            self.cmd_ed.clear()

    def _confirm(self, q: str, cmd: str):
        if QtWidgets.QMessageBox.question(self, APP, q) == QtWidgets.QMessageBox.StandardButton.Yes:
            self.send(cmd)

    def note(self, s: str):
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + s)

    # ------------------------------------------------------------------ actions
    def tare(self):
        self._confirm("Vynulovat tenzometr (TARE)?\n\nNa snímači nesmí nic tlačit (upevněný motor nevadí, "
                      "nula je pak bez jeho hmotnosti).", "TARE")

    def write_pressure(self):
        vz, vf = self.p_vz.value(), self.p_vf.value()
        if vf <= vz:
            QtWidgets.QMessageBox.warning(self, APP, "Výstup při plném rozsahu musí být větší než při nule.")
            return
        for k, val in (("p_full_psi", f"{self.p_full.value():.1f}"), ("p_zero", f"{vz / 5.0:.5f}"),
                       ("p_span", f"{(vf - vz) / 5.0:.5f}")):
            self._queue(f"SET {k} {val}")

    def write_cfg(self):
        changed = []
        for r in range(self.cfg_tab.rowCount()):
            k = self.cfg_tab.item(r, 0).text()
            v = self.cfg_tab.item(r, 1).text().strip().replace(",", ".")
            if k in self.cfg and v != self.cfg[k]:
                try:
                    float(v)
                except ValueError:
                    QtWidgets.QMessageBox.warning(self, APP, f"{k}: '{v}' není číslo")
                    return
                changed.append((k, v))
        if not changed:
            self.status.showMessage("Žádná změna", 3000)
            return
        for k, v in changed:
            self._queue(f"SET {k} {v}")

    def _queue(self, cmd: str):
        """SET commands one after another (the rampa stores each before taking the next)."""
        self._q.append(cmd)
        if len(self._q) == 1:
            self.send(cmd)

    def _q_next(self):
        if self._q:
            self._q.pop(0)
        if self._q:
            self.send(self._q[0])

    def list_records(self):
        self.recs = []
        self.send("LIST")

    def download(self):
        row = self.rec_tab.currentRow()
        if row < 0 or row >= len(self.recs):
            QtWidgets.QMessageBox.information(self, APP, "Vyber záznam v tabulce.")
            return
        self.send(f"DUMP {self.recs[row][1]}")

    def erase_all(self):
        txt, ok = QtWidgets.QInputDialog.getText(self, "Smazat všechny záznamy",
                                                 "Smaže VŠECHNY záznamy na flash rampy (kalibrace zůstane).\n"
                                                 "Stáhni si je předem. Pro potvrzení napiš SMAZAT:")
        if ok and txt.strip() == "SMAZAT":
            self.send("ERASEALL YES")
            QtCore.QTimer.singleShot(1500, self.list_records)

    def open_bin(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Záznam rampy", self.data_dir, "Záznam (*.bin)")
        if path:
            try:
                self._show_record(rr.decode(open(path, "rb").read()), path)
            except (OSError, ValueError) as ex:
                QtWidgets.QMessageBox.warning(self, APP, str(ex))

    # ------------------------------------------------------------------ incoming
    def on_line(self, s: str):
        p = s.split(",")
        tag = p[0]
        if tag == "$M" and len(p) >= 9:
            self.m = dict(zip(M_FIELDS, p[1:]))
            t = time.time() - self.t0
            self.ch_f.add(t, tah=to_f(self.m["f"]))
            self.ch_p.add(t, tlak=to_f(self.m["p"]))
            self.ch_t.add(t, T1=to_f(self.m["t1"]), T2=to_f(self.m["t2"]))
            if self.show_m.isChecked():
                self.note(s)
            return
        if tag == "$S" and len(p) >= len(S_FIELDS) + 1:
            self.s = dict(zip(S_FIELDS, p[1:]))
            self.s_rx = time.time()
            if self.show_m.isChecked():
                self.note(s)
            return
        if tag == "$D" and len(p) >= 3 and self.dump:
            try:
                self.dump[1] += bytes.fromhex(p[2])
            except ValueError:
                self.note("vadný řádek záznamu")
            self.dl_bar.setValue(int(100 * len(self.dump[1]) / max(1, self.dump[2])))
            return
        self.note(s)
        if tag == "$ID":
            self.ident = ",".join(p[1:])
        elif tag == "$CFG" and len(p) >= 3:
            self.cfg[p[1]] = p[2]
        elif tag == "#OK" or s.startswith("#OK CFG"):
            pass
        if s.startswith("#OK CFG"):
            self._fill_cfg()
        if s.startswith("#OK SET") or (s.startswith("#ERR") and self._q):
            self._q_next()
        if tag == "$R" and len(p) >= len(R_FIELDS) + 1:
            self.res = dict(zip(R_FIELDS, p[1:]))
            QtCore.QTimer.singleShot(500, self.list_records)
        elif tag == "$REC" and len(p) >= 11:
            self.recs.append(p)
        elif s.startswith("#OK LIST"):
            self._fill_recs()
        elif tag == "$DBEGIN" and len(p) >= 4:
            self.dump = [to_i(p[1]), bytearray(), to_i(p[3])]
            self.dl_bar.setValue(0)
            self.tabs.setCurrentIndex(4)
        elif tag == "$DEND" and self.dump:
            self._finish_dump()
        elif s.startswith("#ERR"):
            self.status.showMessage(s, 8000)

    def _finish_dump(self):
        test, img, _n = self.dump
        self.dump = None
        self.dl_bar.setValue(100)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.data_dir, f"rampa_test_{test:03d}_{stamp}")
        try:
            with open(base + ".bin", "wb") as fh:
                fh.write(bytes(img))
            rec = rr.decode(bytes(img))
        except (OSError, ValueError) as ex:
            QtWidgets.QMessageBox.warning(self, APP, f"Záznam {test} se nepodařilo přečíst: {ex}")
            return
        self._show_record(rec, base + ".csv", write=True)

    def _show_record(self, rec: rr.Record, path: str, write: bool = False):
        st = rr.stats(rec)
        if write:
            try:
                rr.write_csv(rec, path, st)
            except OSError as ex:
                self.note(f"CSV: {ex}")
        self.rec = rec
        self.plot.set(rec, st)
        u = rec.unit
        cls = rr.motor_class(st["impulse"], st["avg"]) if rec.calibrated and st["ignition"] else "—"
        kind = "zkušební záznam" if rec.cold else ("zážeh" if rec.t0_index >= 0 else "bez zážehu")
        warn = "" if rec.closed else " · <b style='color:%s'>neuzavřený (výpadek napájení?)</b>" % C["danger"]
        self.rec_info.setText(
            f"<b>Test #{rec.test_no}</b> · {kind} · {len(rec.t)} vzorků @ {rec.sps} S/s{warn}<br>"
            f"Špička <b>{f_or(st['peak'], '{:.1f}')} {u}</b> v T{f_or(st['t_peak'], '{:+.3f}')} s · "
            f"impulz <b>{f_or(st['impulse'], '{:.2f}')} {u}·s</b> · hoření <b>{f_or(st['burn'], '{:.3f}')} s</b> · "
            f"průměr {f_or(st['avg'], '{:.1f}')} {u} · třída <b>{cls}</b><br>"
            f"zpoždění zážehu {f_or(st['ign_delay'], '{:.3f}')} s · max. tlak {f_or(st['pmax'], '{:.0f}')} psi · "
            f"T1 max {f_or(st['t1max'], '{:.1f}')} °C · T2 max {f_or(st['t2max'], '{:.1f}')} °C<br>"
            f"<span style='color:{C['subtle']}'>{path}</span>")
        self.note(f"záznam #{rec.test_no} → {path}")

    def _fill_cfg(self):
        self.cfg_tab.setRowCount(len(self.cfg))
        for r, (k, v) in enumerate(self.cfg.items()):
            a = QtWidgets.QTableWidgetItem(k)
            a.setFlags(a.flags() & ~Qt.ItemFlag.ItemIsEditable)
            b = QtWidgets.QTableWidgetItem(v)
            c = QtWidgets.QTableWidgetItem(CFG_HELP.get(k, ""))
            c.setFlags(c.flags() & ~Qt.ItemFlag.ItemIsEditable)
            for col, it in enumerate((a, b, c)):
                self.cfg_tab.setItem(r, col, it)
        self.cfg_tab.resizeColumnsToContents()
        if "arm_servo" in self.cfg:
            self.as_cb.setCurrentIndex(max(0, min(4, to_i(self.cfg["arm_servo"]))))
            self.as_safe.setValue(to_i(self.cfg.get("safe_us"), 1000))
            self.as_arm.setValue(to_i(self.cfg.get("arm_us"), 2000))
            self.as_settle.setValue(to_i(self.cfg.get("arm_settle_ms"), 1500))
        pz, ps, pf = to_f(self.cfg.get("p_zero")), to_f(self.cfg.get("p_span")), to_f(self.cfg.get("p_full_psi"))
        if pz == pz and ps == ps:
            self.p_vz.setValue(pz * 5)
            self.p_vf.setValue((pz + ps) * 5)
            self.p_full.setValue(pf)

    def _fill_recs(self):
        self.rec_tab.setRowCount(len(self.recs))
        for r, p in enumerate(self.recs):
            flags = to_i(p[6])
            kind = "zkušební" if flags & 0x20 else ("zážeh" if flags & 0x01 else "—") if to_i(p[5]) else "neuzavřený"
            unit = "N" if flags & 0x40 else "µV/V"
            vals = [p[1], kind, p[4], p[7], f"{to_f(p[8]):.1f} {unit}", f"{to_f(p[9]):.2f} {unit}·s", f"{to_f(p[10]):.2f} s"]
            for c, v in enumerate(vals):
                self.rec_tab.setItem(r, c, QtWidgets.QTableWidgetItem(v))
        self.rec_tab.resizeColumnsToContents()
        if self.recs:
            self.rec_tab.selectRow(len(self.recs) - 1)

    # ------------------------------------------------------------------ refresh
    def refresh(self):
        s, m = self.s, self.m
        age = time.time() - self.s_rx if s else 1e9
        live = age < 2
        unit = "N" if to_f(self.cfg.get("lc_scale"), 0.0) else "µV/V"
        if self.ch_f.unit != unit:
            self.ch_f.unit = unit
        # chips
        self.chips["fw"].set(bool(self.ident) or None, f"Deska: {self.ident.split(',')[1] if ',' in self.ident else '—'}")
        if s:
            proto = s["proto"]
            stand = to_i(s["sstate"], -1)
            mode = {"ZENITH": "Zenith na pogu", "ARIADNA": "Daidalos na pogu"}.get(proto, "test motoru" if stand >= 0 else "bez rakety")
            self.chips["mode"].set(True, f"Režim: {mode}")
            if stand >= 0:
                st_txt, st_ok = STAND_CZ.get(stand, str(stand)), (True if stand == 0 else "warn" if stand in (1, 2, 3, 7) else "hot" if stand in (4, 5) else False)
            else:
                rs = to_i(s["rstate"])
                st_txt = ROCKET_STATES[rs] if rs < len(ROCKET_STATES) else str(rs)
                st_ok = True if rs in (1, 2) else "warn" if rs in (3, 4) else "hot" if rs == 5 else False
            self.chips["state"].set(st_ok if live else None, f"Stav: {st_txt}")
            vb = to_i(s["vbat"]) / 1000
            self.chips["bat"].set(vb >= 7.2 or ("warn" if vb >= 6.6 else False), f"Baterie: {cz(vb, 2)} V")
            self.chips["key"].set(to_i(s["key"]) == 1 or "warn", ("Klíč: sepnut" if to_i(s["key"]) else
                                  ("Klíč: čeká na servo" if to_i(s.get("arm_servo")) else "Klíč: vypnut")))
            cont = "".join(str(i + 1) if to_i(s[f"c{i + 1}"]) >= CONT_MIN_MV else "–" for i in range(3))
            self.chips["cont"].set(True if cont != "–––" else "warn", f"Rozbušky: {cont}")
            self.chips["ads"].set(to_i(s["ads"]) == 1, f"ADS1220: {s['sps']} S/s" if to_i(s["ads"]) else "ADS1220: NEMĚŘÍ")
            fr = to_i(s["free_kb"]) / max(1, to_i(s["total_kb"]))
            self.chips["flash"].set((True if fr > 0.1 else "warn") if to_i(s["flash"]) else False,
                                    f"Flash: {fr * 100:.0f} % volno" if to_i(s["flash"]) else "Flash: CHYBÍ")
            self.chips["lora"].set(True if to_i(s["lora"]) else None, "LoRa: zap" if to_i(s["lora"]) else "LoRa: vyp")
        if not live:
            for k in ("state",):
                self.chips[k].set(None, "Stav: bez dat")

        # overview
        if s:
            P, Lk, Ms = self.ov_pow, self.ov_link, self.ov_meas
            P["vbat"].setText(f"{cz(to_i(s['vbat']) / 1000, 2)} V  (min 6,6 V)")
            P["v5"].setText(f"{cz(to_i(s['v5']) / 1000, 3)} V")
            P["umb"].setText(f"{s['umb_ma']} mA")
            P["key"].setText("zapnut" if to_i(s["key"]) else "vypnut – zážeh nemožný")
            P["btn"].setText("STISKNUTO (ABORT)" if to_i(s["btn"]) else "uvolněno")
            for i in range(3):
                mv = to_i(s[f"c{i + 1}"])
                self.ov_pyro[f"c{i + 1}"].setText(f"{mv} mV  {'kontinuita OK' if mv >= CONT_MIN_MV else 'bez rozbušky'}")
            self.ov_pyro["sel"].setText(f"{s['ch']} · {s['fire_ms']} ms")
            asv = to_i(s.get("arm_servo"))
            self.ov_pyro["arm"].setText(f"servo {asv}: {AS_NAMES.get(to_i(s.get('as')), '?')}" if asv else "není (jen klíč)")
            self.as_vals["state"].setText(f"servo {asv}: {AS_NAMES.get(to_i(s.get('as')), '?')}" if asv else "není nastaveno – jen klíč")
            self.as_vals["key"].setText("SEPNUT – pyro napájení připojeno" if to_i(s["key"]) else "rozepnut")
            self.as_vals["close"].setText(f"{s.get('as_ms')} ms po natočení" if to_i(s.get("as_ms")) else "—")
            for i, (_sp, st) in enumerate(self.sv_rows):
                us = to_i(s.get(f"sv{i + 1}"))
                st.setText(("drží " + f"{us} µs" if us else "nedrží (bez pulzů)") + ("  · BEZPEČNOSTNÍ SPÍNAČ" if asv == i + 1 else ""))
            rf, sf = bits(to_i(s["rfaults"]), ROCKET_FAULTS), bits(to_i(s["sfaults"]), SL_FAULTS)
            self.ov_pyro["rfaults"].setText(", ".join(rf) or "žádné")
            self.ov_pyro["sfaults"].setText(", ".join(sf) or "žádné")
            Lk["proto"].setText({"ZENITH": "Zenith (UART 115200, napájí raketu)", "ARIADNA": "Daidalos / Ariadna (I²C 0x2B)"}
                                .get(s["proto"], "nic (hledá raketu)" if to_i(s["stand"]) == 0 else "nic – režim test motoru"))
            rs = to_i(s["rstate"])
            Lk["rstate"].setText(ROCKET_STATES[rs] if rs < len(ROCKET_STATES) else str(rs))
            Lk["stand"].setText(STAND_CZ.get(to_i(s["sstate"]), "neaktivní") if to_i(s["stand"]) else "neaktivní")
            Lk["lora"].setText(("zapnuto" if to_i(s["lora"]) else "vypnuto (raketa na pogu / po startu)") +
                               (f" · {s['rssi']} dBm" if to_i(s["lora"]) else ""))
            lm = to_i(s["link_ms"], -1)
            Lk["link"].setText(f"paket před {cz(lm / 1000, 1)} s" if lm >= 0 else "zatím nic")
            Lk["fw"].setText(f"{to_i(s['fw']) / 100:.2f}".replace(".", ","))
            Ms["core"].setText("běží" if to_i(s["c1ok"]) else "STOJÍ")
            Ms["ads"].setText(f"{s['sps']} vzorků/s" if to_i(s["ads"]) else "neodpovídá / nepřevádí")
            Ms["flash"].setText(f"JEDEC {s['jedec']} {'W25Q128 OK' if s['jedec'] == 'EF4018' else '?'}" if to_i(s["flash"]) else "neodpovídá")
            Ms["free"].setText(f"{to_i(s['free_kb']) // 1024} / {to_i(s['total_kb']) // 1024} MB")
            Ms["recs"].setText(f"{s['ndir']} (poslední #{s['test_no']})")
            Ms["rec"].setText(f"{REC_STATES[to_i(s['rec'])] if to_i(s['rec']) < len(REC_STATES) else s['rec']}"
                              + (f" · {s['progress']} %" if to_i(s["rec"]) in (1, 3) else ""))
            Ms["drop"].setText(s["dropped"])
            # checks: why the ground station could / could not arm
            ch = max(1, min(3, to_i(s["ch"], 1)))
            sfl = to_i(s["sflags"])
            checks = [
                ((to_i(s["key"]) == 1) or ("warn" if to_i(s.get("arm_servo")) else False),
                 "Klíč J4 sepnut" if not to_i(s.get("arm_servo")) else
                 ("Klíč + servo sepnuty" if to_i(s["key"]) else "Klíč: druhý spínač sepne servo až při ARM / PREPARE")),
                (to_i(s[f"c{ch}"]) >= CONT_MIN_MV, f"Rozbuška {ch} má kontinuitu"),
                (to_i(s["vbat"]) >= 6600, "Baterie ≥ 6,6 V"),
                (to_i(s["ads"]) == 1, "ADS1220 měří"),
                (bool(sfl & 0x80), "Tenzometr kalibrován (jinak síla v µV/V)"),
                (bool(sfl & 0x40), "Flash OK a dost místa"),
                (to_i(s["btn"]) == 0, "Tlačítko ABORT uvolněno"),
                (s["proto"] == "NONE", "Žádná raketa na pogu"),
            ]
            for (d, t), (ok, txt) in zip(self.ov_check_rows, checks):
                d.set(ok)
                t.setText(txt)
        if self.res:
            r = self.res
            u = "N" if to_i(r["flags"]) & 0x40 else "µV/V"
            hang = to_i(r["hang"]) == 1
            self.ov_res.setText(
                f"Test #{r['test_no']}: špička {cz(to_f(r['peak']), 1)} {u}, impulz {cz(to_f(r['impulse']), 2)} {u}·s, "
                f"hoření {cz(to_f(r['burn']), 2)} s, max. tlak {cz(to_f(r['pmax']), 0)} psi"
                + ("\n⚠ ROZBUŠKA ODPÁLENA, TAH NEZAZNAMENÁN – nepřibližuj se 5 minut" if hang else ""))

        # measurement
        if m:
            f, uvv = to_f(m["f"]), to_f(m["uvv"])
            self.big["f"][0].setText(f"{f_or(f, '{:.2f}')} {unit}")
            self.big["f"][1].setText(f"signál {f_or(uvv, '{:.2f}')} µV/V" + ("" if unit == "N" else " · NEKALIBROVÁNO"))
            pp, pv = to_f(m["p"]), to_f(m["pv"])
            self.big["p"][0].setText(f"{f_or(pp, '{:.1f}')} psi")
            self.big["p"][1].setText(f"{f_or(pp * 0.0689476 if pp == pp else None, '{:.2f}')} bar · výstup čidla {f_or(pv, '{:.3f}')} V"
                                     + (" · NEPŘIPOJENO?" if not 0.3 < pv < 4.9 else ""))
            t1, t2 = to_f(m["t1"]), to_f(m["t2"])
            self.big["t"][0].setText(f"{f_or(t1, '{:.1f}')} / {f_or(t2, '{:.1f}')} °C")
            self.big["t"][1].setText("T1 / T2 (— = termistor nepřipojen)")
            self.cal_lc["raw"].setText(f"{f_or(uvv, '{:.3f}')} µV/V")
            self.cal_lc["off"].setText(self.cfg.get("lc_offset", "—"))
            sc = to_f(self.cfg.get("lc_scale"), 0.0)
            self.cal_lc["scale"].setText(f"{sc:.6g} N/kód" if sc else "nekalibrováno")
            self.cal_lc["f"].setText(f"{f_or(f, '{:.3f}')} {unit}   ({f_or(f / 9.80665 if unit == 'N' and f == f else None, '{:.3f}')} kg)")
            self.cal_p["v"].setText(f"{f_or(pv, '{:.4f}')} V")
            self.cal_p["psi"].setText(f"{f_or(pp, '{:.2f}')} psi")
            self.cal_p["zero"].setText(f"{cz(to_f(self.cfg.get('p_zero'), 0.1) * 5, 3)} V")
            self.cal_p["range"].setText(f"{self.cfg.get('p_full_psi', '—')} psi při "
                                        f"{cz((to_f(self.cfg.get('p_zero'), 0.1) + to_f(self.cfg.get('p_span'), 0.8)) * 5, 2)} V")

        armed = s and to_i(s.get("sstate"), -1) in (1, 2, 3, 4, 5)
        self.tare_btn.setEnabled(bool(s) and not armed)
        self.dl_btn.setEnabled(self.dump is None)

    def closeEvent(self, ev):
        self.stop_reader()
        super().closeEvent(ev)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--screenshot")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--tab", type=int, default=0)
    a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    vs.setup_app(app, "VoltinoLabs.Rampa.Diagnostika")
    app.setStyleSheet(vs.QSS)
    w = RampaDiag(a.port, a.demo)
    w.tabs.setCurrentIndex(a.tab)
    if a.screenshot:
        w.resize(1536, 900)
        w.show()
        QtCore.QTimer.singleShot(int(a.seconds * 1000), lambda: (w.grab().save(a.screenshot), app.quit()))
    else:
        w.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
