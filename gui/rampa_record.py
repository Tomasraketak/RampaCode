"""Records of the rampa thrust stand (W25Q128) — decoding, conversion, statistics, CSV.

Byte layout = Rampa_MCU/thrust_stand.h (ts_hdr_t, ts_end_t, ts_frame_t, ts_cfg_t).
No Qt in here: usable from scripts, e.g.

    python rampa_record.py dump.bin            # prints the statistics, writes dump.csv

A record is what the rampa sends for "DUMP <n>": header (128 B), footer (128 B,
missing when the power was cut during the recording) and 16-byte frames at
the full ADC rate (1000 / 2000 S/s). Frames end at the first one whose CRC-8
fails (unwritten flash = 0xFF).
"""
from __future__ import annotations

import csv
import math
import struct
import sys
from dataclasses import dataclass, field

HDR_MAGIC = 0x31535452      # "RTS1"
END_MAGIC = 0x31444E45      # "END1"
FRAMES_OFF = 256
CFG_V1 = struct.Struct("<IHHi11fHHBBH2xH")                 # ts_cfg_t v1, 68 B (records before the arming servo)
CFG = struct.Struct("<IHHi11fHHBBHBxHHHH")                 # ts_cfg_t v2, 74 B
HDR = struct.Struct("<IHHIHBBHHHH")                        # first 24 B of ts_hdr_t
END = struct.Struct("<IIiBBH7fhhI")                        # first 52 B of ts_end_t
FRAME = struct.Struct("<IiHHHBB")                          # ts_frame_t, 16 B
CFG_KEYS_V1 = ["magic", "ver", "seq", "lc_offset", "lc_scale", "p_zero", "p_span", "p_full_psi", "adc_vref", "p_div",
               "th_r0", "th_beta", "th_t0_c", "th_rfix", "ign_thr_n", "fire_ms", "rec_s", "pyro_ch", "rate_2k", "cd_s",
               "crc"]
CFG_KEYS = CFG_KEYS_V1[:-1] + ["arm_servo", "arm_us", "safe_us", "arm_settle_ms", "crc"]
UVV_PER_CODE = 1.0e6 / (128.0 * 8388608.0)
STATE_NAMES = ["SAFE", "ARMING", "ARMED", "COUNTDOWN", "FIRING", "BURN", "ABORT", "RECORD"]
FR_FIRE, FR_TH1, FR_TH2, FR_NOLC = 1, 2, 4, 8


def crc16(b: bytes) -> int:
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def crc8(b: bytes) -> int:
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


# ---------------------------------------------------------------- conversions (= thrust_stand.h)
def force(cfg: dict, code: float) -> float:
    d = code - cfg["lc_offset"]
    return d * cfg["lc_scale"] if cfg["lc_scale"] else d * UVV_PER_CODE


def press_v(cfg: dict, psum: float) -> float:
    return psum / (16.0 * 4095.0) * cfg["adc_vref"] * cfg["p_div"]


def press_psi(cfg: dict, psum: float, v5_mv: int) -> float:
    v5 = v5_mv / 1000.0 if 4000 < v5_mv < 5600 else 5.0
    if cfg["p_span"] <= 0:
        return float("nan")
    return (press_v(cfg, psum) / v5 - cfg["p_zero"]) / cfg["p_span"] * cfg["p_full_psi"]


def therm_c(cfg: dict, ratio: float) -> float:
    if not 0.005 < ratio < 0.995:
        return float("nan")
    r = cfg["th_rfix"] * (1.0 / ratio - 1.0)
    inv = 1.0 / (cfg["th_t0_c"] + 273.15) + math.log(r / cfg["th_r0"]) / cfg["th_beta"]
    return 1.0 / inv - 273.15


@dataclass
class Record:
    test_no: int = 0
    sps: int = 0
    cold: bool = False
    pyro_ch: int = 0
    fire_ms: int = 0
    rec_s: int = 0
    v5_mv: int = 5000
    cfg: dict = field(default_factory=dict)
    closed: bool = False
    end: dict = field(default_factory=dict)
    t: list = field(default_factory=list)          # s from T-0 (or record start)
    f: list = field(default_factory=list)          # N (or uV/V)
    uvv: list = field(default_factory=list)
    p: list = field(default_factory=list)          # psi
    pv: list = field(default_factory=list)         # V at the transducer
    t1: list = field(default_factory=list)
    t2: list = field(default_factory=list)
    fire: list = field(default_factory=list)
    state: list = field(default_factory=list)
    t0_index: int = -1
    bad_tail: int = 0                              # frames after the end that failed CRC

    @property
    def calibrated(self) -> bool:
        return bool(self.cfg.get("lc_scale"))

    @property
    def unit(self) -> str:
        return "N" if self.calibrated else "uV/V"


def decode(img: bytes) -> Record:
    if len(img) < FRAMES_OFF:
        raise ValueError("record too short")
    h = HDR.unpack_from(img, 0)
    if h[0] != HDR_MAGIC or crc16(img[:126]) != struct.unpack_from("<H", img, 126)[0]:
        raise ValueError("bad record header")
    r = Record()
    _m, ver, r.test_no, _blocks, r.sps, cold, r.pyro_ch, r.fire_ms, r.rec_s, _cd, r.v5_mv = h
    r.cold = bool(cold)
    if ver >= 2:
        r.cfg = dict(zip(CFG_KEYS, CFG.unpack_from(img, 24)))
    else:
        r.cfg = dict(zip(CFG_KEYS_V1, CFG_V1.unpack_from(img, 24)))
    e = END.unpack_from(img, 128)
    if e[0] == END_MAGIC and crc16(img[128:254]) == struct.unpack_from("<H", img, 254)[0]:
        r.closed = True
        keys = ["magic", "n_frames", "t0_frame", "flags", "_r", "v5_mv", "peak_n", "impulse_ns", "burn_s", "avg_n",
                "t_peak_s", "ign_delay_s", "pmax_psi", "t1max_dc", "t2max_dc", "dropped"]
        r.end = dict(zip(keys, e))
        if 4000 < r.end["v5_mv"] < 5600:
            r.v5_mv = r.end["v5_mv"]
    n_max = (len(img) - FRAMES_OFF) // FRAME.size
    if r.closed:
        n_max = min(n_max, r.end["n_frames"])
    cfg = r.cfg
    th = [float("nan"), float("nan")]
    t_us0 = None
    for i in range(n_max):
        o = FRAMES_OFF + i * FRAME.size
        raw = img[o:o + FRAME.size]
        if crc8(raw[:15]) != raw[15]:
            r.bad_tail = n_max - i
            break
        t_us, lc, p, th1, th2, flags, _c = FRAME.unpack(raw)
        if t_us0 is None:
            t_us0 = t_us
        if r.t0_index < 0 and flags & FR_FIRE:
            r.t0_index = i
        th[0] = therm_c(cfg, th1 / 65535.0) if th1 else float("nan")
        th[1] = therm_c(cfg, th2 / 65535.0) if th2 else float("nan")
        r.t.append(t_us * 1e-6)
        r.f.append(force(cfg, lc) if not flags & FR_NOLC else float("nan"))
        r.uvv.append(lc * UVV_PER_CODE if not flags & FR_NOLC else float("nan"))
        r.p.append(press_psi(cfg, p, r.v5_mv))
        r.pv.append(press_v(cfg, p))
        r.t1.append(th[0])
        r.t2.append(th[1])
        r.fire.append(bool(flags & FR_FIRE))
        r.state.append(flags >> 4)
    if r.closed and r.end["t0_frame"] >= 0:
        r.t0_index = r.end["t0_frame"]
    if 0 <= r.t0_index < len(r.t):
        z = r.t[r.t0_index]
        r.t = [x - z for x in r.t]
    return r


def stats(r: Record, thr_min: float | None = None) -> dict:
    """Burn statistics at the full rate (same rules as the rampa: threshold = max(5 % peak, ign_thr_n))."""
    out = dict(peak=0.0, t_peak=0.0, impulse=0.0, burn=0.0, avg=0.0, ign_delay=0.0, pmax=0.0, ignition=False,
               t1max=float("nan"), t2max=float("nan"))
    n = len(r.t)
    if n < 2:
        return out
    start = r.t0_index if r.t0_index >= 0 else 0
    idx = [i for i in range(start, n) if r.f[i] == r.f[i]]
    if not idx:
        return out
    ip = max(idx, key=lambda i: r.f[i])
    out["peak"], out["t_peak"] = r.f[ip], r.t[ip]
    thr = max(0.05 * out["peak"], thr_min if thr_min is not None else r.cfg.get("ign_thr_n", 2.0))
    above = [i for i in idx if r.f[i] > thr]
    pv = [x for x in r.p if x == x]
    out["pmax"] = max(pv) if pv else float("nan")
    for k, arr in (("t1max", r.t1), ("t2max", r.t2)):
        v = [x for x in arr if x == x]
        out[k] = max(v) if v else float("nan")
    if above and out["peak"] > r.cfg.get("ign_thr_n", 2.0):
        a, b = above[0], above[-1]
        imp = 0.0
        for i in range(max(a, 1), b + 1):
            if r.f[i] == r.f[i] and r.f[i - 1] == r.f[i - 1]:
                imp += 0.5 * (r.f[i] + r.f[i - 1]) * (r.t[i] - r.t[i - 1])
        out.update(ignition=True, impulse=imp, burn=r.t[b] - r.t[a], ign_delay=r.t[a] if r.t0_index >= 0 else 0.0)
        out["avg"] = imp / out["burn"] if out["burn"] > 0 else 0.0
    return out


def motor_class(impulse_ns: float, avg_n: float) -> str:
    if not impulse_ns or impulse_ns < 0.3125:
        return "–"
    if impulse_ns <= 0.625:
        letter = "1/4A"
    elif impulse_ns <= 1.25:
        letter = "1/2A"
    else:
        k = max(0, math.ceil(math.log2(impulse_ns / 2.5)))
        letter = "ABCDEFGHIJKLMNO"[k] if k < 15 else "O+"
    return f"{letter}{max(0, round(avg_n or 0))}"


def write_csv(r: Record, path: str, st: dict | None = None):
    st = st or stats(r)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# rampa thrust stand record", r.test_no])
        w.writerow(["# sample rate S/s", r.sps])
        w.writerow(["# cold (no igniter)", int(r.cold)])
        w.writerow(["# igniter", r.pyro_ch, "pulse ms", r.fire_ms])
        w.writerow(["# force unit", r.unit])
        for k in ("peak", "t_peak", "impulse", "burn", "avg", "ign_delay", "pmax", "t1max", "t2max"):
            w.writerow([f"# {k}", f"{st[k]:.4f}" if isinstance(st[k], float) else st[k]])
        w.writerow(["# motor class", motor_class(st["impulse"], st["avg"]) if r.calibrated else "-"])
        for k, v in r.cfg.items():
            w.writerow([f"# cfg.{k}", v])
        w.writerow(["t_s", f"force_{'N' if r.calibrated else 'uVV'}", "loadcell_uVV", "pressure_psi", "pressure_V",
                    "t1_C", "t2_C", "igniter", "state"])
        for i in range(len(r.t)):
            w.writerow([f"{r.t[i]:.6f}", f"{r.f[i]:.3f}", f"{r.uvv[i]:.3f}", f"{r.p[i]:.2f}", f"{r.pv[i]:.4f}",
                        f"{r.t1[i]:.2f}", f"{r.t2[i]:.2f}", int(r.fire[i]),
                        STATE_NAMES[r.state[i]] if r.state[i] < len(STATE_NAMES) else r.state[i]])


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    rec = decode(open(sys.argv[1], "rb").read())
    s = stats(rec)
    print(f"test #{rec.test_no}: {len(rec.t)} frames @ {rec.sps} S/s, closed={rec.closed}, unit {rec.unit}")
    for k, v in s.items():
        print(f"  {k:10s} {v}")
    out = sys.argv[1].rsplit(".", 1)[0] + ".csv"
    write_csv(rec, out, s)
    print("written", out)
