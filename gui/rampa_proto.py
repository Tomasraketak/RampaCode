"""Protocol definitions, parsers, Go/No-Go checks, and pure-Python simulator for Rampa PCB1.

Pure Python (no Qt dependency) so it can be unit-tested anywhere and reused by CLI tools.
Supports both:
  1. New structured USB diagnostic protocol ($ID, $S, $EV, $I2C, #OK, #ERR)
  2. Legacy 1 Hz human-readable line (`RAMPA scan port 2 st 1 flags 0x45 ...`)
     from older Rampa_MCU firmware so the GUI works immediately even before reflashing.
"""
from __future__ import annotations

import math
import re
import time

# ---------------------------------------------------------------- constants
VBAT_MIN_MV = 6600
CONT_MIN_MV = 350
UMB_OVERCURRENT_MA = 2500

PROTO_NAMES = {
    0: "Hledá raketu (scan)",
    1: "Zenith (UART 115200)",
    2: "Daidalos / Ariadna V2 (I²C 0x2B)",
}

PORT_NAMES = {
    0: "Odpojeno (Hi-Z)",
    1: "UART0 (GP0 TX / GP1 RX)",
    2: "I²C0 slave 0x2B (GP0 SDA / GP1 SCL)",
}

RAMPA_STATES = {
    0: "OFFLINE",
    1: "IDLE",
    2: "READY",
    3: "COUNTDOWN",
    4: "COMMITTED",
    5: "FIRED",
    6: "ABORT",
    7: "FAULT",
}

RAMPA_FLAG_BITS = [
    (0, "KLÍČ J4"),
    (1, "CONT1"),
    (2, "CONT2"),
    (3, "CONT3"),
    (4, "PYRO ARM"),
    (5, "LoRa VYP"),
    (6, "UMB_5V"),
    (7, "READY"),
]

RAMPA_FAULT_BITS = [
    (0, "SLABÁ BAT"),
    (1, "BEZ KLÍČE"),
    (2, "BEZ KONT."),
    (3, "TIMEOUT"),
    (4, "RAKETA NO-GO"),
    (5, "CHYBA HW"),
    (6, "TLAČ. ABORT"),
    (7, "ČASOVÁNÍ"),
]

HW_BITS = [
    (0, "TCA9534 (0x20)"),
    (1, "INA226 VBAT (0x40)"),
    (2, "INA226 UMB (0x41)"),
    (3, "ADS1115 (0x48)"),
    (4, "LoRa SX1278"),
]

TCA_PINS = [
    (0, "P0 BUZZER", "OUT"),
    (1, "P1 ROCKET_EN (UMB_5V)", "OUT"),
    (2, "P2 KEY (J4)", "IN"),
    (3, "P3 LORA_RST", "OUT"),
    (4, "P4 LORA_EN (U12)", "OUT"),
    (5, "P5 LORA_DIO0", "IN"),
    (6, "P6 FIRE_LED", "OUT"),
    (7, "P7 BTN SW1 (aktivní L)", "IN"),
]

KNOWN_I2C_DEVS = {
    0x20: "TCA9534 (I/O expander)",
    0x40: "INA226 (baterie VBAT)",
    0x41: "INA226 (UMB_5V raketa)",
    0x48: "ADS1115 (kontinuity + 5V)",
}

S_FIELDS = [
    "t_ms", "proto", "port", "state", "flags", "faults", "hw_mask",
    "vbat_mv", "ibat_ma", "umb_mv", "umb_ma", "v5_mv",
    "c1_mv", "c2_mv", "c3_mv",
    "key", "btn", "pyro_arm", "fire_mask",
    "lora_on", "rocket_pwr", "umb_trip", "locate",
    "diag", "diag_arm", "t_rem_ms", "pyro_ch", "fire_ms",
    "last_seq", "cmd_age_ms", "exp_in", "exp_out",
    "lora_tx", "lora_rx",
]

_LEGACY_RE = re.compile(
    r"^RAMPA\s+(scan|ZENITH|ARIADNA)\s+port\s+(\d+)\s+st\s+(\d+)\s+"
    r"flags\s+0x([0-9A-Fa-f]+)\s+faults\s+0x([0-9A-Fa-f]+)\s+"
    r"vbat\s+(\d+)\s+umb\s+(\d+)\s+mA\s+cont\s+(\d+)/(\d+)/(\d+)\s+"
    r"key\s+(\d+)\s+btn\s+(\d+)\s+lora\s+(\d+)"
)
_PROTO_STR_TO_ID = {"scan": 0, "ZENITH": 1, "ARIADNA": 2}


def to_int(val, default: int = 0) -> int:
    try:
        return int(float(val))
    except (TypeError, ValueError):
        return default


def parse_status_line(line: str) -> dict | None:
    """Parse a `$S,...` line into a dictionary of typed ints. Returns None if invalid."""
    if not line.startswith("$S,"):
        return None
    parts = line.split(",")[1:]
    if len(parts) < len(S_FIELDS):
        return None
    d = {k: to_int(parts[i]) for i, k in enumerate(S_FIELDS)}
    d["legacy"] = 0
    return d


def parse_legacy_line(line: str) -> dict | None:
    """Parse the 1 Hz `RAMPA ...` debug line emitted by both old and new firmware."""
    m = _LEGACY_RE.match(line.strip())
    if not m:
        return None
    proto = _PROTO_STR_TO_ID.get(m.group(1), 0)
    flags = int(m.group(4), 16)
    faults = int(m.group(5), 16)
    vbat_mv = int(m.group(6))
    umb_ma = int(m.group(7))
    c1, c2, c3 = int(m.group(8)), int(m.group(9)), int(m.group(10))
    key = int(m.group(11))
    btn = int(m.group(12))
    lora_on = int(m.group(13))
    hw_ok = 0 if (faults & (1 << 5)) else 0x0F
    if lora_on:
        hw_ok |= 0x10
    return {
        "t_ms": 0,
        "proto": proto,
        "port": int(m.group(2)),
        "state": int(m.group(3)),
        "flags": flags,
        "faults": faults,
        "hw_mask": hw_ok,
        "vbat_mv": vbat_mv,
        "ibat_ma": -1,
        "umb_mv": -1,
        "umb_ma": umb_ma,
        "v5_mv": -1,
        "c1_mv": c1,
        "c2_mv": c2,
        "c3_mv": c3,
        "key": key,
        "btn": btn,
        "pyro_arm": 1 if (flags & (1 << 4)) else 0,
        "fire_mask": 0,
        "lora_on": lora_on,
        "rocket_pwr": 1 if (flags & (1 << 6)) else 0,
        "umb_trip": 0,
        "locate": 0,
        "diag": 0,
        "diag_arm": 0,
        "t_rem_ms": -1,
        "pyro_ch": 1,
        "fire_ms": 1000,
        "last_seq": 0,
        "cmd_age_ms": -1,
        "exp_in": (0x04 if key else 0) | (0 if btn else 0x80),
        "exp_out": (0x02 if (flags & (1 << 6)) else 0) | (0x18 if lora_on else 0),
        "lora_tx": 0,
        "lora_rx": 0,
        "legacy": 1,
    }


def parse_i2c_scan_line(line: str) -> list[int] | None:
    """Parse `$I2C,0x20,0x40,...` -> list of integer addresses."""
    if not line.startswith("$I2C"):
        return None
    parts = [p.strip() for p in line.split(",")[1:] if p.strip()]
    addrs: list[int] = []
    for p in parts:
        try:
            addrs.append(int(p, 16) if p.lower().startswith("0x") else int(p))
        except ValueError:
            pass
    return addrs


def evaluate_checks(s: dict) -> list[tuple[str, str]]:
    """Return Go/No-Go checklist items as `(level, text)` where level is 'ok'|'warn'|'bad'."""
    if not s:
        return [("warn", "Deska není připojena — vyberte COM port nebo Simulaci.")]

    items: list[tuple[str, str]] = []
    legacy = bool(s.get("legacy"))
    hw = s.get("hw_mask", 0)
    faults = s.get("faults", 0)
    vbat = s.get("vbat_mv", 0)
    key = bool(s.get("key"))
    btn = bool(s.get("btn"))
    ch = max(1, min(3, s.get("pyro_ch", 1)))
    conts = (s.get("c1_mv", 0), s.get("c2_mv", 0), s.get("c3_mv", 0))

    if legacy:
        items.append(("warn", "Starší firmware rampy (jen 1 Hz čtení) — pro interaktivní DIAG a testy nahrajte aktuální Rampa_MCU.ino."))

    if (hw & 0x0B) != 0x0B or (faults & (1 << 5)):
        missing = []
        if not (hw & 1):
            missing.append("TCA9534 (0x20)")
        if not (hw & 2):
            missing.append("INA226 VBAT (0x40)")
        if not (hw & 4) and not legacy:
            missing.append("INA226 UMB (0x41)")
        if not (hw & 8):
            missing.append("ADS1115 (0x48)")
        items.append(("bad", "Chyba interní sběrnice I²C1 — neodpovídá: " + (", ".join(missing) if missing else "periferie HW")))
    else:
        if not legacy and not (hw & 4):
            items.append(("warn", "INA226 UMB (0x41) neodpovídá na sběrnici I²C1 (proud do rakety není měřen)."))
        else:
            items.append(("ok", "Interní I²C1 periferie OK (TCA9534, INA226 ×2, ADS1115)."))

    if vbat < VBAT_MIN_MV:
        items.append(("bad", f"Nízké napětí baterie: {vbat / 1000:.2f} V (minimum 6,60 V pro 2S).".replace(".", ",")))
    else:
        items.append(("ok", f"Baterie rampy OK: {vbat / 1000:.2f} V.".replace(".", ",")))

    if btn:
        items.append(("bad", "Stisknuto lokální tlačítko ABORT (SW1) — zážeh i DIAG jsou zablokovány."))
    else:
        items.append(("ok", "Lokální tlačítko ABORT (SW1) v klidu."))

    if not key:
        items.append(("warn", "Bezpečnostní klíč J4 je vypnutý (SAFE) — zážeh ani pyro lišta nepůjdou zapnout."))
    else:
        items.append(("ok", "Bezpečnostní klíč J4 sepnutý (ARMED)."))

    if conts[ch - 1] < CONT_MIN_MV:
        items.append(("warn", f"Zvolený kanál CH{ch} (J{4 + ch}) nemá kontinuitu rozbušky ({conts[ch - 1]} mV < {CONT_MIN_MV} mV)."))
    else:
        items.append(("ok", f"Kontinuita kanálu CH{ch} OK ({conts[ch - 1]} mV)."))

    if s.get("umb_trip"):
        items.append(("bad", f"Nadproud na napájení rakety UMB_5V (> {UMB_OVERCURRENT_MA} mA) — výstup byl vypnut."))

    if faults & (1 << 3):
        items.append(("bad", "Latched chyba: TIMEOUT spojení s raketou během odpočtu (reset: SAFE)."))
    if faults & (1 << 4):
        items.append(("bad", "Latched chyba: Raketa shodila příznak GO (reset: SAFE)."))
    if faults & (1 << 7):
        items.append(("bad", "Latched chyba: Chyba časování odpočtu / COMMIT (reset: SAFE)."))

    if s.get("diag"):
        items.append(("warn", "Aktivní stolní DIAG režim (pyro testy povoleny bez rakety)."))

    return items


def format_fw(fw_num: int) -> str:
    if fw_num <= 0:
        return "—"
    return f"{fw_num // 100}.{fw_num % 100:02d}"


# ---------------------------------------------------------------- pure-Python simulator
class RampaSimCore:
    """Simulates a Rampa PCB1 over USB with the exact same text protocol as Rampa_MCU.ino."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.stream = False
        self.proto = 0
        self.port = 2
        self.state = 1  # IDLE
        self.key = True
        self.btn = False
        self.vbat_mv = 7960
        self.ibat_ma = 215
        self.umb_mv = 5020
        self.umb_ma = 165
        self.v5_mv = 5040
        self.cont_mv = [1240, 1180, 15]
        self.rocket_pwr = True
        self.umb_trip = False
        self.lora_on = True
        self.lora_ok = True
        self.lora_tx = 3
        self.lora_rx = 0
        self.locate = False
        self.diag = False
        self.diag_arm = False
        self.diag_fire_ch = 0
        self.diag_fire_end = 0.0
        self.buzz_end = 0.0
        self.led_override = -1
        self.rgb_override = -1
        self.lora_override = -1
        self.pyro_ch = 1
        self.fire_ms = 1000
        self.latched = 0
        self._last_beacon = time.monotonic()

    def _now_ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000)

    def _flags_and_faults(self) -> tuple[int, int]:
        now = time.monotonic()
        f = 0
        if self.key:
            f |= 1 << 0
        for i in range(3):
            if self.cont_mv[i] >= CONT_MIN_MV:
                f |= 1 << (1 + i)
        if self.diag_arm:
            f |= 1 << 4
        if not self.lora_on:
            f |= 1 << 5
        if self.rocket_pwr and not self.umb_trip:
            f |= 1 << 6
        fm = 0
        if self.vbat_mv < VBAT_MIN_MV:
            fm |= 1 << 0
        if not self.key:
            fm |= 1 << 1
        if not (f & (1 << self.pyro_ch)):
            fm |= 1 << 2
        if self.btn:
            fm |= 1 << 6
        if fm == 0:
            f |= 1 << 7
        _ = now
        return f, fm | self.latched

    def status_line(self) -> str:
        now = time.monotonic()
        t = now - self.t0
        # subtle realistic sensor ripple
        vbat = int(self.vbat_mv + 12 * math.sin(t * 0.7))
        ibat = int(self.ibat_ma + (165 if self.rocket_pwr else 0) + 8 * math.sin(t * 1.3))
        umb_mv = int(5020 + 6 * math.cos(t * 0.9)) if (self.rocket_pwr and not self.umb_trip) else 0
        umb_ma = int(165 + 9 * math.sin(t * 1.1)) if (self.rocket_pwr and not self.umb_trip) else 0
        v5 = int(self.v5_mv + 5 * math.sin(t * 0.5))
        c = [max(0, int(self.cont_mv[i] + 4 * math.sin(t * (0.8 + i * 0.4)))) for i in range(3)]
        if self.diag_fire_end and now >= self.diag_fire_end:
            self.diag_fire_end = 0.0
            self.diag_fire_ch = 0
        if self.lora_on and now - self._last_beacon >= 1.0:
            self._last_beacon = now
            self.lora_tx += 1
        if self.proto == 0:
            self.port = 1 if (int(t / 0.6) % 2) else 2
        flags, faults = self._flags_and_faults()
        hw_mask = 0x1F if self.lora_ok else 0x0F
        fire_mask = (1 << (self.diag_fire_ch - 1)) if self.diag_fire_ch else 0
        exp_in = (0x04 if self.key else 0) | (0 if self.btn else 0x80)
        exp_out = 0
        if (self.buzz_end and now < self.buzz_end) or (self.locate and (int(t * 1000) % 2000) < 500):
            exp_out |= 1 << 0
        if self.rocket_pwr and not self.umb_trip:
            exp_out |= 1 << 1
        if self.lora_on:
            exp_out |= (1 << 3) | (1 << 4)
        if self.led_override == 1 or (self.state == 6 and (int(t * 1000) % 500) < 250):
            exp_out |= 1 << 6
        vals = [
            self._now_ms(), self.proto, self.port, self.state, flags, faults, hw_mask,
            vbat, ibat, umb_mv, umb_ma, v5, c[0], c[1], c[2],
            int(self.key), int(self.btn), int(self.diag_arm), fire_mask,
            int(self.lora_on), int(self.rocket_pwr), int(self.umb_trip), int(self.locate),
            int(self.diag), int(self.diag_arm), -1, self.pyro_ch, self.fire_ms,
            0, -1, exp_in, exp_out, self.lora_tx, self.lora_rx,
        ]
        return "$S," + ",".join(str(v) for v in vals)

    def handle_command(self, cmd_text: str) -> list[str]:
        parts = cmd_text.strip().split()
        if not parts:
            return []
        op = parts[0].upper()
        now = time.monotonic()
        t_ms = self._now_ms()

        if op == "HELLO":
            self.stream = True
            return ["$ID,Rampa PCB1,100,Zenith+Daidalos (SIM)", self.status_line()]
        if op in ("STATUS", "PING"):
            return [self.status_line()]
        if op == "HELP":
            return ["#OK CMDS: HELLO STATUS STREAM DIAG ARM FIRE ROCKETPWR LOCATE BUZZ LED RGB LORA I2C ABORT SAFE"]
        if op == "STREAM" and len(parts) >= 2:
            self.stream = to_int(parts[1]) != 0
            return [f"#OK STREAM {int(self.stream)}"]
        if op == "ABORT":
            self.diag = False
            self.diag_arm = False
            self.diag_fire_ch = 0
            self.diag_fire_end = 0.0
            self.led_override = -1
            self.rgb_override = -1
            self.lora_override = -1
            self.state = 6
            return [f"$EV,{t_ms},ABORT (sim)", "#OK ABORT"]
        if op == "SAFE":
            self.diag = False
            self.diag_arm = False
            self.diag_fire_ch = 0
            self.diag_fire_end = 0.0
            self.led_override = -1
            self.rgb_override = -1
            self.lora_override = -1
            self.latched = 0
            self.state = 1
            return [f"$EV,{t_ms},USB SAFE -> IDLE", "#OK SAFE"]
        if op == "ROCKETPWR" and len(parts) >= 2:
            self.rocket_pwr = to_int(parts[1]) != 0
            self.umb_trip = False
            return [f"#OK ROCKETPWR {int(self.rocket_pwr)}", f"$EV,{t_ms},USB ROCKETPWR {int(self.rocket_pwr)}"]
        if op == "LOCATE" and len(parts) >= 2:
            self.locate = to_int(parts[1]) != 0
            return [f"#OK LOCATE {int(self.locate)}"]
        if op == "BUZZ":
            ms = max(50, min(5000, to_int(parts[1], 300) if len(parts) >= 2 else 300))
            self.buzz_end = now + ms / 1000.0
            return [f"#OK BUZZ {ms}"]
        if op == "I2C" and len(parts) >= 2 and parts[1].upper() == "SCAN":
            return ["$I2C,0x20,0x40,0x41,0x48", "#OK I2C SCAN 4"]
        if op == "DIAG" and len(parts) >= 2:
            on = to_int(parts[1]) != 0
            if not on:
                self.diag = False
                self.diag_arm = False
                self.diag_fire_ch = 0
                self.diag_fire_end = 0.0
                self.led_override = -1
                self.rgb_override = -1
                self.lora_override = -1
                return ["#OK DIAG 0", f"$EV,{t_ms},DIAG mode OFF"]
            self.diag = True
            return ["#OK DIAG 1", f"$EV,{t_ms},DIAG mode ON"]
        if op == "ARM" and len(parts) >= 2:
            on = to_int(parts[1]) != 0
            if not on:
                self.diag_arm = False
                self.diag_fire_ch = 0
                self.diag_fire_end = 0.0
                return ["#OK ARM 0", f"$EV,{t_ms},DIAG pyro rail OFF"]
            if not self.diag:
                return ["#ERR ARM requires DIAG 1"]
            if not self.key:
                return ["#ERR ARM key switch J4 is OFF"]
            if self.btn:
                return ["#ERR ARM abort button SW1 pressed"]
            self.diag_arm = True
            return ["#OK ARM 1", f"$EV,{t_ms},DIAG pyro rail ARMED"]
        if op == "FIRE" and len(parts) >= 3:
            ch = to_int(parts[1])
            ms = 300
            confirmed = False
            if len(parts) == 3 and parts[2].upper() == "CONFIRM":
                confirmed = True
            elif len(parts) >= 4 and parts[3].upper() == "CONFIRM":
                ms = max(50, min(2000, to_int(parts[2], 300)))
                confirmed = True
            if not confirmed:
                return ["#ERR FIRE syntax: FIRE <1..3> [ms] CONFIRM"]
            if ch < 1 or ch > 3:
                return ["#ERR FIRE channel must be 1..3"]
            if not self.diag or not self.diag_arm:
                return ["#ERR FIRE requires DIAG 1 and ARM 1"]
            self.diag_fire_ch = ch
            self.diag_fire_end = now + ms / 1000.0
            return [f"#OK FIRE {ch} {ms}", f"$EV,{t_ms},DIAG FIRE ch {ch} ({ms} ms)"]
        if op == "LED" and len(parts) >= 2:
            if not self.diag:
                return ["#ERR LED requires DIAG 1"]
            arg = parts[1].upper()
            self.led_override = -1 if arg == "AUTO" else (1 if to_int(arg) else 0)
            return [f"#OK LED {arg}"]
        if op == "RGB" and len(parts) >= 2:
            if not self.diag:
                return ["#ERR RGB requires DIAG 1"]
            arg = parts[1].upper()
            valid = {"AUTO": -1, "OFF": 0, "R": 1, "G": 2, "B": 3, "Y": 4, "C": 5, "M": 6, "W": 7}
            if arg not in valid:
                return ["#ERR RGB use AUTO|OFF|R|G|B|Y|C|M|W"]
            self.rgb_override = valid[arg]
            return [f"#OK RGB {arg}"]
        if op == "LORA" and len(parts) >= 2:
            if not self.diag:
                return ["#ERR LORA requires DIAG 1"]
            arg = parts[1].upper()
            if arg == "AUTO":
                self.lora_override = -1
                self.lora_on = True
            elif arg == "OFF":
                self.lora_override = 0
                self.lora_on = False
            elif arg == "ON":
                self.lora_override = 1
                self.lora_on = True
            elif arg == "BEACON":
                self.lora_override = 1
                self.lora_on = True
                self.lora_tx += 1
            else:
                return ["#ERR LORA use AUTO|ON|OFF|BEACON"]
            return [f"#OK LORA {arg}"]
        return [f"#ERR unknown command: {parts[0]}"]
