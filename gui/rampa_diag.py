#!/usr/bin/env python3
"""
Rampa · Diagnostika (standalone USB diagnostic console for Rampa PCB1)

Built in the same VoltinoLabs style and architecture as Zenith (zenith_diag.py)
and Daidalos (daidalos_diag.py). Connect Rampa PCB1 directly via USB (115200 Bd)
without needing Zenith or the ground station.

  * Header       port (or "Simulace"), connect, board / protocol / key / pyro chips, ABORT
  * Second bar   state / firmware / faults / Go-No-Go chips + DIAG, SAFE, Buzzer, I2C Scan, CSV log, Open .ino
  * Přehled      Board & power, Pogo umbilical & LoRa, Safety & igniters, live PCB1 block diagram,
                 I2C1 & TCA9534 peripheral grid, and Go/No-Go checklist
  * Grafy        live rolling charts of VBAT/5V, UMB_5V/current, CH1-3 continuity, battery current
  * Pyro a HW    DIAG pyro rail arming + hold-to-fire CH1-3 (J5-J7), UMB_5V switch, buzzer/locate,
                 red fire LED, RGB SK6812 color test, LoRa Ra-02 test & beacon, I2C1 bus scan
  * Konzole      raw USB serial monitor + command line

    pip install -r gui/requirements.txt
    python gui/rampa_diag.py                # pick the COM port in the window
    python gui/rampa_diag.py --demo         # run against built-in simulator
"""
from __future__ import annotations

import argparse
import csv
import datetime
import os
import subprocess
import sys
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import voltino_style as vs  # noqa: E402
from voltino_style import C, Card, Chip, Dot, button, label  # noqa: E402
import rampa_proto as rp  # noqa: E402
from rampa_widgets import (  # noqa: E402
    BAD,
    DIM,
    GOOD,
    WARN,
    DemoReader,
    Flags,
    HoldButton,
    KV,
    PadDiagramWidget,
    SerialReader,
    StripChart,
    list_ports,
)

APP_NAME = "Rampa · Diagnostika"
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def user_dir() -> str:
    d = os.environ.get("RAMPA_DATA") or os.path.join(os.path.expanduser("~"), "Documents", "Rampa")
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        d = os.path.expanduser("~")
    return d


class RampaDiagWindow(QtWidgets.QMainWindow):
    def __init__(self, port: str | None = None, demo: bool = False):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1380, 900)
        self.reader = None
        self.port_name = ""
        self.s: dict = {}
        self.last_rx = 0.0
        self.last_hello = 0.0
        self.fw_info = ""
        self.i2c_addrs: list[int] | None = None
        self._last_checks: list[tuple[str, str]] | None = None
        self.t0 = time.monotonic()
        self.rec_file = None
        self.rec_writer = None

        root = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.setCentralWidget(root)

        outer.addWidget(self._build_header())
        outer.addWidget(self._build_diag_bar())

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._scroll(self._build_overview()), "Přehled")
        self.tabs.addTab(self._build_graphs(), "Grafy")
        self.tabs.addTab(self._scroll(self._build_hw_tab()), "Pyro a testy HW")
        self.tabs.addTab(self._build_console_tab(), "Konzole a protokol")
        outer.addWidget(self.tabs, 1)

        QtGui.QShortcut(QtGui.QKeySequence(Qt.Key.Key_Escape), self, activated=self.disarm)

        self.ui_timer = QtCore.QTimer(self, interval=200, timeout=self._refresh_ui)
        self.ui_timer.start()

        self.refresh_ports()
        self._refresh_ui()
        if demo:
            self.port_box.setCurrentIndex(self.port_box.findData("DEMO"))
            self.connect_toggle()
        elif port:
            idx = self.port_box.findData(port)
            if idx < 0:
                self.port_box.addItem(port, port)
                idx = self.port_box.count() - 1
            self.port_box.setCurrentIndex(idx)
            self.connect_toggle()

    @staticmethod
    def _scroll(w: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        s = QtWidgets.QScrollArea()
        s.setWidgetResizable(True)
        s.setWidget(w)
        return s

    # ------------------------------------------------------------ header
    def _build_header(self) -> QtWidgets.QWidget:
        h = QtWidgets.QFrame()
        h.setObjectName("header")
        lay = QtWidgets.QHBoxLayout(h)
        lay.setContentsMargins(20, 12, 20, 12)
        lay.setSpacing(10)

        brand = QtWidgets.QVBoxLayout()
        brand.setSpacing(0)
        brand.addWidget(label("RAMPA PCB1 · PICO 2 · DIAGNOSTIKA", "kicker"))
        word = QtWidgets.QHBoxLayout()
        word.setSpacing(0)
        word.addWidget(label("Rampa", "title"))
        word.addWidget(label(" Console", "wordAccent"))
        word.addStretch(1)
        brand.addLayout(word)
        lay.addLayout(brand)
        lay.addSpacing(12)

        self.port_box = QtWidgets.QComboBox()
        self.port_box.setMinimumWidth(220)
        lay.addWidget(self.port_box)

        b_ref = button("Obnovit", tip="Znovu načíst seznam sériových portů (COM)")
        b_ref.clicked.connect(self.refresh_ports)
        lay.addWidget(b_ref)

        self.conn_btn = button("Připojit", "primary")
        self.conn_btn.clicked.connect(self.connect_toggle)
        lay.addWidget(self.conn_btn)

        lay.addStretch(1)

        self.chip_board = Chip("Odpojeno")
        self.chip_proto = Chip("Protokol: —")
        self.chip_key = Chip("Klíč: —")
        self.chip_pyro = Chip("Pyro: —")
        for c_ in (self.chip_board, self.chip_proto, self.chip_key, self.chip_pyro):
            lay.addWidget(c_)

        self.abort_btn = button("ABORT", "danger", "Okamžitě zrušit a vypnout pyro (ABORT)", big=True)
        self.abort_btn.clicked.connect(lambda: self._cmd("ABORT"))
        lay.addWidget(self.abort_btn)
        return h

    # ------------------------------------------------------------ second bar
    def _build_diag_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QFrame()
        bar.setObjectName("bar")
        lay = QtWidgets.QHBoxLayout(bar)
        lay.setContentsMargins(20, 8, 20, 8)
        lay.setSpacing(8)

        self.chip_state = Chip("Stav: —")
        self.chip_fw = Chip("FW: —")
        self.chip_fault = Chip("Chyby: —")
        self.chip_go = Chip("Go / No-Go: —")
        for c_ in (self.chip_state, self.chip_fw, self.chip_fault, self.chip_go):
            lay.addWidget(c_)

        lay.addStretch(1)

        self.diag_btn = button(
            "DIAG režim",
            "primary",
            "Zapne/vypne stolní DIAG režim rampy (jen bez rakety na pogu). Povolí testy pyro, LED, RGB a LoRa.",
        )
        self.diag_btn.setCheckable(True)
        self.diag_btn.clicked.connect(self.toggle_diag)
        lay.addWidget(self.diag_btn)

        b_safe = button("SAFE", tip="Zpět do IDLE, smaže zablokovaný ABORT a vypne pyro.")
        b_safe.clicked.connect(lambda: self._cmd("SAFE"))
        lay.addWidget(b_safe)

        b_buzz = button("Bzučák", tip="Krátké pípnutí bzučáku (300 ms)")
        b_buzz.clicked.connect(lambda: self._cmd("BUZZ 300"))
        lay.addWidget(b_buzz)

        b_i2c = button("Sken I²C", tip="Oskenuje interní sběrnici I²C1 (TCA9534, INA226 ×2, ADS1115)")
        b_i2c.clicked.connect(lambda: self._cmd("I2C SCAN"))
        lay.addWidget(b_i2c)

        self.rec_btn = button("Nahrávat", tip="Ukládá každý řádek z rampy do CSV (Documents\\Rampa).")
        self.rec_btn.setCheckable(True)
        self.rec_btn.clicked.connect(self.toggle_record)
        lay.addWidget(self.rec_btn)

        b_ino = button("Firmware (.ino)", tip="Otevře Rampa_MCU\\Rampa_MCU.ino v Arduino IDE")
        b_ino.clicked.connect(self.open_firmware)
        lay.addWidget(b_ino)
        return bar

    # ------------------------------------------------------------ overview tab
    def _build_overview(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)

        c_pwr = Card("Rampa PCB1 · Pico 2", "Deska a napájení")
        self.pwr_kv = KV([
            "Online", "Firmware", "Stav", "DIAG režim",
            "Baterie VBAT (2S)", "Proud baterie", "Větev 5 V (ADS AIN3)",
            "UMB_5V napětí / proud", "Nadproud UMB_5V", "Čas běhu",
        ])
        c_pwr.lay.addWidget(self.pwr_kv)
        c_pwr.lay.addStretch(1)
        g.addWidget(c_pwr, 0, 0)

        c_umb = Card("Pogo J3 + J14 · 433 MHz", "Umbilical a LoRa")
        self.umb_kv = KV([
            "Rozpoznaná raketa", "Aktivní konektor", "Režim portů", "Napájení rakety UMB_5V",
            "Odpočet T−", "Zvolený kanál / pulz", "Poslední seq / stáří",
            "LoRa Ra-02 (433 MHz)", "LoRa TX / RX", "Lokátor bzučák",
        ])
        c_umb.lay.addWidget(self.umb_kv)
        c_umb.lay.addStretch(1)
        g.addWidget(c_umb, 0, 1)

        c_safe = Card("Interlocky · J4 / J5–J7", "Bezpečnost a rozbušky")
        self.safe_kv = KV([
            "Klíč J4 (RBF)", "Tlačítko ABORT (SW1)", "Pyro lišta (GP19)",
            "Aktivně pálí", "Kontinuita CH1 (J5)", "Kontinuita CH2 (J6)", "Kontinuita CH3 (J7)",
        ])
        c_safe.lay.addWidget(self.safe_kv)
        c_safe.lay.addWidget(label("Příznaky (flags)", "subtle"))
        self.flag_chips = Flags(rp.RAMPA_FLAG_BITS)
        c_safe.lay.addWidget(self.flag_chips)
        c_safe.lay.addWidget(label("Chyby (faults)", "subtle"))
        self.fault_chips = Flags(rp.RAMPA_FAULT_BITS, bad_mode=True)
        c_safe.lay.addWidget(self.fault_chips)
        c_safe.lay.addStretch(1)
        g.addWidget(c_safe, 0, 2)

        # Row 1: live PCB1 block diagram + I2C / TCA9534 peripheral health
        c_diag = Card("Vizualizace", "Živé schéma rampy PCB1")
        self.diagram = PadDiagramWidget()
        c_diag.lay.addWidget(self.diagram, 1)
        g.addWidget(c_diag, 1, 0, 1, 2)

        c_hw = Card("Interní I²C1 · GP6/GP7", "Periferie a TCA9534")
        b_scan = button("Sken I²C1")
        b_scan.clicked.connect(lambda: self._cmd("I2C SCAN"))
        c_hw.head.addWidget(b_scan)
        self.hw_chips = Flags(rp.HW_BITS, cols=2)
        c_hw.lay.addWidget(self.hw_chips)
        self.tca_kv = KV([
            "I²C1 nalezené adresy",
            "P0 BUZZER (OUT)",
            "P1 ROCKET_EN (OUT)",
            "P2 KEY J4 (IN)",
            "P3 LORA_RST / P4 EN",
            "P6 FIRE_LED (OUT)",
            "P7 BTN SW1 (IN, akt. L)",
        ])
        c_hw.lay.addWidget(self.tca_kv)
        c_hw.lay.addStretch(1)
        g.addWidget(c_hw, 1, 2)

        # Row 2: Go / No-Go checklist
        c_chk = Card("Kontrola", "Go / No-Go a diagnostika chyb")
        self.check_box = QtWidgets.QVBoxLayout()
        self.check_box.setSpacing(6)
        c_chk.lay.addLayout(self.check_box)
        g.addWidget(c_chk, 2, 0, 1, 3)

        for i in range(3):
            g.setColumnStretch(i, 1)
        return w

    # ------------------------------------------------------------ graphs tab
    def _build_graphs(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(12)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(label("Živé průběhy (posledních 60 s)", "cardTitle"))
        top.addStretch(1)
        b_clr = button("Vymazat grafy")
        b_clr.clicked.connect(self._clear_graphs)
        top.addWidget(b_clr)
        lay.addLayout(top)

        g = QtWidgets.QGridLayout()
        g.setSpacing(12)
        self.ch_volt = StripChart("Napětí VBAT & 5V", ["VBAT", "5V"], "V", span_s=60.0, threshold=(6.6, "min 6,6 V"))
        self.ch_umb = StripChart("Umbilical UMB_5V", ["Proud [mA]", "Napětí [×100 mV]"], "", span_s=60.0)
        self.ch_cont = StripChart(
            "Kontinuita rozbušek (ADS1115)",
            ["CH1 (J5)", "CH2 (J6)", "CH3 (J7)"],
            "mV",
            span_s=60.0,
            threshold=(float(rp.CONT_MIN_MV), f"práh {rp.CONT_MIN_MV} mV"),
        )
        self.ch_ibat = StripChart("Proud baterie (INA226 0x40)", ["I_BAT"], "mA", span_s=60.0)
        g.addWidget(self.ch_volt, 0, 0)
        g.addWidget(self.ch_umb, 0, 1)
        g.addWidget(self.ch_cont, 1, 0)
        g.addWidget(self.ch_ibat, 1, 1)
        lay.addLayout(g, 1)
        return w

    def _clear_graphs(self) -> None:
        for ch in (self.ch_volt, self.ch_umb, self.ch_cont, self.ch_ibat):
            ch.clear()

    # ------------------------------------------------------------ HW & Pyro tab
    def _build_hw_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(20, 16, 20, 20)
        g.setSpacing(16)

        # Left column: Pyro channels (directly modeled on Zenith's Actuators & Pyro tab)
        pc = Card("Rampa PCB1 · J5–J7", "Test pyro kanálů")
        self.arm_chip = Chip("LIŠTA VYPNUTA")
        pc.head.addWidget(self.arm_chip)

        row_arm = QtWidgets.QHBoxLayout()
        self.arm_btn = button("Zapnout pyro lištu…", "primary", "ARM 1: zapne GP19 (vyžaduje DIAG režim a sepnutý klíč J4)", big=True)
        self.arm_btn.clicked.connect(self.arm_clicked)
        self.disarm_btn = button("Vypnout lištu  (Esc)", "", "ARM 0: okamžitě vypne GP19 i všechny kanály", big=True)
        self.disarm_btn.clicked.connect(self.disarm)
        row_arm.addWidget(self.arm_btn, 1)
        row_arm.addWidget(self.disarm_btn, 1)
        pc.lay.addLayout(row_arm)

        row_ms = QtWidgets.QHBoxLayout()
        row_ms.addWidget(label("Délka testovacího pulzu:", "muted"))
        self.pulse_spin = QtWidgets.QSpinBox()
        self.pulse_spin.setRange(50, 2000)
        self.pulse_spin.setSingleStep(50)
        self.pulse_spin.setValue(300)
        self.pulse_spin.setSuffix(" ms")
        row_ms.addWidget(self.pulse_spin)
        row_ms.addStretch(1)
        pc.lay.addLayout(row_ms)

        self.pyro_note = label("", "muted")
        self.pyro_note.setWordWrap(True)
        pc.lay.addWidget(self.pyro_note)

        self.pyro_rows = []
        names = (
            "PYRO 1 · J5 (GP16 → Q22)",
            "PYRO 2 · J6 (GP17 → Q23)",
            "PYRO 3 · J7 (GP18 → Q24)",
        )
        for i, nm in enumerate(names):
            fr = QtWidgets.QFrame()
            fr.setStyleSheet(f"QFrame {{ border-top: 1px solid {C['line']}; }} QLabel, QPushButton {{ border-top: none; }}")
            gl = QtWidgets.QGridLayout(fr)
            gl.setContentsMargins(0, 10, 0, 2)
            gl.setHorizontalSpacing(8)
            gl.setVerticalSpacing(6)
            head = QtWidgets.QHBoxLayout()
            head.addWidget(label(nm, "big"))
            head.addStretch(1)
            firing = Chip("PÁLÍ!")
            firing.set("hot")
            firing.setVisible(False)
            cont = Chip("—")
            head.addWidget(firing)
            head.addWidget(cont)
            gl.addLayout(head, 0, 0, 1, 2)
            raw = label("ADS1115 —", "subtle")
            gl.addWidget(raw, 1, 0)
            fire = HoldButton(f"Podržet: ODPÁLIT CH{i + 1}")
            fire.held.connect(lambda k=i + 1: self.fire_channel(k))
            gl.addWidget(fire, 1, 1)
            gl.setColumnStretch(1, 1)
            pc.lay.addWidget(fr)
            self.pyro_rows.append({"cont": cont, "raw": raw, "firing": firing, "fire": fire})

        pc.lay.addStretch(1)
        warn = label(
            "Pyro test provádějte jen s testovací žárovkou nebo odporem, nikdy s ostrou rozbuškou u motoru! "
            "Odpálení vyžaduje aktivní DIAG režim, sepnutý klíč J4, baterii ≥ 6,6 V a zapnutou pyro lištu.",
            "subtle",
        )
        warn.setWordWrap(True)
        pc.lay.addWidget(warn)
        g.addWidget(pc, 0, 0, 2, 1)

        # Right column top: Power, Buzzer, LED, RGB, LoRa manual controls
        pw = Card("Napájení a signalizace", "Ruční testy periferií")
        self.hw_btn_map: dict[str, QtWidgets.QPushButton] = {}
        self.diag_only_btns: list[QtWidgets.QPushButton] = []

        def add_group(title: str, items: list[tuple[str, str, str, bool]]):
            pw.lay.addWidget(label(title, "subtle"))
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            for txt, cmd, tip, need_diag in items:
                b = button(txt, tip=tip or cmd)
                b.clicked.connect(lambda _=False, c_=cmd: self._cmd(c_))
                row.addWidget(b)
                self.hw_btn_map[cmd] = b
                if need_diag:
                    self.diag_only_btns.append(b)
            row.addStretch(1)
            pw.lay.addLayout(row)

        add_group("Napájení rakety UMB_5V (TCA9534 P1 · buck U2 · J3.2 + J14.2)", [
            ("UMB_5V ZAP", "ROCKETPWR 1", "Povolí 5V napájení rakety na J3.2 + J14.2 (pro Daidalos zůstane vypnuté)", False),
            ("UMB_5V VYP", "ROCKETPWR 0", "Vypne napájení rakety na J3.2 + J14.2", False),
        ])
        add_group("Bzučák (TCA9534 P0)", [
            ("Pípnout 300 ms", "BUZZ 300", "", False),
            ("Pípnout 1 s", "BUZZ 1000", "", False),
            ("Lokátor ZAP", "LOCATE 1", "Periodické pípání každé 2 s", False),
            ("Lokátor VYP", "LOCATE 0", "", False),
        ])
        add_group("Červené výstražné světlo (TCA9534 P6 · jen DIAG)", [
            ("Světlo ZAP", "LED 1", "", True),
            ("Světlo VYP", "LED 0", "", True),
            ("Světlo AUTO", "LED AUTO", "", True),
        ])
        add_group("RGB LED SK6812 (GP20 · jen DIAG)", [
            ("AUTO", "RGB AUTO", "", True),
            ("Červená", "RGB R", "", True),
            ("Zelená", "RGB G", "", True),
            ("Modrá", "RGB B", "", True),
            ("Žlutá", "RGB Y", "", True),
            ("Bílá", "RGB W", "", True),
            ("Vyp", "RGB OFF", "", True),
        ])
        add_group("LoRa M1 Ra-02 SX1278 433 MHz (SPI1 + U12 · jen DIAG)", [
            ("LoRa AUTO", "LORA AUTO", "", True),
            ("LoRa ZAP", "LORA ON", "Zapne U12 a inicializuje SX1278", True),
            ("LoRa VYP", "LORA OFF", "", True),
            ("Testovací beacon", "LORA BEACON", "Okamžitě vyšle jeden 433 MHz beacon", True),
        ])
        pw.lay.addStretch(1)
        g.addWidget(pw, 0, 1)

        # Right column bottom: Quick event log
        evc = Card("Události", "Poslední odpovědi a události z desky")
        b_ev_clr = button("Vymazat")
        evc.head.addWidget(b_ev_clr)
        self.ev_view = QtWidgets.QPlainTextEdit()
        self.ev_view.setReadOnly(True)
        self.ev_view.setMaximumBlockCount(400)
        self.ev_view.setMinimumHeight(210)
        b_ev_clr.clicked.connect(self.ev_view.clear)
        evc.lay.addWidget(self.ev_view, 1)
        g.addWidget(evc, 1, 1)

        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 1)
        return w

    # ------------------------------------------------------------ console tab
    def _build_console_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(20, 16, 20, 20)
        lay.setSpacing(12)

        lc = Card("USB Serial · 115200 Bd", "Konzole a protokol")
        self.show_stream = QtWidgets.QCheckBox("zobrazovat proud $S (10 Hz)")
        lc.head.addWidget(self.show_stream)
        b_help = button("HELP", tip="Vypíše dostupné příkazy z desky")
        b_help.clicked.connect(lambda: self._cmd("HELP"))
        lc.head.addWidget(b_help)
        b_clr = button("Vymazat")
        lc.head.addWidget(b_clr)

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        b_clr.clicked.connect(self.log_view.clear)
        lc.lay.addWidget(self.log_view, 1)

        row = QtWidgets.QHBoxLayout()
        self.cmd_line = QtWidgets.QLineEdit()
        self.cmd_line.setPlaceholderText("Příkaz pro rampu: HELLO, STATUS, I2C SCAN, BUZZ 500, DIAG 1, ROCKETPWR 0, SAFE, ABORT  (Enter)")
        self.cmd_line.returnPressed.connect(self._send_raw)
        row.addWidget(self.cmd_line, 1)
        b_send = button("Odeslat", "primary")
        b_send.clicked.connect(self._send_raw)
        row.addWidget(b_send)
        lc.lay.addLayout(row)

        lay.addWidget(lc, 1)
        return w

    # ------------------------------------------------------------ connection
    def refresh_ports(self) -> None:
        cur = self.port_box.currentData()
        self.port_box.clear()
        for dev, lbl in list_ports():
            self.port_box.addItem(lbl, dev)
        self.port_box.addItem("Simulace (demo bez hardwaru)", "DEMO")
        if cur is not None:
            idx = self.port_box.findData(cur)
            if idx >= 0:
                self.port_box.setCurrentIndex(idx)

    def connect_toggle(self) -> None:
        if self.reader is not None:
            self._disconnect()
            return
        port = self.port_box.currentData()
        if not port:
            return
        self.port_name = str(port)
        self.s = {}
        self.fw_info = ""
        self.i2c_addrs = None
        self.last_rx = 0.0
        self.last_hello = time.monotonic()
        self._clear_graphs()

        if port == "DEMO":
            self.reader = DemoReader()
        else:
            self.reader = SerialReader(port, 115200)
        self.reader.line.connect(self._on_line)
        self.reader.status.connect(self._on_status)
        self.reader.start()

        self.conn_btn.setText("Odpojit")
        self.port_box.setEnabled(False)

    def _disconnect(self) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.s = {}
        self.fw_info = ""
        self.last_rx = 0.0
        self.conn_btn.setText("Připojit")
        self.port_box.setEnabled(True)
        self.chip_board.set("", "Odpojeno")
        self._refresh_ui()

    def _send_hello(self) -> None:
        if self.reader is not None:
            self.last_hello = time.monotonic()
            self.reader.send("HELLO")

    def _on_status(self, msg: str) -> None:
        self._append_log(f"[port] {msg}", is_event=True)
        if msg.startswith("připojeno"):
            self.chip_board.set("warn", f"Čekám na data ({self.port_name})")
            QtCore.QTimer.singleShot(150, self._send_hello)
        else:
            self.chip_board.set("bad", msg)

    def _cmd(self, text: str) -> None:
        if self.reader is None:
            self._append_log(f"[chyba] Nejste připojeni (příkaz '{text}' neodeslán)", is_event=True)
            return
        self._append_log(f">>> {text}", is_event=True)
        self.reader.send(text)

    def _send_raw(self) -> None:
        txt = self.cmd_line.text().strip()
        if not txt:
            return
        self.cmd_line.clear()
        self._cmd(txt)

    # ------------------------------------------------------------ actions
    def toggle_diag(self) -> None:
        want = self.diag_btn.isChecked()
        self._cmd(f"DIAG {1 if want else 0}")

    def arm_clicked(self) -> None:
        if not self.s.get("diag"):
            QtWidgets.QMessageBox.warning(self, APP_NAME, "Nejprve zapněte DIAG režim (tlačítko v horní liště).")
            return
        res = QtWidgets.QMessageBox.question(
            self,
            "Zapnout pyro lištu rampy?",
            "Opravdu zapnout PYRO_ARM (GP19) v DIAG režimu?\n\n"
            "Ujistěte se, že na svorkách J5–J7 nejsou ostré rozbušky u motoru (použijte testovací žárovku nebo odpor).",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.No,
        )
        if res == QtWidgets.QMessageBox.StandardButton.Yes:
            self._cmd("ARM 1")

    def disarm(self) -> None:
        if self.reader is not None:
            self._cmd("ARM 0")

    def fire_channel(self, ch: int) -> None:
        ms = self.pulse_spin.value()
        self._cmd(f"FIRE {ch} {ms} CONFIRM")

    def toggle_record(self) -> None:
        if self.rec_btn.isChecked():
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(user_dir(), f"rampa_{ts}.csv")
            try:
                self.rec_file = open(path, "w", newline="", encoding="utf-8")
                self.rec_writer = csv.writer(self.rec_file)
                self.rec_writer.writerow(["pc_time", "line"])
                self.rec_btn.setText("Zastavit záznam")
                self._append_log(f"[záznam] Ukládám do {path}", is_event=True)
            except OSError as ex:
                self.rec_btn.setChecked(False)
                QtWidgets.QMessageBox.warning(self, APP_NAME, f"Nelze otevřít soubor pro záznam:\n{ex}")
        else:
            if self.rec_file:
                try:
                    self.rec_file.close()
                except OSError:
                    pass
                self.rec_file = None
                self.rec_writer = None
            self.rec_btn.setText("Nahrávat")
            self._append_log("[záznam] Ukončeno", is_event=True)

    def open_firmware(self) -> None:
        ino = os.path.join(REPO_ROOT, "Rampa_MCU", "Rampa_MCU.ino")
        if not os.path.isfile(ino):
            QtWidgets.QMessageBox.warning(self, APP_NAME, f"Soubor nenalezen:\n{ino}")
            return
        try:
            if sys.platform == "win32":
                os.startfile(ino)  # type: ignore[attr-defined]
            elif sys.platform == "darwin":
                subprocess.Popen(["open", ino])
            else:
                subprocess.Popen(["xdg-open", ino])
        except Exception as ex:
            QtWidgets.QMessageBox.warning(self, APP_NAME, f"Nepodařilo se otevřít {ino}:\n{ex}")

    # ------------------------------------------------------------ incoming lines
    def _append_log(self, text: str, is_event: bool = False) -> None:
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        line = f"{ts}  {text}"
        self.log_view.appendPlainText(line)
        if is_event:
            self.ev_view.appendPlainText(line)

    def _on_line(self, line: str) -> None:
        if self.rec_writer is not None:
            self.rec_writer.writerow([datetime.datetime.now().isoformat(timespec="milliseconds"), line])

        now = time.monotonic()

        if line.startswith("$S,"):
            parsed = rp.parse_status_line(line)
            if parsed is not None:
                self.s = parsed
                self.last_rx = now
                self._push_charts(now - self.t0, parsed)
            if self.show_stream.isChecked():
                self._append_log(line)
            return

        if line.startswith("$ID,"):
            parts = line.split(",")
            board = parts[1] if len(parts) > 1 else "Rampa"
            fw = rp.format_fw(rp.to_int(parts[2])) if len(parts) > 2 else "—"
            note = parts[3] if len(parts) > 3 else ""
            self.fw_info = f"{board} · FW {fw}" + (f" ({note})" if note else "")
            self.last_rx = now
            self._append_log(line, is_event=True)
            return

        if line.startswith("$I2C"):
            self.i2c_addrs = rp.parse_i2c_scan_line(line)
            self._append_log(line, is_event=True)
            return

        if line.startswith("$EV,"):
            parts = line.split(",", 2)
            msg = parts[2] if len(parts) >= 3 else line
            self._append_log(f"[událost] {msg}", is_event=True)
            return

        if line.startswith("#OK") or line.startswith("#ERR"):
            self._append_log(line, is_event=True)
            return

        # Legacy 1 Hz line ("RAMPA scan port 2 st 1 ..."):
        # Parse it if we haven't received a structured $S within the last 1.5 s.
        if line.startswith("RAMPA "):
            if not self.s or self.s.get("legacy") or (now - self.last_rx > 1.5):
                leg = rp.parse_legacy_line(line)
                if leg is not None:
                    self.s = leg
                    self.last_rx = now
                    self._push_charts(now - self.t0, leg)
            if self.show_stream.isChecked() or self.s.get("legacy"):
                self._append_log(line)
            return

        self._append_log(line, is_event=True)

    def _push_charts(self, t: float, d: dict) -> None:
        vbat_v = d["vbat_mv"] / 1000.0
        v5_v = d["v5_mv"] / 1000.0 if d["v5_mv"] >= 0 else None
        self.ch_volt.add(t, vbat_v, v5_v)
        umb_v10 = d["umb_mv"] / 100.0 if d["umb_mv"] >= 0 else None
        self.ch_umb.add(t, float(d["umb_ma"]), umb_v10)
        self.ch_cont.add(t, float(d["c1_mv"]), float(d["c2_mv"]), float(d["c3_mv"]))
        if d["ibat_ma"] >= 0:
            self.ch_ibat.add(t, float(d["ibat_ma"]))

    # ------------------------------------------------------------ UI refresh
    def _refresh_ui(self) -> None:
        now = time.monotonic()
        connected = self.reader is not None
        fresh = connected and self.last_rx > 0 and (now - self.last_rx) < 3.0

        # If connected to a board but no $S yet, re-send HELLO occasionally
        if connected and (not fresh or self.s.get("legacy")) and (now - self.last_hello) > 2.5:
            if not self.s.get("legacy"):
                self._send_hello()

        s = self.s if fresh else {}
        legacy = bool(s.get("legacy"))
        diag = bool(s.get("diag"))
        diag_arm = bool(s.get("diag_arm"))
        key = bool(s.get("key"))
        btn = bool(s.get("btn"))
        state = s.get("state", 0)
        state_txt = rp.RAMPA_STATES.get(state, str(state)) if fresh else "—"
        proto = s.get("proto", 0)
        faults = s.get("faults", 0)
        flags = s.get("flags", 0)

        # Header chips
        if not connected:
            self.chip_board.set("", "Odpojeno")
        elif not fresh:
            self.chip_board.set("warn", f"Bez dat ({self.port_name})")
        elif legacy:
            self.chip_board.set("warn", "Starší FW (jen čtení)")
        else:
            self.chip_board.set("ok", f"Online · {self.port_name}")

        if fresh:
            conn = s.get("conn", 0)
            c_tag = {1: " · J3", 2: " · J14"}.get(conn, "")
            p_short = {0: "Hledá (J3+J14)", 1: f"Zenith UART{c_tag}", 2: f"Daidalos I²C{c_tag}"}.get(proto, "?")
            self.chip_proto.set("ok" if proto != 0 else "warn", p_short)
            self.chip_key.set("ok" if key else "warn", "Klíč J4: SEPNUT" if key else "Klíč J4: SAFE")
            if s.get("fire_mask"):
                self.chip_pyro.set("hot", "PYRO PÁLÍ!")
            elif s.get("pyro_arm"):
                self.chip_pyro.set("warn", "Pyro lišta: ZAP")
            else:
                self.chip_pyro.set("", "Pyro lišta: VYP")
        else:
            self.chip_proto.set("", "Protokol: —")
            self.chip_key.set("", "Klíč: —")
            self.chip_pyro.set("", "Pyro: —")

        # Second bar chips
        if fresh:
            st_kind = "ok" if state in (1, 2) else ("hot" if state in (4, 5) else ("bad" if state in (6, 7) else "warn"))
            self.chip_state.set(st_kind, f"Stav: {state_txt}")
            self.chip_fw.set("ok" if not legacy else "warn", self.fw_info or ("Starší FW" if legacy else "FW 1.00"))
            self.chip_fault.set("bad" if faults else "ok", f"Chyby: 0x{faults:02X}" if faults else "Chyby: 0")
            ready = bool(flags & (1 << 7)) and not faults
            self.chip_go.set("ok" if ready else "warn", "RAMPA READY" if ready else "NO-GO / ČEKÁ")
        else:
            self.chip_state.set("", "Stav: —")
            self.chip_fw.set("", "FW: —")
            self.chip_fault.set("", "Chyby: —")
            self.chip_go.set("", "Go / No-Go: —")

        # Sync DIAG toggle button without re-triggering signal
        self.diag_btn.blockSignals(True)
        self.diag_btn.setChecked(diag)
        self.diag_btn.setEnabled(fresh and not legacy and proto == 0)
        self.diag_btn.blockSignals(False)

        # Overview KV cards
        if not fresh:
            self.pwr_kv.clear()
            self.umb_kv.clear()
            self.safe_kv.clear()
            self.tca_kv.clear()
            self.flag_chips.set(None)
            self.fault_chips.set(None)
            self.hw_chips.set(None)
        else:
            vbat_v = s["vbat_mv"] / 1000.0
            self.pwr_kv.set("Online", "ano (starší FW)" if legacy else "ano", WARN if legacy else GOOD)
            self.pwr_kv.set("Firmware", self.fw_info or ("starší (bez $S)" if legacy else "1.00"))
            self.pwr_kv.set("Stav", state_txt, GOOD if state in (1, 2) else (BAD if state in (6, 7) else WARN))
            self.pwr_kv.set("DIAG režim", "AKTIVNÍ" if diag else "vypnut", WARN if diag else DIM)
            self.pwr_kv.set("Baterie VBAT (2S)", f"{vs.cz(vbat_v, 2)} V  ({s['vbat_mv']} mV)", GOOD if s["vbat_mv"] >= rp.VBAT_MIN_MV else BAD)
            self.pwr_kv.set("Proud baterie", f"{s['ibat_ma']} mA" if s["ibat_ma"] >= 0 else "—")
            if s["v5_mv"] >= 0:
                v5v = s["v5_mv"] / 1000.0
                self.pwr_kv.set("Větev 5 V (ADS AIN3)", f"{vs.cz(v5v, 2)} V", GOOD if 4.6 <= v5v <= 5.4 else WARN)
            else:
                self.pwr_kv.set("Větev 5 V (ADS AIN3)", "—", DIM)
            umb_v_str = f"{vs.cz(s['umb_mv'] / 1000.0, 2)} V / " if s["umb_mv"] >= 0 else ""
            self.pwr_kv.set("UMB_5V napětí / proud", f"{umb_v_str}{s['umb_ma']} mA", BAD if s["umb_trip"] else None)
            self.pwr_kv.set("Nadproud UMB_5V", "ODPOJENO (> 2,5 A)" if s["umb_trip"] else "OK", BAD if s["umb_trip"] else GOOD)
            self.pwr_kv.set("Čas běhu", f"{vs.cz(s['t_ms'] / 1000.0, 1)} s" if s["t_ms"] > 0 else "—")

            conn = s.get("conn", 0)
            self.umb_kv.set("Rozpoznaná raketa", rp.PROTO_NAMES.get(proto, str(proto)), GOOD if proto else WARN)
            self.umb_kv.set("Aktivní konektor", rp.CONN_NAMES.get(conn, str(conn)), GOOD if conn else WARN)
            self.umb_kv.set("Režim portů", rp.PORT_NAMES.get(s["port"], str(s["port"])))
            umb_out = bool(flags & (1 << 6))
            self.umb_kv.set("Napájení rakety UMB_5V", "ZAPNUTO" if umb_out else "VYPNUTO", GOOD if umb_out else DIM)
            self.umb_kv.set("Odpočet T−", f"T−{vs.cz(s['t_rem_ms'] / 1000.0, 2)} s" if s["t_rem_ms"] >= 0 else "—")
            self.umb_kv.set("Zvolený kanál / pulz", f"CH{s['pyro_ch']} / {s['fire_ms']} ms")
            age_txt = f"{s['cmd_age_ms']} ms" if s["cmd_age_ms"] >= 0 else "—"
            self.umb_kv.set("Poslední seq / stáří", f"#{s['last_seq']} / {age_txt}")
            self.umb_kv.set("LoRa Ra-02 (433 MHz)", "ZAPNUTO (beacon)" if s["lora_on"] else "VYPNUTO", GOOD if s["lora_on"] else DIM)
            self.umb_kv.set("LoRa TX / RX", f"{s['lora_tx']} / {s['lora_rx']}")
            self.umb_kv.set("Lokátor bzučák", "ZAP" if s["locate"] else "vyp", WARN if s["locate"] else DIM)

            self.safe_kv.set("Klíč J4 (RBF)", "SEPNUT (ARMED)" if key else "VYPNUT (SAFE)", GOOD if key else WARN)
            self.safe_kv.set("Tlačítko ABORT (SW1)", "STISKNUTO!" if btn else "v klidu", BAD if btn else GOOD)
            self.safe_kv.set("Pyro lišta (GP19)", "ZAPNUTA (VPYRO_ARMED)" if s["pyro_arm"] else "vypnuta", WARN if s["pyro_arm"] else DIM)
            fm = s["fire_mask"]
            fchs = [f"CH{i + 1}" for i in range(3) if fm & (1 << i)]
            self.safe_kv.set("Aktivně pálí", ", ".join(fchs) if fchs else "žádný", BAD if fchs else DIM)
            for idx, key_name in enumerate(("Kontinuita CH1 (J5)", "Kontinuita CH2 (J6)", "Kontinuita CH3 (J7)")):
                mv = s[f"c{idx + 1}_mv"]
                ok = mv >= rp.CONT_MIN_MV
                self.safe_kv.set(key_name, f"{mv} mV  ({'OK' if ok else 'rozpojeno'})", GOOD if ok else DIM)

            self.flag_chips.set(flags)
            self.fault_chips.set(faults)
            self.hw_chips.set(s["hw_mask"], bad_when_off=(0, 1, 3))

            # TCA9534 & I2C scan
            if self.i2c_addrs is not None:
                if self.i2c_addrs:
                    self.tca_kv.set("I²C1 nalezené adresy", ", ".join(f"0x{a:02X}" for a in self.i2c_addrs), GOOD)
                else:
                    self.tca_kv.set("I²C1 nalezené adresy", "žádné!", BAD)
            else:
                self.tca_kv.set("I²C1 nalezené adresy", "klikněte na Sken I²C1", DIM)

            ei, eo = s["exp_in"], s["exp_out"]
            self.tca_kv.set("P0 BUZZER (OUT)", "HIGH (pípá)" if (eo & 0x01) else "LOW", WARN if (eo & 0x01) else DIM)
            self.tca_kv.set("P1 ROCKET_EN (OUT)", "HIGH (UMB_5V ZAP)" if (eo & 0x02) else "LOW", GOOD if (eo & 0x02) else DIM)
            self.tca_kv.set("P2 KEY J4 (IN)", "HIGH (sepnut)" if (ei & 0x04) else "LOW (vypnut)", GOOD if (ei & 0x04) else WARN)
            self.tca_kv.set("P3 LORA_RST / P4 EN", f"RST={'H' if (eo & 0x08) else 'L'} / EN={'H' if (eo & 0x10) else 'L'}")
            self.tca_kv.set("P6 FIRE_LED (OUT)", "HIGH (svítí)" if (eo & 0x40) else "LOW", WARN if (eo & 0x40) else DIM)
            self.tca_kv.set("P7 BTN SW1 (IN, akt. L)", "LOW (STISKNUTO!)" if not (ei & 0x80) else "HIGH (v klidu)", BAD if not (ei & 0x80) else GOOD)

        # Live block diagram
        self.diagram.set_state(s)

        # Go / No-Go checklist
        self._rebuild_checks(rp.evaluate_checks(s))

        # Pyro & HW tab controls
        can_diag = fresh and not legacy and diag
        can_arm = can_diag and key and not btn and s.get("vbat_mv", 0) >= rp.VBAT_MIN_MV
        can_fire = can_arm and diag_arm

        self.arm_chip.set("warn" if s.get("pyro_arm") else "", "LIŠTA ZAPNUTA" if s.get("pyro_arm") else "LIŠTA VYPNUTA")
        self.arm_btn.setEnabled(can_arm and not diag_arm)
        self.disarm_btn.setEnabled(fresh and not legacy and (diag_arm or bool(s.get("pyro_arm"))))

        if not fresh:
            self.pyro_note.setText("Připojte rampu přes USB (nebo zvolte Simulaci).")
        elif legacy:
            self.pyro_note.setText("Na rampě běží starší firmware bez USB DIAG příkazů. Nahrajte aktuální Rampa_MCU.ino.")
        elif proto != 0:
            self.pyro_note.setText("Na pogu je připojena raketa — stolní pyro testy jsou zablokovány.")
        elif not diag:
            self.pyro_note.setText("Pro aktivaci pyro testů zapněte v horní liště DIAG režim.")
        elif btn:
            self.pyro_note.setText("Je stisknuto lokální tlačítko ABORT (SW1) — pyro zablokováno.")
        elif not key:
            self.pyro_note.setText("Bezpečnostní klíč J4 je vypnutý — pro zapnutí pyro lišty jej sepněte.")
        elif s.get("vbat_mv", 0) < rp.VBAT_MIN_MV:
            self.pyro_note.setText("Baterie je pod 6,60 V — pyro lištu nelze zapnout.")
        elif not diag_arm:
            self.pyro_note.setText("DIAG aktivní a interlocky OK. Můžete zapnout pyro lištu (GP19).")
        else:
            self.pyro_note.setText("PYRO LIŠTA ZAPNUTA! Pro testovací pulz podržte tlačítko vybraného kanálu (1,2 s).")

        for i, row in enumerate(self.pyro_rows):
            if fresh:
                mv = s[f"c{i + 1}_mv"]
                ok = mv >= rp.CONT_MIN_MV
                row["cont"].set("ok" if ok else "", f"{mv} mV · OK" if ok else f"{mv} mV · rozpojeno")
                row["raw"].setText(f"ADS1115 AIN{i}: {mv} mV  (práh {rp.CONT_MIN_MV} mV)")
                row["firing"].setVisible(bool(s["fire_mask"] & (1 << i)))
            else:
                row["cont"].set("", "—")
                row["raw"].setText(f"ADS1115 AIN{i}: —")
                row["firing"].setVisible(False)
            row["fire"].setEnabled(can_fire)

        for b in self.diag_only_btns:
            b.setEnabled(can_diag)
        for cmd_k, b in self.hw_btn_map.items():
            if b not in self.diag_only_btns:
                b.setEnabled(fresh and not legacy)

    def _rebuild_checks(self, items: list[tuple[str, str]]) -> None:
        if items == self._last_checks:
            return
        self._last_checks = list(items)
        while self.check_box.count():
            it = self.check_box.takeAt(0)
            w = it.widget()
            if w is not None:
                w.deleteLater()
        for level, text in items:
            row_w = QtWidgets.QWidget()
            hl = QtWidgets.QHBoxLayout(row_w)
            hl.setContentsMargins(0, 1, 0, 1)
            hl.setSpacing(8)
            dot = Dot()
            dot.set(level)
            hl.addWidget(dot)
            lbl = label(text)
            lbl.setWordWrap(True)
            hl.addWidget(lbl, 1)
            self.check_box.addWidget(row_w)

    def closeEvent(self, ev) -> None:
        if self.rec_file:
            try:
                self.rec_file.close()
            except OSError:
                pass
        if self.reader is not None:
            self.reader.stop()
        super().closeEvent(ev)


def main() -> int:
    ap = argparse.ArgumentParser(description="Rampa · Diagnostika (standalone USB GUI)")
    ap.add_argument("--port", help="Sériový port (např. COM5 nebo /dev/ttyACM0)")
    ap.add_argument("--demo", action="store_true", help="Spustit v režimu simulace bez hardwaru")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    vs.setup_app(app, "VoltinoLabs.Rampa.Diagnostika")
    win = RampaDiagWindow(port=args.port, demo=args.demo)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
