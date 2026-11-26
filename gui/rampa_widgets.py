"""Widgets and serial/demo transport for Rampa · Diagnostika.

Follows the VoltinoLabs conventions of Zenith (gui/zenith_widgets.py,
gui/zenith_common.py) and Daidalos (Ariadna_V2/Diagnostics/daidalos_diag.py).
"""
from __future__ import annotations

from collections import deque
import math
import time

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

import voltino_style as vs
from voltino_style import C, Chip, label
import rampa_proto as rp


GOOD, WARN, BAD, DIM = C["positive"], C["accent"], C["danger"], C["subtle"]


# ---------------------------------------------------------------- helpers
class KV(QtWidgets.QWidget):
    """A two-column grid of name : value labels (same as Zenith / Daidalos)."""

    def __init__(self, names: list[str]):
        super().__init__()
        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(4)
        self.v: dict[str, QtWidgets.QLabel] = {}
        for row, n in enumerate(names):
            g.addWidget(label(n, "muted"), row, 0)
            val = label("—", "value")
            val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            g.addWidget(val, row, 1)
            self.v[n] = val
        g.setColumnStretch(1, 1)

    def set(self, name: str, text: str, color: str | None = None) -> None:
        w = self.v.get(name)
        if w is None:
            return
        if w.text() != text:
            w.setText(text)
        st = f"color: {color};" if color else ""
        if w.styleSheet() != st:
            w.setStyleSheet(st)

    def clear(self) -> None:
        for n in self.v:
            self.set(n, "—", DIM)


class Flags(QtWidgets.QWidget):
    """A grid of status chips, one per bit (wraps cleanly inside narrow cards)."""

    def __init__(self, bits_: list[tuple[int, str]], bad_mode: bool = False, cols: int = 4):
        super().__init__()
        self.bad_mode = bad_mode
        lay = QtWidgets.QGridLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(5)
        lay.setVerticalSpacing(5)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.Fixed)
        self.bits = bits_
        self.chips = [Chip(n) for _, n in bits_]
        for idx, c_ in enumerate(self.chips):
            lay.addWidget(c_, idx // cols, idx % cols)
        for c in range(cols):
            lay.setColumnStretch(c, 1)

    def set(self, mask: int | None, bad_when_off: tuple[int, ...] = ()) -> None:
        for (bit, name), c_ in zip(self.bits, self.chips):
            if mask is None:
                c_.set("", name)
            elif mask & (1 << bit):
                c_.set("bad" if self.bad_mode else "ok", name)
            else:
                c_.set("bad" if bit in bad_when_off else "", name)


class HoldButton(QtWidgets.QPushButton):
    """Fires only after being held down for `hold_ms`; shows fill progress."""
    held = QtCore.pyqtSignal()

    def __init__(self, text: str, hold_ms: int = 1200):
        super().__init__(text)
        self.hold_ms = hold_ms
        self.setProperty("kind", "danger")
        self._t0 = 0.0
        self._prog = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(f"Držte {hold_ms / 1000:.1f} s".replace(".", ","))

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self._t0 = time.monotonic()
            self._timer.start()
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        self._cancel()
        super().mouseReleaseEvent(e)

    def leaveEvent(self, e):
        self._cancel()
        super().leaveEvent(e)

    def changeEvent(self, e):
        if e.type() == QtCore.QEvent.Type.EnabledChange and not self.isEnabled():
            self._cancel()
        super().changeEvent(e)

    def _cancel(self):
        self._timer.stop()
        self._prog = 0.0
        self.update()

    def _tick(self):
        self._prog = (time.monotonic() - self._t0) * 1000 / self.hold_ms
        if self._prog >= 1.0:
            self._cancel()
            self.held.emit()
        self.update()

    def paintEvent(self, e):
        super().paintEvent(e)
        if self._prog > 0:
            p = QtGui.QPainter(self)
            col = QtGui.QColor(C["danger"])
            col.setAlpha(75)
            r = QtCore.QRectF(self.rect()).adjusted(1, 1, -1, -1)
            r.setWidth(r.width() * min(1.0, self._prog))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(col)
            p.drawRoundedRect(r, 7, 7)


# ---------------------------------------------------------------- StripChart
class StripChart(QtWidgets.QWidget):
    """Rolling time-series chart styled for VoltinoLabs (like zenith_widgets.StripChart)."""

    COLORS = (C["accent"], C["positive"], C["cyan"], C["danger"])

    def __init__(
        self,
        title: str,
        names: list[str],
        unit: str = "",
        span_s: float = 60.0,
        threshold: tuple[float, str] | None = None,
    ):
        super().__init__()
        self.title = title
        self.names = names
        self.unit = unit
        self.span = span_s
        self.threshold = threshold  # optional (value, label) dashed reference line
        self.data: list[deque[tuple[float, float]]] = [deque(maxlen=1500) for _ in names]
        self.setMinimumHeight(180)

    def add(self, t: float, *values: float | None) -> None:
        for i, v in enumerate(values):
            if i < len(self.data) and v is not None and math.isfinite(v):
                self.data[i].append((t, float(v)))
                cutoff = t - self.span * 1.15
                while self.data[i] and self.data[i][0][0] < cutoff:
                    self.data[i].popleft()
        self.update()

    def clear(self) -> None:
        for d in self.data:
            d.clear()
        self.update()

    @staticmethod
    def nice_ticks(lo: float, hi: float, n: int) -> tuple[float, list[float]]:
        span = max(hi - lo, 1e-9)
        raw = span / max(n, 1)
        mag = 10.0 ** math.floor(math.log10(raw))
        for m in (1.0, 2.0, 2.5, 5.0, 10.0):
            if m * mag >= raw:
                step = m * mag
                break
        else:
            step = 10.0 * mag
        start = math.ceil(lo / step - 1e-9) * step
        ticks: list[float] = []
        v = start
        while v <= hi + step * 1e-6 and len(ticks) < 20:
            ticks.append(v)
            v += step
        return step, ticks

    @staticmethod
    def fmt_tick(v: float, step: float) -> str:
        nd = 0 if step >= 1 else max(1, min(3, -int(math.floor(math.log10(max(step, 1e-6))))))
        return vs.cz(v, nd)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QtGui.QColor(C["bg_raised"]))

        f = p.font()
        f.setPixelSize(11)
        p.setFont(f)
        fm = QtGui.QFontMetrics(f)

        allv = [v for buf in self.data for _, v in buf]
        if self.threshold is not None and allv:
            allv.append(self.threshold[0])
        if allv:
            lo, hi = min(allv), max(allv)
            if hi - lo < 1e-6:
                lo, hi = lo - 1.0, hi + 1.0
            pad = (hi - lo) * 0.1
            lo, hi = lo - pad, hi + pad
            step, ticks = self.nice_ticks(lo, hi, 4)
            labels = [self.fmt_tick(v, step) for v in ticks]
            left = max(44, max((fm.horizontalAdvance(t) for t in labels), default=36) + 12)
        else:
            lo, hi, step, ticks, labels = 0.0, 1.0, 0.5, [], []
            left = 48

        top, right, bottom = 24, 12, 22
        pw, ph = max(20, w - left - right), max(20, h - top - bottom)

        # header + legend
        p.setPen(QtGui.QColor(C["text"]))
        p.drawText(
            QtCore.QRectF(8, 3, w - 16, 18),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            f"{self.title}" + (f" [{self.unit}]" if self.unit else ""),
        )
        x = w - right
        for i in reversed(range(len(self.names))):
            last = self.data[i][-1][1] if self.data[i] else None
            txt = "■ " + self.names[i] + ("" if last is None else " " + self.fmt_tick(last, step / 10 if allv else 0.1))
            tw = fm.horizontalAdvance(txt) + 10
            x -= tw
            p.setPen(QtGui.QColor(self.COLORS[i % len(self.COLORS)]))
            p.drawText(QtCore.QRectF(x, 3, tw, 18), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, txt)

        frame = QtCore.QRectF(left, top, pw, ph)
        p.fillRect(frame, QtGui.QColor(C["surface"]))
        grid = QtGui.QPen(QtGui.QColor(255, 255, 255, 22), 1)
        axis_txt = QtGui.QColor(C["subtle"])

        tstep, tticks = self.nice_ticks(-self.span, 0.0, max(2, int(pw / 80)))
        for tv in tticks:
            px = left + pw * (tv + self.span) / self.span
            p.setPen(grid)
            p.drawLine(QtCore.QPointF(px, top), QtCore.QPointF(px, top + ph))
            p.setPen(axis_txt)
            is_now = abs(tv) < 1e-9
            p.drawText(
                QtCore.QRectF(px - (48 if is_now else 28), top + ph + 3, 56, 16),
                Qt.AlignmentFlag.AlignRight if is_now else Qt.AlignmentFlag.AlignHCenter,
                "teď" if is_now else self.fmt_tick(tv, tstep) + " s",
            )

        if not allv:
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 35), 1))
            p.drawRect(frame)
            p.setPen(axis_txt)
            p.drawText(frame, Qt.AlignmentFlag.AlignCenter, "žádná data")
            p.end()
            return

        for v, txt in zip(ticks, labels):
            py = top + ph * (hi - v) / (hi - lo)
            zero = abs(v) < step * 1e-6
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 65 if zero else 22), 1))
            p.drawLine(QtCore.QPointF(left, py), QtCore.QPointF(left + pw, py))
            p.setPen(axis_txt)
            p.drawText(
                QtCore.QRectF(0, py - 8, left - 6, 16),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                txt,
            )

        # optional threshold dashed line
        if self.threshold is not None:
            tv, tlbl = self.threshold
            if lo <= tv <= hi:
                py = top + ph * (hi - tv) / (hi - lo)
                pen = QtGui.QPen(QtGui.QColor(C["danger"]), 1, Qt.PenStyle.DashLine)
                p.setPen(pen)
                p.drawLine(QtCore.QPointF(left, py), QtCore.QPointF(left + pw, py))
                p.drawText(QtCore.QRectF(left + 6, py - 15, 160, 14), Qt.AlignmentFlag.AlignLeft, tlbl)

        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 38), 1))
        p.drawRect(frame)

        t1 = max(buf[-1][0] for buf in self.data if buf)
        t0 = t1 - self.span
        p.save()
        p.setClipRect(frame)
        for i, buf in enumerate(self.data):
            if len(buf) < 2:
                continue
            path = QtGui.QPainterPath()
            first = True
            for t, v in buf:
                px = left + pw * (t - t0) / self.span
                py = top + ph * (hi - v) / (hi - lo)
                if first:
                    path.moveTo(px, py)
                    first = False
                else:
                    path.lineTo(px, py)
            p.setPen(QtGui.QPen(QtGui.QColor(self.COLORS[i % len(self.COLORS)]), 1.8))
            p.drawPath(path)
        p.restore()
        p.end()


# ---------------------------------------------------------------- PadDiagramWidget
class PadDiagramWidget(QtWidgets.QWidget):
    """Live hardware block diagram of Rampa PCB1 (power, interlocks, pyro, pogo, LoRa, LEDs)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.s: dict = {}
        self.setMinimumSize(520, 250)

    def set_state(self, s: dict) -> None:
        self.s = dict(s) if s else {}
        self.update()

    def _box(self, p: QtGui.QPainter, rect: QtCore.QRectF, title: str, lines: list[str], border_col: str, fill_col: str | None = None):
        p.setPen(QtGui.QPen(QtGui.QColor(border_col), 1.4))
        p.setBrush(QtGui.QColor(fill_col or C["surface"]))
        p.drawRoundedRect(rect, 9, 9)
        f = p.font()
        f.setPixelSize(11)
        f.setWeight(QtGui.QFont.Weight.DemiBold)
        p.setFont(f)
        p.setPen(QtGui.QColor(border_col))
        p.drawText(rect.adjusted(9, 6, -8, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop, title)
        f.setWeight(QtGui.QFont.Weight.Normal)
        f.setPixelSize(11)
        p.setFont(f)
        p.setPen(QtGui.QColor(C["text"]))
        y = rect.top() + 24
        for ln in lines:
            p.drawText(QtCore.QRectF(rect.left() + 9, y, rect.width() - 16, 16), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, ln)
            y += 16

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QtGui.QColor(C["bg_raised"]))

        s = self.s
        online = bool(s)
        vbat = s.get("vbat_mv", 0)
        v5 = s.get("v5_mv", -1)
        key = bool(s.get("key"))
        btn = bool(s.get("btn"))
        arm = bool(s.get("pyro_arm"))
        fire_mask = s.get("fire_mask", 0)
        conts = [s.get("c1_mv", 0), s.get("c2_mv", 0), s.get("c3_mv", 0)]
        proto = s.get("proto", 0)
        umb_on = bool(s.get("flags", 0) & (1 << 6))
        umb_ma = s.get("umb_ma", 0)
        umb_trip = bool(s.get("umb_trip"))
        lora_on = bool(s.get("lora_on"))
        exp_out = s.get("exp_out", 0)
        state = s.get("state", 0)

        mx, my = 12.0, 10.0
        col_w = (w - mx * 4) / 3.0
        row_h = (h - my * 3) / 2.0

        # 1. Battery & Power
        bat_col = DIM if not online else (GOOD if vbat >= rp.VBAT_MIN_MV else BAD)
        v5_txt = f"5V větev: {vs.cz(v5 / 1000, 2)} V" if v5 >= 0 else "5V větev: —"
        bat_lines = [
            f"VBAT: {vs.cz(vbat / 1000, 2)} V" if online else "VBAT: —",
            f"Proud: {s.get('ibat_ma')} mA" if online and s.get("ibat_ma", -1) >= 0 else "Proud: —",
            v5_txt,
        ]
        r_bat = QtCore.QRectF(mx, my, col_w, row_h)
        self._box(p, r_bat, "NAPÁJENÍ (2S + 5V)", bat_lines, bat_col)

        # 2. Safety Interlocks (Key + Abort Button)
        int_col = DIM if not online else (BAD if btn else (GOOD if key else WARN))
        int_lines = [
            f"Klíč J4: {'SEPNUT (ARMED)' if key else 'VYPNUT (SAFE)'}" if online else "Klíč J4: —",
            f"Tlač. SW1: {'STISKNUTO!' if btn else 'v klidu'}" if online else "Tlač. SW1: —",
            f"Lišta GP19: {'ZAPNUTA' if arm else 'vypnuta'}" if online else "Lišta GP19: —",
        ]
        r_int = QtCore.QRectF(mx, my * 2 + row_h, col_w, row_h)
        self._box(p, r_int, "BEZPEČNOSTNÍ ŘETĚZEC", int_lines, int_col)

        # 3. Pyro Channels (center, full height)
        pyro_col = DIM if not online else (BAD if fire_mask else (WARN if arm else C["line_strong"]))
        r_pyro = QtCore.QRectF(mx * 2 + col_w, my, col_w, h - my * 2)
        p.setPen(QtGui.QPen(QtGui.QColor(pyro_col), 1.6 if (arm or fire_mask) else 1.2))
        p.setBrush(QtGui.QColor(C["surface"]))
        p.drawRoundedRect(r_pyro, 9, 9)
        f = p.font()
        f.setPixelSize(11)
        f.setWeight(QtGui.QFont.Weight.DemiBold)
        p.setFont(f)
        p.setPen(QtGui.QColor(BAD if fire_mask else (WARN if arm else C["accent_soft"])))
        p.drawText(r_pyro.adjusted(10, 7, -10, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop, "PYRO KANÁLY (J5 · J6 · J7)")

        ch_h = (r_pyro.height() - 42) / 3.0
        for i in range(3):
            cy = r_pyro.top() + 30 + i * ch_h
            mv = conts[i]
            ok = mv >= rp.CONT_MIN_MV
            firing = bool(fire_mask & (1 << i))
            sel = (s.get("pyro_ch", 1) == i + 1) and online
            f.setWeight(QtGui.QFont.Weight.Normal)
            p.setFont(f)
            lbl = f"CH{i + 1} (J{5 + i})" + (" ★" if sel else "")
            p.setPen(QtGui.QColor(BAD if firing else (GOOD if ok else C["muted"])))
            p.drawText(QtCore.QRectF(r_pyro.left() + 10, cy, r_pyro.width() - 20, 16), Qt.AlignmentFlag.AlignLeft, lbl)
            st_txt = "ZÁŽEH!" if firing else (f"{mv} mV · OK" if online and ok else (f"{mv} mV · —" if online else "—"))
            p.drawText(QtCore.QRectF(r_pyro.left() + 10, cy, r_pyro.width() - 20, 16), Qt.AlignmentFlag.AlignRight, st_txt)

            # continuity progress bar
            bar = QtCore.QRectF(r_pyro.left() + 10, cy + 19, r_pyro.width() - 20, 10)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor(C["bg"]))
            p.drawRoundedRect(bar, 4, 4)
            frac = min(1.0, max(0.0, mv / 2000.0)) if online else 0.0
            fill = QtCore.QRectF(bar.left(), bar.top(), bar.width() * frac, bar.height())
            p.setBrush(QtGui.QColor(BAD if firing else (GOOD if ok else C["subtle"])))
            p.drawRoundedRect(fill, 4, 4)
            # threshold tick at 350 mV
            tx = bar.left() + bar.width() * (rp.CONT_MIN_MV / 2000.0)
            p.setPen(QtGui.QPen(QtGui.QColor(C["accent"]), 1.2))
            p.drawLine(QtCore.QPointF(tx, bar.top() - 2), QtCore.QPointF(tx, bar.bottom() + 2))

        # 4. Pogo Umbilical J3
        umb_col = DIM if not online else (BAD if umb_trip else (GOOD if proto != 0 else C["cyan"]))
        proto_short = {0: "Hledá raketu", 1: "Zenith (UART)", 2: "Daidalos (I²C)"}.get(proto, "?")
        umb_lines = [
            f"Raketa: {proto_short}" if online else "Raketa: —",
            f"UMB_5V: {'ZAP' if umb_on else 'VYP'} ({umb_ma} mA)" if online else "UMB_5V: —",
            f"Nadproud: {'ODPOJENO!' if umb_trip else 'OK'}" if online else "Nadproud: —",
        ]
        r_umb = QtCore.QRectF(mx * 3 + col_w * 2, my, col_w, row_h)
        self._box(p, r_umb, "POGO UMBILICAL (J3)", umb_lines, umb_col)

        # 5. LoRa & Annunciators
        lora_col = DIM if not online else (GOOD if lora_on else C["muted"])
        fire_led = bool(exp_out & (1 << 6))
        buz = bool(exp_out & (1 << 0))
        lora_lines = [
            f"LoRa 433: {'ZAP (beacon)' if lora_on else 'VYP'}" if online else "LoRa 433: —",
            f"TX / RX: {s.get('lora_tx', 0)} / {s.get('lora_rx', 0)}" if online else "TX / RX: —",
            f"Světlo: {'ZAP' if fire_led else 'vyp'} · Bzučák: {'ZAP' if buz else 'vyp'}" if online else "Signalizace: —",
        ]
        r_lora = QtCore.QRectF(mx * 3 + col_w * 2, my * 2 + row_h, col_w, row_h)
        self._box(p, r_lora, "LORA & SIGNALIZACE", lora_lines, lora_col)

        # RGB indicator dot inside LoRa/Annunciators box corner
        rgb_colors = {
            1: "#3b82f6" if proto != 2 else "#06b6d4",
            2: "#22c55e",
            3: "#f59e0b",
            4: "#ef4444",
            5: "#ef4444",
            6: "#d946ef",
            7: "#eab308",
        }
        rgb_hex = rgb_colors.get(state, C["subtle"]) if online else C["subtle"]
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 90), 1))
        p.setBrush(QtGui.QColor(rgb_hex))
        p.drawEllipse(QtCore.QPointF(r_lora.right() - 16, r_lora.top() + 14), 6, 6)

        # Connecting arrows
        pen = QtGui.QPen(QtGui.QColor(255, 255, 255, 45), 1.4)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(r_bat.center().x(), r_bat.bottom()), QtCore.QPointF(r_int.center().x(), r_int.top()))
        p.drawLine(QtCore.QPointF(r_int.right(), r_int.center().y()), QtCore.QPointF(r_pyro.left(), r_int.center().y()))
        p.drawLine(QtCore.QPointF(r_pyro.right(), r_umb.center().y()), QtCore.QPointF(r_umb.left(), r_umb.center().y()))
        p.end()


# ---------------------------------------------------------------- Serial & Demo transport
class SerialReader(QtCore.QThread):
    """Reads text lines from a serial port in a thread; auto-reconnects on unplug."""
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port, self.baud = port, baud
        self._ser = None
        self._run = True
        self._tx: deque[str] = deque()

    def send(self, text: str) -> None:
        self._tx.append(text)

    def stop(self) -> None:
        self._run = False
        self.wait(1500)

    def run(self) -> None:
        if serial is None:
            self.status.emit("pyserial není nainstalován (pip install pyserial)")
            return
        while self._run:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
                self._ser.dtr = True
                self.status.emit(f"připojeno {self.port}")
                buf = b""
                while self._run:
                    while self._tx:
                        self._ser.write((self._tx.popleft() + "\n").encode("ascii", errors="replace"))
                    chunk = self._ser.read(4096)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        s = raw.decode(errors="replace").strip()
                        if s:
                            self.line.emit(s)
            except Exception as ex:
                self.status.emit(f"odpojeno: {ex}")
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                for _ in range(20):
                    if not self._run:
                        break
                    time.sleep(0.05)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


class DemoReader(QtCore.QObject):
    """In-process simulator using RampaSimCore; same interface as SerialReader."""
    line = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.sim = rp.RampaSimCore()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._tick)

    def start(self) -> None:
        self.status.emit("připojeno Simulace")
        self._timer.start()

    def stop(self) -> None:
        self._timer.stop()

    def send(self, text: str) -> None:
        for out in self.sim.handle_command(text):
            self.line.emit(out)

    def _tick(self) -> None:
        if self.sim.stream:
            self.line.emit(self.sim.status_line())


def list_ports() -> list[tuple[str, str]]:
    """Return [(device, display_label), ...] with USB/Pico ports sorted first."""
    if serial is None:
        return []
    ports = list(serial.tools.list_ports.comports())

    def score(p) -> tuple[int, str]:
        desc = f"{p.description or ''} {p.manufacturer or ''}".lower()
        usb = 0 if ("pico" in desc or "rp2" in desc or "usb" in desc or "serial" in desc) else 1
        return (usb, p.device)

    ports.sort(key=score)
    out = []
    for p in ports:
        desc = (p.description or "").strip()
        lbl = f"{p.device} — {desc}" if desc and desc != "n/a" else p.device
        out.append((p.device, lbl))
    return out
