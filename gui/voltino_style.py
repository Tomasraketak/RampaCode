"""VoltinoLabs design system for the Rampa diagnostic GUI.

Same tokens and style sheet as the Daidalos and Zenith GUIs
(Zenith-flight-computer/gui/voltino_style.py, Daidalos_Landing_Rocket_Code):
dark ground, one amber accent, Inter/Segoe UI, cards with a kicker,
outlined status chips, outline buttons (primary = amber, danger = red).
"""
from __future__ import annotations

import os
import sys

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

# ---------------------------------------------------------------- tokens
C = {
    "bg": "#161826",
    "bg_raised": "#1c1e2b",
    "surface": "#232532",
    "text": "#e9e9ed",
    "muted": "#b2b6ca",
    "subtle": "#75798c",
    "line": "rgba(233, 233, 237, 0.10)",
    "line_strong": "rgba(233, 233, 237, 0.18)",
    "accent": "#f5a51c",
    "accent_soft": "#ffcf7a",
    "accent_strong": "#ffb734",
    "accent_tint": "rgba(245, 165, 28, 0.12)",
    "positive": "#5dd39e",
    "danger": "#f2777a",
    "danger_tint": "rgba(242, 119, 122, 0.14)",
    "cyan": "#4cc9f0",
}

QSS = f"""
QWidget {{ background: {C['bg']}; color: {C['text']}; font-size: 13px; }}
QToolTip {{ background: {C['surface']}; color: {C['text']}; border: 1px solid {C['line_strong']}; padding: 6px; }}
QFrame#card {{ background: {C['bg_raised']}; border: 1px solid {C['line']}; border-radius: 14px; }}
QFrame#card QWidget {{ background: transparent; }}
QFrame#header {{ background: {C['bg']}; border-bottom: 1px solid {C['line']}; }}
QFrame#bar {{ background: {C['bg_raised']}; border-bottom: 1px solid {C['line']}; }}
QLabel#kicker {{ color: {C['accent_soft']}; font-size: 11px; font-weight: 600; letter-spacing: 1.5px; }}
QLabel#title {{ font-size: 22px; font-weight: 500; }}
QLabel#wordAccent {{ font-size: 22px; font-weight: 500; color: {C['accent']}; }}
QLabel#cardTitle {{ font-size: 16px; font-weight: 500; }}
QLabel#muted {{ color: {C['muted']}; }}
QLabel#subtle {{ color: {C['subtle']}; font-size: 12px; }}
QLabel#value {{ color: {C['text']}; font-size: 13px; font-family: "JetBrains Mono", Menlo, Consolas, monospace; }}
QLabel#big {{ font-size: 15px; font-weight: 600; }}
QLabel#chip {{ border: 1px solid {C['line_strong']}; border-radius: 6px; padding: 3px 8px; color: {C['muted']}; font-size: 12px; }}
QLabel#chip[state="ok"]   {{ color: {C['positive']}; border-color: rgba(93, 211, 158, 0.45); }}
QLabel#chip[state="bad"]  {{ color: {C['danger']};   border-color: rgba(242, 119, 122, 0.55); }}
QLabel#chip[state="warn"] {{ color: {C['accent']};   border-color: rgba(245, 165, 28, 0.55); }}
QLabel#chip[state="hot"]  {{ color: {C['bg']}; background: {C['danger']}; border-color: {C['danger']}; font-weight: 600; }}
QLabel#dot {{ border-radius: 6px; background: {C['surface']}; border: 1px solid {C['line_strong']}; }}
QLabel#dot[state="ok"]   {{ background: {C['positive']}; border-color: {C['positive']}; }}
QLabel#dot[state="bad"]  {{ background: {C['danger']}; border-color: {C['danger']}; }}
QLabel#dot[state="warn"] {{ background: {C['accent']}; border-color: {C['accent']}; }}

QPushButton {{ background: transparent; color: {C['muted']}; border: 1px solid {C['line_strong']};
              border-radius: 8px; padding: 6px 12px; font-weight: 500; }}
QPushButton:hover {{ color: {C['text']}; border-color: rgba(233, 233, 237, 0.35); }}
QPushButton:focus {{ border: 2px solid {C['accent']}; padding: 5px 11px; }}
QPushButton:disabled {{ color: rgba(178, 182, 202, 0.45); border-color: rgba(233, 233, 237, 0.08); }}
QPushButton[kind="primary"] {{ color: {C['accent']}; border-color: {C['accent']}; }}
QPushButton[kind="primary"]:hover {{ background: {C['accent_tint']}; }}
QPushButton[kind="danger"] {{ color: {C['danger']}; border-color: {C['danger']}; }}
QPushButton[kind="danger"]:hover {{ background: {C['danger_tint']}; }}
QPushButton[size="big"] {{ padding: 10px 14px; font-size: 14px; }}
QPushButton[size="big"]:focus {{ padding: 9px 13px; }}
QPushButton[kind="primary"]:checked {{ background: {C['accent_tint']}; }}
QPushButton[kind="danger"]:disabled {{ color: rgba(242, 119, 122, 0.4); border-color: rgba(242, 119, 122, 0.2); }}
QPushButton[kind="primary"]:disabled {{ color: rgba(245, 165, 28, 0.45); border-color: rgba(245, 165, 28, 0.25); }}

QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{ background: {C['surface']}; color: {C['text']};
    border: 1px solid {C['line_strong']}; border-radius: 6px; padding: 4px 6px; min-height: 20px;
    selection-background-color: {C['accent']}; selection-color: {C['bg']}; }}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{ border: 1px solid {C['accent']}; }}
QComboBox QAbstractItemView {{ background: {C['surface']}; border: 1px solid {C['line_strong']}; }}
QSpinBox::up-button, QSpinBox::down-button {{ width: 14px; border: none; }}
QCheckBox {{ color: {C['muted']}; spacing: 6px; background: transparent; }}
QCheckBox::indicator {{ width: 14px; height: 14px; border: 1px solid {C['line_strong']}; border-radius: 4px; background: {C['surface']}; }}
QCheckBox::indicator:checked {{ background: {C['accent']}; border-color: {C['accent']}; }}

QPlainTextEdit {{ background: {C['surface']}; color: {C['muted']}; border: 1px solid {C['line']}; border-radius: 8px;
    font-family: "JetBrains Mono", Menlo, Consolas, monospace; font-size: 12px; }}
QListWidget {{ background: transparent; border: none; }}
QListWidget::item {{ padding: 2px 0; }}
QProgressBar {{ background: {C['surface']}; border: 1px solid {C['line']}; border-radius: 6px; text-align: center;
               color: {C['muted']}; min-height: 16px; }}
QProgressBar::chunk {{ background: rgba(245, 165, 28, 0.55); border-radius: 5px; }}
QScrollArea {{ border: none; }}
QScrollBar:vertical {{ background: {C['bg']}; width: 10px; }}
QScrollBar::handle:vertical {{ background: {C['surface']}; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
QMessageBox QLabel {{ color: {C['text']}; }}
QTabWidget::pane {{ border: none; border-top: 1px solid {C['line']}; }}
QTabBar::tab {{ background: transparent; padding: 8px 18px; color: {C['muted']}; border: none; }}
QTabBar::tab:hover {{ color: {C['text']}; }}
QTabBar::tab:selected {{ color: {C['accent']}; border-bottom: 2px solid {C['accent']}; }}
"""


def cz(x: float, nd: int = 1, sign: bool = False) -> str:
    """Czech number: decimal comma, real minus sign."""
    s = f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"
    return s.replace(".", ",").replace("-", "−")


def repolish(w: QtWidgets.QWidget) -> None:
    w.style().unpolish(w)
    w.style().polish(w)


def label(text: str = "", name: str = "") -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    if name:
        lab.setObjectName(name)
    return lab


def button(text: str, kind: str = "", tip: str = "", big: bool = False) -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if big:
        b.setProperty("size", "big")
    if tip:
        b.setToolTip(tip)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    return b


_STATE = {True: "ok", False: "bad", None: "", "ok": "ok", "bad": "bad", "warn": "warn", "info": "warn", "hot": "hot"}


class Chip(QtWidgets.QLabel):
    """VoltinoLabs status chip: outlined, coloured text. set(True|False|None|'warn'|'hot', text)."""

    def __init__(self, text: str = "", state=None):
        super().__init__(text)
        self.setObjectName("chip")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setProperty("state", _STATE.get(state, ""))

    def set(self, state, text: str | None = None):
        if text is not None and self.text() != text:
            self.setText(text)
        st = _STATE.get(state, "")
        if self.property("state") != st:
            self.setProperty("state", st)
            repolish(self)


class Dot(Chip):
    """12 px status light for check lists."""

    def __init__(self):
        super().__init__("")
        self.setObjectName("dot")
        self.setFixedSize(12, 12)


class Card(QtWidgets.QFrame):
    """Card with an amber kicker and a title."""

    def __init__(self, kicker: str, title: str = "", body: bool = False):
        super().__init__()
        self.setObjectName("card")
        self.lay = QtWidgets.QVBoxLayout(self)
        self.lay.setContentsMargins(16, 14, 16, 16)
        self.lay.setSpacing(10)
        head = QtWidgets.QHBoxLayout()
        col = QtWidgets.QVBoxLayout()
        col.setSpacing(2)
        col.addWidget(label(kicker.upper(), "kicker"))
        if title:
            col.addWidget(label(title, "cardTitle"))
        head.addLayout(col)
        head.addStretch(1)
        self.head = head
        self.lay.addLayout(head)
        if body:
            self.body = QtWidgets.QWidget()
            self.lay.addWidget(self.body, 1)


def pick_font(app: QtWidgets.QApplication) -> None:
    fams = set(QtGui.QFontDatabase.families())
    for name in ("Inter", "Inter Variable", "Segoe UI", "Helvetica Neue", "Noto Sans", "DejaVu Sans"):
        if name in fams:
            f = QtGui.QFont(name)
            f.setPixelSize(13)
            app.setFont(f)
            return


def set_windows_app_id(app_id: str) -> None:
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(app_id)
    except Exception:
        pass


def app_icon() -> QtGui.QIcon:
    """tools/windows/rampa.ico (taskbar/window icon, same as the desktop shortcut)."""
    here = os.path.dirname(os.path.abspath(__file__))
    for name in ("rampa.ico", "rampa.png"):
        path = os.path.join(here, "..", "tools", "windows", name)
        if os.path.isfile(path):
            return QtGui.QIcon(path)
    return QtGui.QIcon()


def setup_app(app: QtWidgets.QApplication, app_id: str) -> None:
    set_windows_app_id(app_id)
    icon = app_icon()
    if not icon.isNull():
        app.setWindowIcon(icon)
    QtCore.QLocale.setDefault(QtCore.QLocale(QtCore.QLocale.Language.Czech, QtCore.QLocale.Country.CzechRepublic))
    pick_font(app)
    app.setStyleSheet(QSS)
