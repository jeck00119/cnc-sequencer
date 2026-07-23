"""
CNC Sequencer
=============
A simple standalone CNC control GUI: jog the machine, save named positions,
and run those positions as a sequence — N times in a loop (or forever).

XY-only CoreXY machine (Marlin, sensorless homing), with a live camera view
(V4L2 webcam or Basler via pypylon) and per-position photo capture. Serial
layer and move-wait pattern reused from the proven pcb-projector-overlay
project.

Usage: python3 cnc_sequencer.py
"""

import sys
import os
import math
import json
import glob
import time
import threading
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QLineEdit, QCheckBox, QFrame, QScrollArea,
    QPlainTextEdit, QSizePolicy, QComboBox,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap

from serial_worker import SerialWorker
from sequence_runner import SequenceRunner

# Camera is optional: if OpenCV isn't installed the CNC features still work,
# the camera pane just shows "OpenCV not available".
try:
    import cv2
    from camera import CameraController
    _HAS_CAMERA = True
    _CAMERA_ERR = ""
except Exception as _e:
    cv2 = None
    CameraController = None
    _HAS_CAMERA = False
    _CAMERA_ERR = str(_e)

# Basler support is optional on top of that (needs pypylon).
try:
    from basler_camera import BaslerController, _HAS_PYLON, _PYLON_ERR
except Exception as _be:
    BaslerController = None
    _HAS_PYLON = False
    _PYLON_ERR = str(_be)


DEFAULT_PORT = ("/dev/serial/by-id/usb-STMicroelectronics_MARLIN_"
                "STM32G0B1RE_CDC_in_FS_Mode_20793673594D-if00")
DATA_FILE = Path(__file__).resolve().parent / "cnc_data.json"
CAPTURE_DIR = Path(__file__).resolve().parent / "captures"
DEFAULT_CAM = "/dev/video0"
CAM_W, CAM_H = 1280, 720

STEP_SIZES = [0.1, 1, 5, 10, 50]      # preset jog steps, mm
CUSTOM_STEP_MIN, CUSTOM_STEP_MAX = 0.01, 200.0   # custom-step field range, mm

STYLE = """
QWidget { background: #14151b; color: #d0d3da; font-size: 13px; }
QLabel { color: #a6aab5; background: transparent; }
QToolTip {
    background: #23252d; color: #d5d8df; border: 1px solid #3a3d48;
    padding: 5px 8px; font-size: 12px;
}
QPushButton {
    background: #262933; color: #d0d3da; border: 1px solid #3a3d48;
    border-radius: 5px; padding: 7px 14px;
}
QPushButton:hover { background: #313540; border-color: #4a4e5a; }
QPushButton:pressed { background: #1d1f26; }
QPushButton:disabled { color: #565a66; background: #1d1f25; border-color: #2a2c34; }
#accent:disabled, #go:disabled, #danger:disabled, #jogHome:disabled,
#stepActive:disabled, #capOn:disabled, #stopBig:disabled {
    color: #565a66; background: #1d1f25; border-color: #2a2c34;
}
QFrame#row QPushButton, QFrame#rowActive QPushButton { padding: 3px 5px; }
#accent { color: #4dc3f0; border-color: #2c657e; font-weight: bold; }
#accent:hover { background: #16232b; border-color: #4dc3f0; }
#go { color: #58c96b; border-color: #2f6a3a; font-weight: bold; }
#go:hover { background: #17281b; border-color: #58c96b; }
#danger { color: #e86060; border-color: #7c3434; font-weight: bold; }
#danger:hover { background: #2b1717; border-color: #e86060; }
#runBtn {
    color: #ffffff; background: #1f6b2f; border: 1px solid #58c96b;
    font-weight: bold; font-size: 13px; border-radius: 6px;
}
#runBtn:hover { background: #268038; }
#runBtn:disabled { color: #565a66; background: #1d1f25; border-color: #2a2c34; }
#stopBig {
    color: #ffffff; background: #8a1f1f; border: 2px solid #e05555;
    border-radius: 7px; font-size: 14px; font-weight: bold; padding: 8px 24px;
}
#stopBig:hover { background: #a62626; }
#stopBig:pressed { background: #6f1515; }
#jogBtn { font-size: 19px; }
#jogHome { font-size: 19px; color: #4dc3f0; border-color: #2c657e; }
#jogHome:hover { background: #16232b; border-color: #4dc3f0; }
#stepActive {
    color: #4dc3f0; border: 1px solid #4dc3f0; background: #16232b;
    font-weight: bold;
}
QLineEdit {
    background: #101116; color: #e2e4ea; border: 1px solid #3a3d48;
    border-radius: 5px; padding: 7px 9px;
    selection-background-color: #2c657e;
}
QLineEdit:focus { border-color: #4dc3f0; }
QLineEdit:disabled { color: #565a66; background: #17181d; }
QComboBox {
    background: #262933; color: #d0d3da; border: 1px solid #3a3d48;
    border-radius: 5px; padding: 7px 9px;
}
QComboBox:hover { border-color: #4a4e5a; }
QComboBox QAbstractItemView {
    background: #23252d; color: #d0d3da; border: 1px solid #3a3d48;
    selection-background-color: #2c657e;
}
QCheckBox { color: #c4c7d0; background: transparent; spacing: 6px; }
QCheckBox:disabled { color: #565a66; }
QCheckBox::indicator {
    width: 18px; height: 18px; border: 1px solid #4a4e5a;
    border-radius: 4px; background: #101116;
}
QCheckBox::indicator:hover { border-color: #4dc3f0; }
QCheckBox::indicator:checked { background: #2c657e; border-color: #4dc3f0; }
QFrame#card { background: #1c1e26; border: 1px solid #2c2f39; border-radius: 8px; }
QFrame#row { background: #22242d; border: 1px solid #2e313c; border-radius: 5px; }
QFrame#rowActive { background: #14303f; border: 1px solid #4dc3f0; border-radius: 5px; }
QPlainTextEdit {
    background: #0e0f14; color: #9aa0a6; border: 1px solid #2c2f39;
    border-radius: 5px; font-family: 'DejaVu Sans Mono', monospace;
    font-size: 12px;
}
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: transparent; width: 9px; }
QScrollBar::handle:vertical { background: #33363f; border-radius: 4px; min-height: 30px; }
QScrollBar::handle:vertical:hover { background: #454956; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { background: transparent; height: 9px; }
QScrollBar::handle:horizontal { background: #33363f; border-radius: 4px; min-width: 30px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
#title { color: #4dc3f0; font-size: 17px; font-weight: bold; }
#posReadout {
    color: #eceef2; font-family: 'DejaVu Sans Mono', monospace;
    font-size: 21px; font-weight: bold;
}
#sectionHdr { color: #7f8494; font-size: 12px; font-weight: bold; }
#statusGood, #statusBad, #statusWait {
    border-radius: 12px; padding: 5px 14px; font-weight: bold; font-size: 12px;
}
#statusGood { color: #58c96b; background: #142a1a; border: 1px solid #2f6a3a; }
#statusBad  { color: #e86060; background: #2a1515; border: 1px solid #7c3434; }
#statusWait { color: #d9a23d; background: #2a2312; border: 1px solid #7c6522; }
#capOn { color: #d9a6f2; border: 1px solid #a35cc4; background: #241a2b; font-weight: bold; }
#camView { background: #000000; color: #5c6270; border: 1px solid #2c2f39; border-radius: 6px; }
#progress { color: #4dc3f0; font-weight: bold; font-size: 12px; }
"""


class CameraView(QLabel):
    """Image view that reports zoom (wheel), pan (left-drag) and double-click,
    so the window can zoom into the full-resolution frame and go fullscreen."""
    wheelZoom = pyqtSignal(int)         # +1 = zoom in, -1 = zoom out
    dragPan = pyqtSignal(int, int)      # dx, dy in view pixels
    doubleClicked = pyqtSignal()

    def __init__(self, text=""):
        super().__init__(text)
        self._drag_pos = None

    def wheelEvent(self, e):
        self.wheelZoom.emit(1 if e.angleDelta().y() > 0 else -1)

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag_pos = e.pos()

    def mouseMoveEvent(self, e):
        if self._drag_pos is not None:
            d = e.pos() - self._drag_pos
            self._drag_pos = e.pos()
            self.dragPan.emit(d.x(), d.y())

    def mouseReleaseEvent(self, e):
        self._drag_pos = None

    def mouseDoubleClickEvent(self, e):
        self.doubleClicked.emit()


class FullscreenPreview(QWidget):
    """Fullscreen window holding a CameraView. ESC or double-click closes it."""
    closed = pyqtSignal()
    resized = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera — Fullscreen (ESC to exit)")
        self.setStyleSheet("background:#000000;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.view = CameraView("")
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setStyleSheet("background:#000000;")
        lay.addWidget(self.view)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.resized.emit()   # let the owner re-fit even a frozen stream

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.close()

    def closeEvent(self, e):
        self.closed.emit()
        e.accept()


class CncSequencer(QWidget):
    # results marshalled from helper threads back onto the GUI thread
    _capture_result = pyqtSignal(bool, str)         # success, path or error
    _cam_open_result = pyqtSignal(bool, str, str)   # success, log note, color

    def __init__(self):
        super().__init__()
        self.setWindowTitle("CNC Sequencer")
        self.resize(1300, 920)
        self.setMinimumSize(1200, 740)

        self.worker = None
        self.runner = None
        self._connected = False
        self.position = (0.0, 0.0)
        self._step = 10.0
        self.positions = []          # list of {name, x, y, enabled, capture}
        self._pos_rows = []          # QFrame per position row (display order)
        self._active_row = -1

        # camera state
        self.camera = None
        self._cam_opening = False
        self._cam_error = False
        self._cam_last_seq = None
        self._cam_last_preview = None    # latest downscaled BGR frame (fit view)
        self._cam_last_full = None       # latest full-res BGR frame (zoom source)
        self._cam_status_shown = None
        self._cam_last_frame_t = 0.0         # monotonic time of newest frame
        self._cam_active_source = "webcam"   # "webcam" | "basler"
        self._source_loading = False
        self._webcam_device = DEFAULT_CAM
        self._basler_serial = ""
        # zoom / pan / fullscreen (view-only; capture always saves full frame)
        self._cam_zoom = 1.0             # 1.0 = fit; >1 = zoomed in
        self._cam_center = [0.5, 0.5]    # crop center as fraction of full frame
        self._fs = None                  # FullscreenPreview while active

        self._build_ui()
        self.setStyleSheet(STYLE)
        self._load_config()
        self._rebuild_positions()
        self._set_ui_state()

        # Camera preview pump — pulls the latest cached frame; never blocks the
        # GUI on a slow/dead device (the grabber thread owns the blocking read).
        # Runs only while a camera is open (started in _on_camera_open).
        self._cam_timer = QTimer(self)
        self._cam_timer.timeout.connect(self._update_camera_view)
        self._capture_result.connect(self._on_manual_capture_result)
        self._cam_open_result.connect(self._on_cam_open_result)

    # ==================================================================
    #  UI construction
    # ==================================================================

    def _card(self):
        f = QFrame()
        f.setObjectName("card")
        return f

    def _hdr(self, num, text, hint=""):
        """Section header: optional accent step number + title + dim hint."""
        parts = []
        if num:
            parts.append(f'<span style="color:#4dc3f0;">{num}&nbsp;·</span> ')
        parts.append(text)
        if hint:
            parts.append('&nbsp; <span style="color:#5c6270;'
                         f' font-weight:normal;">— {hint}</span>')
        lbl = QLabel("".join(parts))
        lbl.setObjectName("sectionHdr")
        lbl.setTextFormat(Qt.RichText)
        return lbl

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # Header bar — status, live position and emergency stop, always visible
        outer.addWidget(self._build_header())

        split = QHBoxLayout()
        split.setSpacing(10)
        outer.addLayout(split, 1)

        # Left column — the machine workflow, top to bottom:
        # ① connect → ② jog → ③ positions scroll; ④ run stays pinned below
        # so the Run/Pause/Stop controls never scroll out of view.
        leftw = QWidget()
        leftw.setMinimumWidth(660)
        leftw.setMaximumWidth(675)
        leftcol = QVBoxLayout(leftw)
        leftcol.setContentsMargins(0, 0, 0, 0)
        leftcol.setSpacing(10)
        split.addWidget(leftw, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        leftcol.addWidget(scroll, 1)
        body = QWidget()
        scroll.setWidget(body)
        root = QVBoxLayout(body)
        root.setContentsMargins(0, 0, 4, 0)
        root.setSpacing(10)

        root.addWidget(self._build_connection())
        root.addWidget(self._build_jog())
        root.addWidget(self._build_saved())
        root.addStretch()

        leftcol.addWidget(self._build_sequence())

        # Right column — camera on top, console (log + raw G-code) below
        right = QVBoxLayout()
        right.setSpacing(10)
        split.addLayout(right, 1)
        right.addWidget(self._build_camera(), 1)
        right.addWidget(self._build_log())

    def _build_header(self):
        bar = self._card()
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 10, 10, 10)
        lay.setSpacing(16)

        title = QLabel("CNC SEQUENCER")
        title.setObjectName("title")
        lay.addWidget(title)

        self._status = QLabel("● DISCONNECTED")
        self._status.setObjectName("statusBad")
        lay.addWidget(self._status)

        lay.addStretch(1)

        self._pos_label = QLabel("X: 0.00    Y: 0.00")
        self._pos_label.setObjectName("posReadout")
        self._pos_label.setToolTip("Current machine position (from M114)")
        lay.addWidget(self._pos_label)

        lay.addStretch(1)

        self._stop_big = QPushButton("STOP")
        self._stop_big.setObjectName("stopBig")
        self._stop_big.setMinimumSize(170, 48)
        self._stop_big.setToolTip(
            "EMERGENCY STOP (M410) — instantly halts all motion and aborts "
            "a running sequence")
        self._stop_big.clicked.connect(self._on_emergency_stop)
        lay.addWidget(self._stop_big)
        return bar

    def _set_status(self, text, kind):
        """Update the header status pill. kind: 'good' | 'bad' | 'wait'."""
        self._status.setText("● " + text)
        self._status.setObjectName(
            {"good": "statusGood", "bad": "statusBad",
             "wait": "statusWait"}[kind])
        self._refresh_style(self._status)

    def _build_connection(self):
        card = self._card()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)
        lay.addWidget(self._hdr("1", "CONNECT"))

        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(QLabel("Port:"))
        self._port = QLineEdit(DEFAULT_PORT)
        self._port.setToolTip("Serial port of the CNC board (115200 baud)")
        row.addWidget(self._port, 1)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        row2.setSpacing(8)
        self._detect_btn = QPushButton("Auto-detect")
        self._detect_btn.setToolTip(
            "Scan /dev/serial/by-id and /dev/ttyACM* for the board")
        self._detect_btn.clicked.connect(self._on_autodetect)
        row2.addWidget(self._detect_btn)
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setObjectName("accent")
        self._connect_btn.setToolTip(
            "Open the port and boot the machine — USB reset + auto-home ×3, "
            "takes ~15–20 s (watch the console)")
        self._connect_btn.clicked.connect(self._on_connect)
        row2.addWidget(self._connect_btn, 1)
        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.setToolTip(
            "Close the serial connection (also cancels a connect in progress)")
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        row2.addWidget(self._disconnect_btn)
        lay.addLayout(row2)
        return card

    def _build_jog(self):
        card = self._card()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(10)
        lay.addWidget(self._hdr("2", "JOG", "move the machine manually"))

        mid = QHBoxLayout()
        mid.setSpacing(22)

        # D-pad with Home in the center
        grid = QGridLayout()
        grid.setSpacing(8)
        arrows = [("▲", 0, 1, "Y", 1, "Jog Y + one step"),
                  ("▼", 2, 1, "Y", -1, "Jog Y − one step"),
                  ("◀", 1, 0, "X", -1, "Jog X − one step"),
                  ("▶", 1, 2, "X", 1, "Jog X + one step")]
        self._jog_btns = []
        for text, r, c, axis, sign, tip in arrows:
            b = QPushButton(text)
            b.setObjectName("jogBtn")
            b.setFixedSize(58, 58)
            b.setToolTip(tip)
            b.clicked.connect(
                lambda _=False, a=axis, s=sign: self._on_jog(a, s))
            grid.addWidget(b, r, c)
            self._jog_btns.append(b)
        self._home_btn = QPushButton("⌂")
        self._home_btn.setObjectName("jogHome")
        self._home_btn.setFixedSize(58, 58)
        self._home_btn.setToolTip("Home XY (G28) — re-find machine zero")
        self._home_btn.clicked.connect(self._on_home)
        grid.addWidget(self._home_btn, 1, 1)
        mid.addLayout(grid)

        # Step size + feed rate
        right = QVBoxLayout()
        right.setSpacing(8)
        right.addWidget(QLabel("Step size (mm):"))
        step_row = QHBoxLayout()
        step_row.setSpacing(6)
        self._step_btns = []
        for val in STEP_SIZES:
            sb = QPushButton(str(val))
            sb.setFixedSize(58, 34)
            sb.setToolTip("Distance moved per arrow click")
            sb.clicked.connect(lambda _=False, v=val: self._on_step(v))
            step_row.addWidget(sb)
            self._step_btns.append((sb, val))
        self._step_custom = QLineEdit()
        self._step_custom.setFixedSize(76, 34)
        self._step_custom.setAlignment(Qt.AlignCenter)
        self._step_custom.setMaxLength(6)
        self._step_custom.setPlaceholderText("custom")
        self._step_custom.setToolTip(
            f"Custom jog step in mm ({CUSTOM_STEP_MIN:g}–{CUSTOM_STEP_MAX:g})"
            " — type a value and it becomes the active step"
            " ('.' or ',' decimals both work)")
        self._step_custom.textEdited.connect(self._on_step_custom)
        step_row.addWidget(self._step_custom)
        step_row.addStretch()
        right.addLayout(step_row)

        feed_row = QHBoxLayout()
        feed_row.setSpacing(8)
        feed_row.addWidget(QLabel("Feed rate:"))
        self._feed = QLineEdit("3000")
        self._feed.setFixedWidth(84)
        self._feed.setToolTip("Speed for jogs and sequence moves (mm/min)")
        feed_row.addWidget(self._feed)
        feed_row.addWidget(QLabel("mm/min"))
        feed_row.addStretch()
        right.addLayout(feed_row)
        right.addStretch()
        mid.addLayout(right, 1)

        lay.addLayout(mid)
        return card

    def _build_saved(self):
        card = self._card()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)
        lay.addWidget(self._hdr("3", "POSITIONS", "the list runs top → bottom"))

        save_row = QHBoxLayout()
        save_row.setSpacing(8)
        self._name_input = QLineEdit()
        self._name_input.setPlaceholderText("position name (optional)")
        self._name_input.setMaxLength(40)   # matches the capture-filename cap
        self._name_input.setToolTip(
            "Name for the position about to be saved (Enter also saves)")
        self._name_input.returnPressed.connect(
            lambda: self._save_btn.isEnabled() and self._on_save_current())
        save_row.addWidget(self._name_input, 1)
        self._save_btn = QPushButton("+ Save position")
        self._save_btn.setObjectName("go")
        self._save_btn.setToolTip(
            "Save the machine's current X/Y as a new step in the list")
        self._save_btn.clicked.connect(self._on_save_current)
        save_row.addWidget(self._save_btn)
        lay.addLayout(save_row)

        self._pos_container = QWidget()
        self._pos_layout = QVBoxLayout(self._pos_container)
        self._pos_layout.setContentsMargins(0, 0, 0, 0)
        self._pos_layout.setSpacing(6)
        lay.addWidget(self._pos_container)

        self._empty_hint = QLabel(
            "No positions yet — connect, jog somewhere, then press "
            "“+ Save position”.")
        self._empty_hint.setWordWrap(True)
        self._empty_hint.setStyleSheet("color: #6a6f7c; font-style: italic;")
        lay.addWidget(self._empty_hint)
        return card

    def _build_sequence(self):
        card = self._card()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)
        lay.addWidget(self._hdr("4", "RUN", "loop through the enabled positions"))

        cfg = QHBoxLayout()
        cfg.setSpacing(8)
        cfg.addWidget(QLabel("Dwell:"))
        self._dwell = QLineEdit("1.0")
        self._dwell.setFixedWidth(64)
        self._dwell.setToolTip(
            "Seconds to wait at each position (auto-photos are taken after "
            "this settle time)")
        cfg.addWidget(self._dwell)
        cfg.addWidget(QLabel("s"))
        cfg.addSpacing(22)
        cfg.addWidget(QLabel("Loops:"))
        self._loops = QLineEdit("3")
        self._loops.setFixedWidth(64)
        self._loops.setToolTip("How many times to run the whole list")
        cfg.addWidget(self._loops)
        cfg.addSpacing(10)
        self._infinite = QCheckBox("∞ forever")
        self._infinite.setToolTip("Loop until Stop is pressed")
        # _set_ui_state owns ALL enable/disable logic (incl. loops-vs-∞)
        self._infinite.toggled.connect(lambda _on: self._set_ui_state())
        cfg.addWidget(self._infinite)
        cfg.addStretch()
        lay.addLayout(cfg)

        btns = QHBoxLayout()
        btns.setSpacing(8)
        self._run_btn = QPushButton("▶  Run")
        self._run_btn.setObjectName("runBtn")
        self._run_btn.setMinimumHeight(42)
        self._run_btn.setToolTip(
            "Move to each enabled position, top to bottom")
        self._run_btn.clicked.connect(self._on_run)
        btns.addWidget(self._run_btn, 2)
        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setMinimumHeight(42)
        self._pause_btn.setToolTip(
            "Pause after the current move (press again to resume)")
        self._pause_btn.clicked.connect(self._on_pause)
        btns.addWidget(self._pause_btn, 1)
        self._seq_stop_btn = QPushButton("Stop")
        self._seq_stop_btn.setObjectName("danger")
        self._seq_stop_btn.setMinimumHeight(42)
        self._seq_stop_btn.setToolTip(
            "Stop the run — sends an M410 halt, same as the big STOP")
        self._seq_stop_btn.clicked.connect(self._on_seq_stop)
        btns.addWidget(self._seq_stop_btn, 1)
        lay.addLayout(btns)

        self._progress = QLabel("Idle")
        self._progress.setObjectName("progress")
        self._progress.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._progress)
        return card

    def _build_log(self):
        card = self._card()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)
        lay.addWidget(self._hdr("", "CONSOLE", "serial log · raw G-code"))
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)
        self._log.setFixedHeight(160)
        lay.addWidget(self._log)

        raw = QHBoxLayout()
        raw.setSpacing(8)
        self._raw_input = QLineEdit()
        self._raw_input.setPlaceholderText("type raw G-code, Enter to send…")
        self._raw_input.setToolTip(
            "Send any G-code directly to the machine (e.g. M114, G28 X)")
        self._raw_input.returnPressed.connect(self._on_raw)
        raw.addWidget(self._raw_input, 1)
        self._raw_send_btn = QPushButton("Send")
        self._raw_send_btn.clicked.connect(self._on_raw)
        raw.addWidget(self._raw_send_btn)
        lay.addLayout(raw)
        return card

    def _build_camera(self):
        card = self._card()
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)

        hdr_row = QHBoxLayout()
        hdr_row.addWidget(self._hdr(
            "", "CAMERA",
            "wheel = zoom · drag = pan · double-click = fullscreen"))
        hdr_row.addStretch()
        self._cam_status = QLabel("Camera off")
        self._cam_status.setStyleSheet("color:#7f8494; font-weight:bold;")
        hdr_row.addWidget(self._cam_status)
        lay.addLayout(hdr_row)

        row = QHBoxLayout()
        row.setSpacing(8)
        self._cam_source = QComboBox()
        self._cam_source.addItem("Webcam (V4L2)", "webcam")
        self._cam_source.addItem("Basler (pylon)", "basler")
        if not _HAS_PYLON:
            item = self._cam_source.model().item(1)   # grey out Basler
            if item is not None:
                item.setEnabled(False)
        self._cam_source.setToolTip(
            "Camera type: any /dev/video* webcam, or a Basler via pypylon")
        self._cam_source.currentIndexChanged.connect(self._on_source_changed)
        row.addWidget(self._cam_source)
        self._cam_device_label = QLabel("Device:")
        row.addWidget(self._cam_device_label)
        self._cam_device = QLineEdit(DEFAULT_CAM)
        self._cam_device.setToolTip(
            "Webcam: /dev/video* node · Basler: serial number "
            "(blank = first camera found)")
        row.addWidget(self._cam_device, 1)
        self._cam_open_btn = QPushButton("Open")
        self._cam_open_btn.setObjectName("accent")
        self._cam_open_btn.setToolTip(
            "Start the live view (opens in the background)")
        self._cam_open_btn.clicked.connect(self._on_camera_open)
        row.addWidget(self._cam_open_btn)
        self._cam_close_btn = QPushButton("Close")
        self._cam_close_btn.setToolTip("Stop the live view")
        self._cam_close_btn.clicked.connect(self._on_camera_close)
        row.addWidget(self._cam_close_btn)
        lay.addLayout(row)

        self._cam_view = CameraView(
            "Camera off — choose a source above and press Open")
        self._cam_view.setObjectName("camView")
        self._cam_view.setAlignment(Qt.AlignCenter)
        self._cam_view.setMinimumSize(480, 320)
        self._cam_view.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._cam_view.setCursor(Qt.OpenHandCursor)
        self._cam_view.wheelZoom.connect(self._on_wheel_zoom)
        self._cam_view.dragPan.connect(self._on_drag_pan)
        self._cam_view.doubleClicked.connect(self._toggle_fullscreen)
        lay.addWidget(self._cam_view, 1)

        # Toolbar:  −  <zoom%>  +   Fit          ⛶ Fullscreen
        tools = QHBoxLayout()
        tools.setSpacing(8)
        self._zoom_out_btn = QPushButton("−")
        self._zoom_out_btn.setFixedSize(44, 32)
        self._zoom_out_btn.setToolTip("Zoom out")
        self._zoom_out_btn.clicked.connect(lambda: self._on_wheel_zoom(-1))
        tools.addWidget(self._zoom_out_btn)
        self._zoom_label = QLabel("100%")
        self._zoom_label.setAlignment(Qt.AlignCenter)
        self._zoom_label.setFixedWidth(56)
        self._zoom_label.setToolTip("Current zoom level")
        self._zoom_label.setStyleSheet("color:#9aa0a6; font-weight:bold;")
        tools.addWidget(self._zoom_label)
        self._zoom_in_btn = QPushButton("+")
        self._zoom_in_btn.setFixedSize(44, 32)
        self._zoom_in_btn.setToolTip("Zoom in (mouse wheel on the image too)")
        self._zoom_in_btn.clicked.connect(lambda: self._on_wheel_zoom(1))
        tools.addWidget(self._zoom_in_btn)
        self._zoom_fit_btn = QPushButton("Fit")
        self._zoom_fit_btn.setMinimumHeight(32)
        self._zoom_fit_btn.setToolTip("Reset zoom to fit the window")
        self._zoom_fit_btn.clicked.connect(self._on_zoom_fit)
        tools.addWidget(self._zoom_fit_btn)
        tools.addStretch(1)
        self._fs_btn = QPushButton("⛶  Fullscreen")
        self._fs_btn.setMinimumHeight(32)
        self._fs_btn.setToolTip(
            "Fullscreen view (double-click the image also toggles, ESC exits)")
        self._fs_btn.clicked.connect(self._toggle_fullscreen)
        tools.addWidget(self._fs_btn)
        lay.addLayout(tools)

        cap_row = QHBoxLayout()
        cap_row.setSpacing(10)
        self._capture_btn = QPushButton("📷  Capture photo")
        self._capture_btn.setObjectName("accent")
        self._capture_btn.setMinimumHeight(38)
        self._capture_btn.setToolTip(
            "Save a full-resolution photo to captures/ right now")
        self._capture_btn.clicked.connect(self._on_capture)
        cap_row.addWidget(self._capture_btn)
        self._cam_last_capture = QLabel("no captures yet")
        self._cam_last_capture.setStyleSheet("color:#6a6f7c;")
        self._cam_last_capture.setToolTip("Most recent capture file")
        cap_row.addWidget(self._cam_last_capture, 1)
        lay.addLayout(cap_row)

        if not _HAS_CAMERA:
            self._cam_open_btn.setEnabled(False)
            self._capture_btn.setEnabled(False)
            for b in (self._zoom_in_btn, self._zoom_out_btn,
                      self._zoom_fit_btn, self._fs_btn):
                b.setEnabled(False)
            self._cam_status.setText("OpenCV not available")
            self._cam_view.setText("OpenCV not available")
        self._set_cam_buttons()
        return card

    def _set_cam_buttons(self):
        """Enable/disable the camera controls to match the camera state, so
        the UI suggests the one action that makes sense next."""
        opening = self._cam_opening
        is_open = bool(self.camera and self.camera.is_open)
        self._cam_open_btn.setEnabled(
            _HAS_CAMERA and not opening and not is_open)
        # Close must also clear a LOST camera (controller present, grabber dead)
        self._cam_close_btn.setEnabled(
            opening or is_open or self._cam_error or self.camera is not None)
        # source/device are locked while the camera runs — close first
        self._cam_source.setEnabled(not opening and not is_open)
        self._cam_device.setEnabled(not opening and not is_open)

    # ==================================================================
    #  Saved-position rows
    # ==================================================================

    def _rebuild_positions(self):
        while self._pos_layout.count():
            item = self._pos_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._pos_rows = []

        self._empty_hint.setVisible(len(self.positions) == 0)

        for i, p in enumerate(self.positions):
            row = QFrame()
            row.setObjectName("row")
            rl = QHBoxLayout(row)
            rl.setContentsMargins(10, 6, 8, 6)
            rl.setSpacing(8)

            cb = QCheckBox()
            cb.setChecked(p.get("enabled", True))
            cb.setToolTip("Include this position when the sequence runs")
            cb.stateChanged.connect(
                lambda st, idx=i: self._on_toggle(idx, st))
            rl.addWidget(cb)

            num = QLabel(f"{i + 1}")
            num.setStyleSheet("color: #5c6270; font-weight: bold;")
            num.setFixedWidth(18)
            num.setToolTip("Run order")
            rl.addWidget(num)

            lbl = QLabel(f"{p['name']}")
            lbl.setTextFormat(Qt.PlainText)
            lbl.setStyleSheet("color: #e8eaee; font-weight: bold;")
            lbl.setMinimumWidth(70)
            lbl.setToolTip(p["name"])   # full name when the label is squeezed
            rl.addWidget(lbl)
            coord = QLabel(f"X {p['x']:.1f}   Y {p['y']:.1f}")
            coord.setStyleSheet(
                "color: #8b8f9a; font-family: 'DejaVu Sans Mono', monospace;")
            rl.addWidget(coord, 1)

            cap = QPushButton("📷")
            cap.setCheckable(True)
            cap.setChecked(p.get("capture", False))
            cap.setObjectName("capOn" if p.get("capture", False) else "")
            cap.setFixedSize(40, 32)
            cap.setToolTip(
                "Auto-photo: capture a picture when the sequence stops here "
                "(lit purple = on)")
            cap.toggled.connect(
                lambda on, idx=i: self._on_toggle_capture(idx, on))
            rl.addWidget(cap)

            go = QPushButton("Go")
            go.setObjectName("go")
            go.setFixedSize(50, 32)
            go.setToolTip("Move the machine here now")
            go.setProperty("needsConn", True)
            go.clicked.connect(lambda _=False, idx=i: self._on_go(idx))
            rl.addWidget(go)
            up = QPushButton("↑")
            up.setFixedSize(36, 32)
            up.setToolTip("Move up (runs earlier)")
            up.clicked.connect(lambda _=False, idx=i: self._on_move_row(idx, -1))
            rl.addWidget(up)
            dn = QPushButton("↓")
            dn.setFixedSize(36, 32)
            dn.setToolTip("Move down (runs later)")
            dn.clicked.connect(lambda _=False, idx=i: self._on_move_row(idx, 1))
            rl.addWidget(dn)
            dl = QPushButton("✕")
            dl.setObjectName("danger")
            dl.setFixedSize(36, 32)
            dl.setToolTip("Delete this position")
            dl.clicked.connect(lambda _=False, idx=i: self._on_delete_row(idx))
            rl.addWidget(dl)

            self._pos_layout.addWidget(row)
            self._pos_rows.append(row)
        self._highlight_row(self._active_row)
        self._set_ui_state()

    def _highlight_row(self, idx):
        for i, row in enumerate(self._pos_rows):
            row.setObjectName("rowActive" if i == idx else "row")
            row.setStyleSheet("")  # force style refresh
            row.style().unpolish(row)
            row.style().polish(row)

    # ==================================================================
    #  Position handlers
    # ==================================================================

    def _on_save_current(self):
        name = self._name_input.text().strip()
        if not name:
            # auto-name must stay unique: after deletions len()+1 can collide
            # with an existing pN
            existing = {p["name"] for p in self.positions}
            n = len(self.positions) + 1
            name = f"p{n}"
            while name in existing:
                n += 1
                name = f"p{n}"
        self.positions.append({
            "name": name,
            "x": round(self.position[0], 2),
            "y": round(self.position[1], 2),
            "enabled": True,
            "capture": False,
        })
        self._name_input.clear()
        self._save_config()
        self._rebuild_positions()
        self._log_msg(f"Saved '{name}' at X{self.position[0]:.1f} "
                      f"Y{self.position[1]:.1f}", "#58c96b")

    def _on_toggle(self, idx, state):
        if 0 <= idx < len(self.positions):
            self.positions[idx]["enabled"] = (state == Qt.Checked)
            self._save_config()

    def _on_toggle_capture(self, idx, on):
        if 0 <= idx < len(self.positions):
            self.positions[idx]["capture"] = on
            btn = self.sender()
            if btn is not None:
                btn.setObjectName("capOn" if on else "")
                self._refresh_style(btn)
            self._save_config()

    def _on_go(self, idx):
        if 0 <= idx < len(self.positions):
            p = self.positions[idx]
            self._move_to(p["x"], p["y"])

    def _on_move_row(self, idx, delta):
        j = idx + delta
        if 0 <= idx < len(self.positions) and 0 <= j < len(self.positions):
            self.positions[idx], self.positions[j] = \
                self.positions[j], self.positions[idx]
            self._save_config()
            self._rebuild_positions()

    def _on_delete_row(self, idx):
        if 0 <= idx < len(self.positions):
            p = self.positions.pop(idx)
            self._save_config()
            self._rebuild_positions()
            self._log_msg(f"Deleted '{p['name']}'", "#d9a23d")

    # ==================================================================
    #  Connection
    # ==================================================================

    def _on_autodetect(self):
        cands = (glob.glob("/dev/serial/by-id/*STM*")
                 + glob.glob("/dev/serial/by-id/*stm*")
                 + glob.glob("/dev/serial/by-id/*Marlin*")
                 + glob.glob("/dev/serial/by-id/*marlin*"))
        if not cands:
            for p in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]:
                if os.path.exists(p):
                    cands.append(p)
        if cands:
            self._port.setText(cands[0])
            self._log_msg(f"Found: {cands[0]}", "#58c96b")
        else:
            self._log_msg("No CNC device detected", "#e86060")

    def _on_connect(self):
        if self.worker and self.worker.isRunning():
            self._log_msg("Already connected (or connecting)", "#d9a23d")
            return
        if self.runner and self.runner.isRunning():
            # A live runner still references the old worker — connecting a new
            # one under it would leave STOP aimed at a dead port.
            self._log_msg("Stop the running sequence first", "#e86060")
            return
        port = self._port.text().strip()
        self._set_status("CONNECTING…", "wait")
        self._log_msg(f"Connecting to {port}…", "#4dc3f0")

        self.worker = SerialWorker()
        self.worker.line_received.connect(self._on_line)
        self.worker.position_updated.connect(self._on_position)
        self.worker.connected.connect(self._on_serial_connected)
        self.worker.disconnected.connect(self._on_serial_disconnected)
        self.worker.error.connect(self._on_serial_error)
        # refresh button/field gating the moment the thread actually exits, so
        # Connect can't stay disabled on a dead "connecting" state
        self.worker.finished.connect(self._set_ui_state)
        self.worker.connect_port(port, 115200)
        if self.worker.is_port_open():
            self.worker.start()
            self._set_ui_state()   # reflect the connecting state right away
        else:
            # connect_port failed (missing/busy/permission-denied) and already
            # emitted an error — don't leave the header stuck on "CONNECTING…".
            self.worker = None
            self._set_status("DISCONNECTED", "bad")
            self._set_ui_state()
        self._save_config()

    def _on_disconnect(self):
        if self.runner and self.runner.isRunning():
            self._on_seq_stop()
        w, self.worker = self.worker, None
        if w:
            # Close the port on a helper thread: disconnect_port joins the
            # worker thread (up to 5 s if it is wedged in the boot's usbreset)
            # and that wait must not freeze the GUI. Signals the dying worker
            # still emits are ignored by the sender guards in the slots.
            threading.Thread(target=w.disconnect_port, daemon=True).start()
        self._connected = False
        self._set_status("DISCONNECTED", "bad")
        self._set_ui_state()

    # ==================================================================
    #  CNC movement (direct)
    # ==================================================================

    def _feedrate(self):
        return self._safe_int(self._feed.text(), 3000, minimum=1)

    def _send(self, cmd):
        if self.worker and self._connected:
            self.worker.send(cmd)
        else:
            self._log_msg("Not connected", "#e86060")

    def _on_jog(self, axis, sign):
        f = self._feedrate()
        d = sign * self._step
        self._send("G91")
        self._send(f"G1 {axis}{d:.2f} F{f}")
        self._send("G90")
        self._send("M114")

    def _on_step(self, val):
        self._step = val
        matched = False
        for sb, v in self._step_btns:
            active = (v == val)
            matched = matched or active
            sb.setObjectName("stepActive" if active else "")
            self._refresh_style(sb)
        # A value that isn't one of the presets means the custom field is the
        # active step — highlight it and make sure it displays the value
        # (without rewriting the text mid-typing: ',' decimals parse equal).
        self._step_custom.setObjectName("" if matched else "stepActive")
        if not matched:
            try:
                shown = float(self._step_custom.text().replace(",", "."))
            except ValueError:
                shown = None
            if shown != val:
                self._step_custom.setText(f"{val:g}")
        self._refresh_style(self._step_custom)

    def _on_step_custom(self, text):
        """Typing a valid value in the custom-step field makes it the active
        step. Accepts ',' as decimal separator; out-of-range or garbage input
        is ignored (the previous step stays active)."""
        try:
            v = float(text.replace(",", "."))
        except ValueError:
            return
        if not math.isfinite(v) or not (CUSTOM_STEP_MIN <= v <= CUSTOM_STEP_MAX):
            return
        self._on_step(v)

    def _on_home(self):
        self._send("G28 X Y")
        self._send("M114")

    def _move_to(self, x, y):
        f = self._feedrate()
        self._send("G90")
        self._send(f"G1 X{x:.2f} Y{y:.2f} F{f}")
        self._send("M114")

    def _on_raw(self):
        cmd = self._raw_input.text().strip()
        if not cmd:
            return
        if not cmd.isascii():
            # Smart quotes / ° / NBSP from copy-paste would blow up the ASCII
            # encode on the serial thread — reject here with a real message.
            self._log_msg(
                "G-code must be plain ASCII — check for smart quotes/symbols",
                "#e86060")
            return
        self._send(cmd)
        self._raw_input.clear()

    def _on_emergency_stop(self):
        stopped = False
        if self.runner and self.runner.isRunning():
            self.runner.stop()   # halts the runner and the worker it holds
            stopped = True
        if self.worker:
            # ALSO hit the current link directly: after an unplug + reconnect
            # the runner may still hold the previous worker, and it's the new
            # connection that must halt.
            self.worker.emergency_stop()
            stopped = True
        if not stopped:
            self._log_msg("STOP: not connected — nothing to halt", "#d9a23d")

    # ==================================================================
    #  Camera
    # ==================================================================

    def _apply_source_ui(self):
        """Update the device row (label / placeholder / value) for the source."""
        if self._cam_active_source == "basler":
            self._cam_device_label.setText("Serial:")
            self._cam_device.setPlaceholderText("blank = first Basler")
            self._cam_device.setText(self._basler_serial)
        else:
            self._cam_device_label.setText("Device:")
            self._cam_device.setPlaceholderText("/dev/video0")
            self._cam_device.setText(self._webcam_device)

    def _on_source_changed(self):
        if self._source_loading:
            return
        # remember the field value for the source we're leaving
        if self._cam_active_source == "webcam":
            self._webcam_device = self._cam_device.text().strip()
        else:
            self._basler_serial = self._cam_device.text().strip()
        self._cam_active_source = self._cam_source.currentData()
        self._apply_source_ui()

    def _on_camera_open(self):
        if self._cam_opening or (self.camera and self.camera.is_open):
            return
        source = self._cam_source.currentData()
        if source == "basler":
            if not _HAS_PYLON:
                self._log_msg(f"pypylon not installed — {_PYLON_ERR}", "#e86060")
                return
            cam = BaslerController(serial=self._cam_device.text().strip())
            label = self._cam_device.text().strip() or "first Basler"
        else:
            if not _HAS_CAMERA:
                self._log_msg(f"Camera unavailable: {_CAMERA_ERR}", "#e86060")
                return
            device = self._cam_device.text().strip() or DEFAULT_CAM
            cam = CameraController(device=device, width=CAM_W, height=CAM_H)
            label = device
        self.camera = cam
        self._cam_opening = True
        self._cam_error = False
        self._cam_last_seq = None
        self._cam_last_preview = None
        self._cam_last_full = None
        self._cam_last_frame_t = time.monotonic()
        self._on_zoom_fit()   # every fresh open starts fit-to-view
        self._set_cam_buttons()
        self._log_msg(f"Opening {source} camera ({label})…", "#4dc3f0")
        if not self._cam_timer.isActive():
            self._cam_timer.start(40)   # ~25 fps while opening/open
        self._save_config()

        # Open/probe can take a few seconds — do it off the GUI thread so the
        # window never freezes. The result comes back via the _cam_open_result
        # signal (queued onto the GUI thread), same pattern as manual capture.
        def _worker():
            ok = cam.open()
            if self.camera is not cam:
                # user closed/reopened while we were opening — discard this one
                cam.release()
                return
            note, color = "", ""
            actual = getattr(cam, "actual_source", None)
            if ok and source == "webcam" and actual is not None \
                    and str(actual) != label:
                # the fallback scan substituted a different node — say so
                note = (f"Note: {label} wasn't usable — opened {actual} "
                        f"instead")
                color = "#d9a23d"
            elif ok and source == "basler":
                # show which settings the camera came up with — the app applies
                # none of its own, so this is the pylon Viewer / user-set config
                summary = getattr(cam, "settings_summary", "")
                if summary:
                    note = f"Camera config (from camera/pylon setup): {summary}"
                    color = "#4dc3f0"
            self._cam_open_result.emit(ok, note, color)
        threading.Thread(target=_worker, daemon=True).start()

    def _on_cam_open_result(self, ok, note, color):
        """Camera open finished (helper thread → queued here). All GUI-visible
        state changes happen on the GUI thread."""
        self._cam_opening = False
        self._cam_error = not ok
        if note:
            self._log_msg(note, color or "#d9a23d")
        self._update_camera_view()   # repaint status + button gating right now
        if not ok:
            self._cam_timer.stop()   # nothing to poll until the next Open

    def _on_camera_close(self):
        self._cam_opening = False
        self._cam_timer.stop()
        cam, self.camera = self.camera, None
        if cam:
            # release() may join the grabber up to 1.5 s — do it off the GUI
            # thread so closing a wedged camera can't freeze the UI (incl. STOP).
            threading.Thread(target=cam.release, daemon=True).start()
        self._cam_last_seq = None
        self._cam_last_preview = None
        self._cam_last_full = None
        self._on_zoom_fit()
        self._cam_error = False
        self._cam_status_shown = None
        self._cam_status.setText("Camera off")
        self._cam_status.setStyleSheet("color:#7f8494; font-weight:bold;")
        self._cam_view.clear()
        self._cam_view.setText("Camera off — choose a source above and press Open")
        if self._fs is not None:
            self._fs.view.clear()
            self._fs.view.setText("Camera off")
        self._set_cam_buttons()

    def _on_capture(self):
        if not (self.camera and self.camera.is_open):
            self._log_msg("Camera not open", "#e86060")
            return
        frame = self.camera.capture_frame()
        if frame is None:
            self._log_msg("No frame yet", "#d9a23d")
            return
        cam = self.camera

        # JPEG-encode + disk write off the GUI thread; the result comes back via
        # the _capture_result signal (queued onto the GUI thread).
        def _worker():
            try:
                path = cam.save_capture(frame, "manual", str(CAPTURE_DIR))
                self._capture_result.emit(True, str(path))
            except Exception as e:
                self._capture_result.emit(False, str(e))
        threading.Thread(target=_worker, daemon=True).start()

    def _on_manual_capture_result(self, success, info):
        if success:
            base = os.path.basename(info)
            self._log_msg(f"📷 Saved {base}", "#58c96b")
            self._cam_last_capture.setText(base)
        else:
            self._log_msg(f"Capture failed: {info}", "#e86060")

    def _update_camera_view(self):
        # Status line (only relabel on change to avoid needless repaint).
        is_open = bool(self.camera and self.camera.is_open)
        if self._cam_opening:
            status = ("opening…", "#d9a23d")
        elif is_open:
            if (self._cam_last_seq is not None
                    and time.monotonic() - self._cam_last_frame_t > 3.0):
                # device stopped delivering — don't pretend the frozen frame
                # is live
                status = ("no signal", "#d9a23d")
            else:
                status = ("● LIVE", "#58c96b")
        elif self._cam_error:
            status = ("camera error", "#e86060")
        elif self.camera is not None:
            # controller exists but its grabber shut down (unplug/driver death)
            status = ("camera lost", "#e86060")
        else:
            status = ("Camera off", "#7f8494")
        if status != self._cam_status_shown:
            self._cam_status_shown = status
            self._cam_status.setText(status[0])
            self._cam_status.setStyleSheet(
                f"color:{status[1]}; font-weight:bold;")
            self._set_cam_buttons()   # opening → live/error/lost transitions
            if self._cam_last_preview is None:
                # nothing painted yet — keep the big view's hint in sync too
                hints = {
                    "opening…": "Opening camera…",
                    "camera error":
                        "Camera error — check the device and press Open",
                    "camera lost":
                        "Camera lost — reconnect it and press Open",
                }
                if status[0] in hints:
                    self._cam_view.setText(hints[status[0]])
            if status[0] == "camera lost":
                self._cam_timer.stop()   # grabber is gone; nothing to poll

        if not (self.camera and self.camera.is_open):
            return
        # read_preview() gives an already-downscaled frame for a Basler (the
        # grabber did the 12MP resize off the GUI thread); for a webcam it's the
        # native frame, cheap to scale here. It drives the fit view.
        seq, preview = self.camera.read_preview()
        if preview is None or seq == self._cam_last_seq:
            return   # no new frame since last paint
        self._cam_last_seq = seq
        self._cam_last_preview = preview
        self._cam_last_frame_t = time.monotonic()
        # When zoomed in, also grab the FULL-resolution frame so the crop shows
        # real detail instead of an upscaled preview.
        if self._cam_zoom > 1.0:
            _, full = self.camera.read_latest()
            if full is not None:
                self._cam_last_full = full
        self._render()

    def _render(self):
        """Paint the current frame into the active view (main pane, or the
        fullscreen window if open). At fit (zoom==1) it uses the cheap preview;
        when zoomed in it crops the FULL-resolution frame around _cam_center so
        the extra detail is genuine, then scales that crop to the view."""
        view = self._active_view()
        src = self._cam_last_full if (
            self._cam_zoom > 1.0 and self._cam_last_full is not None) \
            else self._cam_last_preview
        if src is None:
            return
        h, w = src.shape[:2]

        if self._cam_zoom > 1.0:
            cw = max(1, int(round(w / self._cam_zoom)))
            chh = max(1, int(round(h / self._cam_zoom)))
            cx = int(round(self._cam_center[0] * w))
            cy = int(round(self._cam_center[1] * h))
            x0 = min(max(0, cx - cw // 2), w - cw)
            y0 = min(max(0, cy - chh // 2), h - chh)
            frame = src[y0:y0 + chh, x0:x0 + cw]
        else:
            frame = src

        tw = max(1, view.width())
        th = max(1, view.height())
        fh, fw = frame.shape[:2]
        scale = min(tw / fw, th / fh)
        if scale != 1.0:
            if scale < 1.0:
                # INTER_AREA looks best but costs tens of ms on a multi-MP
                # zoom crop — too slow for a 25 fps paint on the GUI thread.
                # Above ~2 MP fall back to INTER_LINEAR (a few ms, fine for a
                # live view; captures are saved from the full frame anyway).
                interp = (cv2.INTER_AREA if fw * fh <= 2_000_000
                          else cv2.INTER_LINEAR)
            else:
                interp = cv2.INTER_LINEAR
            frame = cv2.resize(
                frame, (max(1, int(fw * scale)), max(1, int(fh * scale))),
                interpolation=interp)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h2, w2, ch = rgb.shape
        img = QImage(rgb.data, w2, h2, ch * w2, QImage.Format_RGB888)
        view.setPixmap(QPixmap.fromImage(img))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Re-fit the last frame so resizing while the stream is idle/frozen
        # doesn't leave a mis-sized image.
        if self._cam_last_preview is not None:
            self._render()

    # ------------------------------------------------------------------
    #  Zoom / pan / fullscreen  (view-only; capture always saves full frame)
    # ------------------------------------------------------------------

    def _active_view(self):
        """The CameraView currently being painted: the fullscreen one if the
        fullscreen window is up, else the pane view."""
        return self._fs.view if self._fs is not None else self._cam_view

    def _on_wheel_zoom(self, direction):
        factor = 1.25 if direction > 0 else 1 / 1.25
        self._set_zoom(self._cam_zoom * factor)

    def _on_zoom_fit(self):
        self._cam_center = [0.5, 0.5]
        self._set_zoom(1.0)

    def _set_zoom(self, zoom):
        self._cam_zoom = max(1.0, min(16.0, zoom))
        if self._cam_zoom <= 1.0:
            self._cam_center = [0.5, 0.5]
            # back at fit: stop holding a full-res frame (a 12 MP Basler
            # frame is ~36 MB — don't retain it for the rest of the session)
            self._cam_last_full = None
        self._zoom_label.setText(f"{int(round(self._cam_zoom * 100))}%")
        self._clamp_center()
        # Fetch a full-res frame right away so zooming a slow/frozen stream
        # sharpens immediately instead of waiting for the next grab.
        if self._cam_zoom > 1.0 and self.camera and self.camera.is_open:
            _, full = self.camera.read_latest()
            if full is not None:
                self._cam_last_full = full
        if self._cam_last_preview is not None:
            self._render()

    def _on_drag_pan(self, dx, dy):
        if self._cam_zoom <= 1.0 or self._cam_last_preview is None:
            return
        view = self._active_view()
        vw = max(1, view.width())
        vh = max(1, view.height())
        # Normalise the drag by the DISPLAYED image size, not the raw view
        # size — the image is letterboxed inside the view, and the displayed
        # extent of the crop is min(vw, vh·aspect) regardless of zoom. This
        # keeps panning tracking the cursor 1:1.
        fh, fw = self._cam_last_preview.shape[:2]
        aspect = fw / fh
        disp_w = max(1.0, min(vw, vh * aspect))
        disp_h = max(1.0, min(vh, vw / aspect))
        # Drag the image right (dx>0) → reveal content to its left → move the
        # crop center left. Divide by zoom: the crop is 1/zoom of the frame.
        self._cam_center[0] -= dx / disp_w / self._cam_zoom
        self._cam_center[1] -= dy / disp_h / self._cam_zoom
        self._clamp_center()
        self._render()

    def _clamp_center(self):
        half = 0.5 / self._cam_zoom
        self._cam_center[0] = min(max(half, self._cam_center[0]), 1 - half)
        self._cam_center[1] = min(max(half, self._cam_center[1]), 1 - half)

    def _toggle_fullscreen(self):
        if self._fs is not None:
            self._fs.close()      # emits closed → _on_fs_closed
            return
        fs = FullscreenPreview()
        fs.view.setCursor(Qt.OpenHandCursor)
        fs.view.wheelZoom.connect(self._on_wheel_zoom)
        fs.view.dragPan.connect(self._on_drag_pan)
        fs.view.doubleClicked.connect(self._toggle_fullscreen)
        fs.resized.connect(self._render)
        fs.closed.connect(self._on_fs_closed)
        self._fs = fs
        fs.showFullScreen()
        self._render()

    def _on_fs_closed(self):
        self._fs = None
        if self._cam_last_preview is not None:
            self._render()   # repaint the pane view (stale while fullscreen up)

    # ==================================================================
    #  Sequence run
    # ==================================================================

    def _on_run(self):
        if not self._connected:
            self._log_msg("Connect first", "#e86060")
            return
        if self.runner and self.runner.isRunning():
            return
        steps = [(i, p["name"], p["x"], p["y"], bool(p.get("capture", False)))
                 for i, p in enumerate(self.positions)
                 if p.get("enabled", True)]
        if not steps:
            self._log_msg("No enabled positions to run", "#e86060")
            return
        dwell = self._safe_float(self._dwell.text(), 1.0, minimum=0.0)
        infinite = self._infinite.isChecked()
        loops = self._safe_int(self._loops.text(), 1, minimum=1)

        self.runner = SequenceRunner(
            self.worker, steps, self._feedrate(), dwell, loops, infinite,
            camera_getter=lambda: self.camera, capture_dir=str(CAPTURE_DIR),
            start_xy=self.position)
        self.runner.step_started.connect(self._on_step_started)
        self.runner.progress.connect(self._on_seq_progress)
        self.runner.capture_done.connect(self._on_capture_done)
        self.runner.run_finished.connect(self._on_seq_finished)
        self.runner.stopped.connect(self._on_seq_stopped)
        self.runner.error.connect(self._on_seq_error)
        self.runner.start()

        n_cap = sum(1 for s in steps if s[4])
        if n_cap and not (self.camera and self.camera.is_open):
            self._log_msg(
                f"{n_cap} position(s) set to capture but the camera is off — "
                f"open it any time and captures will resume",
                "#d9a23d")

        self._pause_btn.setText("Pause")
        n = "∞" if infinite else str(loops)
        self._log_msg(f"Run started: {len(steps)} positions × {n}",
                      "#4dc3f0")
        self._set_ui_state()

    def _on_pause(self):
        if not (self.runner and self.runner.isRunning()):
            return
        if self.runner.is_paused():
            self.runner.resume()
            self._pause_btn.setText("Pause")
            self._log_msg("Resumed", "#4dc3f0")
        else:
            self.runner.pause()
            self._pause_btn.setText("Resume")
            self._log_msg("Paused", "#d9a23d")

    def _on_seq_stop(self):
        if self.runner and self.runner.isRunning():
            self.runner.stop()
            # No blocking wait: stop() already M410-halted the machine, and
            # the runner's stopped/error signal does the final UI reset.
            # Waiting here could freeze the GUI for seconds if the runner is
            # mid-capture — show progress instead and let the signal land.
            self._progress.setText("Stopping…")
        self._active_row = -1
        self._highlight_row(-1)
        self._pause_btn.setText("Pause")
        self._set_ui_state()

    def _on_step_started(self, row_idx):
        self._active_row = row_idx
        self._highlight_row(row_idx)

    def _on_capture_done(self, row_idx, success, info):
        if success:
            base = os.path.basename(info)
            self._log_msg(f"📷 {base}", "#58c96b")
            self._cam_last_capture.setText(base)
        else:
            self._log_msg(f"Capture skipped ({info})", "#d9a23d")

    def _on_seq_progress(self, loop, total_loops, step, total_steps):
        loops_txt = "∞" if total_loops == 0 else str(total_loops)
        self._progress.setText(
            f"Loop {loop}/{loops_txt}  ·  Step {step}/{total_steps}")

    def _on_seq_finished(self):
        self._progress.setText("Done ✓")
        self._active_row = -1
        self._highlight_row(-1)
        self._pause_btn.setText("Pause")
        self._log_msg("Sequence complete", "#58c96b")
        self._set_ui_state()

    def _on_seq_stopped(self):
        self._progress.setText("Stopped")
        self._active_row = -1
        self._highlight_row(-1)
        self._pause_btn.setText("Pause")
        self._log_msg("Sequence stopped", "#d9a23d")
        self._set_ui_state()

    def _on_seq_error(self, msg):
        self._progress.setText("Error")
        self._active_row = -1
        self._highlight_row(-1)
        self._pause_btn.setText("Pause")
        self._log_msg(f"Sequence error: {msg}", "#e86060")
        self._set_ui_state()

    # ==================================================================
    #  Serial signal slots
    # ==================================================================

    def _on_line(self, line):
        col = None
        if line.startswith(">"):
            col = "#6aa9ff"
        elif line.startswith("ok"):
            col = "#58c96b"
        elif "error" in line.lower():
            col = "#e86060"
        elif line.startswith("BOOT:"):
            col = "#5c6270"
        self._log_msg(line, col)

    def _on_position(self, x, y, z):
        self.position = (x, y)
        self._pos_label.setText(f"X: {x:.2f}    Y: {y:.2f}")

    def _on_serial_connected(self):
        sender = self.sender()
        if sender is not None and sender is not self.worker:
            return   # from a worker discarded by a disconnect — stale
        self._connected = True
        self._set_status("CONNECTED", "good")
        self._set_ui_state()

    def _on_serial_disconnected(self):
        sender = self.sender()
        if sender is not None and sender is not self.worker:
            # A worker we already detached (background disconnect) winding
            # down — must not stomp the state of a NEW connection attempt.
            return
        self._connected = False
        # A dropped link (unplug / board lost) means a running sequence can't
        # continue — stop it rather than let it spin on a dead worker.
        if self.runner and self.runner.isRunning():
            self.runner.stop()
        self.position = (0.0, 0.0)
        self._pos_label.setText("X: —    Y: —")
        self._set_status("DISCONNECTED", "bad")
        # (worker.finished → _set_ui_state clears any residual "connecting"
        # gating once the thread fully exits)
        self._set_ui_state()

    def _on_serial_error(self, msg):
        self._log_msg(f"ERROR: {msg}", "#e86060")

    # ==================================================================
    #  UI state / helpers
    # ==================================================================

    def _set_ui_state(self):
        running = bool(self.runner and self.runner.isRunning())
        conn = self._connected
        connecting = bool(self.worker and self.worker.isRunning()) and not conn

        self._connect_btn.setEnabled(not conn and not connecting)
        self._port.setEnabled(not conn and not connecting)
        self._detect_btn.setEnabled(not conn and not connecting)
        self._disconnect_btn.setEnabled((conn or connecting) and not running)

        # movement touches the machine → needs a connection and no run active;
        # editing only touches the saved list → allowed offline, but locked
        # during a run so the highlighted row can't drift from what's running.
        movement = conn and not running
        editing = not running
        for b in self._jog_btns:
            b.setEnabled(movement)
        for sb, _v in self._step_btns:
            sb.setEnabled(movement)
        self._step_custom.setEnabled(movement)
        self._home_btn.setEnabled(movement)
        self._save_btn.setEnabled(movement)
        self._raw_input.setEnabled(movement)
        self._raw_send_btn.setEnabled(movement)

        # run parameters are snapshotted when Run is pressed — freeze the
        # fields during a run so they can't silently disagree with what's
        # actually executing
        for w in (self._dwell, self._infinite, self._feed, self._name_input):
            w.setEnabled(editing)
        self._loops.setEnabled(editing and not self._infinite.isChecked())
        for row in self._pos_rows:
            for w in row.findChildren(QPushButton):
                w.setEnabled(movement if w.property("needsConn") else editing)
            for w in row.findChildren(QCheckBox):
                w.setEnabled(editing)

        # sequence controls
        self._run_btn.setEnabled(conn and not running and len(self.positions) > 0)
        self._pause_btn.setEnabled(running)
        self._seq_stop_btn.setEnabled(running)

    def _refresh_style(self, w):
        w.style().unpolish(w)
        w.style().polish(w)

    def _log_msg(self, text, color=None):
        if color:
            self._log.appendHtml(
                f'<span style="color:{color}">{self._esc(text)}</span>')
        else:
            self._log.appendPlainText(text)
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())

    @staticmethod
    def _esc(s):
        return (s.replace("&", "&amp;").replace("<", "&lt;")
                .replace(">", "&gt;"))

    # ==================================================================
    #  Persistence
    # ==================================================================

    @staticmethod
    def _coerce_positions(raw):
        """Validate/repair positions loaded from disk so a hand-edited, schema-
        drifted, or truncated cnc_data.json can never crash startup. Rows that
        aren't a dict with finite numeric x/y are dropped; missing name/enabled
        are defaulted. Returns (clean_list, dropped_count)."""
        out = []
        if not isinstance(raw, list):
            return out, (0 if raw is None else 1)
        for p in raw:
            if not isinstance(p, dict):
                continue
            try:
                x = float(p["x"])
                y = float(p["y"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            out.append({
                # `or` also covers an explicit JSON null / empty-string name,
                # which p.get()'s default would pass through as "None" / ""
                "name": str(p.get("name") or f"p{len(out) + 1}"),
                "x": x,
                "y": y,
                "enabled": CncSequencer._as_bool(p.get("enabled"), True),
                "capture": CncSequencer._as_bool(p.get("capture"), False),
            })
        return out, len(raw) - len(out)

    def _load_config(self):
        try:
            if DATA_FILE.exists():
                data = json.loads(DATA_FILE.read_text())
                if not isinstance(data, dict):
                    data = {}
                # str()-wrap: a hand-edited numeric value (e.g. camera_device: 0,
                # the natural way to write an OpenCV index) would otherwise make
                # QLineEdit.setText raise TypeError and crash startup.
                self._port.setText(str(data.get("port") or DEFAULT_PORT))
                self._webcam_device = str(data.get("camera_device") or DEFAULT_CAM)
                self._basler_serial = str(data.get("basler_serial") or "")
                src = data.get("camera_source", "webcam")
                if src not in ("webcam", "basler"):
                    src = "webcam"
                self._source_loading = True
                sidx = self._cam_source.findData(src)
                if sidx >= 0:
                    self._cam_source.setCurrentIndex(sidx)
                self._source_loading = False
                self._cam_active_source = src
                self._apply_source_ui()
                self._feed.setText(str(data.get("feedrate", 3000)))
                self._dwell.setText(str(data.get("dwell_seconds", 1.0)))
                self._loops.setText(str(data.get("loop_count", 3)))
                self.positions, dropped = self._coerce_positions(
                    data.get("positions", []))
                if dropped > 0:
                    self._log_msg(
                        f"Ignored {dropped} invalid saved position(s)", "#d9a23d")
                self._step = self._safe_float(
                    data.get("step", 10.0), 10.0, minimum=0.1)
                self._on_step(self._step)
        except (OSError, ValueError) as e:
            # ValueError covers json.JSONDecodeError AND UnicodeDecodeError
            # (binary-corrupt file) — a bad config must never crash startup.
            self._log_msg(f"Config load failed: {e}", "#e86060")

    def _save_config(self):
        # keep the per-source remembered value current before writing
        if self._cam_active_source == "webcam":
            self._webcam_device = self._cam_device.text().strip()
        else:
            self._basler_serial = self._cam_device.text().strip()
        data = {
            "port": self._port.text().strip(),
            "camera_source": self._cam_active_source,
            "camera_device": self._webcam_device,
            "basler_serial": self._basler_serial,
            "feedrate": self._feedrate(),
            "dwell_seconds": self._safe_float(
                self._dwell.text(), 1.0, minimum=0.0),
            "loop_count": self._safe_int(self._loops.text(), 3, minimum=1),
            "step": self._step,
            "positions": self.positions,
        }
        try:
            # Write to a temp file, fsync, then atomically replace — a crash
            # or power-loss mid-write can't truncate cnc_data.json (which
            # would silently wipe all saved positions on the next load).
            tmp = DATA_FILE.with_name(DATA_FILE.name + ".tmp")
            with open(tmp, "w") as f:
                f.write(json.dumps(data, indent=2))
                f.flush()
                os.fsync(f.fileno())   # data on disk BEFORE the rename lands
            os.replace(str(tmp), str(DATA_FILE))
        except OSError as e:
            self._log_msg(f"Config save failed: {e}", "#e86060")

    @staticmethod
    def _as_bool(v, default):
        """Coerce a loaded value to bool. Plain bool()/JSON is fine for
        app-written files, but a hand-edited "false"/"no" string is truthy under
        bool(), so handle common string spellings explicitly."""
        if isinstance(v, bool):
            return v
        if isinstance(v, (int, float)):
            return bool(v)
        if isinstance(v, str):
            return v.strip().lower() in ("1", "true", "yes", "on")
        return default

    @staticmethod
    def _safe_float(s, default, minimum=None):
        # Rejects '', 'abc', 'nan', AND 'inf'/'1e400' (which pass float() but
        # would later blow up int() with OverflowError or hang a sleep).
        try:
            v = float(s)
        except (ValueError, TypeError):
            return default
        if not math.isfinite(v):
            return default
        if minimum is not None and v < minimum:
            return minimum
        return v

    @staticmethod
    def _safe_int(s, default, minimum=None):
        try:
            v = float(s)
        except (ValueError, TypeError):
            return default
        if not math.isfinite(v):
            return default
        v = int(v)
        if minimum is not None and v < minimum:
            return minimum
        return v

    def closeEvent(self, event):
        self._save_config()
        self._cam_timer.stop()
        if self._fs is not None:
            self._fs.close()
        if self.runner and self.runner.isRunning():
            self.runner.stop()
            # A runner mid-capture can overshoot the first wait. Wait longer
            # rather than let Qt hit its fatal "QThread destroyed while
            # running" abort during interpreter teardown.
            if not self.runner.wait(3000):
                self.runner.wait(15000)
        if self.worker:
            self.worker.disconnect_port()
            # disconnect_port force-closes the port on timeout, which kicks the
            # thread out of readline — give that a moment to land.
            self.worker.wait(2000)
        # Detach the camera exactly like _on_camera_close does, so an in-flight
        # open sees itself superseded and releases its own controller instead
        # of starting a grabber nobody will ever stop.
        self._cam_opening = False
        cam, self.camera = self.camera, None
        if cam:
            cam.release()
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = CncSequencer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
