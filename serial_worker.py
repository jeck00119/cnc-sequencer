"""
Serial Worker
==============
QThread managing CNC serial communication with Marlin firmware.
Handles USB reset, boot sequence, command queue, and position parsing.

Copied from the pcb-projector-overlay project (proven on Jetson) — standalone,
no external project dependencies.
"""

import os
import re
import time
import threading
import subprocess

import serial
from PyQt5.QtCore import QThread, pyqtSignal


class SerialWorker(QThread):
    line_received = pyqtSignal(str)
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    error = pyqtSignal(str)
    position_updated = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self._serial = None
        self._running = False
        self._queue = []
        self._lock = threading.Lock()

    def _sleep_interruptible(self, seconds):
        """time.sleep that bails out early if the thread is being shut down,
        so disconnect/close during the multi-second boot doesn't leave a
        running QThread (which would abort with 'Destroyed while running')."""
        elapsed = 0.0
        while elapsed < seconds:
            if not self._running:
                return
            time.sleep(0.1)
            elapsed += 0.1

    def connect_port(self, port, baud=115200):
        if not os.path.exists(port):
            self.line_received.emit(f"Port not found: {port}")
            self.error.emit(f"Port not found: {port}")
            return
        try:
            self._serial = serial.Serial(port, baud, timeout=0.5)
            self._running = True
            self.line_received.emit(f"Port opened: {port} @ {baud}")
        except serial.SerialException as e:
            self.line_received.emit(f"Port open FAILED: {e}")
            self.error.emit(str(e))

    def disconnect_port(self):
        self._running = False
        if self._serial and self._serial.is_open:
            try:
                self._serial.cancel_read()
            except Exception:
                pass
        self.wait(5000)
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def send(self, gcode_line):
        with self._lock:
            self._queue.append(gcode_line.strip())

    def is_queue_empty(self):
        with self._lock:
            return len(self._queue) == 0

    def is_port_open(self):
        """True once connect_port() opened the port (the worker may still be
        booting). Public accessor so the GUI never pokes at _running."""
        return self._running

    def flush_queue(self):
        """Clear all pending commands from the queue."""
        with self._lock:
            self._queue.clear()

    def emergency_stop(self):
        """Drop the queue and send M410 directly — all under the same lock the
        run loop writes commands under, so a command already popped by run()
        can never be written to the wire *after* the M410. Anything enqueued
        AFTER this call is intentionally kept: a post-stop jog or the boot's
        auto-home must run, not be silently discarded."""
        with self._lock:
            self._queue.clear()   # inline, not flush_queue() (Lock isn't reentrant)
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(b"M410\n")
                    self._serial.flush()
                    self.line_received.emit("> M410 (EMERGENCY STOP)")
                except Exception:
                    pass

    def run(self):
        # Safety net: any uncaught exception (bad encode, garbled reply parse,
        # driver weirdness) must still tell the GUI the link is gone —
        # otherwise the header stays CONNECTED on a dead thread forever.
        try:
            self._run_impl()
        except Exception as e:
            self.error.emit(f"Serial worker crashed: {e}")
            self.disconnected.emit()

    def _run_impl(self):
        ser = self._serial
        if not ser or not ser.is_open:
            self.error.emit("Serial port not open")
            return

        port_path = ser.port

        # Step 1: USB reset
        self.line_received.emit("USB reset...")
        try:
            ser.close()
        except Exception:
            pass
        try:
            subprocess.run(
                ["sudo", "usbreset", "0483:5740"],
                capture_output=True, timeout=5)
        except Exception:
            pass
        self._sleep_interruptible(3)
        if not self._running:
            return

        # Step 2: Reopen port
        self.line_received.emit("Reopening port...")
        try:
            self._serial = serial.Serial(port_path, 115200, timeout=0.5)
            ser = self._serial
        except serial.SerialException as e:
            self.line_received.emit(f"Reopen failed: {e}")
            self.error.emit(f"Port reopen failed: {e}")
            self.disconnected.emit()
            return

        # Step 3: Wait for board and ping
        self.line_received.emit("Waiting for board...")
        self._sleep_interruptible(3)
        if not self._running:
            return

        ready = False
        for attempt in range(10):
            if not self._running:
                return
            try:
                ser.write(b"M114\n")
                ser.flush()
            except serial.SerialException:
                time.sleep(1)
                continue

            for _ in range(4):
                try:
                    line = ser.readline().decode(
                        "ascii", errors="replace").strip()
                    if line:
                        self.line_received.emit(f"BOOT: {line}")
                        if "ok" in line.lower() or "X:" in line:
                            ready = True
                            self._parse_position(line)
                            break
                except serial.SerialException:
                    break

            if ready:
                self.line_received.emit(
                    f"Board ready ({attempt + 1}s)")
                break
            time.sleep(1)

        if not ready:
            self.error.emit("Board not responding")
            self.disconnected.emit()
            return

        # Drain remaining boot messages. Bounded: a board stuck in a
        # reset/spew loop must not pin us here forever, and a disconnect
        # request must be able to interrupt the drain.
        for _ in range(200):
            if not self._running:
                return
            try:
                line = ser.readline().decode(
                    "ascii", errors="replace").strip()
                if not line:
                    break
                self.line_received.emit(f"BOOT: {line}")
            except serial.SerialException:
                break

        if not self._running:
            return

        # Auto-home 3x for sensorless homing reliability
        with self._lock:
            self._queue.append("G28 X Y")
            self._queue.append("G28 X Y")
            self._queue.append("G28 X Y")
            self._queue.append("M114")

        self.line_received.emit("Connected! Auto-homing...")
        self.connected.emit()

        # Main loop
        while self._running:
            cmd = None
            write_error = None
            with self._lock:
                if self._queue:
                    cmd = self._queue.pop(0)
                    # Write inside the SAME lock emergency_stop() uses, so the
                    # pop+write is atomic w.r.t. M410 — either the command is
                    # written before an incoming stop, or the stop wins the
                    # lock first and clears the queue before this pop happens.
                    try:
                        ser.write((cmd + "\n").encode("ascii"))
                        ser.flush()
                    except serial.SerialException as e:
                        write_error = e
                        cmd = None

            if write_error is not None:
                self.error.emit(str(write_error))
                self.disconnected.emit()
                return

            if cmd:
                self.line_received.emit(f"> {cmd}")

                # Wait for ok — with NO fixed cap. M400/G28 legitimately
                # withhold their 'ok' for as long as the motion takes, and the
                # runner's per-move deadline (which scales with distance/feed
                # and may exceed any cap chosen here) is the authority on
                # stuck moves. Faking completion after a fixed silence would
                # let the sequencer race ahead of the machine. A silent link
                # is surfaced by the periodic notice below instead; Disconnect
                # (cancel_read) or an unplug always breaks the wait.
                silent_count = 0
                while self._running:
                    try:
                        line = ser.readline().decode(
                            "ascii", errors="replace").strip()
                        if line:
                            self.line_received.emit(line)
                            self._parse_position(line)
                            if line.startswith("ok") or \
                                    "error" in line.lower():
                                break
                            silent_count = 0
                        else:
                            silent_count += 1
                            if silent_count % 120 == 0:   # every ~60 s
                                self.line_received.emit(
                                    f"…no reply to '{cmd}' for "
                                    f"{silent_count // 2} s — long move, or "
                                    f"hung board? Disconnect aborts")
                    except serial.SerialException as e:
                        self.error.emit(str(e))
                        self.disconnected.emit()
                        return
            else:
                try:
                    line = ser.readline().decode(
                        "ascii", errors="replace").strip()
                    if line:
                        self.line_received.emit(line)
                        self._parse_position(line)
                except serial.SerialException as e:
                    self.error.emit(str(e))
                    self.disconnected.emit()
                    return

        self.disconnected.emit()

    def _parse_position(self, line):
        m = re.match(r'X:([\d.-]+)\s+Y:([\d.-]+)\s+Z:([\d.-]+)', line)
        if m:
            try:
                self.position_updated.emit(
                    float(m.group(1)), float(m.group(2)), float(m.group(3)))
            except ValueError:
                # '[\d.-]+' also matches non-numbers like '--' or '1.2.3',
                # which a garbled reply (unplug/noise mid-line) can produce —
                # skip the line rather than kill the worker thread.
                pass
