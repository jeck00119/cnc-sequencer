"""
Sequence Runner
===============
QThread that drives the CNC through a list of saved positions, optionally
looping N times (or forever). Each move uses the M400 + is_queue_empty()
handshake so the runner only advances once the physical move has completed.

Pause/stop are responsive: stop checks happen every 100-200ms.
"""

import math
import time
import threading

from PyQt5.QtCore import QThread, pyqtSignal


class SequenceRunner(QThread):
    # step index below is the ORIGINAL row index in the UI list, so the GUI
    # can highlight the correct row.
    step_started = pyqtSignal(int)                 # row index
    step_done = pyqtSignal(int)                    # row index
    loop_started = pyqtSignal(int, int)            # loop (1-based), total (0=inf)
    progress = pyqtSignal(int, int, int, int)      # loop, total_loops, step, total_steps
    capture_done = pyqtSignal(int, bool, str)      # row_idx, success, path or reason
    run_finished = pyqtSignal()                    # completed all loops
    stopped = pyqtSignal()                         # aborted by user
    error = pyqtSignal(str)

    MOVE_TIMEOUT_S = 120

    def __init__(self, worker, steps, feedrate, dwell_s,
                 loop_count, infinite, camera_getter=None, capture_dir=None,
                 start_xy=None):
        """
        worker        : SerialWorker instance (already connected)
        steps         : list of (row_index, name, x, y, capture) — enabled only
        feedrate      : mm/min
        dwell_s       : seconds to pause at each position
        loop_count    : number of loops (ignored if infinite)
        infinite      : run until stopped
        camera_getter : zero-arg callable returning the CURRENT camera
                        controller (or None). Read fresh at every capture so a
                        camera opened/closed/reopened mid-run is picked up.
        capture_dir   : where to write captured images
        start_xy      : (x, y) machine position at run start, used to estimate
                        per-move durations for the timeout
        """
        super().__init__()
        self.worker = worker
        self.steps = steps
        self.feedrate = int(feedrate)
        self.dwell_s = float(dwell_s)
        self.loop_count = int(loop_count)
        self.infinite = bool(infinite)
        self.camera_getter = camera_getter
        self.capture_dir = capture_dir
        self._prev_xy = tuple(start_xy) if start_xy is not None else None
        self._stop_flag = False
        self._pause_event = threading.Event()
        self._pause_event.set()   # not paused

    @staticmethod
    def _safe_name(name):
        safe = "".join(c if (c.isalnum() or c in "-_") else "_"
                       for c in str(name))[:40]
        return safe or "pos"

    def _capture_at(self, row_idx, name, loop):
        """Grab the latest camera frame and save it. Best-effort: a capture
        failure is reported but never aborts the sequence. The camera is
        looked up fresh each time so one opened mid-run is used. Filenames are
        prefixed with the loop cycle (loop001_name_…) so all photos from one
        cycle sort and group together."""
        cam = self.camera_getter() if self.camera_getter else None
        if cam is None:
            self.capture_done.emit(row_idx, False, "no camera")
            return
        try:
            frame = cam.capture_frame()
        except Exception as e:
            self.capture_done.emit(row_idx, False, str(e))
            return
        if frame is None:
            self.capture_done.emit(row_idx, False, "camera not streaming")
            return
        try:
            zone = f"loop{loop:03d}_{self._safe_name(name)}"
            path = cam.save_capture(frame, zone, self.capture_dir)
            self.capture_done.emit(row_idx, True, str(path))
        except Exception as e:
            self.capture_done.emit(row_idx, False, str(e))

    # --- control ---
    def pause(self):
        # Pausing a runner that is already stopping would re-clear the event
        # stop() just set and wedge the run loop on an untimed wait.
        if not self._stop_flag:
            self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def is_paused(self):
        return not self._pause_event.is_set()

    def stop(self):
        self._stop_flag = True
        self._pause_event.set()   # unblock if paused
        if self.worker:
            self.worker.emergency_stop()

    # --- internals ---
    def _check_stop(self):
        """Block while paused; return True if a stop was requested. The wait is
        timed so a stop always gets through, even if a pause races it."""
        while not self._pause_event.wait(0.2):
            if self._stop_flag:
                return True
        return self._stop_flag

    def _interruptible_sleep(self, seconds):
        remaining = seconds
        while remaining > 0:
            if self._stop_flag:
                return
            if not self._pause_event.wait(0.2):
                continue   # paused — don't consume dwell time
            time.sleep(min(0.1, remaining))
            remaining -= 0.1

    def _move_timeout(self, x, y):
        """Per-move deadline in seconds: the fixed floor, extended for
        long/slow moves (distance ÷ feed, doubled, plus margin) so a
        legitimate F30 crawl is not falsely emergency-stopped at 120 s."""
        timeout = float(self.MOVE_TIMEOUT_S)
        if self._prev_xy is not None and self.feedrate > 0:
            dist = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            est = dist / self.feedrate * 60.0
            timeout = max(timeout, est * 2.0 + 30.0)
        self._prev_xy = (x, y)
        return timeout

    def _wait_for_commands(self, commands, timeout_s=None):
        """Queue commands and wait until the queue drains. M400 makes Marlin
        withhold its 'ok' until motion completes, so queue-empty == move done.
        The trailing M114 in each move is load-bearing: it keeps the queue
        non-empty until M400 is acked, so 'queue empty' can't be observed
        mid-move. Stop is checked every 200ms.

        Returns one of:
          'done'         — queue drained, i.e. the move completed
          'stopped'      — a stop was requested
          'disconnected' — the serial worker thread died mid-move
          'timeout'      — the deadline elapsed without completion
        """
        if not self.worker:
            return "disconnected"
        for cmd in commands:
            self.worker.send(cmd)
        # monotonic: an NTP wall-clock step must not expire (or extend) a move
        deadline = time.monotonic() + (timeout_s or self.MOVE_TIMEOUT_S)
        while time.monotonic() < deadline:
            if self._stop_flag:
                return "stopped"
            if not self.worker.isRunning():
                # serial thread exited (unplug / SerialException / board lost)
                return "disconnected"
            if self.worker.is_queue_empty():
                time.sleep(0.2)
                if self.worker.is_queue_empty():
                    return "done"
            time.sleep(0.2)
        return "timeout"

    def run(self):
        if not self.steps:
            self.error.emit("No enabled positions to run")
            return
        total_loops = 0 if self.infinite else self.loop_count
        try:
            loop = 0
            while True:
                if self._stop_flag:
                    break
                loop += 1
                if not self.infinite and loop > self.loop_count:
                    break

                self.loop_started.emit(loop, total_loops)

                for (row_idx, name, x, y, capture) in self.steps:
                    if self._check_stop():
                        self.stopped.emit()
                        return
                    self.step_started.emit(row_idx)
                    self.progress.emit(loop, total_loops,
                                       self._step_number(row_idx),
                                       len(self.steps))
                    timeout_s = self._move_timeout(x, y)
                    status = self._wait_for_commands([
                        "G90",
                        f"G1 X{x:.2f} Y{y:.2f} F{self.feedrate}",
                        "M400",
                        "M114",
                    ], timeout_s)
                    if status == "stopped" or self._stop_flag:
                        self.stopped.emit()
                        return
                    if status == "disconnected":
                        self.error.emit(
                            "Lost connection to the CNC during a move — stopped")
                        return
                    if status == "timeout":
                        # Don't fake success: halt and surface it.
                        if self.worker:
                            self.worker.emergency_stop()
                        self.error.emit(
                            f"Move to '{name}' exceeded {timeout_s:.0f}s — "
                            f"stopped for safety")
                        return
                    self.step_done.emit(row_idx)
                    # Dwell first so the machine settles, then capture (if the
                    # position is flagged) once it's arrived and stationary.
                    self._interruptible_sleep(self.dwell_s)
                    if capture and not self._stop_flag:
                        self._capture_at(row_idx, name, loop)

            if self._stop_flag:
                self.stopped.emit()
            else:
                self.run_finished.emit()
        except Exception as e:
            self.error.emit(str(e))

    def _step_number(self, row_idx):
        """1-based position within the enabled steps for progress display."""
        for i, step in enumerate(self.steps):
            if step[0] == row_idx:
                return i + 1
        return 0
