"""
Basler Camera Controller
========================
A pypylon-backed controller with the SAME interface as camera.CameraController,
so the GUI can drive a Basler ace (USB3 Vision / GigE) camera interchangeably
with a V4L2 webcam.

Basler cameras are NOT V4L2 devices — they never appear as /dev/video* and
OpenCV cannot open them. They are accessed through Basler's pylon runtime via
the pypylon binding (`pip install pypylon`). On Linux a non-root user also needs
a udev rule granting access to Basler USB devices (vendor 2676); see the README.

A background grabber thread (GrabStrategy_LatestImageOnly) caches:
  * `_latest`  — the full-resolution BGR frame, used for capture
  * `_preview` — a downscaled BGR copy, used for the live view, so the GUI thread
                 never has to resize a 12-megapixel frame (that alone is ~60 ms).
"""

import time
import logging
import threading
import cv2
from pathlib import Path

try:
    from pypylon import pylon
    _HAS_PYLON = True
    _PYLON_ERR = ""
except Exception as _e:            # pypylon not installed
    pylon = None
    _HAS_PYLON = False
    _PYLON_ERR = str(_e)

from camera import write_capture   # shared unique-filename save

log = logging.getLogger("basler")

PREVIEW_MAX_W = 800   # downscale target width for the live preview


class BaslerController:
    def __init__(self, serial="", width=None, height=None):
        # `serial` selects a specific camera; blank = first one found.
        # width/height are accepted for interface parity but unused (the Basler
        # runs at its configured full resolution).
        self._serial = (serial or "").strip()
        self.device = self._serial or "(first Basler)"
        self._cam = None
        self._converter = None
        self._lock = threading.Lock()      # protects _latest / _preview / _seq
        self._latest = None                # full-res BGR (for capture)
        self._preview = None               # downscaled BGR (for display)
        self._seq = 0
        self._grab_thread = None
        self._stop_grab = threading.Event()
        self._opened = False
        self.settings_summary = ""   # filled at open(); shown in the GUI log

    @property
    def is_open(self):
        return self._opened

    # ------------------------------------------------------------------
    #  Open
    # ------------------------------------------------------------------

    def open(self):
        """Open the (optionally serial-selected) Basler camera and start the
        grabber. Returns True on success. Runs the blocking device open on the
        caller — the GUI calls this on a worker thread."""
        if not _HAS_PYLON:
            log.error(f"pypylon not available: {_PYLON_ERR}")
            return False
        cam = None
        try:
            tl = pylon.TlFactory.GetInstance()
            devices = tl.EnumerateDevices()
            if not devices:
                log.error("no Basler devices found (check cable/power/udev rule)")
                return False
            dev_info = None
            if self._serial:
                for d in devices:
                    if d.GetSerialNumber() == self._serial:
                        dev_info = d
                        break
                if dev_info is None:
                    log.error(f"Basler serial {self._serial} not connected")
                    return False
            else:
                dev_info = devices[0]

            cam = pylon.InstantCamera(tl.CreateDevice(dev_info))
            cam.Open()

            # Deliberately apply NO camera parameters here. The camera keeps
            # whatever configuration is active — i.e. the user's pylon Viewer
            # setup (and, after a power cycle, the User Set selected as the
            # startup set). Earlier versions forced continuous auto-exposure/
            # gain/white-balance, silently overriding that configuration on
            # every open. Only a summary is read (for the GUI log) so the user
            # can see which settings are live.
            self.settings_summary = self._read_settings_summary(cam)

            conv = pylon.ImageFormatConverter()
            conv.OutputPixelFormat = pylon.PixelType_BGR8packed
            conv.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

            cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        except Exception as e:
            log.error(f"Basler open failed: {e}")
            try:
                if cam is not None:
                    cam.Close()
            except Exception:
                pass
            return False

        self._cam = cam
        self._converter = conv
        with self._lock:
            self._latest = None
            self._preview = None
        self._opened = True
        self._start_grabber()
        return True

    @staticmethod
    def _read_settings_summary(cam):
        """Best-effort one-line snapshot of the camera's active acquisition
        settings, so the log shows which configuration (pylon Viewer / user
        set) is in use. Node names differ across camera families — every read
        is optional and read-only."""
        def read(node):
            try:
                return getattr(cam, node).GetValue()
            except Exception:
                return None

        parts = []
        for auto in ("ExposureAuto", "GainAuto", "BalanceWhiteAuto"):
            v = read(auto)
            if v is not None:
                parts.append(f"{auto}={v}")
        exp = read("ExposureTime")          # USB (SFNC), µs float
        if exp is None:
            exp = read("ExposureTimeAbs")   # older GigE naming
        if exp is not None:
            parts.append(f"exposure {exp:.0f} µs")
        gain = read("Gain")                 # USB (SFNC), dB float
        if gain is not None:
            parts.append(f"gain {gain:.1f} dB")
        else:
            gain_raw = read("GainRaw")      # older GigE naming
            if gain_raw is not None:
                parts.append(f"gain {gain_raw} (raw)")
        return ", ".join(parts)

    # ------------------------------------------------------------------
    #  Background grabber (owns the camera's Stop/Close, like camera.py)
    # ------------------------------------------------------------------

    def _start_grabber(self):
        self._stop_grab.clear()
        self._grab_thread = threading.Thread(
            target=self._grab_loop, name="basler-grab", daemon=True)
        self._grab_thread.start()

    def _grab_loop(self):
        cam = self._cam
        conv = self._converter
        fails = 0
        try:
            while not self._stop_grab.is_set() and cam.IsGrabbing():
                try:
                    # 1 s timeout so the loop re-checks _stop_grab promptly.
                    grab = cam.RetrieveResult(
                        1000, pylon.TimeoutHandling_ThrowException)
                except Exception:
                    # Timeout (normal) or device gone. The small sleep keeps a
                    # device that raises IMMEDIATELY (unplug) from busy-spinning
                    # this loop at 100% CPU until IsGrabbing() goes false.
                    time.sleep(0.05)
                    continue
                try:
                    if grab.GrabSucceeded():
                        # .copy() so _latest is independent of the converter's
                        # reused output buffer (captures happen much later).
                        img = conv.Convert(grab).GetArray().copy()
                        h, w = img.shape[:2]
                        if w > PREVIEW_MAX_W:
                            s = PREVIEW_MAX_W / w
                            preview = cv2.resize(
                                img, (PREVIEW_MAX_W, max(1, int(h * s))),
                                interpolation=cv2.INTER_AREA)
                        else:
                            preview = img
                        with self._lock:
                            self._latest = img
                            self._preview = preview
                            self._seq += 1
                        fails = 0
                    else:
                        fails += 1
                        if fails == 1 or fails % 50 == 0:
                            log.warning(f"Basler grab failed ({fails})")
                finally:
                    grab.Release()
        finally:
            # This thread OWNS teardown so release() never stops/closes the
            # camera out from under an in-flight RetrieveResult.
            try:
                cam.StopGrabbing()
            except Exception:
                pass
            try:
                cam.Close()
            except Exception:
                pass
            # The camera is closed now — reflect that in is_open so the GUI can
            # show "camera lost" instead of LIVE-on-a-frozen-frame, and stop
            # serving stale frames to captures.
            self._opened = False
            with self._lock:
                self._latest = None
                self._preview = None

    # ------------------------------------------------------------------
    #  Consumer API (non-blocking — mirror of CameraController)
    # ------------------------------------------------------------------

    def capture_frame(self):
        """Latest FULL-resolution frame (non-blocking). None if unavailable."""
        with self._lock:
            return self._latest

    def read_latest(self):
        with self._lock:
            return self._seq, self._latest

    def read_preview(self):
        """Downscaled frame for the GUI preview (computed on the grabber thread
        so the GUI never resizes a full-res frame)."""
        with self._lock:
            return self._seq, self._preview

    def save_capture(self, frame, zone_name, save_dir):
        return write_capture(frame, zone_name, save_dir)

    def release(self):
        """Stop the grabber and release the camera. The grabber thread does the
        actual StopGrabbing()/Close() as it exits, so we only signal + join."""
        self._stop_grab.set()
        t = self._grab_thread
        if t is not None and t.is_alive():
            t.join(timeout=2.0)   # RetrieveResult has a 1 s timeout, so this joins
        self._grab_thread = None
        self._opened = False
        self._cam = None
        self._converter = None
        with self._lock:
            self._latest = None
            self._preview = None
            self._seq = 0
