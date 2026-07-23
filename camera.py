"""
Camera Controller
=================
Wraps OpenCV VideoCapture. A background grabber thread continuously reads
frames into a cache so that NO consumer (GUI thread, inspector, hand tracker)
ever blocks on a slow/dead V4L2 device. Consumers read the latest cached frame
instantly via capture_frame() / read_latest().
"""

import time
import logging
import threading
import cv2
from pathlib import Path

log = logging.getLogger("camera")

# Sources (device paths / indices) currently held by an open controller, so a
# fallback scan never grabs a node another camera is already using.
_claimed = set()
_claimed_lock = threading.Lock()


# Name-pick + write must be atomic per process: two concurrent captures in the
# same second (double-clicked Capture button, or manual + sequence capture)
# would otherwise both pass the exists() check and overwrite each other.
_capture_write_lock = threading.Lock()


def write_capture(frame, zone_name, save_dir):
    """Save a frame as <name>.jpg — no timestamp in the filename (the file's
    own modification time carries date/time). When the name is already taken
    (e.g. a repeated run), a _NNN suffix keeps every file instead of
    overwriting. Raises on imwrite failure rather than silently reporting
    success. Shared by the webcam and Basler controllers."""
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    with _capture_write_lock:
        filepath = save_dir / f"{zone_name}.jpg"
        n = 1
        while filepath.exists():
            filepath = save_dir / f"{zone_name}_{n:03d}.jpg"
            n += 1
        if not cv2.imwrite(str(filepath), frame):
            raise IOError(f"cv2.imwrite failed for {filepath}")
    return filepath


class CameraController:
    def __init__(self, device="/dev/video0", width=1920, height=1080):
        self.device = device
        self.width = width
        self.height = height
        self._cap = None
        self._lock = threading.Lock()        # protects _latest / _seq
        self._latest = None                  # most recent BGR frame
        self._seq = 0                        # increments on every new frame
        self._grab_thread = None
        self._stop_grab = threading.Event()
        self._opened_source = None           # actual source that opened

    @property
    def is_open(self):
        return self._cap is not None and self._cap.isOpened()

    # ------------------------------------------------------------------
    #  Open / validate
    # ------------------------------------------------------------------

    def _open_validated(self, source):
        """Open `source` and confirm it actually delivers frames.
        Returns a cv2.VideoCapture or None. A node that opens but cannot
        stream (e.g. a dead /dev/video* node) is rejected here."""
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Must deliver at least one real frame within a few tries
        for _ in range(5):
            ret, frame = cap.read()
            if ret and frame is not None:
                return cap
        cap.release()
        return None

    def open(self):
        """Open camera. Validates it can stream; falls back to scanning
        indices 0-9 if the configured device can't deliver frames.
        Returns True on success."""
        if self.is_open:
            return True
        log.info(f"Opening camera: {self.device}")
        opened_source = None
        # A camera that was just closed can hold the node for ~100-200 ms while
        # its grabber tears down — retry briefly before concluding the device
        # is unusable (otherwise a quick Close→Open falls into the index scan).
        cap = None
        for attempt in range(3):
            cap = self._open_validated(self.device)
            if cap is not None:
                break
            if attempt < 2:
                time.sleep(0.25)
        if cap is not None:
            opened_source = self.device
        else:
            log.warning(f"{self.device} opened but can't stream — "
                        f"scanning indices 0-9")
            for idx in range(10):
                with _claimed_lock:
                    taken = idx in _claimed or f"/dev/video{idx}" in _claimed
                if taken:
                    continue
                cap = self._open_validated(idx)
                if cap is not None:
                    opened_source = idx
                    log.info(f"Opened working camera at index {idx}")
                    break
        if cap is None:
            log.error("No working camera device could be opened")
            return False
        self._cap = cap
        with _claimed_lock:
            self._opened_source = opened_source
            _claimed.add(opened_source)
        self._latest = None
        self._start_grabber()
        return True

    # ------------------------------------------------------------------
    #  Background grabber
    # ------------------------------------------------------------------

    def _start_grabber(self):
        self._stop_grab.clear()
        self._grab_thread = threading.Thread(
            target=self._grab_loop, name=f"grab:{self.device}", daemon=True)
        self._grab_thread.start()

    def _grab_loop(self):
        """Continuously read frames into the cache. Runs off the GUI thread,
        so a slow/dead camera's blocking read() never freezes the UI. This
        thread OWNS the capture: it releases it as it exits, so release() never
        frees the cap out from under an in-flight read()."""
        cap = self._cap
        fails = 0
        try:
            while not self._stop_grab.is_set():
                if cap is None:
                    break
                ret, frame = cap.read()   # paced by camera fps; may block if dead
                if ret and frame is not None:
                    with self._lock:
                        self._latest = frame
                        self._seq += 1
                    fails = 0
                else:
                    fails += 1
                    if fails == 1 or fails % 50 == 0:
                        log.warning(f"{self.device}: frame read failed ({fails})")
                    if fails == 20:
                        # ~2 s of continuous failures: stop serving the stale
                        # last frame, so captures fail loudly ("camera not
                        # streaming") instead of silently saving an old image.
                        with self._lock:
                            self._latest = None
                    time.sleep(0.1)
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    # ------------------------------------------------------------------
    #  Consumer API (non-blocking — returns cached frame)
    # ------------------------------------------------------------------

    def capture_frame(self):
        """Return the most recent frame (non-blocking). None if unavailable.
        The returned array is owned by the caller's read only — the grabber
        replaces the reference rather than mutating in place."""
        with self._lock:
            return self._latest

    def read_latest(self):
        """Return (seq, frame). seq increments per new frame so consumers can
        skip reprocessing duplicates."""
        with self._lock:
            return self._seq, self._latest

    def read_preview(self):
        """Preview frame for the GUI. A V4L2 webcam frame is small enough to
        downscale on the GUI thread, so this is just the latest frame. (The
        Basler controller overrides this with a pre-downscaled copy so the GUI
        never resizes a 12MP frame.)"""
        with self._lock:
            return self._seq, self._latest

    @property
    def actual_source(self):
        """The device/index actually opened — may differ from the requested
        device when the fallback scan substituted a working node."""
        return self._opened_source

    def save_capture(self, frame, zone_name, save_dir):
        """Save a capture image (see module-level write_capture)."""
        return write_capture(frame, zone_name, save_dir)

    def release(self):
        """Stop the grabber and release resources. The grabber thread owns the
        actual cap.release() (in its finally), so we only signal + join here and
        never release the capture concurrently with an in-flight read()."""
        self._stop_grab.set()
        t = self._grab_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.5)
        # If the grabber joined, it already released the cap; if it timed out
        # (wedged in read()), leave the cap to the daemon grabber rather than
        # releasing it out from under the read. Either way, drop our reference.
        if t is None or not t.is_alive():
            self._cap = None
        self._grab_thread = None
        with _claimed_lock:
            src = self._opened_source
            self._opened_source = None
            if src is not None:
                _claimed.discard(src)
        with self._lock:
            self._latest = None
            self._seq = 0
