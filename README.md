# CNC Sequencer

A simple standalone CNC control GUI — like a stripped-down GRBL sender. Jog the
machine, save named positions, then run those positions as a sequence, N times
in a loop (or forever), with a **live camera view** and optional **photo capture
at each position**.

Built for the same XY-only CoreXY machine as the pcb-projector-overlay project
(BigTreeTech SKR Mini E3 V3.0, Marlin firmware, TMC2209 sensorless homing). The
serial and camera layers are copied verbatim from that proven project.

## What it does

- **Jog** the CNC with a D-pad (X/Y only) at selectable step sizes (0.1 / 1 / 5 / 10 / 50 mm, or any custom step in mm typed into the "custom" field)
- **Home** the machine (`G28 X Y`)
- **Save positions** — capture the current X/Y under a name
- **Reorder / enable / delete** saved positions; the list *is* the sequence
- **Live camera view** — see where the CNC is; open/close any `/dev/video*` device
- **Capture photos** — a manual **📷 Capture photo** button, plus a per-position **📷**
  toggle that snaps an image automatically when the sequence arrives there
- **Run the sequence** top-to-bottom, with:
  - a **dwell** (pause) at each position
  - a **loop count** (or **∞** to run forever)
  - live progress ("Loop 2/5 · Step 3/8") and the active row highlighted
- **Pause / Resume / Stop** mid-run
- a big always-on **STOP** button in the header (`M410` emergency stop)
- everything persisted to `cnc_data.json`; captured images land in `captures/`

## Requirements

- Python 3.8+
- PyQt5
- pyserial
- OpenCV (`cv2`) — **optional**, only for the camera view/capture. Without it the
  CNC features work fine and the camera pane shows "OpenCV not available".
- pypylon — **optional**, only for a **Basler** camera (see below). Without it the
  Basler source is greyed out and webcams still work.

```bash
pip install PyQt5 pyserial opencv-python
pip install pypylon            # only if you use a Basler camera
```

(All are already present on the Jetson used for pcb-projector-overlay.)

## Running

```bash
cd ~/Desktop/cnc-sequencer
python3 cnc_sequencer.py
```

### First-time setup

1. **Connect**
   - Click **Auto-detect** (scans `/dev/serial/by-id/*STM*` / `*Marlin*`, then
     `/dev/ttyACM0..2`), or paste the port manually.
   - Click **Connect**. On connect the app performs the proven boot sequence:
     USB reset → reopen at 115200 → wait for the board → **auto-home 3×**
     (sensorless homing needs the repeats for reliability). This takes ~15–20 s;
     watch the log.
2. **Jog** to a spot using the D-pad, pick your step size, set the feed rate.
3. Type a **name** and click **+ Save position**. Repeat for each position.
4. Set **Dwell** (seconds at each stop) and **Loops** (or tick **∞ forever**).
5. Click **▶ Run**.

While a sequence runs, jogging / saving / editing are disabled so nothing else
touches the serial queue. Use **Pause**, **Stop**, or the big **STOP**.

## Camera & capture

The right-hand pane is a live camera view so you can watch where the CNC is. Two
camera **sources** are supported — pick one in the **Source** dropdown:

- **Webcam (V4L2)** — any `/dev/video*` UVC camera (e.g. a Logitech BRIO), via
  OpenCV. Enter the node in **Device** (default `/dev/video0`).
- **Basler (pylon)** — a Basler ace industrial camera (USB3 Vision / GigE) via
  pypylon. Enter the **Serial** (blank = first Basler found). See setup below.

1. Choose the **Source**, set **Device/Serial**, and click **Open**. Opening
   happens on a background thread, so the window never freezes — even on a
   dead/absent camera it just reports "camera error".
2. Click **📷 Capture photo** any time to save the current frame. (Basler captures are
   full sensor resolution; the preview is downscaled on the grabber thread so a
   12 MP frame is never resized on the GUI thread.)
3. To auto-capture during a run, toggle the small **📷** button on any saved
   position. When the sequence arrives there (after the move + the dwell settle),
   it saves a frame automatically before moving on.

Manual captures are written to `captures/` as `manual.jpg` (then
`manual_001.jpg`, `manual_002.jpg`, …). Sequence auto-captures are written as
`loop<NNN>_<name>.jpg` — the loop-cycle prefix makes all photos from one cycle
sort and group together (e.g. `loop002_corner.jpg`). Filenames carry no
date/time — the file's own timestamp metadata has that; when a name is already
taken (e.g. a repeated run), a `_NNN` suffix keeps every file instead of
overwriting. Auto-capture is best-effort: if the camera is off or not
streaming, the sequence logs "Capture skipped" and keeps going — it never
stops the run.

The camera is completely independent of the CNC serial link — you can open/close
it and capture whether or not the machine is connected. Frames are grabbed on a
background thread and only the newest is painted, so the preview never blocks the
GUI (this is the same grabber architecture that fixed the main app's UI lag).

### Basler camera setup (one-time)

Basler cameras are **not** V4L2 devices — they never show up as `/dev/video*` and
OpenCV can't open them. They go through Basler's pylon runtime via pypylon.

1. **Install pypylon** (bundles the pylon runtime; no separate SDK needed):
   ```bash
   pip install pypylon
   ```
   On this Jetson (Python 3.8 / aarch64 / glibc 2.31) the prebuilt
   `pypylon-3.0.1-cp38-cp38-manylinux_2_31_aarch64.whl` installs directly.
2. **udev rule** so a non-root user can access the camera (without it,
   `EnumerateDevices()` returns empty even though `lsusb` shows the camera).

   **On this machine this is already handled** — the pylon Software Suite
   Debian package owns `/etc/udev/rules.d/69-basler-cameras.rules` (a
   `MODE:="0666"` + `TAG+="uaccess"` rule; `2676` is Basler's USB vendor ID).
   **Do not overwrite that file by hand** — it is a dpkg conffile, and
   clobbering it breaks future package upgrades. The older hand-made
   `plugdev` rule was backed up to `69-basler-cameras.rules.manual-backup`
   in this folder.

   Only on a machine with pypylon but *without* the pylon Suite would you add
   a rule manually:
   ```bash
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2676", MODE:="0666", TAG+="uaccess", TAG+="udev-acl"' \
     | sudo tee /etc/udev/rules.d/69-basler-cameras.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger --action=add
   ```
   Replug the camera if it was already connected.
3. Confirm it's visible: `python3 -c "from pypylon import pylon; print([d.GetModelName() for d in pylon.TlFactory.GetInstance().EnumerateDevices()])"`

**Camera settings (exposure / gain / white balance…)** — this app deliberately
applies **none of its own**: whatever configuration is active on the camera is
used as-is, and the console logs a summary of it on every Open ("Camera config
(from camera/pylon setup): …"). Set the image up in the **pylon Viewer**, then
save it via **User Set Control**: save to a User Set (e.g. `UserSet1`) *and*
select that set as the **Startup Set** — otherwise the settings live only in
volatile memory and are lost when the camera loses power (replug), falling
back to the startup set. Close the camera in the Viewer before opening it
here. If the live view is black, the camera's exposure is simply too low —
fix it in the Viewer. (Earlier versions force-enabled continuous
auto-exposure/gain/white-balance on every open; that override has been removed
so the app no longer fights your configuration.)

For a **GigE** Basler instead of USB3, the camera needs an IP on the host's
subnet rather than a udev rule; pypylon then enumerates it the same way.

### Official pylon Viewer GUI

**Installed on this machine**: pylon Software Suite **26.06.2** (with
CodeMeter), installed from Basler's `pylon-26.06.2_linux-aarch64_debs.tar.gz`
via apt — so it is package-managed (`dpkg -l pylon`). Launch the GUI with:

```bash
/opt/pylon/bin/pylonviewer
```

To install on another machine (or upgrade):

- pylon supports Linux aarch64 with **glibc ≥ 2.31**; this Jetson has exactly
  2.31 (Ubuntu 20.04 / JetPack 5.1.1), so it qualifies.
- Basler no longer publishes direct download links — every deep link redirects
  to the portal, so the archive must be fetched by hand (licence click-through):
  <https://www.baslerweb.com/en/downloads/software/> → *pylon Software Suite* →
  *Linux ARM 64 bit*.
- Then run the helper. It handles **both** archive variants Basler ships —
  `pylon-<ver>_linux-aarch64_debs.tar.gz` (Debian packages, installed via apt;
  preferred on Ubuntu) and `pylon_<ver>_aarch64_setup.tar.gz` (plain tarball →
  `/opt/pylon` + `setup-usb.sh`) — and refuses to mix the tarball flavor over
  an apt-managed install:
  ```bash
  ./install_pylon.sh                    # finds the archive in ~/Downloads
  ./install_pylon.sh /path/to/pylon-26.06.2_linux-aarch64_debs.tar.gz
  ```

The Viewer's GUI prerequisites (`libgl1-mesa-dri`, `libgl1-mesa-glx`,
`libxcb-xinerama0`, `libxcb-xinput0`, `libxcb-cursor0`) are already installed on
this machine. Installing `/opt/pylon` does **not** disturb pypylon — pypylon
bundles its own runtime and keeps working regardless.

> **Only one process can open a camera at a time.** Close the camera in the CNC
> Sequencer (**Close** in the camera pane) before opening it in the pylon
> Viewer, or the Viewer will report the device as in use — and vice versa.

## Permissions & USB reset (Jetson / Linux)

The boot sequence runs `sudo usbreset 0483:5740` (the STM32 CDC VID:PID). For
this to work without a password prompt blocking the GUI thread, allow it
passwordless — e.g. add to `/etc/sudoers.d/usbreset`:

```
jetson ALL=(root) NOPASSWD: /usr/bin/usbreset
```

(Use the path `which usbreset` reports — on this machine it is
`/usr/bin/usbreset`.)

(If `usbreset` isn't installed or the sudo rule is missing, the reset step is
skipped gracefully — the app still tries to connect. It just may need a manual
replug if the port is in a bad state.)

Your user must be able to open the serial port. If you get a permission error,
add yourself to the `dialout` group and re-login:

```bash
sudo usermod -aG dialout $USER
```

## How a move completes (why it's reliable)

Each sequence step sends:

```
G90                     ; absolute coordinates
G1 X.. Y.. F<feed>      ; move
M400                    ; Marlin withholds 'ok' until motion finishes
M114                    ; report resulting position
```

The runner waits for the command queue to drain before advancing. Because
`M400` makes the firmware hold its `ok` until the physical move is done,
"queue empty" reliably means "move complete" — the sequencer never races ahead
of the machine. If the link drops mid-move the runner notices and aborts with an
error rather than hanging; a per-move timeout (120 s floor, automatically
extended for long/slow moves based on distance ÷ feed rate) is a final safety
backstop — it stops the machine instead of assuming the move finished.

## Files

| File | Purpose |
|---|---|
| `cnc_sequencer.py` | Main GUI (connection, jog, saved positions, sequence, camera, log) |
| `sequence_runner.py` | `SequenceRunner` QThread — loops positions with dwell/pause/stop + capture |
| `serial_worker.py` | `SerialWorker` QThread — Marlin serial, USB reset, boot, command queue |
| `camera.py` | `CameraController` — webcam (V4L2) background grabber + shared `write_capture` |
| `basler_camera.py` | `BaslerController` — Basler (pypylon) grabber, same interface as the webcam one |
| `cnc_data.json` | Auto-saved settings + positions (created on first save) |
| `captures/` | Saved images (created on first capture) |
| `install_pylon.sh` | Installs Basler's official pylon Suite + Viewer GUI from a downloaded archive (deb or tarball variant) |
| `69-basler-cameras.rules.manual-backup` | Backup of the old hand-made udev rule (superseded by the pylon package's rule) |

### `cnc_data.json` schema

```json
{
  "port": "/dev/serial/by-id/usb-STMicroelectronics_MARLIN_...-if00",
  "camera_source": "webcam",
  "camera_device": "/dev/video0",
  "basler_serial": "",
  "feedrate": 3000,
  "dwell_seconds": 1.0,
  "loop_count": 3,
  "step": 10.0,
  "positions": [
    { "name": "park",   "x": 0.0,   "y": 0.0,   "enabled": true,  "capture": false },
    { "name": "corner", "x": 120.0, "y": 80.0,  "enabled": true,  "capture": true  }
  ]
}
```

## Notes / limits

- **XY only** — this machine has no Z axis; there is no Z control anywhere.
- **Baud is fixed at 115200** (Marlin default here).
- The sequence stops wherever the last position leaves it — it does not return
  to a start/home position automatically. Add a "home" or "park" position as the
  last row if you want that.
- One machine at a time — the app opens a single serial port.
