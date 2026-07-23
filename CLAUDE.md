# CLAUDE.md — cnc-sequencer

Standalone controller for the CoreXY/Marlin CNC (BigTreeTech SKR Mini E3
V3): jog, saved positions, and a sequence runner with optional
per-position image capture (webcam or Basler). Single-window PyQt5 app
(`cnc_sequencer.py`), built from `CNC_STANDALONE_SPEC.md` in the
`pcb-projector-overlay` repo. **README.md is the full manual** —
requirements, degraded modes, capture behavior.

## Setup on a Jetson Orin (JetPack 6 / Ubuntu 22.04 / Python 3.10)

- Clone to `~/Desktop/cnc-sequencer` — the launcher assumes this path
  and user `jetson`.
- Deps (README "Requirements"): PyQt5 + pyserial required;
  opencv-python optional (camera pane); pypylon optional (Basler). The
  shipped `CNC-Sequencer.desktop` runs `/usr/bin/python3`, so install
  for the system interpreter; if PyQt5 has no aarch64 pip wheel, use
  `sudo apt install python3-pyqt5`.
- Basler camera: `./install_pylon.sh` (pylon debs + udev rules), then
  replug the camera. The `69-basler-cameras.rules.manual-backup` file is
  a historical snapshot only — the installer writes the real rules.
- Groups: `sudo usermod -aG dialout,video $USER`, re-login.
- `cp CNC-Sequencer.desktop ~/Desktop/`.
- The app starts fine with no CNC or camera attached (sources grey out)
  — use that as the first smoke test.

## State

- `cnc_data.json` is the entire machine state: serial port (a stable
  `/dev/serial/by-id/...` path tied to the SKR board's serial number —
  the same physical board resolves identically on any device), camera
  source, feedrate, and saved positions. PosA/PosB (X 200 → 205 mm) are
  the 5 mm stereo-step pair used for FoundationStereo Studio captures.
  Commit meaningful changes.
- `captures/` stays untracked on purpose (bulk images).

## Git

- `origin` = github.com/jeck00119/cnc-sequencer (public, the user's
  account).
