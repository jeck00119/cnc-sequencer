#!/usr/bin/env bash
#
# install_pylon.sh — install the official Basler pylon Software Suite (incl.
# the pylon Viewer GUI) from an archive downloaded from Basler's website.
#
# Basler no longer serves direct download links (every deep link 301s to the
# portal), so the archive has to be fetched by hand first:
#
#   https://www.baslerweb.com/en/downloads/software/
#     → pylon Software Suite · Linux ARM 64 bit  (accept the licence)
#
# Basler ships TWO archive variants; this script handles both:
#   pylon-<ver>_linux-aarch64_debs.tar.gz   → Debian packages (preferred on
#                                             Ubuntu; installed via apt)
#   pylon_<ver>_aarch64_setup.tar.gz        → plain tarball (unpacked to
#                                             /opt/pylon + setup-usb.sh)
#
# Then run:   ./install_pylon.sh [path/to/archive.tar.gz]
# With no argument it looks in ~/Downloads, $HOME, ~/Desktop and the current
# directory.

set -euo pipefail

die() { echo "ERROR: $*" >&2; exit 1; }
note() { echo "==> $*"; }

# --- locate the archive -----------------------------------------------------
if [[ $# -ge 1 ]]; then
    ARCHIVE="$1"
    [[ -f "$ARCHIVE" ]] || die "no such file: $ARCHIVE"
else
    note "Looking for a pylon archive in ~/Downloads, \$HOME, ~/Desktop and $(pwd) …"
    # Newest match wins. Deliberately broad: matches both the _debs and the
    # _setup naming. $HOME is included because this box once had
    # XDG_DOWNLOAD_DIR collapsed to $HOME, so downloads landed there.
    ARCHIVE=$(ls -t \
        "$HOME"/Downloads/pylon*aarch64*.tar.gz \
        "$HOME"/pylon*aarch64*.tar.gz \
        "$HOME"/Desktop/pylon*aarch64*.tar.gz \
        ./pylon*aarch64*.tar.gz 2>/dev/null | head -1 || true)
    [[ -n "$ARCHIVE" ]] || die "no pylon aarch64 archive found.
Download it first from https://www.baslerweb.com/en/downloads/software/
(pylon Software Suite → Linux ARM 64 bit), then re-run this script,
or pass the path explicitly:  $0 /path/to/pylon-...-aarch64....tar.gz"
fi
note "Using archive: $ARCHIVE"

# --- sanity checks ----------------------------------------------------------
[[ "$(uname -m)" == "aarch64" ]] || die "this machine is $(uname -m), not aarch64"

if [[ "$ARCHIVE" != *aarch64* ]]; then
    die "'$ARCHIVE' does not look like an aarch64 build — wrong architecture?"
fi

# Guard against a truncated / HTML-error-page download: the portal serves a
# ~200 KB HTML page for bad links, which would otherwise fail confusingly later.
if ! tar -tzf "$ARCHIVE" >/dev/null 2>&1; then
    die "'$ARCHIVE' is not a valid gzip archive.
If it is only a few hundred KB it is probably Basler's HTML portal page rather
than the real download — fetch it again through the website."
fi

# --- extract & detect the variant -------------------------------------------
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

note "Extracting archive…"
tar -C "$WORK" -xzf "$ARCHIVE"

PYLON_DEB=$(find "$WORK" -maxdepth 2 -name "pylon_*.deb" | head -1 || true)
INNER_TAR=$(find "$WORK" -maxdepth 2 -name "pylon*.tar.gz" | head -1 || true)

# --- variant 1: Debian packages (preferred on Ubuntu) ------------------------
if [[ -n "$PYLON_DEB" ]]; then
    note "Debian-package variant detected."
    NEW_VER=$(dpkg-deb -f "$PYLON_DEB" Version 2>/dev/null || echo "?")
    CUR_VER=$(dpkg-query -W -f '${Version}' pylon 2>/dev/null || true)
    if [[ -n "$CUR_VER" && "$CUR_VER" == "$NEW_VER" ]]; then
        note "pylon $CUR_VER is already installed — nothing to do."
        /opt/pylon/bin/pylonviewer --version 2>/dev/null || true
        echo "Launch the GUI with:  /opt/pylon/bin/pylonviewer"
        exit 0
    fi
    CODEMETER=$(find "$WORK" -maxdepth 2 -name "codemeter*.deb" | head -1 || true)
    note "Installing pylon $NEW_VER via apt (this may take a minute)…"
    # --force-confnew: take the package's udev rule; a pre-existing hand-made
    # /etc/udev/rules.d/69-basler-cameras.rules would otherwise make dpkg
    # stop at an interactive conffile prompt.
    sudo apt-get install -y -o Dpkg::Options::="--force-confnew" \
        "$PYLON_DEB" ${CODEMETER:+"$CODEMETER"}

# --- variant 2: plain tarball -----------------------------------------------
elif [[ -n "$INNER_TAR" ]]; then
    note "Tarball (setup) variant detected."
    # Refuse to untar over a dpkg-managed install: that would clobber
    # package-owned files and break future apt upgrades/removals.
    if dpkg -s pylon >/dev/null 2>&1; then
        die "pylon is already installed as a Debian package \
($(dpkg-query -W -f '${Version}' pylon 2>/dev/null)).
Mixing the tarball install into /opt/pylon would corrupt the apt-managed
files. Either download the _debs variant instead, or first remove the
package:  sudo apt-get remove pylon"
    fi
    note "Installing to /opt/pylon (needs sudo)…"
    sudo mkdir -p /opt/pylon
    sudo tar -C /opt/pylon -xzf "$INNER_TAR"
    sudo chmod 755 /opt/pylon
    note "Installing udev rules for Basler USB cameras…"
    if [[ -x /opt/pylon/share/pylon/setup-usb.sh ]]; then
        sudo /opt/pylon/share/pylon/setup-usb.sh
    else
        echo "WARNING: setup-usb.sh not found — skipping udev setup." >&2
    fi

else
    die "unrecognised archive layout — found neither pylon_*.deb nor an inner
pylon*.tar.gz. Contents:
$(find "$WORK" -maxdepth 2 | head -20)"
fi

# --- report -----------------------------------------------------------------
echo
note "Done. Installed version:"
if [[ -x /opt/pylon/bin/pylonviewer ]]; then
    /opt/pylon/bin/pylonviewer --version 2>/dev/null || true
    cat <<'EOF'

Launch the GUI with:

    /opt/pylon/bin/pylonviewer

Unplug and replug the camera once so the udev rules take effect (and log out
and back in once so the pylon environment variables apply).

NOTE: a camera can only be opened by one process at a time — close the camera
in the CNC Sequencer (Camera → Close) before opening it in the pylon Viewer,
and vice versa.
EOF
else
    echo "WARNING: /opt/pylon/bin/pylonviewer not found after install." >&2
    ls /opt/pylon/bin 2>/dev/null >&2 || true
    exit 1
fi
