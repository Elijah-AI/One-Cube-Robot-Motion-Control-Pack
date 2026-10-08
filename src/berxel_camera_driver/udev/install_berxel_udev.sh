#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Install the Berxel USB udev rules (one-time, per host).
#
# The Berxel Hawk camera is accessed through libusb. On most systems the USB
# device node is owned by root and not readable/writable for a normal user, so
# the SDK cannot enumerate/open the camera ("Failed to query device list
# (ret=-1)" / "Not find device!" in the SDK log). Installing these rules grants
# world access (MODE 0666) to every known Berxel USB product ID, and disables
# USB autosuspend (ATTR{power/control}="on") for the dual-pipe camera PIDs so the
# kernel does not suspend a streaming pipe (which would drop the device mid-run).
#
# This is the same rules file shipped in the Berxel SDK root and is required on
# the FIRST run on any machine (see the SDK readMe: run install.sh as root, then
# replug the device). Afterwards UNPLUG and REPLUG the camera.
#
# Usage:  sudo bash install_berxel_udev.sh
#         (if not root it re-executes itself with sudo)
# -----------------------------------------------------------------------------
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/berxel-usb.rules"
RULES_DST="/etc/udev/rules.d/99-berxel-usb.rules"

if [ ! -f "${RULES_SRC}" ]; then
  echo "ERROR: ${RULES_SRC} not found (run from the package udev/ folder)." >&2
  exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
  echo "Not running as root, re-executing with sudo ..."
  exec sudo bash "$0" "$@"
fi

install -m 644 "${RULES_SRC}" "${RULES_DST}"
udevadm control --reload-rules
udevadm trigger

echo "Installed ${RULES_DST}"
echo "Now UNPLUG and REPLUG the Berxel camera, then start the driver."
echo "Verify with:  lsusb | grep -i berxel   (or by 0603 vendor id)"
