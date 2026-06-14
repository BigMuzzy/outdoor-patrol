#!/usr/bin/env bash
# Installs the host udev rule that creates the stable /dev/esp32-chassis
# symlink for the ESP32-S3 micro-ROS chassis controller.
#
# This MUST run on the host (not inside the container): udev and /dev are
# managed by the host kernel, and Docker binds --device nodes at container
# create time. The devcontainer wires this in via "initializeCommand", which
# runs on the host before the container is created.
#
# Idempotent: only reinstalls when the rule is missing or changed.
#
# NOTE: requires sudo. If you don't have passwordless sudo, you'll be
# prompted in the host terminal (or install the rule manually once):
#   sudo cp .devcontainer/99-esp32-chassis.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/99-esp32-chassis.rules"
DEST="/etc/udev/rules.d/99-esp32-chassis.rules"

# Only act on Linux hosts with udev (skip on WSL without udev, macOS, etc.).
if ! command -v udevadm >/dev/null 2>&1; then
	echo "[esp32-udev] udevadm not found; skipping (device passthrough may not work)."
	exit 0
fi

if cmp -s "$SRC" "$DEST" 2>/dev/null; then
	echo "[esp32-udev] rule already up to date."
else
	echo "[esp32-udev] installing $DEST (may prompt for sudo password)..."
	sudo cp "$SRC" "$DEST"
	sudo udevadm control --reload-rules
	sudo udevadm trigger --subsystem-match=tty
	echo "[esp32-udev] installed and reloaded."
fi
