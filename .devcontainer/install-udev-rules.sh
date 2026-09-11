#!/usr/bin/env bash
# Backward-compatible, optional chassis-only setup for devcontainer hosts.
# Full, locally configured role rules are never overwritten at container start.
set -euo pipefail

if ! command -v udevadm >/dev/null 2>&1; then
	echo "[device-udev] udevadm not found; skipping (role symlinks may be unavailable)."
	exit 0
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/scripts/install-device-rules.sh" --chassis-only "$@"
