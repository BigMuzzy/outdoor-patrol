#!/usr/bin/env bash
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
#
# Install role aliases on the host, never inside the deployment container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_RULES="$ROOT/deploy/udev/99-outdoor-patrol.rules"
SOURCE="$DEFAULT_RULES"
# Test seams: isolate filesystem writes and device verification from the host.
RULES_DIR="${OUTDOOR_PATROL_RULES_DIR:-/etc/udev/rules.d}"
DEV_DIR="${OUTDOOR_PATROL_DEV_DIR:-/dev}"
FULL_RULES="$RULES_DIR/99-outdoor-patrol.rules"
LEGACY_RULES="$RULES_DIR/99-esp32-chassis.rules"
CHASSIS_ONLY=false
VERIFY=false

fail() {
    echo "[device-udev] $*" >&2
    exit 1
}

usage() {
    echo "Usage: $0 [--rules-file PATH] [--verify] [--chassis-only]"
    echo "Full install requires a robot-specific GNSS ID_PATH in the rules."
    echo "--verify checks all four stock role devices after installing."
    echo "--chassis-only preserves the devcontainer's optional chassis setup."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rules-file)
            [[ $# -ge 2 ]] || fail "--rules-file requires a path"
            SOURCE="$2"
            shift 2
            ;;
        --verify) VERIFY=true; shift ;;
        --chassis-only) CHASSIS_ONLY=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) fail "Unknown option: $1 (see --help)" ;;
    esac
done

command -v udevadm >/dev/null 2>&1 || fail "udevadm is required on the host"
[[ -r "$SOURCE" ]] || fail "Cannot read rules: $SOURCE"

chassis_rule() {
    awk '$0 !~ /^[[:space:]]*#/ && /SYMLINK\+="op-chassis esp32-chassis"/' "$1"
}

check_legacy_rules() {
    local legacy chassis original
    [[ -f "$LEGACY_RULES" ]] || return 0
    legacy="$(sed '/^[[:space:]]*\(#.*\)\?$/d' "$LEGACY_RULES")"
    chassis="$(chassis_rule "$DEFAULT_RULES")"
    original="$(printf '%s\n' "$chassis" | sed \
        -e 's/op-chassis esp32-chassis/esp32-chassis/' \
        -e 's/, GROUP="dialout"//' -e 's/, MODE="0660"//')"
    if [[ "$legacy" != "$chassis" && "$legacy" != "$original" &&
          "$legacy" != "$(chassis_rule "$SOURCE")" ]]; then
        if $CHASSIS_ONLY && ! $VERIFY; then
            echo "[device-udev] customized chassis rule left unchanged: $LEGACY_RULES; role aliases were not configured or verified" >&2
            exit 0
        fi
        fail "Legacy rules were customized: $LEGACY_RULES. Merge them into $SOURCE and remove the old file explicitly."
    fi
}

verify_devices() {
    local role device target group
    local -A seen=()
    for role in "$@"; do
        device="$DEV_DIR/op-$role"
        [[ -c "$device" ]] || fail "Missing character device: $device"
        target="$(readlink -f -- "$device")"
        [[ -z "${seen[$target]:-}" ]] || fail "$device shares $target with ${seen[$target]}"
        seen["$target"]="$device"
        group="$(stat -Lc '%G' -- "$device")"
        [[ "$group" == dialout ]] || fail "$device has group $group, expected dialout"
        echo "[device-udev] verified $device -> $target"
    done
}

if $CHASSIS_ONLY && [[ -f "$FULL_RULES" ]]; then
    echo "[device-udev] full host rules already installed; leaving them unchanged"
    if $VERIFY; then
        verify_devices chassis
    fi
    exit 0
fi

TEMP_RULES="$(mktemp)"
trap 'rm -f -- "$TEMP_RULES"' EXIT
REMOVE_LEGACY=false
if $CHASSIS_ONLY; then
    chassis_rule "$SOURCE" > "$TEMP_RULES"
    [[ "$(wc -l < "$TEMP_RULES")" -eq 1 ]] || fail "Expected exactly one chassis rule"
    check_legacy_rules
    DEST="$LEGACY_RULES"
    ROLES=(chassis)
else
    if grep -q '@GNSS_ID_PATH@' "$SOURCE"; then
        fail "GNSS identity is unset. Copy $SOURCE, set its exact ID_PATH from udevadm info, and use --rules-file."
    fi
    for role in chassis gnss imu lidar; do
        grep -Eq "^[^#]*SYMLINK\\+=\"op-$role([ \"])" "$SOURCE" ||
            fail "Missing op-$role alias in $SOURCE"
    done
    cp -- "$SOURCE" "$TEMP_RULES"
    DEST="$FULL_RULES"
    ROLES=(chassis gnss imu lidar)
    if [[ -f "$LEGACY_RULES" ]]; then
        check_legacy_rules
        REMOVE_LEGACY=true
    fi
fi

SUDO=()
if [[ $EUID -ne 0 ]]; then
    SUDO=(sudo)
fi
if cmp -s "$TEMP_RULES" "$DEST"; then
    echo "[device-udev] $DEST already up to date"
    if $CHASSIS_ONLY && ! $VERIFY; then
        exit 0
    fi
else
    "${SUDO[@]}" install -D -m 0644 -- "$TEMP_RULES" "$DEST"
fi
if $REMOVE_LEGACY; then
    "${SUDO[@]}" rm -- "$LEGACY_RULES"
fi
# Explicit full installs (or --verify) retry activation even if an earlier
# run copied the file but failed to reload. The unattended chassis hook is
# a no-op for unchanged rules and never waits for unrelated udev events.
"${SUDO[@]}" udevadm control --reload-rules
"${SUDO[@]}" udevadm trigger --subsystem-match=tty
if ! $CHASSIS_ONLY || $VERIFY; then
    "${SUDO[@]}" udevadm settle --timeout=10
fi
echo "[device-udev] installed and reloaded $DEST"
if $VERIFY; then
    verify_devices "${ROLES[@]}"
else
    echo "[device-udev] hardware not verified; use --verify with the stock devices connected"
fi
