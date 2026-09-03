#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_FILE="${SCRIPT_DIR}/stella.rules"

echo "Configure STELLA serial aliases and GPIO/I2C/Xbox gamepad access"
echo "Serial aliases: /dev/RPLIDAR, /dev/RPLIDAR2, /dev/SK120, /dev/AHRS, /dev/MW, /dev/BT"
echo "start copy stella.rules to  /etc/udev/rules.d/"
echo "$RULES_FILE"
sudo install -m 0644 "$RULES_FILE" /etc/udev/rules.d/stella.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle

echo "Configured hardware devices:"
ls -l /dev/RPLIDAR /dev/RPLIDAR2 /dev/SK120 /dev/AHRS /dev/MW \
    /dev/gpiochip4 /dev/i2c-1 2>/dev/null || true

xbox_event=""
for event in /dev/input/event*; do
    [[ -e "$event" ]] || continue
    if [[ "$(udevadm info -q property -n "$event" 2>/dev/null || true)" == \
        *$'ID_BUS=bluetooth'* ]] && \
       [[ "$(udevadm info -a -n "$event" 2>/dev/null || true)" == \
        *'ATTRS{id/vendor}=="045e"'* ]]; then
        xbox_event="$event"
        break
    fi
done

if [[ -n "$xbox_event" ]]; then
    echo "Xbox gamepad input:"
    ls -l "$xbox_event"
fi

failed=0
for device in /dev/gpiochip4 /dev/i2c-1; do
    if [[ ! -e "$device" ]]; then
        echo "ERROR: required device is missing: $device" >&2
        failed=1
    elif [[ ! -r "$device" || ! -w "$device" ]]; then
        echo "ERROR: current user cannot read/write $device" >&2
        failed=1
    fi
done

if [[ -n "$xbox_event" && ! -r "$xbox_event" ]]; then
    echo "ERROR: current user cannot read Xbox input: $xbox_event" >&2
    echo "Reconnect the controller after installing the udev rule." >&2
    failed=1
fi

if (( failed )); then
    echo "udev setup finished, but required hardware access is not ready." >&2
    exit 1
fi

echo "udev setup finished successfully"
