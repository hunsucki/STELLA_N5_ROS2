#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_FILE="${SCRIPT_DIR}/stella.rules"

echo "remap the devices serial port(ttyUSBX, ttySX) to rplidar, SK120, AHRS, Motordriver, Bluetooth"
echo "devices usb connection as /dev/RPLIDAR, /dev/RPLIDAR2, /dev/SK120, /dev/AHRS, /dev/MW, /dev/BT"
echo "check it using the command : ls -l /dev|grep -e ttyUSB -e ttyS0 -e RPLIDAR -e SK120 -e AHRS -e MW"
echo "start copy stella.rules to  /etc/udev/rules.d/"
echo "$RULES_FILE"
sudo cp "$RULES_FILE" /etc/udev/rules.d/stella.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "finish "
