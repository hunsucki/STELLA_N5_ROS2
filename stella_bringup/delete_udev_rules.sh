#!/bin/bash
set -e

echo "delete remap the devices serial port(ttyUSBX,ttySX) to rplidar, SK120, AHRS, Motordriver, Bluetooth"
echo "sudo rm   /etc/udev/rules.d/stella.rules"
sudo rm -f /etc/udev/rules.d/stella.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "finish  delete"
