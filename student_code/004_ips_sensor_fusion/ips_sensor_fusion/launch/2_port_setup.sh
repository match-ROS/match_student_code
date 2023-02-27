#! /bin/sh
# needs to be run as sudo
# script to automate port and CAN setup on scout mini
echo "setting port permissions"
chmod 666 /dev/ttyUSB0
chmod 666 /dev/ttyACM0
chmod 666 /dev/ttyACM1
echo "setting up can0"
sudo modprobe gs_usb
ip link set can0 up type can bitrate 500000
echo "done"
exit 0