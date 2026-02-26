#!/bin/bash
# Enable CAN interface on Jetson Orin Nano
# Pin muxing is required to route CAN signals to the 40-pin header

busybox devmem 0x0c303010 w 0x400
busybox devmem 0x0c303018 w 0x458

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 down 2>/dev/null
ip link set can0 up type can bitrate 125000
