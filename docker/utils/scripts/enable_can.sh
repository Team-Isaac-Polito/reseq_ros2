#!/bin/bash
# Enable CAN interface on Jetson Orin Nano
# Pin muxing is required to route CAN signals to the J17 connector
# Values per NVIDIA CAN docs: https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/HR/ControllerAreaNetworkCan.html

busybox devmem 0x0c303010 w 0xc400
busybox devmem 0x0c303018 w 0xc458

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 down 2>/dev/null

# Configure CAN FD: 1 Mbps Arbitration (80% SP) / 2 Mbps Data (84% SP)
ip link set can0 up type can bitrate 1000000 sample-point 0.80 dbitrate 2000000 dsample-point 0.84 berr-reporting on fd on
