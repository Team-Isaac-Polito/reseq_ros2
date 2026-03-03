#!/usr/bin/env python3
"""Stop the RPLIDAR motor.

The RPLIDAR A2M8 motor keeps spinning after a non-clean shutdown (e.g. container
removal without lifecycle shutdown).  This script sends the STOP scan command
followed by a SET_MOTOR_PWM=0 packet to halt the motor.

Usage (from container or host with pyserial):
    python3 stop_rplidar.py [/dev/ttyUSB0]
"""

import struct
import sys
import time

try:
    import serial
except ImportError:
    print('pyserial not installed.  Run:  pip install pyserial')
    sys.exit(1)

DEVICE = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'


def stop_rplidar(port: str = DEVICE) -> None:
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 1
    s.dsrdtr = False
    s.dtr = False
    s.open()

    # 1. STOP scan command (0xA5 0x25)
    s.write(bytes([0xA5, 0x25]))
    time.sleep(0.3)

    # 2. SET_MOTOR_PWM = 0 → stops motor
    pwm_data = struct.pack('<H', 0)
    header = bytes([0xA5, 0xF0, len(pwm_data)])
    full_cmd = header + pwm_data
    checksum = 0
    for b in full_cmd:
        checksum ^= b
    full_cmd += bytes([checksum])
    s.write(full_cmd)
    time.sleep(0.3)

    s.dtr = False
    s.close()
    print(f'RPLIDAR motor stopped on {port}')


if __name__ == '__main__':
    stop_rplidar()
