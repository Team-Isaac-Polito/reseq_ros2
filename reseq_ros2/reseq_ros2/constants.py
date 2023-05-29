from math import pi

### Translation between CAN packet identifiers and ROS topics ###
# ROS topics will be created based on this file

# From CAN to ROS = ROS publishers
# This contains any feedback coming from the robot
id_to_topic = {
    0x11: "battery/percent",
    0x12: "battery/voltage",
    0x13: "battery/temp",

    0x22: "motor/feedback",
    0X23: "motor/temp",
    0x24: "motor/current",

    0x32: "joint/yaw/feedback",
    0x34: "joint/pitch/feedback",
    0x36: "joint/roll/feedback",
}

# From ROS to CAN = ROS subscribers
# This contains all information (setpoints) sent to the robot
topic_to_id = {
    "motor/setpoint": 0x21,

    "joint/yaw/setpoint": 0x31,
    "joint/pitch/setpoint": 0x33,
    "joint/roll/setpoint": 0x35,
}

### Agevar ###

# GEOMETRIC CONSTANTS
a = 0.15        # [m] Distance between the center of every module and the previous yaw joint
b = 0.1         # [m] Distance between the center of every module and the following yaw joint
d = 0.13        # [m] Distance between pair of equivalent wheels
r_eq = 0.06     # [m] Radius of the equivalent wheels

rpm2rads = 2*pi/60
rads2rpm = 60/(2*pi)