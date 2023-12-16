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

    0x42: "end_effector/pitch/feedback",
    0x44: "end_effector/head_pitch/feedback",
    0x46: "end_effector/head_yaw/feedback",
}

# From ROS to CAN = ROS subscribers
# This contains all information (setpoints) sent to the robot
topic_to_id = {
    "motor/setpoint": 0x21,

    "joint/yaw/setpoint": 0x31,
    "joint/pitch/setpoint": 0x33,
    "joint/roll/setpoint": 0x35,

    "end_effector/pitch/setpoint": 0x41,
    "end_effector/head_pitch/setpoint": 0x43,
    "end_effector/head_yaw/setpoint": 0x45,
}

### Agevar ###

# GEOMETRIC CONSTANTS
a = 0.18        # [m] Distance between the center of every module and the previous yaw joint
b = 0.18        # [m] Distance between the center of every module and the following yaw joint
d = 0.21        # [m] Distance between pair of equivalent wheels
r_eq = 0.05     # [m] Radius of the equivalent wheels

rpm2rads = 2*pi/60
rads2rpm = 60/(2*pi)