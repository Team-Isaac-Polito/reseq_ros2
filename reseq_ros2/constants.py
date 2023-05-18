# Translation between CAN packet identifiers and ROS topics
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
