from math import pi
from typing import Final, NamedTuple
from enum import Enum
from std_msgs.msg import Int32, Float32
from reseq_interfaces.msg import Motors
from ament_index_python.packages import get_package_share_directory

class Direction(Enum):
    """
    Represents the possible directions of data though the 
    communication node into ROS: 
    - `IN`: from CAN (robot) to ROS - (ROS publishers)
    - `OUT`: from ROS to CAN - (ROS subscribers)
    """
    IN = 0
    OUT = 1

class ReseQTopic(NamedTuple):
    """
    Data structure for the translation of ROS topics to CAN packets.
    """
    name: str
    can_id: int
    direction: Direction
    data_type: type

topics = (
    ReseQTopic("battery/percent", 0x11, Direction.IN, Float32),
    ReseQTopic("battery/voltage", 0x12, Direction.IN, Float32),
    ReseQTopic("battery/temp", 0x13, Direction.IN, Float32),

    ReseQTopic("motor/setpoint", 0x21, Direction.OUT, Motors),

    ReseQTopic("motor/feedback", 0x22, Direction.IN, Motors),
    ReseQTopic("motor/temp", 0x23, Direction.IN, Motors),
    ReseQTopic("motor/current", 0x24, Direction.IN, Motors),

    ReseQTopic("joint/yaw/setpoint", 0x31, Direction.OUT, Float32),
    ReseQTopic("joint/yaw/feedback", 0x32, Direction.IN, Float32),
    ReseQTopic("joint/pitch/setpoint", 0x33, Direction.OUT, Float32),
    ReseQTopic("joint/pitch/feedback", 0x34, Direction.IN, Float32),
    ReseQTopic("joint/roll/setpoint", 0x35, Direction.OUT, Float32),
    ReseQTopic("joint/roll/feedback", 0x36, Direction.IN, Float32),

    ReseQTopic("end_effector/pitch/setpoint", 0x41, Direction.OUT, Int32),
    ReseQTopic("end_effector/pitch/feedback", 0x42, Direction.IN, Int32),
    ReseQTopic("end_effector/head_pitch/setpoint", 0x43, Direction.OUT, Int32),
    ReseQTopic("end_effector/head_pitch/feedback", 0x44, Direction.IN, Int32),
    ReseQTopic("end_effector/head_yaw/setpoint", 0x45, Direction.OUT, Int32),
    ReseQTopic("end_effector/head_yaw/feedback", 0x46, Direction.IN, Int32),
)

share_folder = get_package_share_directory("reseq_ros2")

### Agevar ###

# GEOMETRIC CONSTANTS
a = 0.18        # [m] Distance between the center of every module and the previous yaw joint
b = 0.18        # [m] Distance between the center of every module and the following yaw joint
d = 0.21        # [m] Distance between pair of equivalent wheels
r_eq = 0.05     # [m] Radius of the equivalent wheels

rpm2rads = 2*pi/60
rads2rpm = 60/(2*pi)

### EnEA, specified in WIKI ###
i_pitch: Final = 567               # [LSB] Initial arm pitch
i_head_pitch: Final = 663          # [LSB] Initial head pitch
i_head_yaw: Final = 512            # [LSB] Initial head yaw
servo_speed: Final = 200           # [LSB] Speed of the servomotor

r_pitch: Final = (200, 750)        # [LSB] Range of arm pitch angles
r_head_pitch: Final = (0, 800)     # [LSB] Range of head pitch angles
r_head_yaw: Final = (0, 1023)      # [LSB] Range of head yaw angles
pitch_conv: Final = 0.75           # [ ] Conversion of pitch increase to head pitch increase

### Scaler, specified in WIKI ###
r_linear_vel: Final = (-0.29297, 0.29297)       # [m/s] Range of the linear velocity
r_inverse_radius: Final = (-1.5385, 1.5385)     # [1/m] Range of the inverse radius of curvature
r_pitch_vel: Final = (-183, 183)                # [LSB/s] Range of pitch velocity
r_head_pitch_vel: Final = (-400, 400)           # [LSB/s] Range of indepepndent head pitch velocity
r_head_yaw_vel: Final = (-455, 455)             # [LSB/s] Range of head yaw velocity
