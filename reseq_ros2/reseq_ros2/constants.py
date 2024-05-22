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

class StateType(Enum):
    MOTOR_FEEDBACK = 0
    JOINT_FEEDBACK = 1
    END_EFFECTOR_FEEDBACK = 2

class ReseQTopic(NamedTuple):
    """
    Data structure for the translation of ROS topics to CAN packets.
    """
    name: str
    can_id: int
    direction: Direction
    data_type: type

class ReseQState(NamedTuple):
    """
    Data structure for JointState operations
    """
    topic: str
    states: list[str]
    state_type: StateType

states = (
    ReseQState("motor/feedback", ["left_front_wheel", "left_back_wheel", "right_front_wheel", "right_back_wheel"], StateType.MOTOR_FEEDBACK),
    
    ReseQState("joint/yaw/feedback", ["joint_y"], StateType.JOINT_FEEDBACK),
    ReseQState("joint/pitch/feedback", ["joint_p"], StateType.JOINT_FEEDBACK),
    ReseQState("joint/roll/feedback", ["joint_r"], StateType.JOINT_FEEDBACK),

    ReseQState("end_effector/pitch/feedback", ["arm_pitch"], StateType.END_EFFECTOR_FEEDBACK),
    ReseQState("end_effector/head_pitch/feedback", ["arm_head_pitch"], StateType.END_EFFECTOR_FEEDBACK),
    ReseQState("end_effector/head_yaw/feedback", ["arm_head_yaw"], StateType.END_EFFECTOR_FEEDBACK)
)

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
    ReseQTopic("end_effector/head_roll/setpoint", 0x45, Direction.OUT, Int32),
    ReseQTopic("end_effector/head_roll/feedback", 0x46, Direction.IN, Int32),
)

share_folder = get_package_share_directory("reseq_ros2")

rpm2rads = 2*pi/60
rads2rpm = 60/(2*pi)

sample_time: Final = 0.08

lsb_to_rads: Final = 5.1183e-3 # [rad/LSB] Conversion from Dynamixel position LSB to radians
