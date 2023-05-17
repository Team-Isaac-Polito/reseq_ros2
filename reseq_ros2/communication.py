import rclpy
import can
import yaml
import os
from can import Message
from rclpy.node import Node
from yaml.loader import SafeLoader
from std_msgs.msg import Float32 # deprecated?
import struct
import reseq_ros2.constants as cc


class Communication(Node):
    def __init__(self):
        super().__init__('communication')
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/reseq_ros2/config.yaml") as f:
            self.data = yaml.load(f, Loader=SafeLoader)
            print(self.data)

        # create publishers and subscribers for each module
        self.pubs = []
        self.subs = []
        for i in range(len(self.data["modules"])):
            self.pubs.append(self.create_module_pubs(self.data["modules"][i]))
            self.subs.append(self.create_module_subs(self.data["modules"][i]))

        # canbus
        try:
            self.canbus = can.interface.Bus(
                channel='vcan0', bustype='socketcan')
            self.notifier = can.Notifier(self.canbus, [self.can_callback])
        except OSError as e:
            print("Error with canbus: " + str(e))

        self.get_logger().info("Communication node started")

    def create_module_pubs(self, info):
        d = {}

        d["motor_setpoint"] = self.create_publisher(
            Float32, f'/module{info["address"]}/motor/setpoint', 10)

        d["joint_yaw_setpoint"] = self.create_publisher(
            Float32, f'/module{info["address"]}/joint/yaw/setpoint', 10)
        d["joint_pitch_setpoint"] = self.create_publisher(
            Float32, f'/module{info["address"]}/joint/pitch/setpoint', 10)
        d["joint_roll_setpoint"] = self.create_publisher(
            Float32, f'/module{info["address"]}/joint/roll/setpoint', 10)
        return d

    def create_module_subs(self, info):
        d = {}
        # using lamba function to add extra arguments (module id and topic name) to the callback
        d["battery_percent"] = self.create_subscription(
            Float32, f'/module{info["address"]}/battery/percent', lambda msg: self.ros_listener_callback(msg, info["address"], "battery_percent"), 10)
        d["battery_voltage"] = self.create_subscription(
            Float32, f'/module{info["address"]}/battery/voltage', lambda msg: self.ros_listener_callback(msg, info["address"], "battery_voltage"), 10)
        d["battery_temperature"] = self.create_subscription(
            Float32, f'/module{info["address"]}/battery/temperature', lambda msg: self.ros_listener_callback(msg, info["address"], "battery_temperature"), 10)

        d["motor_feedback"] = self.create_subscription(
            Float32, f'/module{info["address"]}/motor/feedback', lambda msg: self.ros_listener_callback(msg, info["address"], "motor_feedback"), 10)
        d["battery_current"] = self.create_subscription(
            Float32, f'/module{info["address"]}/battery/current', lambda msg: self.ros_listener_callback(msg, info["address"], "battery_current"), 10)
        d["battery_voltage"] = self.create_subscription(
            Float32, f'/module{info["address"]}/battery/voltage', lambda msg: self.ros_listener_callback(msg, info["address"], "battery_voltage"), 10)

        if info["hasJoint"]:
            d["joint_yaw_feedback"] = self.create_subscription(
                Float32, f'/module{info["address"]}/joint/yaw/feedback', lambda msg: self.ros_listener_callback(msg, info["address"], "joint_yaw_feedback"), 10)
            d["joint_roll_feedback"] = self.create_subscription(
                Float32, f'/module{info["address"]}/joint/roll/feedback', lambda msg: self.ros_listener_callback(msg, info["address"], "joint_roll_feedback"), 10)
            d["joint_pitch_feedback"] = self.create_subscription(
                Float32, f'/module{info["address"]}/joint/pitch/feedback', lambda msg: self.ros_listener_callback(msg, info["address"], "joint_pitch_feedback"), 10)

        return d

    # publish data received from CAN to ROS topic
    def can_callback(self, msg):
        # TODO: decode arbitration ID
        # id = struct.unpack("cccc", bytes(msg.arbitration_id))
        decoded_aid = (0, 0x21, 0x03, 18)

        # create ROS message
        m = Float32()
        m.data = 12.0
        topic_name = cc.id_to_topic[decoded_aid[1]]

        self.pubs[decoded_aid[3]-17][topic_name].publish(m)

    # publish data received from ROS to CAN
    def ros_listener_callback(self, msg, module_num, topic):
        # TODO: create arbitration id from identifier, destination and source
        # based on the topic the message was published to
        msg = can.Message(
            arbitration_id=0x00030411,
            data=[0, 25, 0, 1, 3, 1, 4, 1],
            is_extended_id=True,
        )
        self.canbus.send(msg)
        # package data into bytearray


def main(args=None):
    rclpy.init(args=args)
    communication = Communication()
    rclpy.spin(communication)
    communication.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
