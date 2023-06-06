import can
import rclpy
import reseq_ros2.constants_mk2 as rc
import struct
import yaml
from can import Message
from rclpy.node import Node
from reseq_interfaces.msg import Motors
from std_msgs.msg import Float32  # deprecated?
from yaml.loader import SafeLoader


class Communication(Node):
    def __init__(self):
        super().__init__("communication")
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/config/config_mk2.yaml") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
            print(self.config)

        # create ROS publishers and subscribers for each module
        self.pubs = []
        self.subs = []
        for i in range(len(self.config["modules"])):
            self.pubs.append(self.create_module_pubs(
                self.config["modules"][i]))
            self.subs.append(self.create_module_subs(
                self.config["modules"][i]))

        # connect to CAN bus
        try:
            self.canbus = can.interface.Bus(
                channel=self.config["canbus"]["channel"],
                bustype="socketcan",
            )
            self.notifier = can.Notifier(self.canbus, [self.can_callback])
        except OSError:
            raise

        self.get_logger().info("Communication node started")

    def create_module_pubs(self, info):
        d = {}
        for subtopic in rc.id_to_topic.values():
            if subtopic.split("/")[0] == "joint" and info["hasJoint"] == False:
                continue

            d[subtopic] = self.create_publisher(
                Motors if subtopic.split("/")[0] == "motor" else Float32,
                f"reseq/module{info['address']}/{subtopic}",
                10,
            )

        return d

    def create_module_subs(self, info):
        d = {}
        for subtopic in rc.topic_to_id.keys():
            if subtopic.split("/")[0] == "joint" and info["hasJoint"] == False:
                continue

            d[subtopic] = self.create_subscription(
                Motors if subtopic.split("/")[0] == "motor" else Float32,
                f"reseq/module{info['address']}/{subtopic}",
                # use lamba function to pass extra arguments to the callback
                lambda msg, s=subtopic: self.ros_listener_callback(
                    msg, info["address"], s),
                10,
            )

        return d

    # publish data received from CAN to ROS topic
    def can_callback(self, msg):
        decoded_aid = struct.unpack(
            "4b", msg.arbitration_id.to_bytes(4, "big"))
        module_id = decoded_aid[3] - 17
        topic_name = rc.id_to_topic[decoded_aid[1]]

        print(f"Publishing to {topic_name} on module{module_id+17}")

        # check if the message contains one or two floats
        if len(msg.data) == 4:
            m = Float32()
            data = struct.unpack('f', msg.data)
            m.data = data[0]
        elif len(msg.data) == 8:
            m = Motors()
            data = struct.unpack('ff', msg.data)
            print(data)
            m.left = data[0]
            m.right = data[1]

        try:
            self.pubs[module_id][topic_name].publish(m)
        except TypeError as e:
            # the type of ROS message doesn't match the publisher
            print("TypeError", e)

    # send data received from ROS to CAN
    def ros_listener_callback(self, msg, module_num, topic_name):
        identifier = rc.topic_to_id[topic_name]
        aid = struct.pack("bbbb", 00, identifier, module_num, 0x00)

        print(f"Sending {type(msg)} to module{module_num} via CAN")

        if type(msg) is Float32:
            data = struct.pack('f', msg.data)
        elif type(msg) is Motors:
            data = struct.pack('ff', msg.left, msg.right)

        m = can.Message(
            arbitration_id=int.from_bytes(aid, byteorder="big", signed=False),
            data=data,
            is_extended_id=True,
        )
        self.canbus.send(m)


def main(args=None):
    rclpy.init(args=args)
    try:
        communication = Communication()
    except Exception as err:
        print("Error while starting Communication node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(communication)
        communication.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
