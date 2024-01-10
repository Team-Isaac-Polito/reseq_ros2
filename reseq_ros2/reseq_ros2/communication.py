import can
import rclpy
import reseq_ros2.constants as rc
import struct
import yaml
from rclpy.node import Node
from reseq_interfaces.msg import Motors
from std_msgs.msg import Float32, Int32  # deprecated?
from yaml.loader import SafeLoader

"""ROS node that handles communication between the Jetson and each module via CAN

It receives data from each module via CAN and publishes it to "feedback" ROS topics
for Agevar to read. It receives instructions from Agevar over "motor/setpoint" ROS topics
on the motors velocity and sends it to the modules via CAN. It receives instructions from 
the Scaler node for the end_effector over end_effector/.../setpoint

See team's wiki for details on how data is packaged inside CAN messages.
"""

class Communication(Node):
    def __init__(self):
        super().__init__("communication")
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/reseq_ros2/config.yaml") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
            print(self.config)

        # create ROS publishers and subscribers for each module based on config file
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

    # create ROS publishers for a module based on its properties
    def create_module_pubs(self, info):
        d = {}
        for topic in self.topics_from_direction(rc.Direction.IN):
            if topic.name.split("/")[0] == "joint" and info["hasJoint"] == False:
                continue

            if topic.name.split("/")[0] == "end_effector" and info["hasEndEffector"] == False: 
                continue

            d[topic.name] = self.create_publisher(
                topic.data_type,
                f"reseq/module{info['address']}/{topic.name}",
                10,
            )

        return d

    # create ROS subscribers for a module based on its properties
    def create_module_subs(self, info):
        d = {}
        for topic in self.topics_from_direction(rc.Direction.OUT):
            if topic.name.split("/")[0] == "joint" and info["hasJoint"] == False:
                continue

            if topic.name.split("/")[0] == "end_effector" and info["hasEndEffector"] == False: 
                continue

            d[topic.name] = self.create_subscription(
                topic.data_type,
                f"reseq/module{info['address']}/{topic.name}",
                # use lamba function to pass extra arguments to the callback
                lambda msg, s=topic.name: self.ros_listener_callback(
                    msg, info["address"], s),
                10,
            )

        return d

    # publish data received from CAN to ROS topic
    def can_callback(self, msg):
        decoded_aid = struct.unpack(
            "4b", msg.arbitration_id.to_bytes(4, "big"))
        module_id = decoded_aid[3] - 17
        topic = self.topic_from_id(decoded_aid[1])

        print(f"Publishing to {topic.name} on module{module_id+17}")
        self.get_logger().info(f"Publishing to {topic.name} on module{module_id+17}")

        # check if the message contains one or two floats
        if topic.data_type == Float32:
            m = Float32()
            data = struct.unpack('f', msg.data)
            m.data = data[0]
        elif topic.data_type == Int32:
            m = Int32()
            data = struct.unpack('i', msg.data)
            m.data = data[0]
        elif topic.data_type == Motors:
            m = Motors()
            data = struct.unpack('ff', msg.data)
            print(data)
            m.left = data[0]
            m.right = data[1]

        try:
            self.pubs[module_id][topic.name].publish(m)
        except TypeError as e:
            # the type of ROS message doesn't match the publisher
            print("TypeError", e)
        except IndexError as e:
            print("IndexError", e)
            print(f"{module_id} out of range 0 to {len(self.pubs)-1}")

    # send data received from ROS to CAN
    def ros_listener_callback(self, msg, module_num, topic_name):
        topic = self.topic_from_name(topic_name)
        aid = struct.pack("bbbb", 00, topic.can_id, module_num, 0x00)

        print(f"Sending {type(msg)} to module{module_num} via CAN")

        if topic.data_type is Float32:
            data = struct.pack('f', msg.data)
        elif topic.data_type is Int32:
            data = struct.pack('i', msg.data)
        elif topic.data_type is Motors:
            data = struct.pack('ff', msg.left, msg.right)

        m = can.Message(
            arbitration_id=int.from_bytes(aid, byteorder="big", signed=False),
            data=data,
            is_extended_id=True,
        )
        self.canbus.send(m)
    
    def topics_from_direction(self, d: rc.Direction): 
        return list(filter(lambda x: x.direction == d, rc.topics))
    
    def topic_from_id(self, id: int) -> rc.ReseQTopic:
        return next(filter(lambda x: x.can_id == id, rc.topics))
    
    def topic_from_name(self, name: str) -> rc.ReseQTopic:
        return next(filter(lambda x: x.name == name, rc.topics))


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
