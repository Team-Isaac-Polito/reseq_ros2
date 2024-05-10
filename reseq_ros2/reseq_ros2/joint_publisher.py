from math import pi
from itertools import chain
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32
import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors


class State:
    name: str
    value: float
    def __init__(self, name, value) -> None:
        self.name = name
        self.value = value
    def update(self, value):
        self.value = value
    


class JointPublisher(Node):
    def __init__(self):
        super().__init__("joint_publisher")
        self.get_logger().info("JointPublisher node started")

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        modules = (
            self.declare_parameter("modules", [0])
            .get_parameter_value()
            .integer_array_value
        )

        joints = (
            self.declare_parameter("joints", [0])
            .get_parameter_value()
            .integer_array_value
        )

        end_effector = (
            self.declare_parameter("end_effector", 0)
            .get_parameter_value()
            .integer_value
        )

        self.arm_pitch_origin = (
            self.declare_parameter("arm_pitch_origin", 0)
            .get_parameter_value()
            .integer_value
        )
        self.head_pitch_origin = (
            self.declare_parameter("head_pitch_origin", 0)
            .get_parameter_value()
            .integer_value
        )
        self.head_yaw_origin = (
            self.declare_parameter("head_yaw_origin", 0)
            .get_parameter_value()
            .integer_value
        )

        self.velocity_gain = (
            self.declare_parameter("vel_gain", 0.0)
            .get_parameter_value()
            .double_value
        )

        self.arm_pitch_gain = (
            self.declare_parameter("arm_pitch_gain", 0.0)
            .get_parameter_value()
            .double_value
        )

        self.velocity_states = {}
        self.position_states = {}

        self.init_state(modules, joints, end_effector)
        self.create_subs(modules, joints, end_effector)

        self.get_logger().info(f"States initialized")

        self.create_timer(rc.sample_time, self.broadcast_states)

    def init_state(self, modules: list[int], joints: list[int], end_effector: int):
        for mod in modules:
            id = mod % 16

            self.velocity_states.update({
                (mod, x): State(x + f"_{id}_joint", 0.0)
                for x in self.states_from_type(rc.StateType.MOTOR_FEEDBACK)
            })

            if mod in joints:
                self.position_states.update({
                    (mod, x): State(x + f"_{id}_joint", 0.0)
                    for x in self.states_from_type(rc.StateType.JOINT_FEEDBACK)
                })

            if mod == end_effector:
                self.position_states.update({
                    (mod, x): State(x + f"_{id}_joint", 0.0)
                    for x in self.states_from_type(
                        rc.StateType.END_EFFECTOR_FEEDBACK
                    )
                })

    def create_subs(self, modules: list[int], joints: list[int], end_effector: int):
        for mod in modules:
            for topic in self.topics_from_type(rc.StateType.MOTOR_FEEDBACK):
                self.create_subscription(
                    Motors,
                    f"reseq/module{mod}/{topic}",
                    lambda msg, addr=mod, tp=topic: self.update_callback(
                        msg, addr, tp, rc.StateType.MOTOR_FEEDBACK
                    ), 
                    10,
                )
            if mod in joints:
                for topic in self.topics_from_type(rc.StateType.JOINT_FEEDBACK):
                    self.create_subscription(
                        Float32,
                        f"reseq/module{mod}/{topic}",
                        lambda msg, addr=mod, tp=topic: self.update_callback(
                            msg, addr, tp, rc.StateType.JOINT_FEEDBACK
                        ),
                        10,
                    )
            if mod is end_effector:
                for topic in self.topics_from_type(rc.StateType.END_EFFECTOR_FEEDBACK):
                    self.create_subscription(
                        Int32,
                        f"reseq/module{mod}/{topic}",
                        lambda msg, addr=mod, tp=topic: self.update_callback(
                            msg, addr, tp, rc.StateType.END_EFFECTOR_FEEDBACK
                        ),
                        10,
                    )

    def broadcast_states(self):

        velocity = JointState()
        now = self.get_clock().now()
        velocity.header.stamp = now.to_msg()
        velocity.name = [x.name for x in self.velocity_states.values()]
        velocity.velocity = [x.value for x in self.velocity_states.values()]

        self.joint_pub.publish(velocity)

        position = JointState()
        now = self.get_clock().now()
        position.header.stamp = now.to_msg()
        position.name = [x.name for x in self.position_states.values()]
        position.position = [x.value for x in self.position_states.values()]

        self.joint_pub.publish(position)

    def update_callback(
        self,
        msg: Motors | Float32 | Int32,
        address: int,
        topic: str,
        state_t: rc.StateType,
    ):
        if state_t == rc.StateType.MOTOR_FEEDBACK:
            l, r = msg.left, msg.right

            l *= self.velocity_gain
            r *= self.velocity_gain


            for st in self.states_from_topic(topic):
                if "left" in st:
                    self.velocity_states[(address, st)].update(l)
                else:
                    self.velocity_states[(address, st)].update(r)

        if state_t == rc.StateType.JOINT_FEEDBACK:
            angle = msg.data

            if angle >= 180:
                angle -= 360

            st = self.states_from_topic(topic)[0]
            self.position_states[(address, st)].update(angle * pi / 180.0)

        if state_t == rc.StateType.END_EFFECTOR_FEEDBACK:
            lsb = msg.data
            st = self.states_from_topic(topic)[0]
            if topic == "end_effector/pitch/feedback":
                self.position_states[(address, st)].update(
                    (lsb - self.arm_pitch_origin) * rc.lsb_to_rads * self.arm_pitch_gain
                )
            if topic == "end_effector/head_pitch/feedback":
                self.position_states[(address, st)].update(
                    (lsb - self.head_pitch_origin) * rc.lsb_to_rads * (-1)
                )
            if topic == "end_effector/head_yaw/feedback":
                self.position_states[(address, st)].update(
                    (lsb - self.arm_pitch_origin) * rc.lsb_to_rads
                )
            

    def states_from_type(self, state_type: rc.StateType) -> list[str]:
        return list(
            chain.from_iterable(
                [x.states for x in rc.states if x.state_type == state_type]
            )
        )

    def states_from_topic(self, topic: str) -> list[str]:
        return next(map(lambda x: x.states, filter(lambda x: x.topic == topic, rc.states)))

    def topics_from_type(self, state_type: rc.StateType) -> list[str]:
        return [x.topic for x in rc.states if x.state_type == state_type]


def main(args=None):
    rclpy.init(args=args)
    try:
        joint_publisher = JointPublisher()
    except Exception as err:
        print("Error while starting JointPublisher node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(joint_publisher)
        joint_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
