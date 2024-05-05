from typing import NamedTuple
from itertools import chain
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import reseq_ros2.constants as rc


class State(NamedTuple):
    name: str
    value: float


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

        self.velocity_states = {}
        self.position_states = {}

        self.init_state(modules, joints, end_effector)

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

    def states_from_type(self, state_type: rc.StateType) -> list[str]:
        return list(
            chain.from_iterable(
                [x.states for x in rc.states if x.state_type == state_type]
            )
        )


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
