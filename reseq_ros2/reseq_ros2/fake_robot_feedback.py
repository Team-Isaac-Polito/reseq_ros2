import rclpy
from rclpy.node import Node
import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors
import traceback

"""
ROS node designed to test movement of the digital twin without access to the CAN bus.

It publishes to motor/feedback like the Communication node would do upon receiving
feedback from the robot via CAN bus.
"""

class FakeRobotFeedback(Node):
    def __init__(self):
        super().__init__("fake_robot_feedback")
        self.time = 0
        self.modules = self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value

        self.pubs = []
        for address in self.modules:
            self.pubs.append(self.create_publisher(
                Motors,
                f"/reseq/module{address}/motor/feedback",
                10
            ))

        self.create_timer(rc.sample_time, self.fake_callback)

    def fake_callback(self):
        self.time += 1
        self.time %= 150

        if self.time < 50:  # straight
            l, r = 0.5, 0.5
        elif self.time < 100:  # turn left
            l, r = 0.5, 0.4
        else:  # straight
            l, r = 0.5, 0.5

        msg = Motors()
        msg.left, msg.right = l, r
        for pub in self.pubs:
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        frf = FakeRobotFeedback()
    except Exception as err:
        rclpy.logging.get_logger('fake_robot_feedback').fatal(f"Error while starting FakeRobotFeedback node: {str(err)}\n{traceback.format_exc()}")
        rclpy.shutdown()
    else:
        rclpy.spin(frf)
        frf.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
