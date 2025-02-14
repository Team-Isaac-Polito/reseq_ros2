import traceback

import rclpy
from rclpy.node import Node

import reseq_ros2.constants as rc

"""
ROS node that replicates the setpoint on feedback topics

This node listens to the setpoint topics and publishes the same message on the feedback topics,
creating a closed loop. This is used in the executions without the physical robot, where the
feedback on CAN is not available.
"""


class FeedbackReplicator(Node):
    def __init__(self):
        super().__init__('feedback_replicator')
        self.modules = (
            self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value
        )
        self.joints = (
            self.declare_parameter('joints', [0]).get_parameter_value().integer_array_value
        )
        self.end_effector = (
            self.declare_parameter('end_effector', 0).get_parameter_value().integer_value
        )

        for address in self.modules:
            for topic in self.topics_from_direction(rc.Direction.OUT):
                # We are interested only in setpoint topics
                if 'setpoint' not in topic.name:
                    continue
                # Skip topics that are not related to the current module
                # (from communication node approach)
                if topic.name.split('/')[0] == 'joint' and address not in self.joints:
                    continue
                if topic.name.split('/')[0] == 'end_effector' and not address == self.end_effector:
                    continue

                base_name = f'reseq/module{address}/{topic.name.rsplit("/", 1)[0]}'
                pub = self.create_publisher(topic.data_type, f'{base_name}/feedback', 10)
                self.create_subscription(
                    topic.data_type,
                    f'{base_name}/setpoint',
                    # lambda function to pass the publisher to the callback
                    lambda msg, p=pub: self.ros_replication_callback(msg, p),
                    10,
                )
        self.get_logger().info('FeedbackReplicator node started')

    def ros_replication_callback(self, msg, publisher: rclpy.publisher.Publisher):
        """Callback that replicates the setpoint on the feedback topic"""
        self.get_logger().debug(f'Replicating setpoint: {msg}')
        publisher.publish(msg)  # replicate the setpoint on feedback

    def topics_from_direction(self, d: rc.Direction):
        return list(filter(lambda x: x.direction == d, rc.topics))


def main(args=None):
    rclpy.init(args=args)
    try:
        fr = FeedbackReplicator()
        rclpy.spin(fr)
    except Exception as err:
        rclpy.logging.get_logger('feedback_replicator').fatal(
            f'Error in the FeedbackReplicator node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        fr.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
