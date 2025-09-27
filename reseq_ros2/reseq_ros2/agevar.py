import traceback
from math import cos, sin

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

"""ROS node with control algorithm for snake-like movement

It receives data from the remote over a ROS topic and publishes to the ROS topics
used by the Communication node to set motors velocities.
"""


class Agevar(Node):
    def __init__(self):
        super().__init__('agevar')

        # Declaring parameters and getting values
        self.a = self.declare_parameter('a', 0.0).get_parameter_value().double_value
        self.b = self.declare_parameter('b', 0.0).get_parameter_value().double_value
        self.modules = (
            self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value
        )

        # create the enable/disable service
        self.enabled = True
        self.create_service(SetBool, '/agevar/enable', self.handle_enable)

        self.n_mod = len(self.modules)
        self.yaw_angles = [0] * self.n_mod

        # subscribe to remote (parsed by teleop_twist_joy)
        self.create_subscription(Twist, '/cmd_vel', self.remote_callback, 10)

        self.latest_feedback = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.controller_pubs = []
        for i in range(self.n_mod):
            # create publisher for the controllers
            pub = self.create_publisher(TwistStamped, f'/diff_controller{i + 1}/cmd_vel', 10)
            self.controller_pubs.append(pub)

    def handle_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.enabled = request.data

        if not self.enabled:
            for pub in self.controller_pubs:
                pub.publish(TwistStamped())  # stop all controllers

        response.success = True
        response.message = 'Agevar node enabled' if self.enabled else 'Agevar node disabled'
        self.get_logger().info(response.message)
        return response

    def remote_callback(self, msg: Twist):
        if not self.enabled:
            self.get_logger().debug('Agevar node is disabled, ignoring command')
            return

        # extract information from ROS Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        sign = 1 if linear_vel >= 0 else -1

        modules = list(range(self.n_mod))
        if sign == -1:  # going backwards
            modules.reverse()
            linear_vel = -linear_vel
            angular_vel = -angular_vel

        for mod_id in modules:
            out_msg = TwistStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.twist.linear.x = sign * linear_vel
            out_msg.twist.angular.z = sign * angular_vel

            if self.enabled:
                self.controller_pubs[mod_id].publish(out_msg)

            if mod_id != modules[-1]:  # for every module except the last one
                # invert yaw angle if going backwards
                yaw_angle = sign * self.yaw_angles[mod_id]

                # compute linear and angular velocity of the following module
                linear_vel, angular_vel = self.kinematic(linear_vel, angular_vel, yaw_angle)

                self.get_logger().debug(f'Output lin:{linear_vel}, ang:{angular_vel}, sign:{sign}')

    # given data of a module, compute linear and angular velocities of the next one
    def kinematic(self, linear_vel, angular_vel, yaw_angle):
        linear_out = linear_vel * cos(yaw_angle) + self.a * angular_vel * sin(yaw_angle)
        angular_out = (
            linear_vel * sin(yaw_angle) - self.a * angular_vel * cos(yaw_angle)
        ) / self.b

        return linear_out, angular_out

    # update the latest joint states
    def joint_state_callback(self, msg):
        self.latest_feedback = msg

    # extract yaw angles from the latest joint states
    def get_yaw_angles(self):
        if self.latest_feedback is None:
            return
        for i in range(self.n_mod):
            joint_name = f'mod{i + 1}__yaw_joint'
            if joint_name in self.latest_feedback.name:
                idx = self.latest_feedback.name.index(joint_name)
                self.yaw_angles[i] = self.latest_feedback.position[idx]


def main(args=None):
    rclpy.init(args=args)
    try:
        agevar = Agevar()
        rclpy.spin(agevar)
    except Exception as err:
        rclpy.logging.get_logger('agevar').fatal(
            f'Error in the Agevar node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        agevar.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
