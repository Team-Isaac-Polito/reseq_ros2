import traceback
from math import cos, pi, sin

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32  # deprecated?
from std_srvs.srv import SetBool

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors

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
        self.d = self.declare_parameter('d', 0.0).get_parameter_value().double_value
        self.r_eq = self.declare_parameter('r_eq', 0.0).get_parameter_value().double_value
        self.modules = (
            self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value
        )
        self.joints = (
            self.declare_parameter('joints', [0]).get_parameter_value().integer_array_value
        )
        self.end_effector = (
            self.declare_parameter('end_effector', 0).get_parameter_value().integer_value
        )

        # create the enable/disable service
        self.enabled = True
        self.create_service(SetBool, '/agevar/enable', self.handle_enable)

        self.n_mod = len(self.modules)
        self.yaw_angles = [0] * self.n_mod

        # subscribe to remote (parsed by teleop_twist_joy)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.remote_callback,
            10,
        )

        self.joint_subs = []
        self.motors_pubs = []
        for i in range(self.n_mod):
            address = self.modules[i]

            # subscribe to feedback from joints
            if address in self.joints:
                s = self.create_subscription(
                    Float32,
                    f'reseq/module{address}/joint/yaw/feedback',
                    lambda msg, x=address: self.yaw_feedback_callback(msg, x),
                    10,
                )
                self.joint_subs.append(s)

            # create publisher for the motor topics
            p = self.create_publisher(Motors, f'reseq/module{address}/motor/setpoint', 10)
            self.motors_pubs.append(p)

    def handle_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.enabled = request.data

        if not self.enabled:
            for pub in self.motors_pubs:
                pub.publish(Motors())  # stop all motors

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
        sign = linear_vel > 0

        modules = list(range(self.n_mod))
        if sign == 0:  # going backwards
            modules.reverse()
            linear_vel = -linear_vel
            angular_vel = -angular_vel

        for mod_id in modules:
            # get velocity of left and right motor
            w_right, w_left = self.vel_motors(linear_vel, angular_vel, sign)

            # publish to ROS motor topics
            m = Motors()
            m.right = w_right
            m.left = w_left

            if self.enabled:
                self.motors_pubs[mod_id].publish(m)

            if mod_id != modules[-1]:  # for every module except the last one
                # invert yaw angle if going backwards
                yaw_angle = (1 if sign else -1) * self.yaw_angles[mod_id]

                # compute linear and angular velocity of the following module
                linear_vel, angular_vel = self.kinematic(linear_vel, angular_vel, yaw_angle)

                self.get_logger().debug(f'Output lin:{linear_vel}, ang:{angular_vel}, sign:{sign}')

    # update yaw angle of a joint
    def yaw_feedback_callback(self, msg, module_num):
        angle = msg.data

        # keep the angle between -180 and +180
        if angle >= 180:
            angle -= 360

        # store the angle in radiants
        self.yaw_angles[module_num % 16 - 1] = angle * pi / 180.0

    # given data of a module, compute linear and angular velocities of the next one
    def kinematic(self, linear_vel, angular_vel, yaw_angle):
        linear_out = linear_vel * cos(yaw_angle) + self.a * angular_vel * sin(yaw_angle)
        angular_out = (
            linear_vel * sin(yaw_angle) - self.a * angular_vel * cos(yaw_angle)
        ) / self.b

        return linear_out, angular_out

    # compute motors velocity for each module
    def vel_motors(self, lin_vel, ang_vel, sign):
        w_right = (lin_vel + ang_vel * self.d / 2) / self.r_eq
        w_left = (lin_vel - ang_vel * self.d / 2) / self.r_eq

        if sign == 0:  # backwards
            w_left, w_right = -w_left, -w_right

        # from radiants to rpm
        w_right = w_right * rc.rads2rpm
        w_left = w_left * rc.rads2rpm

        return w_right, w_left


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
