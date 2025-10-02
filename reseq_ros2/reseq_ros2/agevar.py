import traceback
from math import cos, pi, sin

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32  # deprecated?

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

        self.n_mod = len(self.modules)
        self.yaw_angles = [0] * self.n_mod

        # subscribe to remote (parsed by teleop_twist_joy)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.remote_callback,
            10,
        )

        self.diff_controller_pub = []
        for mod in range(1, self.n_mod + 1):
            self.diff_controller_pub.append(
                self.create_publisher(TwistStamped, f'/diff_controller{mod}/cmd_vel', 10)
            )

        self.joints_sub = self.create_subscription(
            JointState, 'joint_states', self.feedback_callback, 10
        )

    def feedback_callback(self, msg: JointState):
        feedback = {}
        for i, name in enumerate(msg.name):
            feedback[name] = {
                'position': msg.position[i],
                'velocity': msg.velocity[i],
                'effort': msg.effort[i],
            }

        # update yaw angle of a joint (joint between one module and another)
        for mod in range(2, self.n_mod + 1):
            joint = feedback.get(f'joint_y_{mod}_joint')
            if joint == None:
                continue
            angle = joint.get('position')
            if angle == None:
                continue
            # store the angle in radiants
            self.yaw_angles[mod - 1] = angle

    def remote_callback(self, msg: Twist):
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
            # send to each diff_controller the TwistStamped msg with the according linear_vel
            # and angular_vel
            t = Twist()
            if sign == 0:
                t.linear.x = -linear_vel
                t.angular.z = -angular_vel
            else:
                t.linear.x = linear_vel
                t.angular.z = angular_vel
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist = t
            # publish to /diff_controller{i}/cmd_vel
            self.diff_controller_pub[mod_id].publish(msg)

            if mod_id != modules[-1]:  # for every module except the last one
                # invert yaw angle if going backwards
                yaw_angle = (1 if sign else -1) * self.yaw_angles[mod_id]

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
