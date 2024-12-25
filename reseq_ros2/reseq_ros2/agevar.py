import traceback
from math import cos, pi, sin
from time import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32  # deprecated?

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors


def Rotz(th):
    return np.array([[cos(th), -sin(th), 0], [sin(th), cos(th), 0], [0, 0, 1]])


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
        self.eta = [[0.0, 0.0, 0.0] for _ in range(self.n_mod)]
        self.etad = [[0.0, 0.0, 0.0] for _ in range(self.n_mod)]

        self.init_conditions()
        self.previous_time = 0

        # subscribe to remote (parsed by teleop_twist_joy)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.remote_callback,
            10,
        )

        self.joint_subs = []
        self.motors_pubs = []
        self.yaw_pubs = []
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

                # create publisher for the joint yaw setpoint topics
                p1 = self.create_publisher(
                    Float32, f'reseq/module{address}/joint/yaw/setpoint', 10
                )
                self.yaw_pubs.append(p1)

            # create publisher for the motor topics
            p2 = self.create_publisher(Motors, f'reseq/module{address}/motor/setpoint', 10)
            self.motors_pubs.append(p2)

    def init_conditions(self):
        for m in range(self.n_mod):
            self.eta[m] = [0.0, (m - 1) * (-self.a - self.b), 0.0]
            self.etad[m] = [0.0, 0.0, 0.0]

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

        # time step
        t = time()
        dt = t - self.previous_time
        self.previous_time = t

        for mod_id in modules:
            # For the first module
            if mod_id == modules[0]:
                # Update angular velocity (yaw rate)
                self.etad[mod_id][0] = angular_vel

                # Integrate angular velocity to get new yaw angle
                self.eta[mod_id][0] += dt * self.etad[mod_id][0]

                # Calculate linear velocities in the local frame and transform to global frame
                vs = Rotz(self.eta[mod_id][0]) @ [linear_vel, 0.0, 0.0]
                self.etad[mod_id][1:3] = vs[0:2]
            else:
                # Compute relative angle between the current and previous module
                th = self.eta[mod_id - 1][0] - self.eta[mod_id][0]

                # Compute output linear and angular velocities
                linear_out, angular_out = self.kinematic(linear_vel, angular_vel, th)

                # Update angular velocity (yaw rate)
                self.etad[mod_id][0] = angular_out

                # Integrate angular velocity to get new yaw angle
                self.eta[mod_id][0] += dt * self.etad[mod_id][0]

                # Calculate linear velocities in the local frame and transform to global frame
                vs = Rotz(self.eta[mod_id][0]) @ [linear_out, 0.0, 0.0]
                self.etad[mod_id][1:3] = vs[0:2]

            # Integrate linear velocities to get new position
            self.eta[mod_id][1] += dt * self.etad[mod_id][1]
            self.eta[mod_id][2] += dt * self.etad[mod_id][2]

            # Calculate linear velocity magnitude for vel_motors
            lin_vel = np.linalg.norm(self.etad[mod_id][1:3])

            # Use angular velocity directly for vel_motors
            ang_vel = self.etad[mod_id][0]

            # Compute motor velocities for each module
            w_right, w_left = self.vel_motors(lin_vel, ang_vel, sign)

            # publish to ROS motor topics
            m = Motors()
            m.right = w_right
            m.left = w_left
            self.motors_pubs[mod_id].publish(m)

            # Publish yaw angle to joint yaw setpoint topic
            if mod_id in self.joints:
                yaw_msg = Float32()
                yaw_msg.data = self.eta[mod_id][0]
                self.yaw_pubs[mod_id].publish(yaw_msg)

            self.get_logger().debug(f'Output lin:{linear_vel}, ang:{angular_vel}, sign:{sign}')

    # update yaw angle of a joint
    def yaw_feedback_callback(self, msg, module_num):
        angle = msg.data

        # keep the angle between -180 and +180
        if angle >= 180:
            angle -= 360

        # store the angle in radians
        self.eta[module_num - 17][0] = angle * pi / 180.0

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

        # from radians to rpm
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
