import traceback
from math import cos, pi, sin
from time import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors


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

        self.position_orientation = {
            m: {'yaw_angle': 0.0, 'x_coord': 0.0, 'y_coord': 0.0} for m in self.modules
        }
        self.velocities = {
            m: {'angular_vel': 0.0, 'x_vel': 0.0, 'y_vel': 0.0} for m in self.modules
        }

        # PD controller parameters
        self.kp = 1.0  # Proportional gain
        self.kd = 0.03  # Derivative gain
        self.kv = 100  # Linear velocity scaling factor
        self.previous_error = {m: 0.0 for m in self.modules}
        self.distance_travelled = {m: 0.0 for m in self.modules}
        self.previous_position = {m: {'x_coord': 0.0, 'y_coord': 0.0} for m in self.modules}

        self.init_conditions()

        # time variables
        self.previous_time = time()
        self.max_dt = 0.1  # Maximum allowable time step in seconds

        # subscribe to remote (parsed by teleop_twist_joy)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.remote_callback,
            10,
        )

        self.joint_subs = {}
        self.motors_pubs = {}
        self.yaw_pubs = {}
        for address in self.modules:
            # subscribe to feedback from joints
            if address in self.joints:
                s = self.create_subscription(
                    Float32,
                    f'reseq/module{address}/joint/yaw/feedback',
                    lambda msg, x=address: self.yaw_feedback_callback(msg, x),
                    10,
                )
                self.joint_subs[address] = s

                # create publisher for the joint yaw setpoint topics
                p1 = self.create_publisher(
                    Float32, f'reseq/module{address}/joint/yaw/setpoint', 10
                )
                self.yaw_pubs[address] = p1

            # create publisher for the motor topics
            p2 = self.create_publisher(Motors, f'reseq/module{address}/motor/setpoint', 10)
            self.motors_pubs[address] = p2

    def init_conditions(self):
        for m in self.modules:
            self.position_orientation[m] = {
                'yaw_angle': 0.0,
                'x_coord': (m - 17) * (-self.a - self.b),
                'y_coord': 0.0,
            }
            self.velocities[m] = {'angular_vel': 0.0, 'x_vel': 0.0, 'y_vel': 0.0}
            self.previous_error[m] = 0.0
            self.distance_travelled[m] = 0.0
            self.previous_position[m] = {'x_coord': (m - 17) * (-self.a - self.b), 'y_coord': 0.0}

    def remote_callback(self, msg: Twist):
        # Extract information from ROS Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        sign = 1 if linear_vel > 0 else -1
        yaw_sign = -1 if angular_vel > 0 else 1

        if sign == -1:  # Going backwards
            modules = self.modules[::-1]
            linear_vel = -linear_vel
            angular_vel = -angular_vel
        else:
            modules = self.modules

        # Time step
        t = time()
        dt = t - self.previous_time
        dt = min(dt, self.max_dt)  # Limit dt to max_dt
        self.previous_time = t

        for idx, mod_id in enumerate(modules):
            # For the first module
            if idx == 0:
                # Update angular velocity (yaw rate)
                self.velocities[mod_id]['angular_vel'] = angular_vel

                kp = self.kp
                kd = self.kd
                kv = self.kv

                # Calculate the desired yaw angle directly
                desired_yaw_angle = (
                    -np.arctan2(
                        self.b * angular_vel,
                        linear_vel + (self.a + self.b) * angular_vel * np.cos(0) * yaw_sign * sign,
                    )
                    * sign
                )

                distance_ratio = min(
                    self.distance_travelled[mod_id] / (self.a + self.b),
                    1.0,
                )

                # PD controller for yaw angle adjustment
                error = desired_yaw_angle - self.position_orientation[mod_id]['yaw_angle']
                derivative = (error - self.previous_error[mod_id]) / dt
                correction = (kp * error + kd * derivative) * linear_vel * kv
                self.previous_error[mod_id] = error

                # Integrate angular velocity to get new yaw angle
                self.position_orientation[mod_id]['yaw_angle'] += correction * dt * distance_ratio

                # Calculate linear velocities in the local frame and transform to global frame
                vs = self.Rotz(self.position_orientation[mod_id]['yaw_angle']) @ [
                    linear_vel,
                    0.0,
                    0.0,
                ]
                self.velocities[mod_id]['x_vel'], self.velocities[mod_id]['y_vel'] = vs[0], vs[1]
            else:
                # Compute relative angle between the current and previous module
                prev_yaw = self.position_orientation[modules[idx - 1]]['yaw_angle']
                current_yaw = self.position_orientation[mod_id]['yaw_angle']
                th = prev_yaw - current_yaw

                # Compute output linear and angular velocities
                linear_out, angular_out = self.kinematic(linear_vel, angular_vel, th)

                # Update angular velocity (yaw rate)
                self.velocities[mod_id]['angular_vel'] = angular_out

                # Integrate the relative angle to get the new yaw angle
                self.position_orientation[mod_id]['yaw_angle'] += th

                # Calculate linear velocities in the local frame and transform to global frame
                vs = self.Rotz(self.position_orientation[mod_id]['yaw_angle']) @ [
                    linear_out,
                    0.0,
                    0.0,
                ]
                self.velocities[mod_id]['x_vel'], self.velocities[mod_id]['y_vel'] = vs[0], vs[1]

            # Update position coordinates
            self.position_orientation[mod_id]['x_coord'] += (
                self.velocities[mod_id]['x_vel'] * dt * 180 / pi
            )
            self.position_orientation[mod_id]['y_coord'] += (
                self.velocities[mod_id]['y_vel'] * dt * 180 / pi
            )

            dx = (
                self.position_orientation[mod_id]['x_coord']
                - self.previous_position[mod_id]['x_coord']
            )
            dy = (
                self.position_orientation[mod_id]['y_coord']
                - self.previous_position[mod_id]['y_coord']
            )

            # Compute distance traveled
            distance_increment = np.sqrt(dx**2 + dy**2)
            self.distance_travelled[mod_id] += distance_increment

            self.previous_position[mod_id]['x_coord'] = self.position_orientation[mod_id][
                'x_coord'
            ]
            self.previous_position[mod_id]['y_coord'] = self.position_orientation[mod_id][
                'y_coord'
            ]

            # Calculate linear velocity magnitude for vel_motors
            lin_vel = np.linalg.norm(
                [self.velocities[mod_id]['x_vel'], self.velocities[mod_id]['y_vel']]
            )

            # Use angular velocity directly for vel_motors
            ang_vel = self.velocities[mod_id]['angular_vel']

            # Compute motor velocities for each module
            w_right, w_left = self.vel_motors(lin_vel, ang_vel, sign)

            # Publish to ROS motor topics
            m = Motors()
            m.right = w_right
            m.left = w_left
            self.motors_pubs[mod_id].publish(m)

            # Publish yaw angle to joint yaw setpoint topic
            if mod_id in self.joints:
                yaw_msg = Float32()
                yaw_msg.data = self.position_orientation[mod_id]['yaw_angle'] * 180 / pi
                self.yaw_pubs[mod_id].publish(yaw_msg)

            self.get_logger().debug(f'Output lin:{linear_vel}, ang:{angular_vel}, sign:{sign}')

    # update yaw angle of a joint
    def yaw_feedback_callback(self, msg, module_num):
        angle = msg.data

        # keep the angle between -180 and +180
        if angle >= 180:
            angle -= 360
        elif angle < -180:
            angle += 360

        # store the angle in radians
        self.position_orientation[module_num]['yaw_angle'] = angle * pi / 180.0

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

        if sign == -1:  # backwards
            w_left, w_right = -w_left, -w_right

        # from radians to rpm
        w_right = w_right * rc.rads2rpm
        w_left = w_left * rc.rads2rpm

        return w_right, w_left

    # rotation matrix around z axis
    def Rotz(self, th):
        return np.array([[cos(th), -sin(th), 0], [sin(th), cos(th), 0], [0, 0, 1]])


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
