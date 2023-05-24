import can
import rclpy
import reseq_ros2.constants as rc
import struct
import yaml
from can import Message
from geometry_msgs.msg import Twist
from rclpy.node import Node
from reseq_interfaces.msg import Motors
from std_msgs.msg import Float32  # deprecated?
from yaml.loader import SafeLoader


class Agevar(Node):
    def __init__(self):
        super().__init__("communication")
        # TODO: read file passed as ROS argument
        with open("src/reseq_ros2/reseq_ros2/config.yaml") as f:
            self.config = yaml.load(f, Loader=SafeLoader)
            print(self.config)

        self.n_mod = len(self.config["modules"])
        self.yaw_angles = [0] * self.n_mod

        # subscribe to remote (parsed by teleop_twist_joy)
        remote_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.remote_callback,
            10,
        )

        self.joint_subs = []
        self.motors_pubs = []
        for i in range(self.n_mod):
            info = self.config["modules"][i]

            # subscribe to feedback from joints
            if info['hasJoint']:
                s = self.create_subscription(
                    Float32,
                    f"reseq/module{info['address']}/joint/yaw/feedback",
                    lambda msg: self.robot_feedback_callback(
                        msg, info["address"]),
                    10,
                )
                self.joint_subs.append(s)

            # create publisher for the motor topics
            p = self.create_publisher(
                Motors,
                f"reseq/module{info['address']}/motor/setpoint",
                10
            )
            self.motors_pubs.append(p)

    def remote_callback(self, msg):
        self.get_logger().info(f"Twist x:{msg.linear.x}, y:{msg.linear.x}")

        # TODO: read data from Twist (parsed by teleop_twist_joy)
        linear_vel = msg.lin_vel
        angular_vel = msg.ang_vel
        sign = msg.sign

        modules = list(range(self.n_mod))
        if sign == 0:  # going backwards
            modules.reverse()

        for mod_id in modules:
            # TODO: compute and publish motor setpoints

            if mod_id != modules[-1]:  # for every module except the last one
                linear_vel, angular_vel = kinematic(
                    linear_vel, angular_vel, self.yaw_angles[mod_id])

    # set yaw angle of a joint
    def yaw_feedback_callback(self, msg, module_num):
        angle = msg.data
        if angle >= 180:
            angle -= 360
        self.yaw_angles[module_num-17] = angle

    # given data of a module, compute linear and angular velocities of the next one
    def kinematic(linear_vel, angular_vel, yaw_angle):
        delta_dot = -(angular_vel * (rc.b + rc.a * cos(yaw_angle)) +
                      linear_vel * sin(yaw_angle)) / rc.b
        yaw_angle = yaw_angle + delta_dot * rc.Ts

        angular_out = angular_vel + delta_dot

        linear_out_x = linear_vel + rc.b * sin(yaw_angle) * angular_out
        linear_out_y = -angular_vel * rc.a - \
            rc.b * cos(yaw_angle) * angular_out
        linear_out = sqrt(linear_out_x**2 + linear_out_y**2)

        return linear_out, angular_out


def main(args=None):
    rclpy.init(args=args)
    try:
        agevar = Agevar()
    except Exception as err:
        print("Error while starting Agevar node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(agevar)
        agevar.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
