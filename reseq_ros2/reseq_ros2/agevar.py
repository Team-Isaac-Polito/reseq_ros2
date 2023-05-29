import rclpy
import reseq_ros2.constants as rc
import struct
import yaml
import rospy
from math import cos, sin, sqrt
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
                    lambda msg, x=info["address"]: self.yaw_feedback_callback(
                        msg, x),
                    10,
                )
                self.joint_subs.append(s)

            # create publisher for the motor topics
            p = self.create_publisher(
                Twist,
                f"diff_cont_{info['address']}/cmd_vel_unstamped",
                10
            )
            self.motors_pubs.append(p)

    def remote_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = linear_vel * msg.angular.z
        sign = (linear_vel > 0)

        modules = list(range(self.n_mod))
        if sign == 0:  # going backwards
            modules.reverse()
            linear_vel = -linear_vel
            angular_vel = -angular_vel

        for mod_id in modules:
            # TODO: compute and publish motor setpoints
            wdx, wsw = self.vel_motors(linear_vel, angular_vel, sign)

            # publish to ROS diff_drive_controller topics
            m = Twist()
            msg.linear.x = linear_vel
            msg.angular.z = angular_vel
            self.motors_pubs[mod_id].publish(m)
        
            if mod_id != modules[-1]:  # for every module except the last one
                linear_vel, angular_vel = self.kinematic(
                    linear_vel, angular_vel, self.yaw_angles[mod_id])

                self.get_logger().info(
                    f"Output lin:{linear_vel}, ang:{angular_vel}, sign:{sign}")

    # set yaw angle of a joint
    def yaw_feedback_callback(self, msg, module_num):
        angle = msg.data

        if angle >= 180:
            angle -= 360
        self.yaw_angles[module_num-17] = angle*math.pi/180.0

    # given data of a module, compute linear and angular velocities of the next one
    def kinematic(self, linear_vel, angular_vel, yaw_angle):
        linear_out = linear_vel * cos(yaw_angle) + rc.a * angular_vel * sin(yaw_angle)
        angular_out = (linear_vel * sin(yaw_angle) - rc.a * angular_vel * cos(yaw_angle)) / rc.b
        
        return linear_out, angular_out

    # compute motors velocity for each module
    def vel_motors(self, lin_vel, ang_vel, sign):
        wdx = (lin_vel+ang_vel*rc.d/2) / rc.r_eq
        wsx = (lin_vel-ang_vel*rc.d/2) / rc.r_eq

        if sign == 0:  # backwards
            wsx, wdx = -wsx, -wdx

        # from radiants to rmp
        wdx = wdx*rc.rads2rpm
        wsx = wsx*rc.rads2rpm

        return wdx, wsx


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
