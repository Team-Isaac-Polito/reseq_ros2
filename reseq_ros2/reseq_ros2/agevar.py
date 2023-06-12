import rclpy
import reseq_ros2.constants as rc
from math import cos, sin, pi
from geometry_msgs.msg import Twist
from rclpy.node import Node
from reseq_interfaces.msg import Motors
from std_msgs.msg import Float32  # deprecated?


class Agevar(Node):
    def __init__(self):
        super().__init__("agevar")

        self.modules = self.declare_parameter("modules", [0]).get_parameter_value().integer_array_value
        self.joints = self.declare_parameter("joints", [0]).get_parameter_value().integer_array_value
        self.a = self.declare_parameter("a", 0.0).get_parameter_value().double_value
        self.b = self.declare_parameter("b", 0.0).get_parameter_value().double_value
        self.d = self.declare_parameter("d", 0.0).get_parameter_value().double_value
        self.r_eq = self.declare_parameter("r_eq", 0.0).get_parameter_value().double_value

        self.get_logger().info(f"Starting with parameters a: {self.a}, b: {self.b}, d: {self.d}, r_eq: {self.r_eq}, modules: {self.modules.tolist()}, joints: {self.joints.tolist()}")

        self.n_mod = len(self.modules)
        self.yaw_angles = [0] * self.n_mod

        # subscribe to remote (parsed by teleop_twist_joy)
        self.remote_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.remote_callback,
            10,
        )

        self.joint_subs = []
        self.motors_pubs = []
        self.diff_pubs = []
        for i in range(self.n_mod):
            address = self.modules[i]

            # subscribe to feedback from joints
            if address in self.joints:
                s = self.create_subscription(
                    Float32,
                    f"reseq/module{address}/joint/yaw/feedback",
                    lambda msg, x=address: self.yaw_feedback_callback(msg, x),
                    10,
                )
                self.joint_subs.append(s)

            # create publisher for the motor topics
            p = self.create_publisher(
                Motors,
                f"reseq/module{address}/motor/setpoint",
                10
            )
            self.motors_pubs.append(p)

            # create publisher for diff_drive_controller
            p = self.create_publisher(
                Twist,
                f"diff_cont_{address}/cmd_vel_unstamped",
                10
            )
            self.diff_pubs.append(p)
            
        self.get_logger().info("Agevar node started!")

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
            wdx, wsw = self.vel_motors(linear_vel, angular_vel, sign)

            # publish to ROS motor topics
            m = Motors()
            m.right = wdx
            m.left = wsw
            self.motors_pubs[mod_id].publish(m)

            # publish to diff_drive_controller
            m = Twist()
            m.linear.x = linear_vel
            m.angular.z = angular_vel
            self.diff_pubs[mod_id].publish(m)

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
        self.yaw_angles[module_num-17] = angle*pi/180.0

    # given data of a module, compute linear and angular velocities of the next one
    def kinematic(self, linear_vel, angular_vel, yaw_angle):
        linear_out = linear_vel * cos(yaw_angle) + self.a * angular_vel * sin(yaw_angle)
        angular_out = (linear_vel * sin(yaw_angle) - self.a * angular_vel * cos(yaw_angle)) / self.b
        
        return linear_out, angular_out

    # compute motors velocity for each module
    def vel_motors(self, lin_vel, ang_vel, sign):
        wdx = (lin_vel + ang_vel * self.d / 2) / self.r_eq
        wsx = (lin_vel - ang_vel * self.d / 2) / self.r_eq

        if sign == 0:  # backwards
            wsx, wdx = -wsx, -wdx

        # from radiants to rmp
        wdx = wdx * rc.rads2rpm
        wsx = wsx * rc.rads2rpm

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
