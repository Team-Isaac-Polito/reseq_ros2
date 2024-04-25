import rclpy
from rclpy.node import Node
import yaml
from yaml.loader import SafeLoader
from geometry_msgs.msg import Twist
from reseq_interfaces.msg import Remote, EndEffector

"""
ROS node that handles scaling of the remote controller data into physical variables used
by the motors

It receives a packet from the remote controller and rescales end_effector data 
(pitch, head_pitch, head_yaw) and the Twist data used by Agevar 
(linear velocity, angular velocity)
"""

class Scaler(Node):
    def __init__(self):
        super().__init__("scaler")
        #Declaring parameters and getting values
        self.r_linear_vel = self.declare_parameter('r_linear_vel', [0.0]).get_parameter_value().double_array_value
        self.r_inverse_radius = self.declare_parameter('r_inverse_radius', [0.0]).get_parameter_value().double_array_value
        self.r_pitch_vel = self.declare_parameter('r_pitch_vel', [0.0]).get_parameter_value().integer_array_value
        self.r_head_pitch_vel = self.declare_parameter('r_head_pitch_vel', [0.0]).get_parameter_value().integer_array_value
        self.r_head_yaw_vel = self.declare_parameter('r_head_yaw_vel', [0.0]).get_parameter_value().integer_array_value
        
        self.create_subscription(
            Remote,
            "/remote",
            self.remote_callback,
            10
        )

        self.enea_pub = self.create_publisher(
            EndEffector,
            "/end_effector",
            10
        )

        self.agevar_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.get_logger().info("Scaler node started")

    def remote_callback(self, data: Remote): 
        #TODO: buttons, switches
        
        cmd_vel = Twist()
        cmd_vel.linear.x = data.right.y # Linear velocity (-1:1)
        cmd_vel.angular.z = - data.right.x # inverse of Radius of curvature (-1:1)

        cmd_vel = self.agevarScaler(cmd_vel)
        self.agevar_pub.publish(cmd_vel)

        end_e = EndEffector()
        end_e.pitch_vel = data.left.y
        end_e.head_pitch_vel = data.left.z
        end_e.head_yaw_vel = data.left.x

        end_e = self.endEffectorScaler(end_e)
        self.enea_pub.publish(end_e)


    def agevarScaler(self, data: Twist): 
        data.linear.x = self.scale(data.linear.x, self.r_linear_vel)
        data.angular.z = self.scale(data.angular.z, self.r_inverse_radius)
        data.angular.z *= data.linear.x # Angular vel
        return data

    def endEffectorScaler(self, data: EndEffector): 
        data.pitch_vel = self.scale(data.pitch_vel, self.r_pitch_vel)
        data.head_pitch_vel = self.scale(data.head_pitch_vel, self.r_head_pitch_vel)
        data.head_yaw_vel = self.scale(data.head_yaw_vel, self.r_head_yaw_vel)
        return data

    def scale(self, val, range):
        return (val+1)/2*(range[1]-range[0]) + range[0]

def main(args=None):
    rclpy.init(args=args)
    try:
        scaler = Scaler()
    except Exception as err:
        print("Error while starting Scaler node: " + str(err))
        rclpy.shutdown()
    else:
        rclpy.spin(scaler)
        scaler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
