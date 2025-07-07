import traceback

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import SetBool, Trigger

"""
ROS node that handles communication between Moveit Servo and the data coming from the remote
controller through Scaler node to control mk2 arm

It receives a Twist message from Scaler which contains linear and angular velocities of the end effector
of mk2 arm. It prepares the TwistStamped message for Moveit Servo.
"""

class MoveitController(Node):
   
    def __init__(self):
        super().__init__('moveit_controller')
        # Declaring parameters and getting values
        self.arm_module_address = (
            self.declare_parameter('arm_module_address', 0).get_parameter_value().integer_value
        )

        # Service that manages linear/angular velocities 
        self.linear_vel_enabled = True
        self.create_service(SetBool, '/moveit_controller/switch_vel', self.switch_vel_type)

        self.planning_frame_id = self.declare_parameter('planning_frame_id', 'roll_6').get_parameter_value().string_value
       
        self.create_subscription(Vector3, '/mk2_arm_vel', self.handle_velocities, 10)

        self.speed_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.activate_servo()

        self.create_subscription(
            JointTrajectory,
            '/mk2_arm_controller/joint_trajectory',
            self.send_joint_trajectory,
            10,
        )
        
        self.pubs = {}
        addr = self.arm_module_address
        self.pubs['pitch_joint_1_2'] = self.create_publisher(
            Float32MultiArray,
            f'reseq/module{addr}/mk2_arm/pitch_joint_1_2/setpoint',
            10,
        )
        self.pubs['pitch_joint_3'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/pitch_joint_3/setpoint',
            10,
        )
        self.pubs['roll_joint_4'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/roll_joint_4/setpoint',
            10,
        )
        self.pubs['pitch_joint_5'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/pitch_joint_5/setpoint',
            10,
        )
        self.pubs['roll_joint_6'] = self.create_publisher(
            Float32,
            f'reseq/module{addr}/mk2_arm/roll_joint_6/setpoint',
            10,
        )
        self.get_logger().info('Node Moveit Controller started successfully')

    def handle_velocities(self, msg: Vector3):
        servo_msg = TwistStamped()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = self.planning_frame_id

        if self.linear_vel_enabled:
            servo_msg.twist.linear = msg
        else:
            servo_msg.twist.angular = msg
        
        self.speed_pub.publish(servo_msg)
    

    def switch_vel_type(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.linear_vel_enabled = request.data

        response.success = True
        response.message = 'Input velocity set to LINEAR' if self.linear_vel_enabled else 'Input velocity set to ANGULAR'
        self.get_logger().info(response.message)
        return response

    
    def activate_servo(self):
        self.activate_service = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.activate_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Switch input service not available, waiting ...')
        
        self.req = Trigger.Request()
        self.activate_service.call_async(self.req)
    
    def send_joint_trajectory(self, msg: JointTrajectory):
        update_diff = True
        names = msg.joint_names
        positions = msg.points[len(msg.points) - 1].positions

        for i in range(0, len(names)):
            if (names[i] == "pitch_joint_1" or names[i] == "roll_joint_2"):
                if update_diff:
                    i0 = self.find_index_by_name(names, "pitch_joint_1")
                    i1 = self.find_index_by_name(names, "roll_joint_2")
                    update_diff = False
                    self.pubs["pitch_joint_1_2"].publish(Float32MultiArray(data = [positions[i0], positions[i1]]))
            else:
                self.pubs[names[i]].publish(Float32(data = positions[i]))

    def find_index_by_name(self, joints, joint_name):
        for i in range(0, len(joints)):
            if joints[i] == joint_name:
                return i


def main(args=None):
    rclpy.init(args=args)
    try:
        moveit_controller = MoveitController()
        rclpy.spin(moveit_controller)
    except Exception as err:
        rclpy.logging.get_logger('moveit_controller').fatal(
            f'Error in the Moveit controller node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        moveit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
