import traceback

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.node import Node
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


        # Service that manages linear/angular velocities 
        self.linear_vel_enabled = True
        self.create_service(SetBool, '/moveit_controller/switch_vel', self.switch_vel_type)

        self.planning_frame_id = self.declare_parameter('planning_frame_id', 'roll_6').get_parameter_value().string_value
       
        self.create_subscription(Vector3, '/mk2_arm_vel', self.handle_velocities, 10)

        self.speed_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.activate_servo()
        
        self.get_logger().info('Moveit controller node started')

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
