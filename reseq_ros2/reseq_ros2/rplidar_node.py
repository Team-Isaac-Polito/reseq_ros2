import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class RplidarNode(Node):
    """
    Wrapper node acting as a bridge:
    Receives a SetBool request from the UI and calls the hardware driver's Empty services.
    """
    def __init__(self):
        super().__init__('rplidar_mode')
        self.callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            SetBool,
            'rplidar_mode/toggle_scan',
            self.handle_toggle,
            callback_group=self.callback_group
        )

        self.start_motor_cli = self.create_client(
            Empty, 
            'start_motor', 
            callback_group=self.callback_group
        )
        self.stop_motor_cli = self.create_client(
            Empty, 
            'stop_motor', 
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Lidar Motor Controller initialized (Bridge Mode)')

    def handle_toggle(self, request, response):
        """Receives True/False request from UI and triggers the hardware motor service."""
        if request.data:
            self.get_logger().info("Motor activation requested...")
            success, message = self.call_motor_service(self.start_motor_cli, "START")
        else:
            self.get_logger().info("Motor stop requested...")
            success, message = self.call_motor_service(self.stop_motor_cli, "STOP")
        
        response.success = success
        response.message = message
        return response

    def call_motor_service(self, client, action_name):
        """Utility method to wait for and call hardware services."""
        if not client.wait_for_service(timeout_sec=2.0):
            err_msg = f"Hardware service {action_name} not found! Is the LiDAR driver running?"
            self.get_logger().error(err_msg)
            return False, err_msg

        client.call_async(Empty.Request())
        return True, f"{action_name} command sent to motor successfully."

def main(args=None):
    rclpy.init(args=args)
    node = RplidarNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()