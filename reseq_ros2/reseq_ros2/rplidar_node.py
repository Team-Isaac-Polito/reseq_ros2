import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class RplidarNode(Node):
    def __init__(self):
        super().__init__('rplidar_mode')
        self.callback_group = ReentrantCallbackGroup()
        
        self.srv = self.create_service(
            SetBool,
            'rplidar_mode/toggle_scan',
            self.handle_toggle,
            callback_group=self.callback_group,
        )

        self.start_motor_cli = self.create_client(Empty, '/start_motor', callback_group=self.callback_group)
        self.stop_motor_cli = self.create_client(Empty, '/stop_motor', callback_group=self.callback_group)

        self.get_logger().info('Lidar Motor Controller Bridge initialized.')

    def handle_toggle(self, request, response):
        if request.data:
            self.get_logger().info('Motor activation requested...')
            success, message = self.call_motor_service(self.start_motor_cli, 'START')
        else:
            self.get_logger().info('Motor stop requested...')
            success, message = self.call_motor_service(self.stop_motor_cli, 'STOP')

        response.success = success
        response.message = message
        return response

    def call_motor_service(self, client, action_name):
        if not client.wait_for_service(timeout_sec=5.0):
            err_msg = f'Hardware service {action_name} not found!'
            self.get_logger().error(err_msg)
            return False, err_msg

        client.call_async(Empty.Request())
        return True, f'{action_name} command sent successfully.'

    def stop_motor_on_exit(self):
        """Ensures the motor stops if this bridge node is closed."""
        self.get_logger().info('Shutting down bridge: stopping motor...')
        if self.stop_motor_cli.wait_for_service(timeout_sec=1.0):
            self.stop_motor_cli.call_async(Empty.Request())
            import time
            time.sleep(0.5)

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
        node.stop_motor_on_exit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()