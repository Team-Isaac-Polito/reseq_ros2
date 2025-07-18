import traceback

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool

import reseq_ros2.constants as rc
from reseq_interfaces.msg import Motors


class PivotController(Node):
    def __init__(self):
        super().__init__('pivot_controller')

        addresses = (
            self.declare_parameter('modules', [0]).get_parameter_value().integer_array_value
        )
        self.d = self.declare_parameter('d', 0.0).get_parameter_value().double_value
        self.r_eq = self.declare_parameter('r_eq', 0.0).get_parameter_value().double_value

        # Publishers for sending motor commands to individual modules
        self.head_publisher = self.create_publisher(
            Motors, f'/reseq/module{min(addresses)}/motor/setpoint', 10
        )
        self.tail_publisher = self.create_publisher(
            Motors, f'/reseq/module{max(addresses)}/motor/setpoint', 10
        )

        self.create_subscription(Twist, '/cmd_vel', self.remote_callback, 10)

        self.enabled = False
        self.pivot_on_head = True
        self.create_service(SetBool, '/pivot_controller/enable', self.handle_enable)
        self.create_service(SetBool, '/pivot_controller/pivot_on_head', self.handle_pivot_on_head)

    def handle_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.enabled = request.data
        response.success = True
        response.message = (
            'Pivot controller enabled' if self.enabled else 'Pivot controller disabled'
        )
        self.get_logger().info(response.message)

        self.head_publisher.publish(Motors())
        self.tail_publisher.publish(Motors())

        return response

    def handle_pivot_on_head(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.pivot_on_head = request.data
        response.success = True
        response.message = 'Pivoting on head' if self.pivot_on_head else 'Pivoting on tail'
        self.get_logger().info(response.message)

        self.head_publisher.publish(Motors())
        self.tail_publisher.publish(Motors())

        return response

    def remote_callback(self, msg: Twist):
        if not self.enabled:
            self.get_logger().debug('Pivot controller is disabled. Skipping command.')
            return

        ang_speed = msg.angular.z

        # differential drive with V=0
        m = Motors()
        m.right = ang_speed * self.d / (2 * self.r_eq) * rc.rads2rpm
        m.left = -m.right

        self.get_logger().debug(
            f"Pivoting {'on head' if self.pivot_on_head else 'on tail'} with values: R={m.right}, L={m.left}"  # noqa
        )

        if self.pivot_on_head:
            self.head_publisher.publish(m)
        else:
            self.tail_publisher.publish(m)


def main(args=None):
    rclpy.init(args=args)
    try:
        pivot_controller = PivotController()
        rclpy.spin(pivot_controller)
    except Exception as err:
        rclpy.logging.get_logger('pivot_controller').fatal(
            f'Error in the Pivot Controller node: {str(err)}\n{traceback.format_exc()}'
        )
        raise err
    else:
        pivot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
