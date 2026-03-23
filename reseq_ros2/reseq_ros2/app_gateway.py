import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool, Trigger

from reseq_interfaces.msg import Remote
from reseq_interfaces.srv import NodeStatus


class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')
        self.callback_group = ReentrantCallbackGroup()
        self.service_mapping = {'thermal': '/activate_thermal', 'velocity': '/vel'}

        self.node_states = {'thermal': False, 'lidar': False, 'velocity': False}

        # ── Arm service clients ────────────────────────────────────────────────
        self._cli_home = self.create_client(Trigger, '/cartesian_arm_controller/go_home')
        self._cli_beak = self.create_client(SetBool, '/cartesian_arm_controller/close_beak')
        self._cli_vel_mode = self.create_client(SetBool, '/cartesian_arm_controller/set_mode')
        self._cli_switch_vel = self.create_client(SetBool, '/cartesian_arm_controller/switch_vel')
        # Track toggle states
        self._beak_closed = False
        self._velocity_mode = True  # True = velocity-while-held
        self._linear_mode = True
        self._prev_bwhite = False
        self._prev_bgreen = False
        self._prev_bred = False
        self._prev_s1 = False
        self._remote_state_initialized = False

        self.sync_states_with_graph()

        self.hw_clients = {}
        for ui_name, hw_service in self.service_mapping.items():
            self.hw_clients[ui_name] = self.create_client(
                SetBool, hw_service, callback_group=self.callback_group
            )

        self.start_motor_cli = self.create_client(
            Empty, '/start_motor', callback_group=self.callback_group
        )
        self.stop_motor_cli = self.create_client(
            Empty, '/stop_motor', callback_group=self.callback_group
        )

        self.create_subscription(Remote, '/remote', self._on_remote, 10)

        for ui_name in self.node_states.keys():
            self.create_service(
                NodeStatus,
                f'/ui/{ui_name}',
                lambda req, res, n=ui_name: self.universal_callback(req, res, n),
                callback_group=self.callback_group,
            )

        self.create_service(
            NodeStatus,
            '/ui/node_status',
            self.handle_node_status,
            callback_group=self.callback_group,
        )
        self.get_logger().info('Gateway App Ready (Multi-Threaded Mode).')

    def _call_async(self, client, request):
        """Fire-and-forget async service call (no blocking)."""
        if not client.service_is_ready():
            self.get_logger().warn(f'{client.srv_name} not ready - skipping')
            return
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().debug(f'{client.srv_name} -> {f.result()}')
        )

    def _on_bwhite_pressed(self):
        self.get_logger().info('BWHITE pressed -> arm home')
        self._call_async(self._cli_home, Trigger.Request())

    def _on_s1_changed(self, closed: bool):
        self._beak_closed = closed
        req = SetBool.Request()
        req.data = closed
        self.get_logger().info(f'S1 -> beak {"CLOSE" if closed else "OPEN"}')
        self._call_async(self._cli_beak, req)

    def _on_bgreen_pressed(self):
        self._velocity_mode = not self._velocity_mode
        req = SetBool.Request()
        req.data = self._velocity_mode
        self.get_logger().info(
            f'BGREEN -> {"VELOCITY" if self._velocity_mode else "POSITION INCREMENT"} mode'
        )
        self._call_async(self._cli_vel_mode, req)

    def _on_bred_pressed(self):
        req = SetBool.Request()
        req.data = not getattr(self, '_linear_mode', True)
        self._linear_mode = req.data
        self.get_logger().info(f'BRED -> {"LINEAR" if self._linear_mode else "ANGULAR"} velocity')
        self._call_async(self._cli_switch_vel, req)

    def _on_remote(self, msg: Remote):
        buttons = list(msg.buttons)
        if len(buttons) < 10:
            self.get_logger().warn('Remote message has fewer than 10 buttons - ignoring')
            return

        if not self._remote_state_initialized:
            self._prev_s1 = buttons[0]
            self._prev_bgreen = buttons[5]
            self._prev_bred = buttons[7]
            self._prev_bwhite = buttons[8]
            self._remote_state_initialized = True
            return

        # New arm button handling
        if buttons[8] and not self._prev_bwhite:
            self._on_bwhite_pressed()
        self._prev_bwhite = buttons[8]

        if buttons[0] != self._prev_s1:
            self._on_s1_changed(bool(buttons[0]))
        self._prev_s1 = buttons[0]

        if buttons[5] and not self._prev_bgreen:
            self._on_bgreen_pressed()
        self._prev_bgreen = buttons[5]

        if buttons[7] and not self._prev_bred:
            self._on_bred_pressed()
        self._prev_bred = buttons[7]

    def universal_callback(self, request, response, module_id):
        action = request.status.lower()
        self.get_logger().info(f'UI request for module {module_id}: {action}')

        if action == 'query':
            state = 'RUNNING' if self.node_states.get(module_id) else 'STOPPED'
            response.success = True
            response.message = f'{module_id}: {state}'
            return response

        if module_id == 'lidar':
            client = self.start_motor_cli if action == 'enable' else self.stop_motor_cli
            hw_req = Empty.Request()
        else:
            client = self.hw_clients.get(module_id)
            hw_req = SetBool.Request(data=(action == 'enable'))

        if client and client.wait_for_service(timeout_sec=2.0):
            client.call_async(hw_req)
            self.node_states[module_id] = action == 'enable'
            response.success = True
            response.message = f'{module_id} {action}ed successfully.'
        else:
            response.success = False
            response.message = f'Error during the execution of service {module_id}.'

        return response

    def handle_node_status(self, request, response):
        """Scan ROS2 graph to synchronize states"""
        try:
            available_services = [s[0] for s in self.get_service_names_and_types()]

            lidar_online = any('/start_motor' in s for s in available_services)
            thermal_online = any('/activate_thermal' in s for s in available_services)
            status_msg = []
            for m, state in self.node_states.items():
                if (m == 'lidar' and not lidar_online) or (m == 'thermal' and not thermal_online):
                    status_msg.append(f'{m}: OFFLINE')
                else:
                    status_msg.append(f'{m}: {"RUNNING" if state else "STOPPED"}')
            response.message = '\n'.join(status_msg)
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def sync_states_with_graph(self):
        available_services = [s[0] for s in self.get_service_names_and_types()]
        self.node_states['lidar'] = any('/start_motor' in s for s in available_services)
        self.node_states['thermal'] = any('/activate_thermal' in s for s in available_services)
        self.get_logger().info(f'Global state found in graph: {self.node_states}')


def main(args=None):
    rclpy.init(args=args)
    node = AppGateway()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.stop_motor_cli.wait_for_service(timeout_sec=1.0):
            node.stop_motor_cli.call_async(Empty.Request())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
