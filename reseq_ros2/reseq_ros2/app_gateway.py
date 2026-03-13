import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty
from reseq_interfaces.srv import NodeStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')

        # Gruppo di callback rientrante per gestire richieste parallele
        self.callback_group = ReentrantCallbackGroup()

        self.service_mapping = {
            'thermal': 'activate_thermal',
            'velocity': 'vel'
        }

        self.node_states = {'thermal': False, 'lidar': False, 'velocity': False}

        # 1. Clients per Thermal e Velocity (SetBool)
        self.hw_clients = {}
        for ui_name, hw_service in self.service_mapping.items():
            self.hw_clients[ui_name] = self.create_client(
                SetBool, hw_service, callback_group=self.callback_group
            )

        # 2. Clients specifici per LiDAR motor (Empty) - Segue le tue regole
        self.start_motor_cli = self.create_client(
            Empty, '/start_motor', callback_group=self.callback_group
        )
        self.stop_motor_cli = self.create_client(
            Empty, '/stop_motor', callback_group=self.callback_group
        )

        # 3. Servizi per Flutter (uno per modulo)
        for ui_name in self.node_states.keys():
            self.create_service(
                NodeStatus,
                f'/ui/{ui_name}',
                lambda req, res, n=ui_name: self.universal_callback(req, res, n),
                callback_group=self.callback_group
            )

        # 4. Servizio Query generale
        self.create_service(
            NodeStatus, '/ui/node_status', self.handle_node_status,
            callback_group=self.callback_group
        )

        self.get_logger().info('Gateway App Ready (Multi-Threaded Mode).')

    def universal_callback(self, request, response, module_id):
        action = request.status.lower()
        self.get_logger().info(f'UI Request for {module_id}: {action}')

        if action == 'query':
            state = 'RUNNING' if self.node_states.get(module_id) else 'STOPPED'
            response.success = True
            response.message = f'{module_id}: {state}'
            return response

        # LOGICA LIDAR: Segue ros2 service call /start_motor o /stop_motor
        if module_id == 'lidar':
            client = self.start_motor_cli if action == 'enable' else self.stop_motor_cli
            # Usiamo un timeout breve per non intasare l'executor
            if client.wait_for_service(timeout_sec=0.5):
                client.call_async(Empty.Request())
                self.node_states['lidar'] = (action == 'enable')
                response.success = True
                response.message = f"LiDAR motor {action}d."
            else:
                response.success = False
                response.message = "LiDAR Hardware not responding."
            return response

        # ALTRI MODULI: Thermal e Velocity
        else:
            client = self.hw_clients.get(module_id)
            if client and client.wait_for_service(timeout_sec=0.5):
                hw_req = SetBool.Request(data=(action == 'enable'))
                client.call_async(hw_req)
                self.node_states[module_id] = hw_req.data
                response.success = True
                response.message = f"{module_id} set to {action}"
            else:
                response.success = False
                response.message = f"Hardware {module_id} not available."
            return response

    def handle_node_status(self, request, response):
        """Scansiona il grafo ROS per sincronizzare gli stati"""
        try:
            available_services = [s[0] for s in self.get_service_names_and_types()]
            self.node_states['lidar'] = any('/start_motor' in s for s in available_services)
            self.node_states['thermal'] = any('activate_thermal' in s for s in available_services)

            status_lines = [f"{m}: {'RUNNING' if s else 'STOPPED'}" for m, s in self.node_states.items()]
            response.message = '\n'.join(status_lines)
            response.success = True
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response



def main(args=None):
    rclpy.init(args=args)
    node = AppGateway()

    # IMPORTANTE: Usiamo il MultiThreadedExecutor invece di rclpy.spin(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motor LiDAR allo spegnimento del gateway
        if node.stop_motor_cli.wait_for_service(timeout_sec=1.0):
            node.stop_motor_cli.call_async(Empty.Request())
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()