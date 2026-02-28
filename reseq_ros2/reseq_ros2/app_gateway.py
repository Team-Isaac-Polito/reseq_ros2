import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class AppGateway(Node):
    def __init__(self):
        super().__init__('app_gateway')

        self.service_mapping = {
            'thermal': 'activate_thermal',
            #'motors': 'hardware/enable_motors',
            #'lights': 'hardware/toggle_lights'
        }
        self.hw_clients = {}
        for ui_name, hw_service in self.service_mapping.items():
            ui_service_name = f'ui/{ui_name}'
            self.create_service(SetBool, ui_service_name, 
                               lambda req, res, n=ui_name: self.universal_callback(req, res, n))
            self.hw_clients[ui_name] = self.create_client(SetBool, hw_service)
            self.get_logger().info(f"Mappato: {ui_service_name} -> {hw_service}")

        self.get_logger().info("Gateway App inizializzato correttamente.")

    def universal_callback(self, request, response, module_id):
        """Callback universale che smista le chiamate verso l'hardware del team"""
        self.get_logger().info(f"Richiesta UI per modulo [{module_id}]: {request.data}")

        client = self.hw_clients.get(module_id)

        if client is None or not client.service_is_ready():
            response.success = False
            response.message = f"Errore: Il servizio hardware per {module_id} non è disponibile."
            self.get_logger().error(response.message)
            return response

        hw_request = SetBool.Request()
        hw_request.data = request.data
        
        client.call_async(hw_request)

        response.success = True
        response.message = f"Comando inoltrato a {module_id}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AppGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
