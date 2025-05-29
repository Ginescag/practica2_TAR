import sys
import rclpy
from rclpy.node import Node
from service_temp.srv import Temperature

class TemperatureClient(Node):
    def __init__(self):
        super().__init__('temperature_client')
        self.cli = self.create_client(Temperature, 'convert_temperature')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio...')
        self.req = Temperature.Request()

    def send_request(self, temp, conv_type):
        self.req.input_temp = float(temp)
        self.req.conversion_type = conv_type
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        self = None
        print('Uso: ros2 run service_temp temperature_client <valor> <Cel_to_Far|Far_to_Cel>')
        return

    temp, conv_type = sys.argv[1], sys.argv[2]
    client = TemperatureClient()
    client.send_request(temp, conv_type)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            resp = client.future.result()
            client.get_logger().info(f"Resultado: {resp.converted_temp}")
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
