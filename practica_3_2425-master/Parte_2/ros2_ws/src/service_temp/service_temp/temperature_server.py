import rclpy
from rclpy.node import Node
from service_temp.srv import Temperature

class TemperatureService(Node):
    def __init__(self):
        super().__init__('temperature_service')
        self.srv = self.create_service(
            Temperature,
            'convert_temperature',
            self.handle_convert
        )
        self.get_logger().info('Servicio de temperatura listo.')

    def handle_convert(self, request, response):
        if request.conversion_type == 'Cel_to_Far':
            response.converted_temp = request.input_temp * 9.0 / 5.0 + 32.0
        elif request.conversion_type == 'Far_to_Cel':
            response.converted_temp = (request.input_temp - 32.0) * 5.0 / 9.0
        else:
            response.converted_temp = 0.0
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
