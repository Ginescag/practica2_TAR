#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from battery_act.action import Battery

class BatteryClient(Node):
    def __init__(self):
        super().__init__('battery_client')
        self._client = ActionClient(self, Battery, 'battery_charge')

    def send_goal(self, target):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server no disponible')
            return
        goal_msg = Battery.Goal()
        goal_msg.target_percentage = int(target)
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rechazado')
            return
        self.get_logger().info('Goal aceptado, esperando resultado...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.current_percentage}%")

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Resultado: {result.warning}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        print('Uso: ros2 run battery_act battery_client <porcentaje_objetivo>')
        return
    client = BatteryClient()
    client.send_goal(sys.argv[1])
    rclpy.spin(client)

if __name__ == '__main__':
    main()
