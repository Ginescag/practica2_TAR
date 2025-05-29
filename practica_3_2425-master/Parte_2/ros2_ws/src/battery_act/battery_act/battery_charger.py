#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from battery_act.action import Battery

class BatteryCharger(Node):
    def __init__(self):
        super().__init__('battery_charger')
        self._action_server = ActionServer(
            self,
            Battery,
            'battery_charge',
            self.execute_callback
        )
        self.get_logger().info('Action server "battery_charge" listo')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Goal recibido: aviso al {goal_handle.request.target_percentage}%")
        current = 100
        feedback = Battery.Feedback()
        rate = self.create_rate(1)  # 1 Hz
        while current > goal_handle.request.target_percentage:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal cancelado')
                return Battery.Result()
            current -= 5
            feedback.current_percentage = current
            goal_handle.publish_feedback(feedback)
            rate.sleep()
        goal_handle.succeed()
        result = Battery.Result()
        result.warning = 'Batería baja, por favor cargue el robot!'
        return result

def main(args=None):
    rclpy.init(args=args)
    node = BatteryCharger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
