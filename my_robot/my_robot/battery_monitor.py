#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

LOW_VOLTAGE_THRESHOLD = 11.5  # Volt


class BatteryVoltageMonitor(Node):
    def __init__(self):
        super().__init__('battery_voltage_monitor')

        self.subscription = self.create_subscription(
            Float32,
            '/battery_voltage',
            self.listener_callback,
            10
        )
        self.subscription  # om warning te vermijden

        self.get_logger().info(
            f'BatteryVoltageMonitor gestart, luistert naar /battery_voltage '
            f'(drempel = {LOW_VOLTAGE_THRESHOLD:.1f} V)'
        )

    def listener_callback(self, msg: Float32):
        voltage = msg.data
        if voltage < LOW_VOLTAGE_THRESHOLD:
            self.get_logger().warn(
                f'WAARSCHUWING: lage batterijspanning: {voltage:.2f} V'
            )
        else:
            self.get_logger().info(
                f'Batterijspanning OK: {voltage:.2f} V'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryVoltageMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
