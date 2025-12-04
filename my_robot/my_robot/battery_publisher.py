#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class BatteryVoltagePublisher(Node):
    def __init__(self):
        super().__init__('battery_voltage_publisher')

        self.publisher_ = self.create_publisher(Float32, '/battery_voltage', 10)

        # elke 60 seconden publishen (voor testen mag je 5.0 nemen)
        timer_period = 60.0
        # timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('BatteryVoltagePublisher gestart, publish naar /battery_voltage')

    def read_battery_voltage(self) -> float:
        # TODO: echte batterij uitlezen, nu gewoon simuleren
        simulated_voltage = 11.7
        return simulated_voltage

    def timer_callback(self):
        voltage = self.read_battery_voltage()
        msg = Float32()
        msg.data = voltage
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published battery voltage: {voltage:.2f} V')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryVoltagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
