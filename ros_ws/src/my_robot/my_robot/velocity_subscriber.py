#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')

        # zelfde instellingen als serial_teleop
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {port} @ {baud}: {e}")
            self.ser = None
        else:
            time.sleep(2.0)
            self.get_logger().info(f"Connected to {port} @ {baud}")

        self.subscription = self.create_subscription(
            String,
            'velocity_cmd',
            self.callback,
            10
        )
        self.subscription

        self.get_logger().info(
            "Velocity subscriber gestart, luistert naar 'velocity_cmd'."
        )

    def send_command(self, cmd: str):
        if self.ser is None:
            self.get_logger().error("Serial not available, cannot send command.")
            return

        full_cmd = cmd.strip() + "\n"
        try:
            self.ser.write(full_cmd.encode())
            self.get_logger().info(f"> {full_cmd.strip()} (naar OpenCR)")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def callback(self, msg: String):
        cmd = msg.data.strip()

        # verwacht strings zoals: "V 50 50" of "V 0 0"
        self.get_logger().info(f"Received velocity command: {cmd}")
        self.send_command(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
