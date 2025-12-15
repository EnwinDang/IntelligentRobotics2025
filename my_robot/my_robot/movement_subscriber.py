#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MovementSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')

        # Subscribe to semantic commands
        self.subscription = self.create_subscription(
            String,
            'key_cmd',
            self.callback,
            10
        )

        # Publish velocity commands for OpenCR bridge
        self.vel_pub = self.create_publisher(String, 'velocity_cmd', 10)

        # Simple speed parameter (optional)
        self.declare_parameter('speed', 30)
        self.speed = int(self.get_parameter('speed').value)

        self.get_logger().info(
            f"MovementSubscriber gestart: key_cmd -> velocity_cmd (speed={self.speed})"
        )

    def send_v(self, left: int, right: int):
        msg = String()
        msg.data = f"V {left} {right}"
        self.vel_pub.publish(msg)
        self.get_logger().info(f"Sent: {msg.data}")

    def callback(self, msg: String):
        direction = msg.data.strip()
        s = self.speed

        if direction == "forward":
            self.send_v(s, s)
        elif direction == "backward":
            self.send_v(-s, -s)
        elif direction == "left":
            self.send_v(s, -s)
        elif direction == "right":
            self.send_v(-s, s)
        else:
            self.get_logger().warn(f"Onbekend commando: {direction}")


def main(args=None):
    rclpy.init(args=args)
    node = MovementSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
