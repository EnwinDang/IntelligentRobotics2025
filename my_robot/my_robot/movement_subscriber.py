#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MovementSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')

        self.subscription = self.create_subscription(
            String,
            'key_cmd',        
            self.callback,
            10
        )
        self.subscription
        self.get_logger().info(
            "Movement subscriber gestart, luistert naar 'key_cmd'."
        )


    def callback(self, msg: String):
        direction = msg.data

        if direction == "forward":
            self.get_logger().info("Robot: vooruit (z)")
        elif direction == "backward":
            self.get_logger().info("Robot: achteruit (s)")
        elif direction == "left":
            self.get_logger().info("Robot: links (a)")
        elif direction == "right":
            self.get_logger().info("Robot: rechts (e)")
        else:
            self.get_logger().warn(f"Onbekend commando: {direction}")


def main(args=None):
    rclpy.init(args=args)
    node = MovementSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
