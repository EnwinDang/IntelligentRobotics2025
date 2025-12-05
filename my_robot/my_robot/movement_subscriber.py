#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MovementSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')

        self.subscription = self.create_subscription(
            String,
            'key_cmd',          # zelfde topic als publisher
            self.callback,
            10
        )
        self.subscription
        self.get_logger().info(
            "Movement subscriber gestart, luistert naar 'key_cmd'."
        )

        # TODO: hier kan je later seriële verbinding openen naar OpenCR
        # bv:
        # import serial
        # self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def callback(self, msg: String):
        direction = msg.data

        # Hier zet je het om naar motorcommando's.
        # Voor nu loggen we het gewoon:
        if direction == "forward":
            self.get_logger().info("Robot: vooruit (z)")
            # bv: stuur D 50 50 1 via serial
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
