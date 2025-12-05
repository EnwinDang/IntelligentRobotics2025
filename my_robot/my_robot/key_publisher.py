#!/usr/bin/env python3
import sys
import tty
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# mapping van toetsen → logical commands
KEYMAP = {
    'z': "forward",
    's': "backward",
    'a': "left",
    'e': "right",
}


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'key_cmd', 10)
        self.get_logger().info(
            "Keyboard publisher gestart. Gebruik z/s/a/e, Ctrl+C om te stoppen."
        )

    def read_key(self) -> str:
        """Lees 1 toets in raw mode (zonder Enter)."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        while rclpy.ok():
            key = self.read_key()

            if key in KEYMAP:
                msg = String()
                msg.data = KEYMAP[key]
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {msg.data}")
            elif key == '\x03':  # Ctrl+C
                break


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
