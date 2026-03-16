#!/usr/bin/env python3
import sys
import tty
import termios

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VelocityKeyboardPublisher(Node):
    def __init__(self):
        super().__init__('velocity_keyboard_publisher')

        self.publisher_ = self.create_publisher(String, 'velocity_cmd', 10)

        # startsnelheden
        self.left_speed = 0
        self.right_speed = 0

        self.step = 5          # increment
        self.min_speed = -100 
        self.max_speed = 100

        self.get_logger().info(
            "Velocity keyboard publisher gestart.\n"
            "z: beide sneller (+5)\n"
            "s: beide trager (-5)\n"
            "a: links +5, rechts -5\n"
            "x: STOP (V 0 0)\n"
        )

    def _read_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def clamp(self, value: int) -> int:
        return max(self.min_speed, min(self.max_speed, value))

    def update_speeds(self, key: str) -> bool:
        if key == 'z':
            # sneller
            self.left_speed = self.clamp(self.left_speed + self.step)
            self.right_speed = self.clamp(self.right_speed + self.step)
        elif key == 's':
            # trager
            self.left_speed = self.clamp(self.left_speed - self.step)
            self.right_speed = self.clamp(self.right_speed - self.step)
        elif key == 'a':
            # links sneller, rechts trager
            self.left_speed = self.clamp(self.left_speed + self.step)
            self.right_speed = self.clamp(self.right_speed - self.step)
        elif key == 'x':
            # stop
            self.left_speed = 0
            self.right_speed = 0
        else:
            return False
        return True

    def loop(self):
        while rclpy.ok():
            key = self._read_key()
            if key == '\x03':  # Ctrl+C om te stoppen
                break

            changed = self.update_speeds(key)
            if not changed:
                continue

            cmd = f"V {self.left_speed} {self.right_speed}"
            msg = String()
            msg.data = cmd
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = VelocityKeyboardPublisher()
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
