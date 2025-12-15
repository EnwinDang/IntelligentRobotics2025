#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MovementSubscriber(Node):
    def _init_(self):
        super()._init_('movement_subscriber')

        # ───────── PARAMETERS ─────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('speed', 30)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        timeout = self.get_parameter('timeout').value
        self.speed = int(self.get_parameter('speed').value)

        # ───────── SERIAL ─────────
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            time.sleep(2.0)  # OpenCR reset delay
            self.get_logger().info(f"Connected to {port} @ {baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Serial error: {e}")

        # ───────── SUBSCRIBER ─────────
        self.subscription = self.create_subscription(
            String,
            'key_cmd',
            self.callback,
            10
        )

        self.get_logger().info(
            f"MovementSubscriber READY (speed={self.speed})"
        )

    # ───────── SERIAL SEND ─────────
    def send_v(self, left: int, right: int):
        if self.ser is None:
            self.get_logger().error("Serial not available")
            return

        cmd = f"V {left} {right}\n"
        try:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"> {cmd.strip()} (naar OpenCR)")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ───────── CALLBACK ─────────
    def callback(self, msg: String):
        direction = msg.data.strip().lower()
        s = self.speed

        if direction == "forward":
            self.send_v(s, s)
        elif direction == "backward":
            self.send_v(-s, -s)
        elif direction == "left":
            self.send_v(s, -s)
        elif direction == "right":
            self.send_v(-s, s)
        elif direction in ("stop", "idle"):
            self.send_v(0, 0)
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


if _name_ == '_main_':
    main()


if _name_ == "_main_":
    main()
