#!/usr/bin/env python3
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MovementSubscriber(Node):
    def _init_(self):
        super()._init_('movement_subscriber')

        # ───────── PARAMETERS (IDENTIEK AAN velocity_subscriber) ─────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('speed', 30)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        timeout = self.get_parameter('timeout').value
        self.speed = int(self.get_parameter('speed').value)

        # ───────── SERIAL OPENEN (IDENTIEK GEDRAG) ─────────
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            time.sleep(2.0)  # OpenCR reset tijd
            self.get_logger().info(f"Connected to {port} @ {baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Failed to open serial {port}: {e}")

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

    # ───────── SERIAL WRITE (100% IDENTIEK PROTOCOL) ─────────
    def send_command(self, cmd: str):
        if self.ser is None:
            self.get_logger().error("Serial not available")
            return

        full_cmd = cmd.strip() + "\n"   # ⚠️ EXACT zoals velocity_subscriber
        try:
            self.ser.write(full_cmd.encode())
            self.get_logger().info(f"> {full_cmd.strip()} (naar OpenCR)")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ───────── CALLBACK ─────────
    def callback(self, msg: String):
        direction = msg.data.strip().lower()
        s = self.speed

        if direction == "forward":
            self.send_command(f"V {s} {s}")
        elif direction == "backward":
            self.send_command(f"V {-s} {-s}")
        elif direction == "left":
            self.send_command(f"V {s} {-s}")
        elif direction == "right":
            self.send_command(f"V {-s} {s}")
        elif direction in ("stop", "idle"):
            self.send_command("V 0 0")
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


if _name_ == "_main_":
    main()
