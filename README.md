# IntelligentRobotics2025

Robotics course assignments using ROS2 Jazzy on a Raspberry Pi 5 + OpenCR board + Dynamixel motors, plus a standalone Sphero BOLT autonomous racing controller.

## Repository structure

```
IntelligentRobotics2025/
├── ros_ws/src/
│   ├── my_robot/                 # Assignments 1–3 (battery, distance, velocity teleop)
│   ├── camera_stream_publisher/  # Camera feed → ROS2 image topic
│   └── serial_teleop/            # Direct keyboard → serial bypass
├── sphero/                       # Sphero BOLT autonomous racing (Assignment 4)
├── firmware/                     # OpenCR embedded firmware (Arduino)
├── opencv/                       # Standalone computer vision demos
└── README.md
```

## Hardware

| Component | Role |
|---|---|
| Ubuntu VM | Runs keyboard publisher nodes |
| Raspberry Pi 5 | Runs subscriber nodes, serial bridge |
| OpenCR board | Motor controller (STM32 MCU) |
| Dynamixel servos | Wheel drive (wheel mode) |
| Sphero BOLT | Bluetooth ball robot (Assignment 4 only) |

**ROS Domain ID: 3** — set on both machines to isolate traffic from other ROS networks on the same LAN.

```bash
export ROS_DOMAIN_ID=3
```

---

## Assignment 1 — Battery monitoring

Simulates battery voltage on the Pi and monitors it from the VM.

| Node | Machine | Package |
|---|---|---|
| `battery_publisher` | Raspberry Pi | `my_robot` |
| `battery_monitor` | Ubuntu VM | `my_robot` |

```bash
# Pi
ros2 run my_robot battery_publisher

# VM
ros2 run my_robot battery_monitor
```

The publisher sends a simulated 11.7 V reading every 60 s on `/battery_voltage`. The monitor logs a warning if voltage drops below **11.5 V**.

---

## Assignment 2 — Distance-mode teleop

Keyboard on the VM drives the robot in discrete moves. A **0.5 s watchdog** on the Pi stops the motors if no command is received.

| Node | Machine |
|---|---|
| `key_publisher` | Ubuntu VM |
| `movement_subscriber` | Raspberry Pi |

```bash
# Pi
ros2 run my_robot movement_subscriber --ros-args -p port:=/dev/ttyACM0 -p speed:=30

# VM
ros2 run my_robot key_publisher
```

| Key | Action |
|---|---|
| `z` | Forward |
| `s` | Backward |
| `a` | Left |
| `e` | Right |
| `Ctrl+C` | Quit |

---

## Assignment 3 — Velocity-mode teleop

Incremental velocity control. Each keypress adjusts speed by ±5 units (clamped to ±100).

| Node | Machine |
|---|---|
| `velocity_publisher` | Ubuntu VM |
| `velocity_subscriber` | Raspberry Pi |

```bash
# Pi
ros2 run my_robot velocity_subscriber --ros-args -p port:=/dev/ttyACM0

# VM
ros2 run my_robot velocity_publisher
```

| Key | Action |
|---|---|
| `z` | Both motors +5 |
| `s` | Both motors −5 |
| `a` | Left +5, Right −5 (turn) |
| `x` | Full stop |
| `Ctrl+C` | Quit |

---

## Assignment 4 — Sphero BOLT autonomous racing

Standalone Python app. Connects to the Sphero BOLT over Bluetooth and follows a predefined track autonomously.

```bash
cd sphero
pip install -r requirements.txt
python task1.py
```

**Joystick controls:**

| Input | Action |
|---|---|
| SELECT | Enter calibration mode (set heading reference) |
| Left/Right stick | Adjust heading during calibration |
| Buttons 1–4 | Speed presets (50 / 70 / 100 / 200) |
| START | Begin autonomous race |

The track is a 9-segment path (~5 m total). Heading 0° = East. Each segment specifies a panel count (1 panel = 50 cm) and angle.

**Battery LED colours:**

| Voltage | LED |
|---|---|
| > 4.1 V | Green |
| 3.9–4.1 V | Yellow |
| 3.7–3.9 V | Orange |
| 3.5–3.7 V | Red |
| < 3.5 V | Auto-shutdown |

---

## Serial protocol (OpenCR)

The Pi communicates with the OpenCR board over serial at **57600 baud** (`/dev/ttyACM0`).

| Command | Format | Description |
|---|---|---|
| Velocity | `V <left> <right>` | Set continuous motor speeds |
| Distance | `D <left> <right> <duration>` | Move for a fixed duration |

Speed values range from **−100 to +100**.

---

## Building the ROS2 workspace

```bash
cd ros_ws
colcon build
source install/setup.bash
```

Dependencies: `rclpy`, `std_msgs`, `pyserial`

---

## Extra packages

### Camera stream
Streams `/dev/video0` as a ROS2 image topic at 640×480, 5 FPS.

```bash
ros2 run camera_stream_publisher cam_node
```

Topic: `/camera/image_raw`

### Serial teleop (bypass)
Direct keyboard → serial control without ROS2 middleware. Useful for hardware debugging.

```bash
ros2 run serial_teleop serial_teleop
```

### OpenCV demos
Standalone scripts in `opencv/` — color object tracking, HSV selector, click tracking, and MediaPipe hand tracking. No installation beyond OpenCV and MediaPipe required.
