# ROS2 Opdrachten – Intelligent Robotics

ROS2 Distro: Jazzy
Robot: Raspberry Pi 5 + OpenCR + Dynamixels

`my_robot` bevat alle drie de ROS2-opdrachten voor Intelligent Robotics:

## 🟦 Opdracht 1 — Battery Publisher & Monitor
### Raspberry Pi – Publisher
```bash
ros2 run my_robot battery_publisher
```

### Ubuntu VM – Monitor
```bash
ros2 run my_robot battery_monitor
```

Deze node luistert op /battery_voltage en geeft een waarschuwing bij een spanning lager dan 11.5V.

## 🟩 Opdracht 2 — Keyboard Teleop (Distance Mode)
### Ubuntu VM – Keyboard Publisher
```bash
ros2 run my_robot key_publisher
```

#### Toetsen:

z = vooruit  
s = achteruit  
a = links  
e = rechts  

### Raspberry Pi – Movement Subscriber
```bash
ros2 run my_robot movement_subscriber
```

Voorbeeldcommando’s die naar de robot worden gestuurd:

D 50 50 1  
D -50 -50 1  


Deze worden via /dev/ttyACM0 naar de OpenCR gestuurd.

## 🟧 Opdracht 3 — Keyboard Teleop (Velocity Mode)
### Ubuntu VM – Velocity Publisher
```bash
ros2 run my_robot velocity_publisher
```

#### Toetsen:

z = sneller (+5)  
s = trager (-5)  
a = links +5, rechts -5  
x = stop (V 0 0)  

### Raspberry Pi – Velocity Subscriber
```bash
ros2 run my_robot velocity_subscriber
```

#### Velocity commandoformaat:

V L R


### 🔧 Build Instructies
Package toevoegen aan workspace
```bash
cd ~/ros2_ws/src
cp -r <pad>/my_robot .
```

### Builden van de workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=3
```

### Netwerkvereisten

Beide systemen moeten hetzelfde ROS-domein gebruiken:

```bash
export ROS_DOMAIN_ID=3
```

En ze moeten op hetzelfde netwerk zitten (hotspot of WiFi).
