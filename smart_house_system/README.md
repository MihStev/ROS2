
***

# 🏠 Smart House System (ROS 2)

This repository contains a ROS 2 package that simulates a smart home environment. The project was developed to demonstrate practical applications of core ROS 2 concepts, including **Topics (Publisher/Subscriber)**, **Services (Client/Server)**, and **Launch files**.

It is built as part of the "Autonomous Mobile Robots" course at the University of Belgrade (School of Electrical Engineering - ETF).

## 🛠️ Environment & Technologies
* **Framework:** ROS 2 (Humble / Iron / Jazzy)
* **Language:** Python 3 (`rclpy`)
* **Standard Interfaces:** `std_msgs`, `std_srvs`

---

## 🏗️ System Architecture

The smart home system consists of multiple independent ROS 2 nodes communicating with each other. The project is divided into three main phases:

### 1. Sensors & Topics (Continuous Data Stream)
These nodes use the Publisher/Subscriber pattern to simulate environmental sensors.
* 🌡️ **Temperature Sensor (`temp_sensor`)**
  * Publishes to: `/home/temperature`
  * Message Type: `std_msgs/msg/Float64`
  * Behavior: Publishes random values between 18.0°C and 30.0°C at 1 Hz.
* 💧 **Humidity Sensor (`humidity_sensor`)**
  * Publishes to: `/home/humidity`
  * Message Type: `std_msgs/msg/Float64`
  * Behavior: Publishes random values between 30.0% and 70.0% at 1 Hz.
* 💡 **Light Sensor (`light_sensor`)**
  * Publishes to: `/home/lighting`
  * Message Type: `std_msgs/msg/Int64`
  * Behavior: Publishes random values between 0 and 1000 lux at 1 Hz.
* 🖥️ **System Dashboard (`system_visualization`)**
  * Subscribes to all three sensor topics and prints the current state of the house in a human-readable format.

### 2. Services (On-Demand Actions)
These nodes use the Request/Response pattern for immediate actions.
* 🚪 **Smart Door Lock (`lock`)**
  * Service: `/home/door/lock`
  * Service Type: `std_srvs/srv/SetBool`
  * Behavior: Locks or unlocks the door based on the request (`True`/`False`) and returns the current state with a message.
* 🚨 **Alarm System (`alarm`)**
  * Service: `/home/alarm/activate` (Type: `std_srvs/srv/SetBool`)
  * Subscribes to: `/home/temperature`
  * Behavior: Can be activated/deactivated via the service. When active, it monitors the temperature. If the temperature goes out of the safe range (15°C - 35°C), it triggers a warning log.

---

## 🚀 How to Build and Run

### 1. Build the Workspace
Navigate to the root of your ROS 2 workspace and build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select smart_house smart_house_interfaces
```

### 2. Source the Environment
Before running any nodes, make sure to source the setup file:
```bash
source ~/ros2_ws/install/setup.bash
```

### 3. Run the Entire System (Launch File)
Instead of running each node in a separate terminal, you can start the entire smart house simulation using the provided launch file:
```bash
ros2 launch smart_house_system smart_house.launch.py
```

---

## 🎮 Interacting with the System via CLI

Once the system is running, you can interact with the services using the ROS 2 Command Line Interface (CLI) from a new terminal (don't forget to source it first!).

**Lock the Door:**
```bash
ros2 service call /home/door/lock std_srvs/srv/SetBool "{data: true}"
```

**Unlock the Door:**
```bash
ros2 service call /home/door/lock std_srvs/srv/SetBool "{data: false}"
```

**Activate the Alarm:**
```bash
ros2 service call /home/alarm/activate std_srvs/srv/SetBool "{data: true}"
```

*Alternatively, you can use the `rqt` GUI tool to call these services visually.*

---
**Author:** Mihajlo Stevanovic  
**License:** Apache License 2.0