# TurtleBot3 Differential Drive Control (ROS 2)

This repository contains a ROS 2 implementation of kinematic control for a differential drive robot (TurtleBot3 Burger). The project demonstrates closed-loop control using polar coordinates, teleoperation, and custom ROS 2 services.

This project was developed as an assignment for the Autonomous Mobile Robots (AMR) course.

## 🌟 Features

- **Manual Mode (Teleoperation):** Direct control of the robot using keyboard inputs (W/A/S/D) to set linear and angular velocities.
- **Autonomous Mode (Closed-Loop Control):**
  - Uses a polar coordinate kinematic controller ($\rho, \alpha, \beta$) to navigate to a target $(X, Y)$ coordinate.
  - **Smart Reverse Driving:** If the target is located behind the robot ($|\alpha| > \pi/2$), the robot will intelligently drive in reverse instead of rotating fully around its axis.
  - **Velocity Scaling:** Maintains a **constant linear velocity** while perfectly preserving the mathematical trajectory by appropriately scaling the angular velocity.
- **Custom ROS 2 Services:** Clean client-server architecture separating user inputs from the control loop.

## 📂 Repository Structure

The project consists of two ROS 2 packages:
1. `turtlebot3_interfaces`: A CMake package containing custom `.srv` definitions.
2. `turtlebot3_control`: A Python package containing the core logic (client and server nodes).

```text
TurtleBot3-Differential-Drive-Control/
├── turtlebot3_control/                  # Python control package
│   ├── turtlebot3_control/
│   │   ├── __init__.py
│   │   ├── client_mode.py               # User interface (Service Client)
│   │   └── mode_service.py              # Main kinematic controller (Service Server)
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
└── turtlebot3_interfaces/               # Custom interfaces package
    ├── srv/
    │   └── SetMode.srv                  # Custom Service definition
    ├── CMakeLists.txt
    └── package.xml

⚙️ Prerequisites

  - OS: Ubuntu 20.04 / 22.04
  - ROS 2: Jazzy or newer
  - Simulation: Gazebo and TurtleBot3 ROS 2 packages (turtlebot3_gazebo)
  - Python 3

Ensure you have the TurtleBot3 simulation packages installed:

sudo apt install ros-<your_ros_distro>-turtlebot3-gazebo

🛠️ Build Instructions

1.  Clone this repository into the src folder of your ROS 2 workspace (e.g.,
    ~/ros2_ws/src/):

cd ~/ros2_ws/src
git clone <your_repository_url> TurtleBot3-Differential-Drive-Control

2.  Navigate to the root of your workspace and build the packages using colcon.
    Note: turtlebot3_interfaces is built first automatically because
    turtlebot3_control depends on it.

cd ~/ros2_ws
colcon build --packages-select turtlebot3_interfaces turtlebot3_control

3.  Source the workspace:

source install/setup.bash

🚀 How to Run

To run the full system, you will need three separate terminal windows. Don't
forget to source your workspace (source install/setup.bash) in every new
terminal!

1. Launch the Gazebo Simulation

Export the TurtleBot3 model environment variable and start the simulation:

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

(Note: Ensure Gazebo is NOT paused. Click the "Play" button in the bottom left
corner of the Gazebo GUI if the simulation is paused).

2. Start the Robot Controller (Server Node)

In a new terminal, run the main control loop. This node subscribes to /odom,
computes the kinematics, and publishes to /cmd_vel using TwistStamped messages.

source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_control service

3. Start the User Interface (Client Node)

In a third terminal, run the client node. This provides a CLI menu to choose the
mode and input target coordinates or manual commands.

source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_control client

📡 Custom Service Details

Service Name: /set_mode Service Type: turtlebot3_interfaces/srv/SetMode

The .srv file structure handles the mode switching, manual commands, and target
global coordinates:

int8 mode           # 0 = IDLE, 1 = MANUAL, 2 = AUTO
string manual_cmd   # 'w', 'a', 's', 'd'
float64 target_x
float64 target_y
---
bool success
string message

📝 License

This project is open-source and created for educational purposes.


***

**Zašto je ovaj README dobar:**
1. **Prikazuje strukturu** baš onako kako si je ti poređao (ubacio sam nazive koje imaš na slici).
2. Očigledno objašnjava **šta tvoj kod radi** (ističe kretanje unazad i skaliranje, što obično donosi dodatne bodove na ocenjivanju).
3. Ima **"How to Run"** sekciju napisanu "za idiote" korak po korak – kako se tačno pokreće okruženje, server, pa onda klijent. Ovo je najbitnije profesorima i asistentima kada pregledaju kod!
