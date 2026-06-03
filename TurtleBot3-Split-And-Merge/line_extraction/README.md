# Line Extraction — ROS2 Package

ROS2 Python package that detects lines from 2D LiDAR scans using the **iterative Split-and-Merge** algorithm and visualizes them in RViz.

## Algorithm

The iterative Split-and-Merge algorithm works as follows:

1. Start with the entire point cloud as one segment
2. Check if all points fit a line within a distance threshold (0.08 m)
3. If not, split at the point with the maximum perpendicular distance
4. Repeat until all segments fit a line or are too short (< 5 points)

Each detected line is represented in **Hesse normal form**: `(ρ, θ)` where `ρ` is the distance from the origin to the line and `θ` is the angle of the normal vector.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribed |
| `/lines` | `visualization_msgs/MarkerArray` | Published |

## Running

### Prerequisites

- ROS2 Humble
- TurtleBot3 simulation packages

```bash
sudo apt install ros-humble-turtlebot3-simulations ros-humble-turtlebot3
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch

Open 4 terminals, all sourced with:
```bash
source ~/ros2_ws/install/setup.bash
```

**Terminal 1 — Gazebo simulation:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
```

**Terminal 2 — Line extraction node:**
```bash
ros2 run line_extraction line_extraction_node
```

**Terminal 3 — RViz:**
```bash
ros2 run rviz2 rviz2
```
In RViz: set Fixed Frame to `odom`, add `/scan` (LaserScan) and `/lines` (MarkerArray).

**Terminal 4 — Teleoperation:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

## Terminal Output Example

```
[INFO] [split_and_merge_node]: Line 1: rho=-1.599, theta=3.026, 302 ----- 356
[INFO] [split_and_merge_node]: Line 2: rho=-2.206, theta=1.815, 295 ----- 302
...
[INFO] [split_and_merge_node]: 13 lines found, time: 0.970 ms
```

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `threshold` | 0.08 m | Max perpendicular distance for line fit |
| Min segment | 5 points | Segments shorter than this are discarded |
