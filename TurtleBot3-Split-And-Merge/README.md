# TurtleBot3 — Split-and-Merge Line Detection

A ROS 2 package that detects straight-line segments from a TurtleBot3 laser scan using the **Split-and-Merge** algorithm, then visualises them in RViz via `visualization_msgs/MarkerArray`.

Built as part of the Autonomous Mobile Robots course at ETF Belgrade.

---

## Task Description

Given a laser scan from the robot's environment, use the Split-and-Merge algorithm to:

1. Extract line parameters (distance `ρ` and angle `θ` in Hesse normal form) from the raw 2-D point cloud.
2. Print each detected line's parameters to the terminal.
3. Publish the lines as `visualization_msgs/MarkerArray` markers so they can be rendered in RViz.
4. Measure and report the execution time of the algorithm per scan.

---

## Repository Structure

```
TurtleBot3-Split-And-Merge/
├── line_extraction/              # Split-and-Merge implementation (ament_python)
│   ├── line_extraction/
│   │   ├── __init__.py
│   │   └── node_line_extraction.py   # Core algorithm + ROS 2 node
│   ├── test/                         # Linting tests (flake8, pep257, copyright)
│   ├── package.xml
│   └── setup.py
├── turtlebot3/                   # Official TurtleBot3 packages (submodule)
└── turtlebot3_simulations/       # Official TurtleBot3 simulation packages (submodule)
```

---

## Algorithm

### Overview

The iterative **Split-and-Merge** algorithm segments a sequence of 2-D points (produced by converting a polar laser scan to Cartesian coordinates) into a set of straight-line segments.

### Steps

1. Start with the entire point cloud as a single candidate segment `[0, n-1]`.
2. For each segment `[i, j]`, compute the perpendicular distance from every interior point to the line defined by the endpoints `P_i` and `P_j`.
3. **Split:** If the maximum perpendicular distance exceeds the threshold (0.08 m), split the segment at that point and add the two sub-segments to the queue.
4. **Accept:** If the segment fits within the threshold **and** contains more than 5 points, store it as a detected line.
5. Repeat until the queue is empty.

> The merge step (joining collinear adjacent segments) is omitted in this implementation; the split phase alone produces clean results for structured indoor environments.

### Line Representation — Hesse Normal Form

Each accepted segment is described by two parameters:

| Parameter | Description |
|-----------|-------------|
| `ρ` (rho) | Perpendicular distance from the origin to the line (metres) |
| `θ` (theta) | Angle of the normal vector to the line (radians) |

```
rho   = nx·x1 + ny·y1
theta = atan2(ny, nx)

where (nx, ny) is the unit normal of the line through P1 and P2.
```

### Point-to-Line Distance

```
d = |((y2-y1)·x0 − (x2-x1)·y0 + x2·y1 − y2·x1)| / sqrt((y2-y1)² + (x2-x1)²)
```

---

## Package: `line_extraction`

### Node: `split_and_merge_node`

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 2-D laser scan from the robot |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lines` | `visualization_msgs/MarkerArray` | Detected line segments as red `LINE_LIST` markers |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `threshold` | `0.08` m | Maximum perpendicular distance for a point to still be considered on the line |

---

## Build

All commands are run from the workspace root (`~/ros2_ws`).

```bash
# Build only the line_extraction package
colcon build --packages-select line_extraction

# Source the install tree
source ~/ros2_ws/install/setup.bash
```

---

## Running

Four terminals are needed. Set the robot model once before launching:

```bash
export TURTLEBOT3_MODEL=burger
```

**Terminal 1 — Gazebo simulation**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
```

**Terminal 2 — Line extraction node**

```bash
ros2 run line_extraction line_extraction_node
```

**Terminal 3 — Keyboard teleoperation**

```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

**Terminal 4 — RViz visualisation**

```bash
ros2 run rviz2 rviz2
```

Inside RViz:
- Set **Fixed Frame** to `base_scan` (or `odom`).
- Add a **LaserScan** display on topic `/scan`.
- Add a **MarkerArray** display on topic `/lines`.

---

## Terminal Output

Each scan cycle prints all detected lines followed by a timing summary:

```
[INFO] Line 1: rho=-1.599, theta=3.026, 302 ----- 356
[INFO] Line 2: rho=-2.206, theta=1.815, 295 ----- 302
[INFO] Line 3: rho= 0.214, theta=1.562, 248 ----- 295
...
[INFO] 13 lines found, time: 0.970 ms
```

The index range (`302 ----- 356`) refers to the point indices in the filtered Cartesian array for that scan frame.

---

## Node Graph

```
turtlebot3_gazebo ──/scan──► split_and_merge_node ──/lines──► rviz2
teleop_keyboard   ──/cmd_vel──► turtlebot3_gazebo
```

---

## Dependencies

- ROS 2 Humble (or newer)
- `rclpy`
- `sensor_msgs`
- `visualization_msgs`
- `geometry_msgs`
- `turtlebot3_gazebo` (for simulation)
- `turtlebot3_teleop` (for manual driving)
