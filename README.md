# ROS2
Collection of ROS 2 projects and nodes focusing on sensor fusion, automated systems, and robotics simulation. Developed as part of my studies at the University of Belgrade (ETF)

## Projects

| Project | Description |
|---------|-------------|
| [smart_house_system](smart_house_system) | Simulated smart-home sensors (temperature, humidity, light) with services and live visualization |
| [TurtleBot3-Differential-Drive-Control](TurtleBot3-Differential-Drive-Control) | Teleoperation and polar-coordinate point-to-point control of a TurtleBot3, switched through a custom service |
| [TurtleBot3-Split-And-Merge](TurtleBot3-Split-And-Merge) | Line extraction from a 2-D laser scan with Split-and-Merge, visualized in RViz |
| [TurtleBot3-EKF-Localization](TurtleBot3-EKF-Localization) | EKF localization against a known line map, using wheel encoders and lidar line features, evaluated against Gazebo ground truth |

`turtlebot3/` and `turtlebot3_simulations/` are the official ROBOTIS packages, included so that the workspace builds as is.
