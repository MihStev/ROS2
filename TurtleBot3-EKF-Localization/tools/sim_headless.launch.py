"""TurtleBot3 maze world without the Gazebo GUI, plus a ground-truth pose bridge.

Used to produce the results in the README:
    ros2 launch tools/sim_headless.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(tb3, 'worlds', 'maze.world')

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s --headless-rendering -v2 ', world],
                              'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5'}.items()),
        # Ground truth: poses of all moving models, straight from the simulator
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/world/default/dynamic_pose/info', '/ground_truth/poses')],
        ),
    ])
