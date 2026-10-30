import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ekf_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mission = LaunchConfiguration('mission')
    use_ekf = LaunchConfiguration('use_ekf')
    rviz = LaunchConfiguration('rviz')
    map_path = LaunchConfiguration('map_path')
    use_scan = LaunchConfiguration('use_scan')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'mission', default_value='false',
            description='Start the waypoint mission (square 1 m x 1 m)'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='Mission feedback: /ekf_pose (true) or /odom (false)'),
        DeclareLaunchArgument(
            'use_scan', default_value='true',
            description='EKF correction step on (true) or prediction only (false)'),
        # The line map is in the Gazebo world frame, so the filter has to start
        # from the spawn pose (turtlebot3_maze.launch.py default: -2.0, -0.5).
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_path',
            default_value=os.path.join(pkg_share, 'maps', 'maze_lines.yaml')),

        Node(
            package='ekf_localization',
            executable='ekf_line_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_path': map_path,
                'use_scan': use_scan,
                'initial_x': x_pose,
                'initial_y': y_pose,
                'initial_theta': yaw,
            }],
        ),
        Node(
            package='ekf_localization',
            executable='split_and_merge_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='ekf_localization',
            executable='waypoint_mission',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_ekf': use_ekf,
                'odom_x': x_pose,
                'odom_y': y_pose,
                'odom_yaw': yaw,
            }],
            condition=IfCondition(mission),
        ),
        # /odom starts at the spawn pose, so map -> odom is the spawn pose.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', x_pose, '--y', y_pose, '--yaw', yaw,
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'ekf_localization.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
