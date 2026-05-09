from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    simulator_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        tb3_gazebo_dir, 'launch', 'empty_world.launch.py'
    )),
    launch_arguments={
            'x_pose': '0.0',   # X coord
            'y_pose': '0.0',   # Y coord
            'z_pose': '0.0',   # Z coord
            'yaw': '0.0'       # Ugao rotacije (theta) oko Z ose
        }.items()
    )

    service_node = Node(
        package='turtlebot3_control',
        executable='service',
        name='robot_control',
        output='screen'
    )
    client_node = Node(
        package='turtlebot3_control',
        executable='client',
        name='mode_client',
        output='screen',
        prefix=['gnome-terminal --']  # Open in a new terminal window
    )

    return LaunchDescription([
        simulator_launch,
        service_node,
        client_node
    ])