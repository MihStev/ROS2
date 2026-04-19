from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package = 'smart_house',
             executable = 'temp_sensor',
             name = 'temp_sensor',
             output='screen',
             ),
        Node(package = 'smart_house',
        executable = 'humidity_sensor',
        name = 'humidity_sensor',
        output='screen',
        ),
        Node(package = 'smart_house',
        executable = 'light_sensor',
        name = 'light_sensor',
        output='screen',
        ),
        Node(package = 'smart_house',
        executable = 'system_visualization',
        name = 'system_visualization',
        output='screen',
        ),
        Node(package = 'smart_house_interfaces',
        executable = 'lock',
        name = 'lock',
        output='screen',
        ),
        Node(package = 'smart_house_interfaces',
        executable = 'alarm',
        name = 'alarm',
        output='screen',
        ),
        
    ])