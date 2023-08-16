from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
        package='marvin_car_ctrl',
        executable='marvin_car_joy',
    )
    launch_description = LaunchDescription([node1])
    return launch_description
