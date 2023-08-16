from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
	Node(
		package='marvin_car_ctrl',
		executable='marvin_car_keyboard',
	),	
	])
