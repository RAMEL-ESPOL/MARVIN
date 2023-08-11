
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    package_name = 'marvin_description'
    urdf_path = os.path.join(FindPackageShare(package=package_name).find(
        package_name), 'urdf/marvin.urdf.xacro')

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('marvin_description'),
         'rviz', 'description.rviz']
    )

    robot_description_config = xacro.process_file(urdf_path)
    params = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument(
            name='publish_joints',
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints"))
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz"))
        ),
    ])

# sources:
# https://navigation.ros.org/setup_guides/index.html#
# https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
# https://github.com/ros2/rclcpp/issues/940
