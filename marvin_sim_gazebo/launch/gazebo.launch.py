import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('marvin_sim_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('marvin_description'), 'launch',
         'description.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('marvin_sim_gazebo'),
         'rviz', 'description.rviz']
    )

    gazebo_params_file = os.path.join(get_package_share_directory(
        'marvin_sim_gazebo'), 'config', 'gazebo_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        DeclareLaunchArgument(
            name='rviz_gaz',
            default_value='false',
            description='Run rviz'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={'use_sim_time': 'true',
                              }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            launch_arguments={'use_sim_time': 'true'
                              }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "marvin_description",
                       "-x", '0.0',
                       "-y", '0.0',
                       "-z", '0.08',
                       "-Y", '0.0']
        ),

        Node(
            package='marvin_sim_gazebo',
            executable='command_timeout.py',
            name='command_timeout'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz_gaz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])
