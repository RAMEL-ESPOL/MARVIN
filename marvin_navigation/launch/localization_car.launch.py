import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    toolbox_launch_path = os.path.join(get_package_share_directory("slam_toolbox"),
                                       'launch', 'online_async_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(toolbox_launch_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'params_file': 'src/marvin_navigation/config/localization.yaml'
                              }.items()
        )
    ])
