import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    localization_launch_path = PathJoinSubstitution(
        [FindPackageShare('marvin_navigation'), 'launch',
         'localization.launch.py']
    )

    nav2_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    return LaunchDescription([
       # IncludeLaunchDescription(
       #     PythonLaunchDescriptionSource(localization_launch_path),
       # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_path),
            launch_arguments={'use_sim_time': 'true',
                              }.items()
        ),
    ])
