#!/usr/bin/env python3
import os
from time import sleep

import launch
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python import get_package_share_directory


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='False')
    return LaunchDescription([
        Node(
            package='slam_gmapping',
            node_namespace='transform',
            node_executable='transform',
            output='screen',
            parameters=[{'parents_frame': "odom",
                         'child_frame': "base_link"
                         }]
        ),
        TimerAction(period=1.0, actions=[
            Node(
                package='slam_gmapping',
                node_namespace='slam_gmapping',
                node_executable='slam_gmapping',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
        ])
    ])
