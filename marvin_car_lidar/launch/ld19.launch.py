#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            # ldlidar publisher node
            package='marvin_car_lidar',
            node_executable='marvin_car_lidar_node',
            name='marvin_car_lidar_node',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': '/scan'},
                {'port_name': '/dev/rplidar'},
                {'frame_id': 'lidar'},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),
    ])
