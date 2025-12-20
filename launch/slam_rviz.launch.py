#!/usr/bin/env python3
"""SLAM RViz Launch"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_mqtt_server')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam_view.rviz')
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_slam',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
