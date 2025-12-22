"""
웹 콘솔 런치 파일
브라우저에서 SLAM/Nav2 제어 가능

Usage:
    ros2 launch slam_mqtt_server web_console.launch.py
    ros2 launch slam_mqtt_server web_console.launch.py port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('ip', default_value='0.0.0.0',
                            description='Web server bind IP'),
        DeclareLaunchArgument('port', default_value='8080',
                            description='Web server port'),
        
        # Nav2 Web Server
        Node(
            package='slam_mqtt_server',
            executable='nav2_web_server',
            name='nav2_web_server',
            output='screen',
            parameters=[{
                'ip': LaunchConfiguration('ip'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
