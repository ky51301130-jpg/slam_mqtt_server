#!/usr/bin/env python3
"""
통합 Launch - ros2 launch slam_mqtt_server unified.launch.py
옵션: use_ai:=false, rviz:=slam|nav2|auto, foxglove:=true
"""
import os
import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals


def get_latest_map_yaml():
    """최신 nav2 맵 yaml 경로 반환"""
    map_folder = "/home/kim1/save/renewed_map"
    yamls = sorted(glob.glob(os.path.join(map_folder, "nav2_final_map_*.yaml")), reverse=True)
    if yamls:
        return yamls[0]
    return ""


def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_mqtt_server')
    latest_map = get_latest_map_yaml()
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_ai', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='auto', description='slam, nav2, auto (자동감지), 또는 빈값'),
        DeclareLaunchArgument('foxglove', default_value='true', description='Foxglove Bridge 활성화'),
        DeclareLaunchArgument('yolo_model', default_value='/home/kim1/model/best.pt'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.5'),
        DeclareLaunchArgument('map_yaml', default_value=latest_map, description='Nav2용 맵 yaml 경로'),
        
        # 0. Static TF (map -> odom -> base_link) - 로봇 없을 때 테스트용
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=LaunchConfigurationEquals('rviz', 'nav2')
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            condition=LaunchConfigurationEquals('rviz', 'nav2')
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'rplidar_link'],
            condition=LaunchConfigurationEquals('rviz', 'nav2')
        ),
        
        # 1. 통합 서버
        Node(package='slam_mqtt_server', executable='unified_server', output='screen', emulate_tty=True),
        
        # 2. MQTT 브릿지
        Node(package='slam_mqtt_server', executable='server_mqtt_bridge', output='screen', emulate_tty=True),
        
        # 3. 맵 빌더
        Node(package='slam_mqtt_server', executable='nav2_map_builder', output='screen', emulate_tty=True),
        
        # 3-1. Map Server (Nav2 모드용 - 맵 퍼블리시)
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_yaml'),
                'topic_name': 'map',
                'frame_id': 'map',
            }],
            condition=LaunchConfigurationEquals('rviz', 'nav2')
        ),
        
        # 3-2. Lifecycle Manager (map_server 활성화)
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
            }],
            condition=LaunchConfigurationEquals('rviz', 'nav2')
        ),
        
        # 4. AI 비전 (선택)
        Node(
            package='slam_mqtt_server', executable='ai_vision_analyzer', output='screen', emulate_tty=True,
            parameters=[{'yolo_model': LaunchConfiguration('yolo_model'), 'robot_ip': LaunchConfiguration('robot_ip')}],
            condition=IfCondition(LaunchConfiguration('use_ai'))
        ),
        
        # 5. Foxglove Bridge (웹 시각화)
        Node(
            package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
                'topic_whitelist': ['.*'],
            }],
            condition=IfCondition(LaunchConfiguration('foxglove'))
        ),
        
        # 6. RViz (SLAM 모드)
        Node(
            package='rviz2', executable='rviz2', name='rviz_slam',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'slam_view.rviz')],
            condition=LaunchConfigurationEquals('rviz', 'slam')
        ),
        
        # 7. RViz (Nav2 모드)
        Node(
            package='rviz2', executable='rviz2', name='rviz_nav2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'nav2_view.rviz')],
            condition=LaunchConfigurationEquals('rviz', 'nav2')
        ),
    ])
