#!/usr/bin/env python3
"""
=============================================================================
                    SLAM MQTT Server 통합 Launch
=============================================================================
사용법:
    ros2 launch slam_mqtt_server unified.launch.py
    ros2 launch slam_mqtt_server unified.launch.py ai:=false

모드 전환:
    - 로봇에서 /robot_mode 토픽으로 "SLAM" 또는 "NAV2" 발행
    - server_mqtt_bridge가 자동으로 감지하여 모드별 처리
=============================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_mqtt_server')
    
    # ─────────────────────────────────────────────────────────────
    # Launch Arguments (단순화)
    # ─────────────────────────────────────────────────────────────
    ai_arg = DeclareLaunchArgument('ai', default_value='true', description='AI 비전 활성화')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.0.5')
    model_arg = DeclareLaunchArgument('model', default_value='/home/kim1/model/best.pt')
    
    return LaunchDescription([
        # Arguments
        ai_arg,
        robot_ip_arg,
        model_arg,
        
        # ─────────────────────────────────────────────────────────
        # 시작 메시지
        # ─────────────────────────────────────────────────────────
        LogInfo(msg=''),
        LogInfo(msg='=' * 60),
        LogInfo(msg='🚀 SLAM MQTT Server 시작'),
        LogInfo(msg='=' * 60),
        LogInfo(msg=''),
        LogInfo(msg='📡 모드 자동 전환:'),
        LogInfo(msg='   로봇에서 /robot_mode 토픽으로 SLAM|NAV2|IDLE 발행'),
        LogInfo(msg='   → 서버가 자동으로 감지하여 모드별 동작 수행'),
        LogInfo(msg=''),
        LogInfo(msg='=' * 60),
        
        # ─────────────────────────────────────────────────────────
        # 핵심 노드들 (항상 실행)
        # ─────────────────────────────────────────────────────────
        
        # 1. 통합 서버 (Flask 업로드 + 네트워크 모니터)
        Node(
            package='slam_mqtt_server',
            executable='unified_server',
            name='unified_server',
            output='screen',
            emulate_tty=True,
        ),
        
        # 2. MQTT 브릿지 (모드 감지 + 자동 전환)
        Node(
            package='slam_mqtt_server',
            executable='server_mqtt_bridge',
            name='server_mqtt_bridge',
            output='screen',
            emulate_tty=True,
        ),
        
        # 3. 맵 빌더 (SLAM 사이클 맵 → Nav2 통합 맵)
        Node(
            package='slam_mqtt_server',
            executable='nav2_map_builder',
            name='nav2_map_builder',
            output='screen',
            emulate_tty=True,
        ),
        
        # ─────────────────────────────────────────────────────────
        # 선택적 노드
        # ─────────────────────────────────────────────────────────
        
        # 4. AI 비전 (ArUco + YOLO)
        Node(
            package='slam_mqtt_server',
            executable='ai_vision_analyzer',
            name='ai_vision_analyzer',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'model_path': LaunchConfiguration('model'),
            }],
            condition=IfCondition(LaunchConfiguration('ai')),
        ),
        
    ])
