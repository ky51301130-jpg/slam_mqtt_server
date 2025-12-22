#!/usr/bin/env python3
"""
=============================================================================
                    웹 기반 RViz Launch 파일
=============================================================================
rosbridge_server + 웹 RViz 브릿지를 실행하여 
Foxglove Studio에서 접속할 수 있도록 합니다.

사용법:
    ros2 launch slam_mqtt_server web_rviz.launch.py

    # Nav2 모드로 실행
    ros2 launch slam_mqtt_server web_rviz.launch.py mode:=nav2
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # 모드 선택 (slam / nav2)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='RViz mode: slam or nav2'
    )
    
    # rosbridge_server (핵심!)
    # Foxglove Studio가 이 WebSocket에 연결
    rosbridge_server = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
            '--ros-args',
            '-p', 'port:=9090',
            '-p', 'address:=0.0.0.0',  # 모든 IP에서 접근 허용
        ],
        name='rosbridge_websocket',
        output='screen'
    )
    
    # 웹 RViz 브릿지 노드
    web_rviz_bridge = Node(
        package='slam_mqtt_server',
        executable='web_rviz_bridge',
        name='web_rviz_bridge',
        output='screen',
    )
    
    # tf2_web_republisher (TF를 웹에서 볼 수 있게)
    tf2_web = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_web_republisher', 'tf2_web_republisher',
        ],
        name='tf2_web_republisher',
        output='screen'
    )
    
    return LaunchDescription([
        mode_arg,
        
        LogInfo(msg='='*60),
        LogInfo(msg='🌐 Web RViz 서버 시작'),
        LogInfo(msg='='*60),
        LogInfo(msg=''),
        LogInfo(msg='📡 Foxglove Studio 접속:'),
        LogInfo(msg='   1. https://foxglove.dev/studio 접속'),
        LogInfo(msg='   2. Open connection → Rosbridge'),
        LogInfo(msg='   3. ws://192.168.0.3:9090'),
        LogInfo(msg=''),
        LogInfo(msg='='*60),
        
        rosbridge_server,
        web_rviz_bridge,
        # tf2_web,  # 필요시 주석 해제
    ])
