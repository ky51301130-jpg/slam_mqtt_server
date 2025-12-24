#!/usr/bin/env python3
"""
=============================================================================
                    PINKY 서버 통합 Launch (ALL-IN-ONE)
=============================================================================
모든 서버 서비스를 한번에 실행합니다:
  - MQTT Bridge (로봇 통신)
  - Map Builder (맵 통합)
  - AI Vision (ArUco + YOLO)
  - Web Server (웹 UI - pinky_navigation)
  - Upload Server (Flask)

사용법:
    ros2 launch slam_mqtt_server pinky_server.launch.py
    ros2 launch slam_mqtt_server pinky_server.launch.py ai:=false
    ros2 launch slam_mqtt_server pinky_server.launch.py web_port:=8080

접속:
    웹 UI: http://192.168.0.3:8080
    Grafana: http://192.168.0.3:3000
=============================================================================
"""
import os
import socket
import shutil
import glob
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, 
    ExecuteProcess, RegisterEventHandler, OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.event_handlers import OnShutdown, OnProcessStart


# ─────────────────────────────────────────────────────────────
# 초기화 함수들
# ─────────────────────────────────────────────────────────────
SAVE_MAP_PATH = '/home/kim1/save/map'
SAVE_COLLISION_PATH = '/home/kim1/save/collision'
SAVE_AI_PATH = '/home/kim1/save/ai_detections'
CYCLE_STATE_FILE = '/tmp/slam_cycle_state.json'


def cleanup_on_shutdown(context, *args, **kwargs):
    """종료 시 임시 상태 파일 정리"""
    print("\n" + "=" * 70)
    print("🛑 PINKY 서버 종료 - 상태 초기화 중...")
    print("=" * 70)
    
    # 사이클 상태 파일 삭제 (다음 시작 시 새로 시작)
    if os.path.exists(CYCLE_STATE_FILE):
        os.remove(CYCLE_STATE_FILE)
        print(f"   ✓ 사이클 상태 초기화: {CYCLE_STATE_FILE}")
    
    # /tmp 내 임시 맵 파일 정리
    tmp_maps = glob.glob('/tmp/slam_*.pgm') + glob.glob('/tmp/slam_*.yaml')
    for f in tmp_maps:
        try:
            os.remove(f)
            print(f"   ✓ 임시 파일 삭제: {f}")
        except:
            pass
    
    print("\n🔄 다음 시작 시 초기 상태로 시작됩니다.")
    print("=" * 70 + "\n")
    return []


def initialize_on_startup(context, *args, **kwargs):
    """시작 시 초기화"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    print("\n" + "=" * 70)
    print(f"🚀 PINKY 서버 초기화 - {timestamp}")
    print("=" * 70)
    
    # 저장 디렉토리 확인/생성
    for path in [SAVE_MAP_PATH, SAVE_COLLISION_PATH, SAVE_AI_PATH]:
        os.makedirs(path, exist_ok=True)
        print(f"   ✓ 저장 경로 확인: {path}")
    
    # 이전 사이클 상태 파일이 있으면 삭제 (새로 시작)
    if os.path.exists(CYCLE_STATE_FILE):
        os.remove(CYCLE_STATE_FILE)
        print(f"   ✓ 이전 사이클 상태 초기화")
    
    # 오늘 날짜 맵 개수 확인
    today = datetime.now().strftime('%Y%m%d')
    today_maps = glob.glob(os.path.join(SAVE_MAP_PATH, f'map_{today}*.pgm'))
    print(f"   📊 오늘 저장된 맵: {len(today_maps)}개")
    
    print("=" * 70 + "\n")
    return []


def get_local_ip():
    """로컬 IP 주소 가져오기"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


def generate_launch_description():
    local_ip = get_local_ip()
    
    # ─────────────────────────────────────────────────────────────
    # Launch Arguments
    # ─────────────────────────────────────────────────────────────
    ai_arg = DeclareLaunchArgument('ai', default_value='true', description='AI 비전 활성화')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.0.5', description='로봇 IP')
    model_arg = DeclareLaunchArgument('model', default_value='/home/kim1/model/best.pt', description='YOLO 모델')
    web_port_arg = DeclareLaunchArgument('web_port', default_value='8080', description='웹 서버 포트')
    
    return LaunchDescription([
        # ─────────────────────────────────────────────────────────
        # 초기화/종료 이벤트 핸들러
        # ─────────────────────────────────────────────────────────
        OpaqueFunction(function=initialize_on_startup),
        
        RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_on_shutdown)])
        ),
        
        # Arguments
        ai_arg,
        robot_ip_arg,
        model_arg,
        web_port_arg,
        
        # ─────────────────────────────────────────────────────────
        # 시작 메시지
        # ─────────────────────────────────────────────────────────
        LogInfo(msg=''),
        LogInfo(msg='=' * 70),
        LogInfo(msg='🤖 PINKY 서버 통합 시스템 시작'),
        LogInfo(msg='=' * 70),
        LogInfo(msg=''),
        LogInfo(msg='📡 서비스:'),
        LogInfo(msg='   • MQTT Bridge - 로봇 통신'),
        LogInfo(msg='   • Map Builder - SLAM 맵 통합'),
        LogInfo(msg='   • AI Vision - ArUco/YOLO 분석'),
        LogInfo(msg='   • Web Server - 웹 UI'),
        LogInfo(msg='   • Upload Server - 맵 업로드'),
        LogInfo(msg=''),
        LogInfo(msg=f'🌐 접속 주소:'),
        LogInfo(msg=f'   • 웹 UI: http://{local_ip}:8080'),
        LogInfo(msg=f'   • Grafana: http://{local_ip}:3000'),
        LogInfo(msg=f'   • 업로드: http://{local_ip}:5100'),
        LogInfo(msg=''),
        LogInfo(msg='=' * 70),
        
        # ─────────────────────────────────────────────────────────
        # 1. 통합 서버 (Flask 업로드 + 네트워크 모니터)
        # ─────────────────────────────────────────────────────────
        Node(
            package='slam_mqtt_server',
            executable='unified_server',
            name='unified_server',
            output='screen',
            emulate_tty=True,
        ),
        
        # ─────────────────────────────────────────────────────────
        # 2. MQTT 브릿지 (모드 감지 + 자동 전환)
        # ─────────────────────────────────────────────────────────
        Node(
            package='slam_mqtt_server',
            executable='server_mqtt_bridge',
            name='server_mqtt_bridge',
            output='screen',
            emulate_tty=True,
        ),
        
        # ─────────────────────────────────────────────────────────
        # 3. 맵 빌더 (SLAM 사이클 맵 → Nav2 통합 맵)
        # ─────────────────────────────────────────────────────────
        Node(
            package='slam_mqtt_server',
            executable='nav2_map_builder',
            name='nav2_map_builder',
            output='screen',
            emulate_tty=True,
        ),
        
        # ─────────────────────────────────────────────────────────
        # 4. AI 비전 (ArUco + YOLO)
        # ─────────────────────────────────────────────────────────
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
        
        # ─────────────────────────────────────────────────────────
        # 5. Web Server (pinky_navigation - nav2_web_server.py)
        # ─────────────────────────────────────────────────────────
        ExecuteProcess(
            cmd=['python3', '/home/kim1/pinky_pro/src/pinky_pro/pinky_navigation/scripts/nav2_web_server.py'],
            name='nav2_web_server',
            output='screen',
        ),
    ])
