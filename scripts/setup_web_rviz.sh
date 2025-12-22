#!/bin/bash
# =============================================================================
#                    Web RViz 설정 스크립트
# =============================================================================
# rosbridge + Foxglove Studio를 통해 RViz를 웹에서 볼 수 있게 합니다.
#
# 사용법:
#     chmod +x setup_web_rviz.sh
#     ./setup_web_rviz.sh
# =============================================================================

set -e

echo "=============================================="
echo "🌐 Web RViz 설정 시작"
echo "=============================================="

# 1. rosbridge 패키지 설치
echo ""
echo "📦 rosbridge_server 설치..."
sudo apt update
sudo apt install -y ros-jazzy-rosbridge-server ros-jazzy-rosbridge-suite

# 2. tf2_web_republisher (선택사항)
echo ""
echo "📦 tf2_web_republisher 설치..."
sudo apt install -y ros-jazzy-tf2-web-republisher || {
    echo "⚠️ tf2_web_republisher 설치 실패 (선택사항이므로 계속 진행)"
}

# 3. 방화벽 설정
echo ""
echo "🔥 방화벽 포트 열기 (9090)..."
sudo ufw allow 9090/tcp || echo "ufw가 설치되지 않았거나 비활성화됨"

# 4. 빌드
echo ""
echo "🔨 패키지 빌드..."
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select slam_mqtt_server --symlink-install

# 5. 완료 메시지
echo ""
echo "=============================================="
echo "✅ 설치 완료!"
echo "=============================================="
echo ""
echo "📡 실행 방법:"
echo "   source ~/ros2_ws/install/setup.bash"
echo "   ros2 launch slam_mqtt_server web_rviz.launch.py"
echo ""
echo "🌐 외부 접속 방법:"
echo "   1. Foxglove Studio 접속: https://foxglove.dev/studio"
echo "   2. Open connection → Rosbridge (ROS 1 & 2)"
echo "   3. WebSocket URL: ws://192.168.0.3:9090"
echo ""
echo "📱 또는 Foxglove 앱 사용:"
echo "   - iOS/Android 앱 다운로드 가능"
echo "   - 동일한 WebSocket URL로 접속"
echo ""
echo "=============================================="
