# 🌐 Web RViz 설정 가이드

> RViz를 웹 브라우저에서 외부 접속하는 방법

---

## 📋 개요

ROS2의 RViz는 기본적으로 로컬 X11 디스플레이가 필요합니다.  
외부에서 접속하려면 다음 방법 중 하나를 사용합니다:

| 방법 | 장점 | 단점 |
|------|------|------|
| **Foxglove Studio** ⭐ | 웹 브라우저, 모바일 앱 지원 | rosbridge 설치 필요 |
| VNC | RViz 그대로 사용 | 느림, 설정 복잡 |
| X11 Forwarding | 간단 | 느림, SSH 필요 |

---

## 🚀 방법 1: Foxglove Studio (추천)

### 1. 설치

```bash
# 스크립트로 자동 설치
cd ~/ros2_ws/src/slam_mqtt_server/scripts
chmod +x setup_web_rviz.sh
./setup_web_rviz.sh
```

또는 수동 설치:
```bash
# rosbridge 설치
sudo apt install -y ros-jazzy-rosbridge-server ros-jazzy-rosbridge-suite

# 빌드
cd ~/ros2_ws
colcon build --packages-select slam_mqtt_server --symlink-install
source install/setup.bash
```

### 2. 서버 실행

```bash
# Web RViz 서버 시작
ros2 launch slam_mqtt_server web_rviz.launch.py
```

### 3. 외부 접속

1. **Foxglove Studio** 접속: https://foxglove.dev/studio
2. **Open connection** 클릭
3. **Rosbridge (ROS 1 & 2)** 선택
4. WebSocket URL 입력: `ws://192.168.0.3:9090`
5. **Open** 클릭

### 4. 패널 추가

| 패널 | 토픽 | 용도 |
|------|------|------|
| **3D** | /map, /scan, /tf | 맵 + 라이다 시각화 |
| **Image** | /camera/image/compressed | 카메라 뷰 |
| **Plot** | /odom | 속도 그래프 |
| **Raw Messages** | 아무 토픽 | 디버깅 |

---

## 🤖 로봇 패키지 연동 (slam_mqtt_project)

### 아키텍처

```
┌──────────────────┐         ┌──────────────────┐
│   로봇 (0.5)     │  MQTT   │   서버 (0.3)     │
│ slam_mqtt_project│ ←────→  │ slam_mqtt_server │
│                  │         │                  │
│  • 센서 데이터   │         │  • 맵 병합       │
│  • LiDAR /scan   │         │  • AI 분석       │
│  • Odom /odom    │         │  • Web RViz      │
└──────────────────┘         └──────────────────┘
                                     │
                                     ↓ WebSocket
                             ┌──────────────────┐
                             │ 🌐 Foxglove     │
                             │    Studio        │
                             │ (웹 브라우저)    │
                             └──────────────────┘
```

### ROS2 네트워크 설정

로봇과 서버가 같은 ROS2 도메인에 있어야 합니다:

```bash
# 로봇 (192.168.0.5)
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0

# 서버 (192.168.0.3)
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0

# DDS 설정 (Cyclone DDS 사용 시)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/kim1/ros2_ws/cyclonedds.xml
```

### 로봇에서 발행되는 주요 토픽

```
/scan             # LiDAR 스캔 (sensor_msgs/LaserScan)
/odom             # 오도메트리 (nav_msgs/Odometry)
/map              # SLAM 맵 (nav_msgs/OccupancyGrid)
/tf               # 좌표 변환 (tf2_msgs/TFMessage)
/camera/image/compressed  # 카메라 (sensor_msgs/CompressedImage)
```

---

## 🔧 문제 해결

### 연결 안됨

```bash
# rosbridge 포트 확인
sudo lsof -i :9090

# 방화벽 확인
sudo ufw status
sudo ufw allow 9090/tcp

# rosbridge 로그 확인
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

### 토픽이 안보임

```bash
# ROS2 토픽 확인
ros2 topic list

# 네트워크 연결 확인 (로봇 ↔ 서버)
ros2 node list
ros2 topic echo /scan --once
```

### DDS 문제

```bash
# 멀티캐스트 확인
ping -c 3 224.0.0.1

# Cyclone DDS 설정 확인
cat ~/ros2_ws/cyclonedds.xml
```

---

## 📱 모바일 접속

Foxglove는 모바일 앱도 제공합니다:

1. **iOS**: App Store에서 "Foxglove" 검색
2. **Android**: Play Store에서 "Foxglove" 검색
3. 동일한 WebSocket URL로 접속

---

## 🔗 참고 링크

- [Foxglove Studio](https://foxglove.dev/studio)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
- [ROS2 Jazzy 문서](https://docs.ros.org/en/jazzy/)
