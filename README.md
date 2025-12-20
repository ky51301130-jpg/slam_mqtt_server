# 🖥️ SLAM MQTT Server

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.12+-yellow)
![License](https://img.shields.io/badge/License-MIT-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04-orange)

Pinky 로봇 시스템의 **서버 측 ROS2 패키지**입니다.  
MQTT 브릿지, 맵 병합, AI 비전 분석, 모니터링 기능을 제공합니다.

> 🤖 로봇 측 코드는 [slam_mqtt_project](https://github.com/ky51301130-jpg/slam_mqtt_project) 저장소를 참조하세요.

---

## 📋 목차

- [시스템 개요](#-시스템-개요)
- [Quick Start](#-quick-start)
- [노드 설명](#-노드-설명)
- [네트워크 구성](#-네트워크-구성)
- [MQTT 토픽](#-mqtt-토픽)
- [설정 파일](#-설정-파일)
- [모니터링](#-모니터링-grafana--influxdb)
- [개발자 가이드](#-개발자-가이드)
- [트러블슈팅](#-트러블슈팅)

---

## 🎯 시스템 개요

```
┌─────────────────────────────────────────────────────────────┐
│                    서버 (192.168.0.3)                        │
│                                                             │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │  unified_server │  │ server_mqtt     │                  │
│  │  ─────────────  │  │ _bridge         │                  │
│  │  • 맵 업로드    │  │ ─────────────── │                  │
│  │  • 충돌 사진    │  │  ROS2 ↔ MQTT    │                  │
│  │  • 네트워크 모니터│  │  PLC/MCU 연동   │                  │
│  └─────────────────┘  └─────────────────┘                  │
│                                                             │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │ nav2_map_builder│  │ ai_vision       │                  │
│  │ ─────────────── │  │ _analyzer       │                  │
│  │  8맵 ICP 병합   │  │ ─────────────── │                  │
│  │  과반수 투표    │  │  ArUco + YOLO   │                  │
│  └─────────────────┘  └─────────────────┘                  │
│                                                             │
│           ↕ MQTT (localhost:1883)                          │
└─────────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        ↓                 ↓                 ↓
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   🤖 로봇    │  │   📡 MCU     │  │   🏭 PLC     │
│ 192.168.0.5  │  │ 192.168.0.4  │  │192.168.0.155 │
│  SLAM/Nav2   │  │  센서 데이터  │  │  위치 명령   │
└──────────────┘  └──────────────┘  └──────────────┘
```

---

## 🚀 Quick Start

### 1. 의존성 설치

```bash
# ROS2 Jazzy 설치 필요
# https://docs.ros.org/en/jazzy/Installation.html

# Python 패키지
pip3 install paho-mqtt flask ultralytics opencv-python-headless pyyaml requests numpy

# MQTT 브로커
sudo apt install mosquitto mosquitto-clients
```

### 2. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select slam_mqtt_server --symlink-install
source install/setup.bash
```

### 3. 실행

```bash
# 전체 시스템 (AI 포함)
ros2 launch slam_mqtt_server unified.launch.py

# AI 없이 (경량 모드)
ros2 launch slam_mqtt_server unified.launch.py use_ai:=false

# RViz 포함 (SLAM 모드)
ros2 launch slam_mqtt_server unified.launch.py rviz:=slam

# RViz 포함 (Nav2 모드)
ros2 launch slam_mqtt_server unified.launch.py rviz:=nav2
```

---

## 📦 노드 설명

### 1. `unified_server` - 통합 서버 노드

3개 기능을 하나로 통합한 경량 노드:

| 기능 | 설명 |
|------|------|
| 맵 업로드 서버 | Flask `:5100` - 로봇이 맵 파일 업로드 |
| 충돌 사진 수신 | 로봇 충돌 시 사진 다운로드 → `/home/kim1/save/collision/` 저장 |
| 네트워크 모니터 | MCU/Robot/PLC 연결 상태 체크 |

#### 📸 충돌 사진 저장 기능

로봇이 SLAM 모드에서 충돌 감지 시:
1. 로봇이 `collision/photo_ready` MQTT 토픽으로 사진 URL 전송
2. 서버가 HTTP로 사진 다운로드
3. `/home/kim1/save/collision/collision_YYYYMMDD_HHMMSS.jpg`로 저장
4. 최대 100장 유지 (오래된 파일 자동 삭제)

```
저장 경로: /home/kim1/save/collision/
파일 형식: collision_20241220_143052.jpg
용도: YOLO 학습 데이터 / 충돌 원인 분석
```

```bash
# 단독 실행
ros2 run slam_mqtt_server unified_server

# 헬스 체크
curl http://192.168.0.3:5100/health

# 맵 업로드 테스트
curl -X POST -F "file=@map.pgm" http://192.168.0.3:5100/upload

# 저장된 충돌 사진 확인
ls -la /home/kim1/save/collision/
```

### 2. `server_mqtt_bridge` - MQTT 브릿지

ROS2 토픽과 MQTT 메시지 양방향 변환:

```
MQTT → ROS2:
  /mcu/sensors      → /mqtt/mcu_sensors
  /plc/location     → /mqtt/plc_location
  /plc/goal         → /mqtt/plc_goal

ROS2 → MQTT:
  /robot_mode       → slam_mode
  /ros/nav2/status  → robot/nav_status
  /ros/nav2/arrived → robot/arrived
```

### 3. `nav2_map_builder` - 맵 병합 노드

SLAM 맵 8장을 ICP 정렬 + 과반수 투표로 병합:

```
┌────────────────────────────────────────┐
│  Input: 8개 맵 (map_*.pgm)             │
│  ↓                                     │
│  1. ICP 회전 정렬 (첫 번째 맵 기준)     │
│  2. 중심점 기반 위치 정렬              │
│  3. 과반수 투표 (5/8 이상 = 벽)        │
│  ↓                                     │
│  Output: merged_map.pgm/yaml           │
│  → /map 토픽으로 발행                  │
│  → Nav2에서 바로 사용                  │
└────────────────────────────────────────┘
```

### 4. `ai_vision_analyzer` - AI 비전 노드 (선택)

| 기능 | 설명 |
|------|------|
| ArUco 감지 | HOME/PORT 마커 (ID 0~4) |
| YOLO 감지 | 장애물 (PORT_A, PORT_B) |

```bash
# YOLO 모델 경로 지정
ros2 launch slam_mqtt_server unified.launch.py yolo_model:=/path/to/best.pt
```

---

## 🌐 네트워크 구성

| 장치 | IP 주소 | 역할 |
|------|---------|------|
| 서버 PC | `192.168.0.3` | ROS2, MQTT 브로커, 모니터링 |
| 로봇 | `192.168.0.5` | Raspberry Pi, SLAM/Nav2 |
| MCU | `192.168.0.4` | 센서 데이터 (ESP32) |
| PLC | `192.168.0.155` | 위치 명령, 포트 상태 |

### 포트 번호

| 포트 | 서비스 |
|------|--------|
| 1883 | MQTT (Mosquitto) |
| 5100 | 맵 업로드 서버 (Flask) |
| 5000 | 충돌 사진 서버 (로봇) |
| 5200 | 카메라 스트리밍 (로봇) |
| 8086 | InfluxDB |
| 3000 | Grafana |
| 8765 | Foxglove |

---

## 📡 MQTT 토픽

### 구독 (외부 → 서버)

| 토픽 | 발신자 | 데이터 |
|------|--------|--------|
| `/mcu/sensors` | MCU | `{"lux": 500, "temp": 25.5}` |
| `/plc/location` | PLC | `"A"` or `"B"` |
| `/plc/goal` | PLC | `{"x": 1.0, "y": 2.0, "yaw": 0.0}` |
| `/plc/port_status` | PLC | `{"A": 1, "B": 0}` |
| `robot/nav_result` | 로봇 | `{"goal": "A", "success": true}` |
| `collision/photo_ready` | 로봇 | `{"url": "http://192.168.0.5:5000/photos/collision_*.jpg"}` |

> 💡 **충돌 사진 흐름**: 로봇이 `collision/photo_ready` 발행 → 서버가 URL에서 사진 다운로드 → `/home/kim1/save/collision/`에 저장 → YOLO 학습 데이터로 활용

### 발행 (서버 → 외부)

| 토픽 | 수신자 | 데이터 |
|------|--------|--------|
| `slam_mode` | 로봇 | `"slam"` / `"nav2"` / `"idle"` |
| `robot/nav_status` | PLC | `{"status": "navigating"}` |
| `robot/arrived` | PLC | `true` / `false` |
| `network/connectivity` | 모니터링 | `{"mcu": 100, "robot": 100, "plc": 0}` |
| `mqtt/pinky/detection` | 모니터링 | YOLO 감지 결과 |
| `mqtt/pinky/aruco` | 모니터링 | ArUco 감지 결과 |

---

## ⚙️ 설정 파일

모든 설정은 `config.py`에서 중앙 관리:

```python
from slam_mqtt_server.config import NET, ROS, MQTT, Path, Setting

# 네트워크
NET.SERVER_IP      # "192.168.0.3"
NET.ROBOT_IP       # "192.168.0.5"
NET.MCU_IP         # "192.168.0.4"
NET.PLC_IP         # "192.168.0.155"

# 경로
Path.RAW_MAP       # "/home/kim1/save/map"          - SLAM 원본 맵
Path.MERGED_MAP    # "/home/kim1/save/renewed_map"  - 병합된 Nav2 맵
Path.COLLISION     # "/home/kim1/save/collision"    - 충돌 사진 (YOLO 학습용)
Path.AI_DETECTIONS # "/home/kim1/save/ai_detections" - AI 감지 결과 이미지

# 설정
Setting.CYCLE_COUNT      # 8 (맵 병합 개수)
Setting.MAX_PHOTOS       # 100 (최대 충돌 사진 수)
Setting.PING_TIMEOUT     # 1.0초
Setting.MAP_INTERVAL     # 1.0초
```

### 📁 저장 폴더 구조

```
/home/kim1/save/
├── map/                    # SLAM 원본 맵 (8장)
│   ├── map_20241220_140000.pgm
│   ├── map_20241220_140000.yaml
│   └── ...
├── renewed_map/            # 병합된 Nav2 최종 맵
│   ├── nav2_final_map_20241220_143000.pgm
│   └── nav2_final_map_20241220_143000.yaml
├── collision/              # 🔥 충돌 사진 (YOLO 학습 데이터)
│   ├── collision_20241220_143052.jpg
│   ├── collision_20241220_143215.jpg
│   └── ... (최대 100장, 자동 정리)
└── ai_detections/          # AI 감지 결과 이미지
    ├── aruco/              # ArUco 마커 감지
    ├── obstacles/          # SLAM 모드 장애물
    └── nav2_obstacles/     # Nav2 모드 장애물
```

### 설정 변경

```python
# config.py 수정
class NET:
    SERVER_IP = "192.168.0.100"  # 새 서버 IP
    ROBOT_IP = "192.168.0.200"   # 새 로봇 IP
```

---

## 📊 모니터링 (Grafana + InfluxDB)

### 자동 설치

```bash
cd ~/ros2_ws
./setup_monitoring.sh
```

### 수동 설치

```bash
# InfluxDB v2
wget https://dl.influxdata.com/influxdb/releases/influxdb2-2.7.4-amd64.deb
sudo dpkg -i influxdb2-2.7.4-amd64.deb
sudo systemctl enable influxdb --now

# Telegraf
sudo apt install telegraf
sudo cp telegraf.conf /etc/telegraf/telegraf.conf
sudo systemctl restart telegraf

# Grafana
sudo apt install grafana
sudo systemctl enable grafana-server --now
```

### 대시보드 접속

```
Grafana: http://localhost:3000
ID: admin / PW: admin

대시보드 Import: grafana_dashboard.json
```

### 모니터링 항목

- 🌐 네트워크 연결 상태 (MCU/Robot/PLC)
- 🌡️ MCU 센서 (온도, 습도, 조도)
- 🚦 PLC PORT 상태
- 🤖 로봇 모드 분포
- 🔋 배터리 전압
- 🎯 ArUco 마커 검출
- 💻 서버 CPU/RAM 사용률

---

## 🛠️ 개발자 가이드

### 프로젝트 구조

```
slam_mqtt_server/
├── slam_mqtt_server/
│   ├── __init__.py
│   ├── config.py                  # 📌 중앙 설정 파일
│   ├── unified_server.py          # 통합 서버 노드
│   ├── server_mqtt_bridge.py      # MQTT 브릿지
│   ├── nav2_map_builder.py        # 맵 병합 노드
│   └── ai_vision_analyzer.py      # AI 비전 (선택)
├── launch/
│   ├── unified.launch.py          # 📌 메인 런치 파일
│   ├── slam_rviz.launch.py
│   └── nav2_rviz.launch.py
├── rviz/
│   ├── slam_view.rviz
│   └── nav2_view.rviz
├── docs/
│   ├── DATA_EXCHANGE_GUIDE.md     # 데이터 교환 가이드
│   └── ICP_ALGORITHM_GUIDE.md     # ICP 알고리즘 설명
├── config/
│   ├── telegraf.conf              # Telegraf 설정
│   └── grafana_dashboard.json     # Grafana 대시보드
├── scripts/
│   └── setup_monitoring.sh        # 모니터링 설치 스크립트
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
└── README.md
```

### 개별 노드 실행 (디버깅)

```bash
# 통합 서버만
ros2 run slam_mqtt_server unified_server

# MQTT 브릿지만
ros2 run slam_mqtt_server server_mqtt_bridge

# 맵 빌더만
ros2 run slam_mqtt_server nav2_map_builder

# AI 비전만
ros2 run slam_mqtt_server ai_vision_analyzer
```

### 로그 확인

```bash
# ROS2 노드 정보
ros2 node info /unified_server

# MQTT 메시지 모니터링
mosquitto_sub -h localhost -t "#" -v

# InfluxDB 데이터 확인
influx query 'from(bucket:"slam_data") 
  |> range(start: -5m) 
  |> filter(fn: (r) => r._measurement == "network_status")' \
  --org slam_org
```

---

## 🐛 트러블슈팅

### 1. MQTT 연결 실패

```bash
# Mosquitto 상태 확인
sudo systemctl status mosquitto

# 재시작
sudo systemctl restart mosquitto

# 테스트 메시지
mosquitto_pub -h localhost -t "test" -m "hello"
mosquitto_sub -h localhost -t "test"
```

### 2. Flask 포트 충돌

```bash
# 기존 프로세스 확인
sudo lsof -i :5100

# 프로세스 종료
kill -9 <PID>
```

### 3. 맵 병합 안됨

```bash
# 맵 파일 확인
ls -la /home/kim1/save/map/

# 최소 8개 필요
# map_YYYYMMDD_HHMMSS.pgm
```

### 4. AI 비전 오류

```bash
# YOLO 모델 확인
ls -la /home/kim1/model/best.pt

# 카메라 스트리밍 확인
curl http://192.168.0.5:5200/image.jpg -o test.jpg
```

### 5. 네트워크 모니터링 안됨

```bash
# ping 테스트
ping -c 1 192.168.0.5  # Robot
ping -c 1 192.168.0.4  # MCU
ping -c 1 192.168.0.155  # PLC
```

---

## 📚 관련 문서

- [DATA_EXCHANGE_GUIDE.md](docs/DATA_EXCHANGE_GUIDE.md) - 로봇-서버 데이터 교환 가이드
- [ICP_ALGORITHM_GUIDE.md](docs/ICP_ALGORITHM_GUIDE.md) - 맵 병합 ICP 알고리즘 설명
- [MONITORING_SETUP_GUIDE.md](docs/MONITORING_SETUP_GUIDE.md) - Grafana 모니터링 상세 가이드

---

## 🔧 기술 스택

| 항목 | 버전 |
|------|------|
| ROS2 | Jazzy Jalisco |
| Ubuntu | 24.04 LTS |
| Python | 3.12+ |
| Flask | 3.0+ |
| MQTT | Mosquitto 2.0+ |
| AI | YOLOv8 (Ultralytics) |
| Monitoring | InfluxDB v2 + Telegraf + Grafana |

---

## 📄 License

MIT License

---

## 👥 Contributors

- **Server Integration & Monitoring**: 2024
- **Original SLAM System**: Pinky Project
- **GitHub**: [@ky51301130-jpg](https://github.com/ky51301130-jpg)

---

## 🔗 관련 저장소

| 저장소 | 설명 | 실행 위치 |
|--------|------|----------|
| 🖥️ [slam_mqtt_server](https://github.com/ky51301130-jpg/slam_mqtt_server) | 서버 측 코드 (현재) | PC (192.168.0.3) |
| 🤖 [slam_mqtt_project](https://github.com/ky51301130-jpg/slam_mqtt_project) | 로봇 측 코드 | Raspberry Pi (192.168.0.5) |

