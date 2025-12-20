#!/usr/bin/env python3
"""
=============================================================================
                    서버 통합 설정 파일
=============================================================================
로봇의 topics.py 구조와 동기화된 설정 파일입니다.
IP, 포트, 경로, ROS2 토픽, MQTT 토픽을 한 곳에서 관리합니다.

사용법:
    from slam_mqtt_project.config import ROS, MQTT, NET, Path, Setting
=============================================================================
"""


# ============================================================================
# 네트워크 설정 (로봇 topics.py의 NET 클래스와 동기화)
# ============================================================================

class NET:
    """네트워크 설정 - IP 주소 및 포트 중앙 관리"""
    
    # IP 주소
    SERVER_IP = "192.168.0.3"       # PC 서버 (MQTT, Flask, InfluxDB)
    ROBOT_IP = "192.168.0.5"        # 로봇 (라즈베리파이)
    MCU_IP = "192.168.0.4"          # MCU (ESP32 등)
    PLC_IP = "192.168.0.155"        # PLC
    
    # MQTT
    MQTT_PORT = 1883
    
    # Flask 서버 포트
    COLLISION_PORT = 5000           # 충돌 사진 (로봇에서 제공)
    MAP_UPLOAD_PORT = 5100          # 맵 업로드 (PC 서버)
    STREAMING_PORT = 5200           # 카메라 스트리밍 (로봇에서 제공)
    
    # 서버 전용 포트
    INFLUXDB_PORT = 8086
    GRAFANA_PORT = 3000
    FOXGLOVE_PORT = 8765
    
    # 업로드 URL
    @classmethod
    def map_upload_url(cls):
        return f"http://{cls.SERVER_IP}:{cls.MAP_UPLOAD_PORT}/upload"
    
    @classmethod
    def collision_photo_url(cls, filename=""):
        base = f"http://{cls.ROBOT_IP}:{cls.COLLISION_PORT}/photos"
        return f"{base}/{filename}" if filename else base
    
    @classmethod
    def stream_url(cls):
        return f"http://{cls.ROBOT_IP}:{cls.STREAMING_PORT}/image.jpg"


# ============================================================================
# ROS2 토픽 (로봇 topics.py의 ROS 클래스와 동기화)
# ============================================================================

class ROS:
    """ROS2 토픽 정의 - 로봇과 동일한 구조"""
    
    # ======================== 센서 ========================
    SCAN = "/scan"                          # LiDAR 스캔 (LaserScan)
    ODOM = "/odom"                          # 오도메트리 (Odometry)
    ULTRASONIC = "/ultrasonic"              # 초음파 거리 (Float32)
    
    # ======================== 모터/주행 ========================
    CMD_VEL = "cmd_vel"                     # 속도 명령 (Twist)
    
    # ======================== 로봇 상태 ========================
    ROBOT_MODE = "/robot_mode"              # SLAM/NAV2/IDLE (String)
    ROBOT_STATUS = "ros/robot_status"       # 상태 메시지 (String)
    
    # ======================== 자율 주행 (SLAM) ========================
    AUTO_DRIVE_ENABLE = "ros/auto_drive/enable"   # 자율 주행 시작/정지 (Bool)
    AUTO_DRIVE_ACTIVE = "ros/auto_drive/active"   # 주행 중 여부 (Bool)
    CAMERA_TRIGGER = "ros/camera_trigger"         # 카메라 캡처 트리거 (Bool)
    
    # ======================== 맵 저장 ========================
    MAP = "/map"                            # OccupancyGrid 맵 (시스템)
    MAP_SAVER_SAVED = "ros/map_saver/saved"       # 저장된 맵 번호 (Int32)
    MAP_SAVER_CYCLE = "ros/map_saver/cycle_complete"   # 사이클 완료 (String)
    MAP_SAVER_COMPLETE = "ros/map_saver/map_complete"  # 맵 완성 (Bool)
    
    # 서버 내부용 (업로드 후 발행)
    MAP_SAVER_CYCLE_LOCAL = "/map_saver/cycle_complete"  # 서버 내부 사이클 완료
    
    # ======================== Odom 리셋 ========================
    ODOM_RESET = "ros/odom/reset"           # Odom 리셋 명령 (Bool)
    
    # ======================== 귀환 ========================
    ROBOT_AT_HOME = "ros/robot/at_home"     # 홈 위치 도착 (Bool)
    ROBOT_RETURN_HOME = "ros/robot/return_home"   # 귀환 명령 (Bool)
    
    # ======================== Nav2 ========================
    NAV2_MODE = "ros/nav2/mode"             # Nav2 모드 (String)
    NAV2_MAP_READY = "ros/nav2/map_ready"   # 맵 로드 완료 (Bool)
    NAV2_GOAL = "ros/nav2/goal"             # Goal 목표 (PoseStamped)
    NAV2_CANCEL = "ros/nav2/cancel"         # Goal 취소 (Bool)
    NAV2_STATUS = "ros/nav2/status"         # 진행 상태 (String/JSON)
    NAV2_ARRIVED = "ros/nav2/arrived"       # 도착 여부 (Bool)
    INITIAL_POSE = "/initialpose"           # AMCL 초기 위치 (시스템)
    
    # 하위호환 별칭
    MAP_READY = NAV2_MAP_READY
    
    # ======================== 충돌 감지 ========================
    COLLISION_TRIGGER = "ros/collision/trigger"       # 충돌 트리거 (Bool)
    COLLISION_PHOTO_READY = "ros/collision/photo_ready"  # 사진 준비 (String/JSON)
    COLLISION_PHOTO_SAVED = "ros/collision/photo_saved"  # 사진 저장 완료 (String)
    COLLISION_IMAGE = "ros/collision/image"           # 압축 이미지 (CompressedImage)
    
    # ======================== MQTT 브릿지 (외부→ROS2) ========================
    MQTT_MCU_SENSORS = "mqtt/mcu_sensors"       # MCU 센서 데이터 (String/JSON)
    MQTT_PLC_LOCATION = "mqtt/plc_location"     # PLC 위치 명령 (String)
    MQTT_PLC_GOAL = "mqtt/plc_goal"             # PLC 좌표 명령 (String/JSON)
    
    # ======================== 배터리 ========================
    BATTERY_VOLTAGE = "ros/battery/voltage"     # 전압 (Float32)
    BATTERY_PRESENT = "ros/battery/present"     # 배터리 잔량 (Float32)
    
    # ======================== QR 코드 (장애물 마커) ========================
    QR_OBSTACLE = "ros/qr/obstacle"             # QR 장애물 감지 (String: 장애물 유형)
    QR_DETECTED = "/qr/detected"                # QR 감지 (하위호환)
    
    # ======================== ArUco 마커 (HOME + PORT) ========================
    ARUCO_HOME_DETECTED = "ros/aruco/home_detected"   # HOME 마커 감지 (String/JSON)
    ARUCO_PORT_DETECTED = "ros/aruco/port_detected"   # PORT 마커 감지 (String/JSON)
    
    # ======================== 카메라/AI ========================
    CAMERA_IMAGE = "ros/camera/image"           # 카메라 이미지 (CompressedImage)
    OBSTACLE_ACTION = "ros/obstacle/action"     # 장애물 동작 (String)
    STOP_ROBOT = "ros/stop_robot"               # 긴급 정지 (Bool)
    
    # ======================== AI 서버 응답 ========================
    AI_DETECTION = "mqtt/pinky/detection"       # 감지 결과 (String/JSON)
    AI_OBSTACLE_TYPE = "mqtt/pinky/obstacle_type"   # 장애물 유형 (String)
    AI_ARUCO = "/ai/aruco_detected"             # AI ArUco 감지 (하위호환)
    AI_OBSTACLE = "/ai/obstacle_detected"       # AI 장애물 감지 (하위호환)
    
    # ======================== 슬램 모드 ========================
    SLAM_MODE = "/slam_mode"                    # SLAM 모드 (Bool)


# ============================================================================
# MQTT 토픽 (로봇 topics.py의 MQTT 클래스와 동기화)
# ============================================================================

class MQTT:
    """MQTT 토픽 정의 - 로봇과 동일한 구조"""
    
    # ======================== 연결 설정 ========================
    HOST = NET.SERVER_IP
    PORT = NET.MQTT_PORT
    
    # ======================== 구독 (외부 → 서버) ========================
    SUB_MCU_SENSORS = "/mcu/sensors"         # MCU 센서 (Lux 등)
    SUB_PLC = "plc/#"                        # PLC 명령 (location, goal)
    SUB_PLC_LOCATION = "/plc/location"       # PLC 위치 명령
    SUB_PLC_GOAL = "plc/goal"                # PLC 좌표 명령
    SUB_PLC_PORT_STATUS = "/plc/port_status" # PLC 포트 상태
    SUB_NAV_RESULT = "robot/nav_result"      # Nav2 결과 (로봇→서버)
    
    # 구독 토픽 리스트
    SUBSCRIBE_LIST = [
        SUB_MCU_SENSORS,
        SUB_PLC_LOCATION,
        SUB_PLC_GOAL,
        SUB_NAV_RESULT,
    ]
    
    # ======================== 발행 (서버 → 외부) ========================
    # 로봇 상태
    SLAM_MODE = "slam_mode"                     # 로봇 모드
    
    # Nav2 상태
    NAV_STATUS = "robot/nav_status"             # 진행 상태
    NAV_RESULT = "robot/nav_result"             # 최종 결과
    NAV_ARRIVED = "robot/arrived"               # 도착 알림
    NAVIGATE_TO_POSE = "robot/navigate_to_pose" # Goal 요청 모니터링
    
    # SLAM/맵
    MAP_CYCLE_COMPLETE = "ros/map_cycle_complete"   # 맵 저장 사이클
    COLLISION_PHOTO = "collision/photo_ready"       # 충돌 사진
    
    # 센서
    BATTERY_STATUS = "battery/status"           # 배터리
    
    # 장애물 (QR 마커로 인식)
    QR_OBSTACLE = "obstacle/qr_detected"        # QR 장애물 감지
    QR_DETECTED = "qr_detected"                 # 하위호환
    
    # ArUco (HOME + PORT)
    ARUCO_HOME = "aruco/home_detected"          # HOME 마커 감지
    ARUCO_PORT = "aruco/port_detected"          # PORT 마커 감지
    
    # AI Vision (서버 발행)
    AI_DETECTION = "mqtt/pinky/detection"       # 객체 감지
    AI_ARUCO = "mqtt/pinky/aruco"               # ArUco 감지
    AI_OBSTACLE = "mqtt/pinky/port_obstacle"    # 장애물 유형
    
    # 네트워크
    NETWORK = "network/connectivity"            # 네트워크 상태


# ============================================================================
# ArUco 마커 설정 (로봇 topics.py의 ARUCO 클래스와 동기화)
# ============================================================================

class ARUCO:
    """ArUco 마커 관련 설정"""
    
    # ======================== 마커 ID ========================
    PORT_A_ID = 0  # SLAM HOME 역할
    PORT_B_ID = 1
    PORT_C_ID = 2
    PORT_D_ID = 3
    PORT_E_ID = 4
    
    # HOME은 PORT_A와 동일
    HOME_ID = PORT_A_ID
    
    # 도킹용 마커 ID 세트
    DOCK_MARKER_IDS = {PORT_A_ID, PORT_B_ID}
    
    # 마커 ID → 이름 매핑
    PORT_MAP = {
        0: "PORT_A",
        1: "PORT_B",
        2: "PORT_C",
        3: "PORT_D",
        4: "PORT_E",
    }
    
    # ======================== 도킹 파라미터 ========================
    DOCK_SIZE_MIN = 180
    DOCK_SIZE_MAX = 350
    DOCK_SIZE_TARGET = 250
    DOCK_CENTER_TOLERANCE = 0.10


# ============================================================================
# 파일 경로 설정
# ============================================================================

class Path:
    """파일 경로"""
    RAW_MAP = "/home/kim1/save/map"
    MERGED_MAP = "/home/kim1/save/renewed_map"
    COLLISION = "/home/kim1/save/collision"
    QR_DB = "/home/kim1/nav2_maps/qr_positions.yaml"
    YOLO_MODEL = "/home/kim1/model/best.pt"
    AI_DETECTIONS = "/home/kim1/save/ai_detections"


# ============================================================================
# 시스템 설정
# ============================================================================

class Setting:
    """시스템 설정값"""
    # 타임아웃
    NETWORK_INTERVAL = 5.0
    PING_TIMEOUT = 1.0
    PING_COUNT = 4
    PLC_TIMEOUT = 60.0
    CAMERA_TIMEOUT = 5.0
    MAP_INTERVAL = 1.0
    MQTT_KEEPALIVE = 60
    
    # 파일
    MAX_FILES = 10
    MAX_SIZE = 50 * 1024 * 1024
    MAX_PHOTOS = 100
    ALLOWED_EXT = {'png', 'pgm', 'yaml', 'yml', 'jpg', 'jpeg'}
    
    # 맵
    CYCLE_COUNT = 8
    MAP_RESOLUTION = 0.05
    CANVAS_MARGIN = 200
    PIXEL_FREE = 0
    PIXEL_OCCUPIED = 100
    PIXEL_UNKNOWN = 205
    OCCUPIED_THRESH = 0.65
    FREE_THRESH = 0.196
    
    # 배터리 (V)
    BATTERY_LOW = 11.5
    BATTERY_CRITICAL = 11.0


# ============================================================================
# 하위 호환성 별칭 (기존 코드 지원)
# ============================================================================

# 네트워크
Net = type('Net', (), {
    'SERVER': NET.SERVER_IP,
    'ROBOT': NET.ROBOT_IP,
    'MCU': NET.MCU_IP,
    'PLC': NET.PLC_IP,
})

NetworkDevices = Net

# 포트
Port = type('Port', (), {
    'MQTT': NET.MQTT_PORT,
    'UPLOAD': NET.MAP_UPLOAD_PORT,
    'INFLUXDB': NET.INFLUXDB_PORT,
    'GRAFANA': NET.GRAFANA_PORT,
    'CAMERA': NET.STREAMING_PORT,
    'COLLISION': NET.COLLISION_PORT,
})

Ports = type('Ports', (), {
    'UPLOAD_SERVER': NET.MAP_UPLOAD_PORT,
})

# 경로
Paths = type('Paths', (), {
    'RAW_MAP_FOLDER': Path.RAW_MAP,
    'MERGED_MAP_FOLDER': Path.MERGED_MAP,
    'COLLISION_PHOTO_FOLDER': Path.COLLISION,
    'QR_DATABASE': Path.QR_DB,
})

# 타임아웃
Timeouts = type('Timeouts', (), {
    'NETWORK_CHECK_INTERVAL': Setting.NETWORK_INTERVAL,
    'PING_TIMEOUT': Setting.PING_TIMEOUT,
    'PING_COUNT': Setting.PING_COUNT,
    'PLC_MQTT_TIMEOUT': Setting.PLC_TIMEOUT,
    'CAMERA_STREAM_TIMEOUT': Setting.CAMERA_TIMEOUT,
    'MAP_BUILD_INTERVAL': Setting.MAP_INTERVAL,
})

# 파일 설정
FileSettings = type('FileSettings', (), {
    'MAX_FILES_PER_TYPE': Setting.MAX_FILES,
    'MAX_CONTENT_SIZE': Setting.MAX_SIZE,
    'ALLOWED_EXTENSIONS': Setting.ALLOWED_EXT,
    'MAX_COLLISION_PHOTOS': Setting.MAX_PHOTOS,
})

# MQTT 설정
MQTTSettings = type('MQTTSettings', (), {
    'BROKER_HOST': 'localhost',
    'BROKER_PORT': NET.MQTT_PORT,
    'QOS': 0,
    'KEEPALIVE': Setting.MQTT_KEEPALIVE,
})

# 토픽 하위호환 (Topic 클래스)
Topic = type('Topic', (), {
    'ROS_MCU_SENSORS': ROS.MQTT_MCU_SENSORS,
    'ROS_PLC_LOCATION': ROS.MQTT_PLC_LOCATION,
    'ROS_PLC_GOAL': ROS.MQTT_PLC_GOAL,
    'MAP_CYCLE': ROS.MAP_SAVER_CYCLE_LOCAL,
    'COLLISION_READY': ROS.COLLISION_PHOTO_READY,
    'COLLISION_SAVED': ROS.COLLISION_PHOTO_SAVED,
    'SLAM_MODE': ROS.SLAM_MODE,
    'ROBOT_MODE': ROS.ROBOT_MODE,
    'NAV_STATUS': ROS.NAV2_STATUS,
    'NAV_ARRIVED': ROS.NAV2_ARRIVED,
    'MAP': ROS.MAP,
    'MAP_READY': ROS.NAV2_MAP_READY,
    'QR_DETECTED': ROS.QR_DETECTED,
    'BATTERY': ROS.BATTERY_VOLTAGE,
    'AI_ARUCO': ROS.AI_ARUCO,
    'AI_OBSTACLE': ROS.AI_OBSTACLE,
    'MQTT_MCU': MQTT.SUB_MCU_SENSORS,
    'MQTT_PLC_LOC': MQTT.SUB_PLC_LOCATION,
    'MQTT_PLC_GOAL': MQTT.SUB_PLC_GOAL,
    'MQTT_PLC_PORT': '/plc/port_status',
    'MQTT_MAP_CYCLE': MQTT.MAP_CYCLE_COMPLETE,
    'MQTT_COLLISION': MQTT.COLLISION_PHOTO,
    'MQTT_SLAM_MODE': MQTT.SLAM_MODE,
    'MQTT_NAV_STATUS': MQTT.NAV_STATUS,
    'MQTT_NAV_RESULT': MQTT.NAV_RESULT,
    'MQTT_NAV_ARRIVED': MQTT.NAV_ARRIVED,
    'MQTT_ARUCO': MQTT.AI_ARUCO,
    'MQTT_OBSTACLE': MQTT.AI_OBSTACLE,
    'MQTT_NAV_CMD': MQTT.NAVIGATE_TO_POSE,
    'MQTT_QR': MQTT.QR_DETECTED,
    'MQTT_BATTERY': MQTT.BATTERY_STATUS,
    'MQTT_NETWORK': MQTT.NETWORK,
})


# ============================================================================
# 유틸리티 함수
# ============================================================================

def get_camera_url(robot_ip=None):
    """카메라 스트리밍 URL"""
    return f"http://{robot_ip or NET.ROBOT_IP}:{NET.STREAMING_PORT}/image.jpg"


def get_collision_url(robot_ip=None, filename=""):
    """충돌 사진 URL"""
    base = f"http://{robot_ip or NET.ROBOT_IP}:{NET.COLLISION_PORT}/photos"
    return f"{base}/{filename}" if filename else base


def print_topic_summary():
    """토픽 요약 출력 (디버깅용)"""
    print("=" * 60)
    print("ROS2 Topics (from robot):")
    for k, v in vars(ROS).items():
        if not k.startswith('_') and isinstance(v, str):
            print(f"  {k:30} = {v}")
    
    print("\n" + "=" * 60)
    print("MQTT Topics:")
    print("  Subscribe:")
    for topic in MQTT.SUBSCRIBE_LIST:
        print(f"    {topic}")
    print("  Publish:")
    for k, v in vars(MQTT).items():
        if not k.startswith('_') and not k.startswith('SUB') and isinstance(v, str):
            print(f"    {k:25} = {v}")


if __name__ == '__main__':
    print_topic_summary()
    print("\n" + "=" * 60)
    print(f"네트워크: SERVER={NET.SERVER_IP}, ROBOT={NET.ROBOT_IP}")
    print(f"경로: MAP={Path.RAW_MAP}")
    print(f"설정: CYCLE={Setting.CYCLE_COUNT}")
