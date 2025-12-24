#!/usr/bin/env python3
"""
통합 서버 노드 - 맵 업로드, 충돌 사진, 네트워크 모니터링
서버(192.168.0.3)에서 실행
"""
from __future__ import annotations
import json
import logging
import os
import re
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from flask import Flask, request, jsonify
from werkzeug.utils import secure_filename

try:
    import paho.mqtt.client as mqtt
    MQTT_OK = True
except ImportError:
    MQTT_OK = False

from slam_mqtt_server.config import (
    ROS, MQTT as MQTT_TOPICS, NET,
    Paths, FileSettings, Ports, NetworkDevices, Timeouts, MQTTSettings
)

# Flask App
app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = Paths.RAW_MAP_FOLDER
app.config['MAX_CONTENT_LENGTH'] = FileSettings.MAX_CONTENT_SIZE

_ros_node = None


def set_ros_node(node):
    global _ros_node
    _ros_node = node


@app.route('/upload', methods=['POST'])
def upload_file():
    """맵 파일 업로드 (pgm, yaml, png 모두 지원)"""
    if 'file' not in request.files:
        return jsonify({'error': 'No file'}), 400
    
    file = request.files['file']
    if not file.filename:
        return jsonify({'error': 'No filename'}), 400
    
    ext = file.filename.rsplit('.', 1)[-1].lower()
    if ext not in FileSettings.ALLOWED_EXTENSIONS:
        return jsonify({'error': 'Invalid extension'}), 400
    
    filename = secure_filename(file.filename)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    new_name = f"map_{ts}.{ext}"
    path = os.path.join(app.config['UPLOAD_FOLDER'], new_name)
    file.save(path)
    
    # pgm 파일일 때만 사이클 트리거
    if ext == 'pgm' and _ros_node:
        _ros_node.trigger_map_cycle(new_name)
    
    return jsonify({'message': 'Uploaded', 'filename': new_name, 'type': ext}), 200


@app.route('/upload_map', methods=['POST'])
def upload_map_files():
    """맵 파일 세트 업로드 (pgm + yaml + png 동시)"""
    uploaded = []
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    for key in ['pgm', 'yaml', 'png', 'file']:
        if key in request.files:
            file = request.files[key]
            if file and file.filename:
                ext = file.filename.rsplit('.', 1)[-1].lower()
                if ext in FileSettings.ALLOWED_EXTENSIONS:
                    new_name = f"map_{ts}.{ext}"
                    path = os.path.join(app.config['UPLOAD_FOLDER'], new_name)
                    file.save(path)
                    uploaded.append({'type': ext, 'filename': new_name})
    
    # pgm이 업로드되었으면 사이클 트리거
    pgm_file = next((f for f in uploaded if f['type'] == 'pgm'), None)
    if pgm_file and _ros_node:
        _ros_node.trigger_map_cycle(pgm_file['filename'])
    
    if uploaded:
        return jsonify({'message': 'Uploaded', 'files': uploaded}), 200
    return jsonify({'error': 'No valid files'}), 400


@app.route('/health')
def health():
    return jsonify({'status': 'ok'}), 200


@app.route('/status')
def server_status():
    """서버 전체 상태 API"""
    import glob
    
    # 맵 정보
    map_folder = "/home/kim1/save/renewed_map"
    raw_folder = "/home/kim1/save/map"
    today = datetime.now().strftime('%Y%m%d')
    
    nav2_maps = sorted(glob.glob(os.path.join(map_folder, "nav2_final_map_*.yaml")), reverse=True)
    today_raw_maps = glob.glob(os.path.join(raw_folder, f"map_{today}*.pgm"))
    
    latest_map = None
    if nav2_maps:
        latest = nav2_maps[0]
        import yaml
        with open(latest, 'r') as f:
            cfg = yaml.safe_load(f)
        latest_map = {
            'name': os.path.basename(latest),
            'ports': cfg.get('ports', {}),
            'resolution': cfg.get('resolution', 0.05)
        }
    
    # PLC/MCU 상태
    plc_online = False
    plc_port_a = 'offline'
    plc_port_b = 'offline'
    if _ros_node:
        now = time.time()
        plc_online = _ros_node.plc_last_seen and (now - _ros_node.plc_last_seen < 30)
        plc_port_a = _ros_node.plc_port_a
        plc_port_b = _ros_node.plc_port_b
    
    # qr_positions 로드
    qr_positions = {}
    qr_path = "/home/kim1/nav2_maps/qr_positions.yaml"
    if os.path.exists(qr_path):
        import yaml
        with open(qr_path, 'r') as f:
            qr_positions = yaml.safe_load(f) or {}
    
    return jsonify({
        'server': 'online',
        'timestamp': datetime.now().isoformat(),
        'maps': {
            'nav2_count': len(nav2_maps),
            'today_raw_count': len(today_raw_maps),
            'latest': latest_map
        },
        'ports': qr_positions,
        'plc': {
            'online': plc_online,
            'port_a': plc_port_a,
            'port_b': plc_port_b
        },
        'links': {
            'web_ui': 'http://192.168.0.3:8080',
            'grafana': 'http://192.168.0.3:3000',
            'upload': 'http://192.168.0.3:5100'
        }
    }), 200


@app.route('/')
def dashboard():
    """서버 대시보드 UI"""
    html = '''<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>PINKY 서버 대시보드</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; min-height: 100vh; }
        .header { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 20px; text-align: center; }
        .header h1 { font-size: 2em; margin-bottom: 5px; }
        .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .card { background: #16213e; border-radius: 12px; padding: 20px; box-shadow: 0 4px 15px rgba(0,0,0,0.3); }
        .card h3 { color: #667eea; margin-bottom: 15px; display: flex; align-items: center; gap: 10px; }
        .status-dot { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }
        .online { background: #00ff88; box-shadow: 0 0 10px #00ff88; }
        .offline { background: #ff4757; box-shadow: 0 0 10px #ff4757; }
        .info-row { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #2a3f5f; }
        .info-row:last-child { border-bottom: none; }
        .info-label { color: #888; }
        .info-value { color: #fff; font-weight: 500; }
        .port-card { background: #1e3a5f; padding: 15px; border-radius: 8px; margin: 10px 0; }
        .port-name { font-size: 1.2em; color: #00d4ff; margin-bottom: 8px; }
        .port-coords { font-family: monospace; color: #aaa; font-size: 0.9em; }
        .link-btn { display: inline-block; background: #667eea; color: white; padding: 10px 20px; 
                    border-radius: 8px; text-decoration: none; margin: 5px; transition: transform 0.2s; }
        .link-btn:hover { transform: scale(1.05); background: #764ba2; }
        .links { text-align: center; margin-top: 20px; }
        .refresh-btn { position: fixed; bottom: 20px; right: 20px; background: #667eea; color: white;
                       border: none; padding: 15px 25px; border-radius: 30px; cursor: pointer; font-size: 1em; }
        .timestamp { text-align: center; color: #666; margin-top: 20px; font-size: 0.9em; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 PINKY 서버 대시보드</h1>
        <p>실시간 시스템 상태</p>
    </div>
    <div class="container">
        <div class="grid">
            <div class="card">
                <h3>📡 서버 상태</h3>
                <div id="server-status">로딩 중...</div>
            </div>
            <div class="card">
                <h3>🗺️ 맵 정보</h3>
                <div id="map-status">로딩 중...</div>
            </div>
            <div class="card">
                <h3>📍 포트 좌표</h3>
                <div id="port-status">로딩 중...</div>
            </div>
            <div class="card">
                <h3>🔌 PLC 상태</h3>
                <div id="plc-status">로딩 중...</div>
            </div>
        </div>
        <div class="links">
            <a href="http://192.168.0.3:8080" target="_blank" class="link-btn">🌐 Web UI</a>
            <a href="http://192.168.0.3:3000" target="_blank" class="link-btn">📊 Grafana</a>
            <a href="/list_maps" target="_blank" class="link-btn">📂 맵 목록</a>
            <a href="/api/detection_images" target="_blank" class="link-btn">🔍 AI 감지</a>
        </div>
        <div class="timestamp" id="timestamp"></div>
    </div>
    <button class="refresh-btn" onclick="loadStatus()">🔄 새로고침</button>
    <script>
        async function loadStatus() {
            try {
                const res = await fetch('/status');
                const data = await res.json();
                
                // 서버 상태
                document.getElementById('server-status').innerHTML = `
                    <div class="info-row">
                        <span class="info-label">상태</span>
                        <span class="info-value"><span class="status-dot online"></span> 온라인</span>
                    </div>
                `;
                
                // 맵 상태
                const maps = data.maps;
                let mapHtml = `
                    <div class="info-row">
                        <span class="info-label">Nav2 맵</span>
                        <span class="info-value">${maps.nav2_count}개</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">오늘 사이클</span>
                        <span class="info-value">${maps.today_raw_count}개</span>
                    </div>
                `;
                if (maps.latest) {
                    mapHtml += `
                        <div class="info-row">
                            <span class="info-label">최신 맵</span>
                            <span class="info-value" style="font-size:0.8em">${maps.latest.name}</span>
                        </div>
                    `;
                }
                document.getElementById('map-status').innerHTML = mapHtml;
                
                // 포트 좌표
                const ports = data.ports;
                let portHtml = '';
                for (const [name, info] of Object.entries(ports)) {
                    const pos = info.position || {};
                    const ori = info.orientation || {};
                    portHtml += `
                        <div class="port-card">
                            <div class="port-name">${name} (ID: ${info.aruco_id})</div>
                            <div class="port-coords">
                                위치: (${pos.x?.toFixed(3) || 0}, ${pos.y?.toFixed(3) || 0})<br>
                                방향: w=${ori.w?.toFixed(3) || 1}, z=${ori.z?.toFixed(3) || 0}
                            </div>
                        </div>
                    `;
                }
                document.getElementById('port-status').innerHTML = portHtml || '<p style="color:#888">포트 정보 없음</p>';
                
                // PLC 상태
                const plc = data.plc;
                document.getElementById('plc-status').innerHTML = `
                    <div class="info-row">
                        <span class="info-label">연결</span>
                        <span class="info-value">
                            <span class="status-dot ${plc.online ? 'online' : 'offline'}"></span>
                            ${plc.online ? '온라인' : '오프라인'}
                        </span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">PORT A</span>
                        <span class="info-value">${plc.port_a}</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">PORT B</span>
                        <span class="info-value">${plc.port_b}</span>
                    </div>
                `;
                
                document.getElementById('timestamp').textContent = '마지막 업데이트: ' + data.timestamp;
            } catch (e) {
                console.error(e);
            }
        }
        loadStatus();
        setInterval(loadStatus, 5000);
    </script>
</body>
</html>'''
    return html


@app.route('/download/<path:filename>')
def download_file(filename):
    """맵 파일 다운로드 (pgm, yaml, qr_positions.yaml)"""
    from flask import send_from_directory
    
    # 맵 파일 디렉토리
    map_folder = "/home/kim1/save/renewed_map"
    qr_folder = "/home/kim1/nav2_maps"
    
    # qr_positions.yaml은 별도 폴더에서
    if filename == 'qr_positions.yaml':
        return send_from_directory(qr_folder, filename)
    
    # 맵 파일 (pgm, yaml)
    filepath = os.path.join(map_folder, filename)
    if os.path.exists(filepath):
        return send_from_directory(map_folder, filename)
    
    return jsonify({'error': 'File not found'}), 404


@app.route('/list_maps')
def list_maps():
    """사용 가능한 맵 목록"""
    import glob
    map_folder = "/home/kim1/save/renewed_map"
    maps = []
    for yaml_file in glob.glob(os.path.join(map_folder, "nav2_final_map_*.yaml")):
        name = os.path.basename(yaml_file)
        pgm_name = name.replace('.yaml', '.pgm')
        if os.path.exists(os.path.join(map_folder, pgm_name)):
            maps.append({'yaml': name, 'pgm': pgm_name})
    return jsonify({'maps': sorted(maps, key=lambda x: x['yaml'], reverse=True)}), 200


# ============================================================================
# Grafana 대시보드용 API 엔드포인트들
# ============================================================================

@app.route('/api/latest_aruco')
def latest_aruco():
    """가장 최근 ArUco 마커 인식 이미지"""
    import glob
    from flask import send_from_directory
    aruco_folder = "/home/kim1/save/ai_detections/aruco"
    files = sorted(glob.glob(os.path.join(aruco_folder, "*.jpg")), key=os.path.getmtime, reverse=True)
    if files:
        filename = os.path.basename(files[0])
        return send_from_directory(aruco_folder, filename)
    return jsonify({'error': 'No ArUco images found'}), 404


@app.route('/api/latest_obstacle')
def latest_obstacle():
    """가장 최근 YOLO 장애물 인식 이미지"""
    import glob
    from flask import send_from_directory
    # nav2_obstacles 먼저, 없으면 obstacles
    folders = ["/home/kim1/save/ai_detections/nav2_obstacles", "/home/kim1/save/ai_detections/obstacles"]
    for folder in folders:
        files = sorted(glob.glob(os.path.join(folder, "*.jpg")), key=os.path.getmtime, reverse=True)
        if files:
            filename = os.path.basename(files[0])
            return send_from_directory(folder, filename)
    return jsonify({'error': 'No obstacle images found'}), 404


@app.route('/api/detection_images')
def detection_images():
    """최근 인식 이미지 정보 (JSON)"""
    import glob
    aruco_folder = "/home/kim1/save/ai_detections/aruco"
    obstacle_folders = ["/home/kim1/save/ai_detections/nav2_obstacles", "/home/kim1/save/ai_detections/obstacles"]
    
    result = {'aruco': None, 'obstacle': None}
    
    # ArUco
    aruco_files = sorted(glob.glob(os.path.join(aruco_folder, "*.jpg")), key=os.path.getmtime, reverse=True)
    if aruco_files:
        f = aruco_files[0]
        result['aruco'] = {
            'filename': os.path.basename(f),
            'url': f"http://192.168.0.3:5100/api/latest_aruco",
            'timestamp': os.path.getmtime(f),
            'count': len(aruco_files)
        }
    
    # Obstacle
    for folder in obstacle_folders:
        files = sorted(glob.glob(os.path.join(folder, "*.jpg")), key=os.path.getmtime, reverse=True)
        if files:
            f = files[0]
            result['obstacle'] = {
                'filename': os.path.basename(f),
                'url': f"http://192.168.0.3:5100/api/latest_obstacle",
                'timestamp': os.path.getmtime(f),
                'count': len(files)
            }
            break
    
    return jsonify(result), 200


@app.route('/api/plc_status')
def plc_status():
    """PLC 포트 상태"""
    if _ros_node:
        now = time.time()
        plc_online = _ros_node.plc_last_seen and (now - _ros_node.plc_last_seen < 30)
        return jsonify({
            'online': plc_online,
            'last_seen': _ros_node.plc_last_seen,
            'port_a': _ros_node.plc_port_a,
            'port_b': _ros_node.plc_port_b
        }), 200
    return jsonify({'online': False}), 200


@app.route('/api/dashboard_links')
def dashboard_links():
    """대시보드 링크 모음"""
    return jsonify({
        'web_rviz': 'http://192.168.0.3:8080',
        'grafana': 'http://192.168.0.3:3000',
        'upload_server': 'http://192.168.0.3:5100'
    }), 200


class UnifiedServerNode(Node):
    """통합 서버 노드"""
    
    def __init__(self):
        super().__init__('unified_server')
        
        # 디렉토리 생성
        os.makedirs(Paths.RAW_MAP_FOLDER, exist_ok=True)
        os.makedirs(Paths.COLLISION_PHOTO_FOLDER, exist_ok=True)
        
        # 상태
        self.slam_mode = False
        self.collision_count = 0
        self.plc_last_seen = 0.0
        self.mcu_last_seen = 0.0
        self.plc_port_a = 'offline'
        self.plc_port_b = 'offline'
        
        # ROS2 Publishers/Subscribers
        self.map_cycle_pub = self.create_publisher(String, ROS.MAP_SAVER_CYCLE, 10)
        self.collision_saved_pub = self.create_publisher(String, ROS.COLLISION_PHOTO_SAVED, 10)
        self.create_subscription(Bool, ROS.SLAM_MODE, self._on_slam_mode, 10)
        # 로봇에서 오는 충돌 사진 토픽 구독
        self.create_subscription(String, ROS.COLLISION_PHOTO_READY, self._on_collision_photo, 10)
        
        # MQTT
        if MQTT_OK:
            self._init_mqtt()
        
        # 타이머
        self.create_timer(Timeouts.NETWORK_CHECK_INTERVAL, self._check_network)
        
        # Flask 스레드
        set_ros_node(self)
        threading.Thread(target=self._run_flask, daemon=True).start()
        
        self.get_logger().info(f"Unified Server 시작 | Upload: http://{NetworkDevices.SERVER}:{Ports.UPLOAD_SERVER}")
    
    def _run_flask(self):
        logging.getLogger('werkzeug').setLevel(logging.WARNING)
        app.run(host='0.0.0.0', port=Ports.UPLOAD_SERVER, debug=False, use_reloader=False)
    
    def trigger_map_cycle(self, filename: str):
        pgm_count = len([f for f in os.listdir(Paths.RAW_MAP_FOLDER) if f.endswith('.pgm')])
        msg = String()
        msg.data = json.dumps({'cycle_number': pgm_count, 'filename': filename, 'timestamp': time.time()})
        self.map_cycle_pub.publish(msg)
        self.get_logger().info(f"맵 사이클: {filename}")
    
    def _init_mqtt(self):
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self._mqtt_on_connect
            self.mqtt_client.on_message = self._mqtt_on_message
            self.mqtt_client.connect_async(MQTTSettings.BROKER_HOST, MQTTSettings.BROKER_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 초기화 실패: {e}")
    
    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe([('/plc/#', 0), ('/mcu/#', 0), (MQTT_TOPICS.COLLISION_PHOTO, 0)])
    
    def _mqtt_on_message(self, client, userdata, msg):
        if msg.topic.startswith('/plc/'):
            self.plc_last_seen = time.time()
            # PLC 포트 상태 파싱
            try:
                payload = json.loads(msg.payload.decode())
                if 'port_a' in payload:
                    self.plc_port_a = payload['port_a']
                if 'port_b' in payload:
                    self.plc_port_b = payload['port_b']
                if msg.topic == '/plc/port_a':
                    self.plc_port_a = payload.get('status', 'unknown')
                elif msg.topic == '/plc/port_b':
                    self.plc_port_b = payload.get('status', 'unknown')
            except:
                pass
        elif msg.topic.startswith('/mcu/'):
            self.mcu_last_seen = time.time()
        elif msg.topic == MQTT_TOPICS.COLLISION_PHOTO and self.slam_mode:
            self._save_collision_photo(msg)
    
    def _save_collision_photo(self, msg):
        try:
            import requests
            payload_str = msg.payload.decode()
            if not payload_str or payload_str.strip() == '':
                return  # 빈 메시지 무시
            
            payload = json.loads(payload_str)
            url = payload.get('url', '')
            if not url:
                return
            
            resp = requests.get(url, timeout=10)
            if resp.status_code != 200:
                return
            
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"collision_{ts}.jpg"
            path = os.path.join(Paths.COLLISION_PHOTO_FOLDER, filename)
            
            with open(path, 'wb') as f:
                f.write(resp.content)
            
            self.collision_count += 1
            self.collision_saved_pub.publish(String(data=json.dumps({
                'filename': filename, 'path': path, 'count': self.collision_count
            })))
            
            # 오래된 파일 정리
            self._cleanup_photos()
        except json.JSONDecodeError:
            pass  # JSON 파싱 실패 무시
        except Exception as e:
            self.get_logger().error(f"충돌 사진 저장 실패: {e}")
    
    def _cleanup_photos(self):
        try:
            photos = sorted([f for f in os.listdir(Paths.COLLISION_PHOTO_FOLDER) 
                           if f.startswith('collision_') and f.endswith('.jpg')])
            while len(photos) > FileSettings.MAX_COLLISION_PHOTOS:
                os.remove(os.path.join(Paths.COLLISION_PHOTO_FOLDER, photos.pop(0)))
        except:
            pass
    
    def _check_network(self):
        devices = {'mcu': NetworkDevices.MCU, 'robot': NetworkDevices.ROBOT, 'plc': NetworkDevices.PLC}
        result = {}
        
        for name, ip in devices.items():
            if name == 'plc':
                status = 100 if self.plc_last_seen and time.time() - self.plc_last_seen < Timeouts.PLC_MQTT_TIMEOUT else 0
            elif name == 'mcu':
                status = 100 if self.mcu_last_seen and time.time() - self.mcu_last_seen < 10.0 else 0
            else:
                status = self._ping(ip)
            
            result[name] = status
            icon = "🟢" if status >= 90 else "🟡" if status >= 50 else "🔴"
            self.get_logger().debug(f"{icon} {name}: {status}%")
        
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.publish('network/connectivity', json.dumps({**result, 'timestamp': time.time()}))
    
    def _ping(self, ip: str) -> int:
        try:
            result = subprocess.run(['ping', '-c', '3', '-W', '1', ip], capture_output=True, text=True, timeout=5)
            match = re.search(r'(\d+)% packet loss', result.stdout)
            return 100 - int(match.group(1)) if match else 0
        except:
            return 0
    
    def _on_slam_mode(self, msg: Bool):
        self.slam_mode = msg.data
    
    def _on_collision_photo(self, msg: String):
        """ROS2 토픽으로 받은 충돌 사진 처리"""
        try:
            import requests
            if not msg.data or msg.data.strip() == '':
                return  # 빈 메시지 무시
            
            data = json.loads(msg.data)
            url = data.get('url', '')
            if not url:
                return  # URL 없으면 조용히 무시
            
            resp = requests.get(url, timeout=10)
            if resp.status_code != 200:
                self.get_logger().warn(f"충돌 사진 다운로드 실패: {resp.status_code}")
                return
            
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"collision_{ts}.jpg"
            path = os.path.join(Paths.COLLISION_PHOTO_FOLDER, filename)
            
            with open(path, 'wb') as f:
                f.write(resp.content)
            
            self.collision_count += 1
            self.get_logger().info(f"📸 충돌 사진 저장: {filename} (#{self.collision_count})")
            
            self.collision_saved_pub.publish(String(data=json.dumps({
                'filename': filename, 'path': path, 'count': self.collision_count
            })))
            self._cleanup_photos()
        except json.JSONDecodeError:
            pass  # JSON 파싱 실패 무시 (빈 메시지 등)
        except Exception as e:
            self.get_logger().error(f"충돌 사진 저장 실패: {e}")
    
    def destroy_node(self):
        if hasattr(self, 'mqtt_client'):
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    if not MQTT_OK:
        print("❌ paho-mqtt 필요: pip install paho-mqtt")
        return
    
    node = UnifiedServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
