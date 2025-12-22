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
