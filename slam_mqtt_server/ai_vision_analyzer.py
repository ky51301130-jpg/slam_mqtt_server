#!/usr/bin/env python3
"""
AI 비전 분석 노드 - ArUco 마커 + YOLO 장애물 감지
서버(192.168.0.3)에서 실행
"""
from __future__ import annotations
import json
import os
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .config import ROS, MQTT

# Optional imports
try:
    import paho.mqtt.client as mqtt
    MQTT_OK = True
except ImportError:
    MQTT_OK = False

try:
    from ultralytics import YOLO
    YOLO_OK = True
except ImportError:
    YOLO_OK = False

try:
    import cv2.aruco as aruco
    ARUCO_OK = True
except ImportError:
    ARUCO_OK = False


@dataclass
class Detection:
    """감지 결과"""
    cls: str
    confidence: float
    bbox: list
    center: list


class AIVisionAnalyzer(Node):
    """ArUco + YOLO 비전 분석 노드"""
    
    # YOLO 클래스 → 장애물 타입 매핑
    CLASS_TO_OBSTACLE = {
        'PORT_A': 'obstacle_at_PORT_A',
        'PORT_B': 'obstacle_at_PORT_B',
    }
    
    def __init__(self):
        super().__init__('ai_vision_analyzer')
        
        # 파라미터
        p = self._declare_params()
        self.camera_url = f"http://{p['robot_ip']}:{p['robot_port']}/image.jpg"
        self.model_path = p['model_path']
        self.confidence = p['confidence']
        self.save_images = p['save_images']
        self.save_path = p['save_path']
        self.mode = p['mode']  # slam 또는 nav2
        
        # 상태
        self.model: Optional[YOLO] = None
        self.model_ok = False
        self.aruco_detector = None
        self.mqtt_client = None
        self.analysis_count = 0
        self.saved_aruco = set()
        self.saved_obstacles = set()
        
        # 초기화
        if ARUCO_OK:
            self._init_aruco()
        if YOLO_OK:
            self._load_model()
        if MQTT_OK:
            self._init_mqtt(p['mqtt_host'], p['mqtt_port'])
        
        # 저장 폴더 생성 (모드별 구분)
        if self.save_images:
            os.makedirs(os.path.join(self.save_path, 'aruco'), exist_ok=True)
            os.makedirs(os.path.join(self.save_path, 'obstacles'), exist_ok=True)
            os.makedirs(os.path.join(self.save_path, 'nav2_obstacles'), exist_ok=True)
            self.get_logger().info(f"📁 저장 폴더: {self.save_path} (모드: {self.mode})")
        
        # ROS2 퍼블리셔
        self.pub_aruco = self.create_publisher(String, ROS.AI_ARUCO, 10)
        self.pub_obstacle = self.create_publisher(String, ROS.AI_OBSTACLE, 10)
        
        # 타이머
        self.create_timer(1.0 / p['analysis_fps'], self._analyze)
        self.create_timer(10.0, self._log_status)
        
        self.get_logger().info(f"AI Vision 시작: ArUco={'✅' if self.aruco_detector else '❌'}, "
                               f"YOLO={'✅' if self.model_ok else '❌'}")
    
    def _declare_params(self) -> dict:
        """파라미터 선언"""
        params = {
            'robot_ip': '192.168.0.5', 'robot_port': 5200,
            'model_path': '/home/kim1/model/best.pt', 'confidence': 0.5,
            'analysis_fps': 1.0, 'mqtt_host': 'localhost', 'mqtt_port': 1883,
            'save_images': True, 'save_path': '/home/kim1/save/ai_detections',
            'mode': 'slam',  # 'slam' 또는 'nav2'
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
            params[name] = self.get_parameter(name).value
        return params
    
    def _init_aruco(self):
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        except Exception as e:
            self.get_logger().error(f"ArUco 초기화 실패: {e}")
    
    def _load_model(self):
        try:
            self.model = YOLO(self.model_path)
            self.model_ok = True
        except Exception as e:
            self.get_logger().error(f"YOLO 로드 실패: {e}")
    
    def _init_mqtt(self, host: str, port: int):
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect_async(host, port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
    
    def _analyze(self):
        try:
            image = self._fetch_image()
            if image is None:
                return
            self.analysis_count += 1
            
            if self.aruco_detector:
                markers = self._detect_aruco(image)
                if markers:
                    self._publish_aruco(markers)
                    self._save_aruco_image(image, markers)
            
            if self.model_ok:
                obstacles = self._detect_obstacles(image)
                if obstacles:
                    self._publish_obstacles(obstacles)
                    self._save_obstacle_image(image, obstacles)
        except Exception as e:
            self.get_logger().error(f"분석 오류: {e}")
    
    def _fetch_image(self) -> Optional[np.ndarray]:
        try:
            resp = requests.get(self.camera_url, timeout=2)
            if resp.status_code == 200:
                return cv2.imdecode(np.frombuffer(resp.content, np.uint8), cv2.IMREAD_COLOR)
        except requests.RequestException:
            pass
        return None
    
    def _detect_aruco(self, image: np.ndarray) -> list:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None:
            return []
        markers = []
        for i, mid in enumerate(ids):
            corner = corners[i][0]
            cx, cy = int(np.mean(corner[:, 0])), int(np.mean(corner[:, 1]))
            markers.append({'id': int(mid[0]), 'corners': corner.tolist(), 'center': [cx, cy]})
            self.get_logger().info(f"ArUco #{mid[0]} @ ({cx}, {cy})")
        return markers
    
    def _detect_obstacles(self, image: np.ndarray) -> list:
        """YOLO로 장애물 감지 (PORT_A/B → obstacle_at_PORT_A/B)"""
        try:
            results = self.model.predict(image, conf=self.confidence, verbose=False)
            obstacles = []
            for r in results:
                for box in r.boxes:
                    cls_name = self.model.names[int(box.cls[0])]
                    # 클래스 → 장애물 타입 매핑
                    obstacle_type = self.CLASS_TO_OBSTACLE.get(cls_name)
                    if obstacle_type:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        obstacles.append(Detection(
                            cls=obstacle_type, 
                            confidence=conf,
                            bbox=[x1, y1, x2, y2], 
                            center=[(x1+x2)//2, (y1+y2)//2]
                        ))
                        self.get_logger().warn(f"🚧 장애물 감지: {obstacle_type} (conf: {conf:.2f}) @ ({(x1+x2)//2}, {(y1+y2)//2})")
            return obstacles
        except Exception as e:
            self.get_logger().error(f"YOLO 오류: {e}")
            return []
    
    def _publish_aruco(self, markers: list):
        data = {'timestamp': datetime.now().isoformat(), 'markers': markers}
        self.pub_aruco.publish(String(data=json.dumps(data)))
        if self.mqtt_client:
            self.mqtt_client.publish(MQTT.AI_ARUCO, json.dumps({'markers': markers, 'timestamp': time.time()}), qos=1)
    
    def _publish_obstacles(self, obstacles: list):
        """장애물 정보 발행 (ROS2 토픽 + MQTT)"""
        obs_list = [{'type': o.cls, 'confidence': o.confidence, 'bbox': o.bbox, 'center': o.center} for o in obstacles]
        timestamp_str = datetime.now().isoformat()
        
        # ROS2 토픽 발행
        data = {'timestamp': timestamp_str, 'count': len(obstacles), 'obstacles': obs_list}
        self.pub_obstacle.publish(String(data=json.dumps(data)))
        
        # MQTT 발행
        if self.mqtt_client:
            mqtt_data = {
                'timestamp': time.time(),
                'timestamp_str': timestamp_str,
                'count': len(obstacles),
                'obstacles': obs_list,
                'summary': [f"{o.cls}({o.confidence:.1%})" for o in obstacles]
            }
            self.mqtt_client.publish(MQTT.AI_OBSTACLE, json.dumps(mqtt_data), qos=1)
        
        # 콘솔 로그
        self.get_logger().info(f"🚧 장애물 {len(obstacles)}개 발행: {', '.join([o.cls for o in obstacles])}")
    
    def _save_aruco_image(self, image: np.ndarray, markers: list):
        if not self.save_images:
            return
        for m in markers:
            mid = m['id']
            if mid in self.saved_aruco:
                continue
            self.saved_aruco.add(mid)
            img = image.copy()
            cv2.polylines(img, [np.array(m['corners'], dtype=np.int32)], True, (0, 255, 0), 3)
            cv2.putText(img, f"#{mid}", tuple(m['center']), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            cv2.imwrite(os.path.join(self.save_path, 'aruco', f"aruco_{mid}_{ts}.jpg"), img)
    
    def _save_obstacle_image(self, image: np.ndarray, obstacles: list):
        """장애물 발견 시 매번 사진 저장 및 로그 발행 (모드별 폴더 구분)"""
        if not self.save_images:
            return
        
        # 모드에 따라 저장 폴더 선택
        if self.mode == 'nav2':
            save_folder = 'nav2_obstacles'
            mode_emoji = '🧭'
        else:
            save_folder = 'obstacles'
            mode_emoji = '🗺️'
        
        for o in obstacles:
            img = image.copy()
            # 바운딩 박스와 레이블 그리기
            cv2.rectangle(img, (o.bbox[0], o.bbox[1]), (o.bbox[2], o.bbox[3]), (0, 0, 255), 3)
            label = f"{o.cls} ({o.confidence:.2f})"
            cv2.putText(img, label, (o.bbox[0], o.bbox[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            
            # 모드 표시 추가
            cv2.putText(img, f"Mode: {self.mode.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            
            # 타임스탬프로 파일명 생성 (매번 새로운 파일)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = f"{o.cls}_{ts}.jpg"
            filepath = os.path.join(self.save_path, save_folder, filename)
            cv2.imwrite(filepath, img)
            
            # 상세 로그 (모드 정보 포함)
            self.get_logger().info(f"{mode_emoji} [{self.mode.upper()}] 📸 장애물 사진 저장: {save_folder}/{filename}")
            self.get_logger().warn(f"⚠️ [{self.mode.upper()}] 장애물 발견! 타입: {o.cls}, 신뢰도: {o.confidence:.2%}, "
                                   f"위치: 중심({o.center[0]}, {o.center[1]}), "
                                   f"영역: ({o.bbox[0]}, {o.bbox[1]}) ~ ({o.bbox[2]}, {o.bbox[3]})")
            
            # 저장된 장애물 타입 기록 (통계용)
            self.saved_obstacles.add(o.cls)
    
    def _log_status(self):
        self.get_logger().info(f"분석: {self.analysis_count}회, ArUco: {len(self.saved_aruco)}, 장애물: {len(self.saved_obstacles)}")
    
    def destroy_node(self):
        if self.mqtt_client:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    if not YOLO_OK:
        print("❌ ultralytics 필요: pip install ultralytics")
        return
    node = AIVisionAnalyzer()
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
