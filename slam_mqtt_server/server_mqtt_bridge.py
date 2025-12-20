#!/usr/bin/env python3
"""
Server-side MQTT Bridge - PLC/MCU ↔ Server ROS2 ↔ Robot MQTT
서버(192.168.0.3)에서 실행
"""
from __future__ import annotations
import json
import os
import signal
import subprocess
import threading
import time
from typing import Optional

import paho.mqtt.client as mqtt
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from slam_mqtt_server.config import MQTT, Paths, ROS, MQTTSettings, NET


class ServerMQTTBridge(Node):
    """서버 MQTT 브릿지 - 양방향 통신"""
    
    ARUCO_TO_PORT = {0: "A", 1: "B"}
    
    def __init__(self):
        super().__init__('server_mqtt_bridge')
        
        # ArUco DB
        self.aruco_db_path = Paths.QR_DATABASE
        self.nav_log_path = os.path.join(os.path.dirname(Paths.QR_DATABASE), 'nav_results.log')
        self.aruco_positions = self._load_aruco_db()
        
        # 상태
        self.last_mode = ""
        self.plc_port_status = {"A": 0, "B": 0}
        self.rviz_process: Optional[subprocess.Popen] = None
        self.rviz_mode: Optional[str] = None
        self.mqtt_connected = False
        
        # ROS2 Subscribers
        subs = [
            (ROS.MAP_SAVER_CYCLE, String, self._on_cycle),
            (ROS.COLLISION_PHOTO_READY, String, self._on_collision),
            (ROS.ROBOT_MODE, String, self._on_robot_mode),
            (ROS.NAV2_STATUS, String, self._on_nav_status),
            (ROS.NAV2_ARRIVED, Bool, self._on_nav_arrived),
            (ROS.QR_DETECTED, String, self._on_qr),
            (ROS.BATTERY_VOLTAGE, Float32, self._on_battery),
            (ROS.MAP_READY, String, self._on_map_ready),
            (ROS.AI_ARUCO, String, self._on_aruco),
            (ROS.AI_OBSTACLE, String, self._on_obstacle),
        ]
        for topic, msg_type, cb in subs:
            self.create_subscription(msg_type, topic, cb, 10)
        
        # ROS2 Publisher
        self.slam_mode_pub = self.create_publisher(Bool, ROS.SLAM_MODE, 10)
        
        # MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._mqtt_on_connect
        self.mqtt_client.on_message = self._mqtt_on_message
        threading.Thread(target=self._mqtt_loop, daemon=True).start()
        
        # 3초 후 Nav2 모드용 맵 자동 전송 시도
        self.create_timer(3.0, self._auto_send_map_once)
        self._map_sent = False
        
        self.get_logger().info("Server MQTT Bridge 시작")
    
    # ==================== MQTT ====================
    
    def _mqtt_loop(self):
        while rclpy.ok():
            try:
                self.mqtt_client.connect(MQTTSettings.BROKER_HOST, MQTTSettings.BROKER_PORT, 60)
                self.mqtt_client.loop_forever()
            except Exception as e:
                self.mqtt_connected = False
                self.get_logger().warn(f"MQTT 재연결 중: {e}")
                time.sleep(10)
    
    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            for topic in [MQTT.SUB_PLC_LOCATION, MQTT.SUB_PLC_GOAL, MQTT.SUB_PLC_PORT_STATUS, MQTT.NAV_RESULT, "server/send_map"]:
                client.subscribe(topic, qos=1)
            self.get_logger().info("MQTT 연결됨")
    
    def _mqtt_on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            topic = msg.topic
            
            if topic == MQTT.SUB_PLC_LOCATION:
                self._handle_plc_location(payload)
            elif topic == MQTT.SUB_PLC_GOAL:
                self._handle_plc_goal(payload)
            elif topic == MQTT.SUB_PLC_PORT_STATUS:
                self._handle_plc_port_status(payload)
            elif topic == MQTT.NAV_RESULT:
                self._handle_nav_result(payload)
            elif topic == "server/send_map":
                # 수동 맵 전송 요청
                self._send_latest_map_to_robot()
        except Exception as e:
            self.get_logger().error(f"MQTT 메시지 오류: {e}")
    
    def _mqtt_pub(self, topic: str, payload: str, qos: int = 1):
        if self.mqtt_connected:
            self.mqtt_client.publish(topic, payload, qos=qos)
    
    # ==================== 맵 전송 (Nav2 모드) ====================
    
    def _auto_send_map_once(self):
        """시작 시 자동으로 최신 맵 전송 (1회)"""
        if self._map_sent:
            return
        self._map_sent = True
        self._send_latest_map_to_robot()
    
    def _send_latest_map_to_robot(self):
        """최신 맵을 로봇에게 MQTT로 전송 (Nav2 모드 활성화)"""
        import glob
        
        map_folder = "/home/kim1/save/renewed_map"
        yamls = sorted(glob.glob(os.path.join(map_folder, "nav2_final_map_*.yaml")), reverse=True)
        
        if not yamls:
            self.get_logger().warn("⚠️ 전송할 맵이 없습니다!")
            return
        
        yaml_path = yamls[0]
        yaml_name = os.path.basename(yaml_path)
        pgm_name = yaml_name.replace('.yaml', '.pgm')
        
        base_url = f"http://{NET.SERVER_IP}:{NET.MAP_UPLOAD_PORT}"
        
        notify = {
            "event": "map_ready",
            "pgm": pgm_name,
            "yaml": yaml_name,
            "download_url": {
                "base": base_url,
                "yaml": f"{base_url}/download/{yaml_name}",
                "pgm": f"{base_url}/download/{pgm_name}",
                "qr_positions": f"{base_url}/download/qr_positions.yaml"
            },
            "timestamp": time.time()
        }
        
        self._mqtt_pub("robot/map_ready", json.dumps(notify))
        self.get_logger().info(f"📤 Nav2 맵 전송: {yaml_name}")
        self.get_logger().info(f"   다운로드: {base_url}/download/{yaml_name}")
    
    # ==================== ArUco DB ====================
    
    def _load_aruco_db(self) -> dict:
        if not os.path.exists(self.aruco_db_path):
            os.makedirs(os.path.dirname(self.aruco_db_path), exist_ok=True)
            default = {
                "PORT_A": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}, "aruco_id": 0},
                "PORT_B": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}, "aruco_id": 1},
                "HOME": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"w": 1.0}, "aruco_id": 2}
            }
            with open(self.aruco_db_path, 'w') as f:
                yaml.dump(default, f)
            return default
        
        try:
            with open(self.aruco_db_path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"ArUco DB 로드 실패: {e}")
            return {}
    
    # ==================== PLC 핸들러 ====================
    
    def _handle_plc_location(self, payload: str):
        try:
            if payload.startswith('{'):
                data = json.loads(payload)
                if "A" in data or "B" in data:
                    self._handle_plc_port_status(payload)
                    return
                aruco_id = data.get("aruco_id")
                port = data.get("port")
            else:
                aruco_id, port = None, payload.strip().upper()
            
            if aruco_id is not None:
                port = self.ARUCO_TO_PORT.get(aruco_id)
            
            if port in ["A", "B"]:
                self._send_aruco_goal(port)
        except Exception as e:
            self.get_logger().error(f"PLC location 오류: {e}")
    
    def _handle_plc_goal(self, payload: str):
        try:
            data = json.loads(payload)
            self._mqtt_pub(MQTT.NAVIGATE_TO_POSE, json.dumps({
                "type": "coordinate", "x": data.get("x", 0.0),
                "y": data.get("y", 0.0), "theta": data.get("theta", 0.0)
            }))
        except Exception as e:
            self.get_logger().error(f"PLC goal 오류: {e}")
    
    def _handle_plc_port_status(self, payload: str):
        try:
            data = json.loads(payload)
            self.plc_port_status["A"] = data.get("A", 0)
            self.plc_port_status["B"] = data.get("B", 0)
        except Exception as e:
            self.get_logger().error(f"PORT 상태 오류: {e}")
    
    def _handle_nav_result(self, payload: str):
        try:
            data = json.loads(payload)
            self.get_logger().info(f"Nav 결과: {data.get('result')}")
            os.makedirs(os.path.dirname(self.nav_log_path), exist_ok=True)
            with open(self.nav_log_path, 'a') as f:
                f.write(f"{time.time()} | {payload}\n")
        except Exception as e:
            self.get_logger().error(f"Nav 결과 오류: {e}")
    
    def _send_aruco_goal(self, port: str):
        port_name = f"PORT_{port}"
        if port_name not in self.aruco_positions:
            return
        goal = self.aruco_positions[port_name]
        self._mqtt_pub(MQTT.NAVIGATE_TO_POSE, json.dumps({
            "type": "aruco_port", "port": port, "location": port_name,
            "position": goal.get("position", {}), "orientation": goal.get("orientation", {})
        }))
    
    # ==================== ROS2 콜백 ====================
    
    def _on_cycle(self, msg: String):
        self._mqtt_pub(MQTT.MAP_CYCLE_COMPLETE, msg.data)
    
    def _on_collision(self, msg: String):
        self._mqtt_pub(MQTT.COLLISION_PHOTO, msg.data)
    
    def _on_robot_mode(self, msg: String):
        mode = msg.data.upper()
        if mode != self.last_mode:
            self.last_mode = mode
            self.slam_mode_pub.publish(Bool(data=(mode == "SLAM")))
            self._mqtt_pub(MQTT.SLAM_MODE, json.dumps({"mode": mode, "timestamp": time.time()}))
            self._launch_rviz(mode)
    
    def _on_nav_status(self, msg: String):
        self._mqtt_pub(MQTT.NAV_STATUS, msg.data)
        try:
            data = json.loads(msg.data)
            if data.get("status") in ["SUCCEEDED", "ABORTED", "CANCELED"]:
                self._mqtt_pub(MQTT.NAV_RESULT, json.dumps({
                    "result": data["status"], "message": data.get("message", "")
                }))
        except:
            pass
    
    def _on_nav_arrived(self, msg: Bool):
        if msg.data:
            self._mqtt_pub(MQTT.NAV_ARRIVED, json.dumps({"arrived": True, "timestamp": time.time()}))
    
    def _on_qr(self, msg: String):
        self._mqtt_pub(MQTT.QR_DETECTED, msg.data)
    
    def _on_battery(self, msg: Float32):
        self._mqtt_pub(MQTT.BATTERY_STATUS, json.dumps({"voltage": round(msg.data, 2)}))
    
    def _on_map_ready(self, msg: String):
        """맵 준비 완료 → 로봇에게 다운로드 URL 알림"""
        try:
            data = json.loads(msg.data)
            pgm_name = data.get("pgm", "")
            yaml_name = data.get("yaml", "")
            
            base_url = f"http://{NET.SERVER_IP}:{NET.MAP_UPLOAD_PORT}"
            
            notify = {
                "event": "map_ready",
                "pgm": pgm_name,
                "yaml": yaml_name,
                "download_url": {
                    "base": base_url,
                    "yaml": f"{base_url}/download/{yaml_name}",
                    "pgm": f"{base_url}/download/{pgm_name}",
                    "qr_positions": f"{base_url}/download/qr_positions.yaml"
                },
                "timestamp": time.time()
            }
            self._mqtt_pub("robot/map_ready", json.dumps(notify))
            self.get_logger().info(f"📤 맵 준비 알림 전송: {yaml_name}")
        except Exception as e:
            self.get_logger().error(f"맵 알림 오류: {e}")
    
    def _on_aruco(self, msg: String):
        try:
            data = json.loads(msg.data)
            for m in data.get('markers', []):
                aruco_id = m.get('id')
                port = self.ARUCO_TO_PORT.get(aruco_id)
                if port and self.plc_port_status.get(port, 0) == 1:
                    self._send_aruco_goal(port)
        except Exception as e:
            self.get_logger().error(f"ArUco 콜백 오류: {e}")
    
    def _on_obstacle(self, msg: String):
        pass  # Grafana에서 직접 MQTT 구독
    
    # ==================== RViz ====================
    
    def _launch_rviz(self, mode: str):
        if mode == "SLAM":
            self._start_rviz("slam")
        elif mode == "NAV2":
            self._start_rviz("nav2")
        else:
            self._stop_rviz()
    
    def _start_rviz(self, mode: str):
        if self.rviz_mode == mode and self.rviz_process and self.rviz_process.poll() is None:
            return
        self._stop_rviz()
        
        script = f"/home/kim1/ros2_ws/{mode}_rviz.sh"
        if not os.path.exists(script):
            return
        
        try:
            self.rviz_process = subprocess.Popen(
                ["gnome-terminal", "--", "bash", script],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setpgrp
            )
            self.rviz_mode = mode
        except Exception as e:
            self.get_logger().error(f"RViz 실행 실패: {e}")
    
    def _stop_rviz(self):
        if self.rviz_process and self.rviz_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
                self.rviz_process.wait(timeout=5)
            except:
                pass
            self.rviz_process = None
            self.rviz_mode = None


def main(args=None):
    rclpy.init(args=args)
    node = ServerMQTTBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_rviz()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
