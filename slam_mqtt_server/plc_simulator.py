#!/usr/bin/env python3
"""
PLC 시뮬레이터 - 3분마다 PORT A/B 신호를 번갈아 발행
테스트용: 실제 PLC가 없을 때 사용

사용법:
    ros2 run slam_mqtt_server plc_simulator
    
    또는 직접 실행:
    python3 plc_simulator.py
"""

import json
import time
import signal
import sys
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("❌ paho-mqtt 설치 필요: pip install paho-mqtt")
    sys.exit(1)


class PLCSimulator:
    """PLC 시뮬레이터 - 주기적으로 위치 명령 발행"""
    
    def __init__(self):
        # 설정
        self.mqtt_host = "192.168.0.3"  # MQTT 브로커 (서버)
        self.mqtt_port = 1883
        self.interval = 180  # 3분 (초)
        
        # 상태
        self.current_port = "A"  # A → B → A 순환
        self.running = True
        self.connected = False
        self.cycle_count = 0
        
        # 토픽
        self.topic_location = "/plc/location"
        self.topic_goal = "plc/goal"
        self.topic_port_status = "/plc/port_status"
        
        # MQTT 클라이언트
        self.client = mqtt.Client(client_id="plc_simulator")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        
        # 시그널 핸들러
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, sig, frame):
        print("\n🛑 종료 중...")
        self.running = False
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"✅ MQTT 브로커 연결됨: {self.mqtt_host}:{self.mqtt_port}")
        else:
            print(f"❌ MQTT 연결 실패: code={rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        print(f"⚠️ MQTT 연결 끊김: code={rc}")
    
    def connect(self):
        """MQTT 브로커 연결"""
        try:
            print(f"🔌 MQTT 브로커 연결 중: {self.mqtt_host}:{self.mqtt_port}")
            self.client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.client.loop_start()
            
            # 연결 대기
            timeout = 5
            while not self.connected and timeout > 0:
                time.sleep(0.5)
                timeout -= 0.5
            
            return self.connected
        except Exception as e:
            print(f"❌ 연결 오류: {e}")
            return False
    
    def send_location(self, port: str):
        """위치 명령 발행 (간단한 문자열)"""
        if not self.connected:
            print("⚠️ MQTT 연결 안됨")
            return False
        
        self.client.publish(self.topic_location, port, qos=1)
        return True
    
    def send_location_json(self, port: str):
        """위치 명령 발행 (JSON 형식)"""
        if not self.connected:
            return False
        
        payload = json.dumps({"port": port, "timestamp": time.time()})
        self.client.publish(self.topic_location, payload, qos=1)
        return True
    
    def send_goal(self, x: float, y: float, theta: float = 0.0):
        """좌표 명령 발행"""
        if not self.connected:
            return False
        
        payload = json.dumps({"x": x, "y": y, "theta": theta})
        self.client.publish(self.topic_goal, payload, qos=1)
        return True
    
    def send_port_status(self, port_a: int, port_b: int):
        """포트 상태 발행 (0=비어있음, 1=물건있음)"""
        if not self.connected:
            return False
        
        payload = json.dumps({"A": port_a, "B": port_b})
        self.client.publish(self.topic_port_status, payload, qos=1)
        return True
    
    def run(self):
        """메인 루프 - 3분마다 A/B 번갈아 신호 발행"""
        if not self.connect():
            print("❌ MQTT 연결 실패. 종료합니다.")
            return
        
        print("")
        print("=" * 60)
        print("🏭 PLC 시뮬레이터 시작")
        print(f"   - 주기: {self.interval}초 ({self.interval // 60}분)")
        print(f"   - 토픽: {self.topic_location}")
        print("   - 패턴: A → B → A → B ...")
        print("=" * 60)
        print("")
        
        # 초기 포트 상태 전송
        self.send_port_status(1, 0)  # A에 물건 있음
        
        while self.running:
            # 현재 시간
            now = datetime.now().strftime("%H:%M:%S")
            
            # 위치 명령 발행
            if self.send_location(self.current_port):
                self.cycle_count += 1
                print(f"[{now}] 📤 PLC 신호 발행: PORT {self.current_port} (#{self.cycle_count})")
                
                # 포트 상태도 함께 업데이트
                if self.current_port == "A":
                    self.send_port_status(1, 0)
                else:
                    self.send_port_status(0, 1)
            
            # 다음 포트로 전환
            self.current_port = "B" if self.current_port == "A" else "A"
            
            # 대기 (1초 단위로 체크하여 빠른 종료 가능)
            print(f"   ⏳ 다음 신호까지 {self.interval}초 대기 (Ctrl+C로 종료)")
            for _ in range(self.interval):
                if not self.running:
                    break
                time.sleep(1)
        
        # 정리
        self.client.loop_stop()
        self.client.disconnect()
        print(f"🏁 PLC 시뮬레이터 종료 (총 {self.cycle_count}회 발행)")


def main():
    """ROS2 노드 없이 독립 실행"""
    simulator = PLCSimulator()
    simulator.run()


def main_ros():
    """ROS2 노드로 실행 (ros2 run 용)"""
    import rclpy
    from rclpy.node import Node
    
    class PLCSimulatorNode(Node):
        def __init__(self):
            super().__init__('plc_simulator')
            self.simulator = PLCSimulator()
            
            # 파라미터
            self.declare_parameter('interval', 180)
            self.declare_parameter('mqtt_host', '192.168.0.3')
            
            self.simulator.interval = self.get_parameter('interval').value
            self.simulator.mqtt_host = self.get_parameter('mqtt_host').value
            
            self.get_logger().info(f"PLC 시뮬레이터 시작 (주기: {self.simulator.interval}초)")
            
            # 별도 스레드에서 실행
            import threading
            self.thread = threading.Thread(target=self.simulator.run, daemon=True)
            self.thread.start()
    
    rclpy.init()
    node = PLCSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.simulator.running = False
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
