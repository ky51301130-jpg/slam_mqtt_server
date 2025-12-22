#!/usr/bin/env python3
"""
=============================================================================
                    RViz 웹 브릿지 서버 (Foxglove 연동)
=============================================================================
ROS2 토픽을 웹 브라우저에서 시각화할 수 있도록 rosbridge_server를 실행합니다.
Foxglove Studio (https://foxglove.dev)에서 접속하여 RViz처럼 사용 가능합니다.

사용법:
    ros2 launch slam_mqtt_server web_rviz.launch.py

외부 접속:
    Foxglove Studio → Open connection → ws://192.168.0.3:9090
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import MarkerArray
import json


class WebRvizBridge(Node):
    """웹 RViz 브릿지 노드 - Foxglove/rosbridge 상태 모니터링"""
    
    def __init__(self):
        super().__init__('web_rviz_bridge')
        
        # QoS 설정
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 토픽 상태 추적
        self.topic_status = {
            '/map': False,
            '/scan': False,
            '/odom': False,
            '/tf': False,
            '/plan': False,
            '/local_plan': False,
        }
        
        # 구독자 (상태 확인용)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_reliable)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_sensor)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_sensor)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        
        # 상태 발행
        self.pub_status = self.create_publisher(String, '/web_rviz/status', 10)
        
        # 상태 타이머 (5초마다)
        self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🌐 Web RViz Bridge 시작!')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('📡 외부 접속 방법:')
        self.get_logger().info('  1. Foxglove Studio 실행')
        self.get_logger().info('     https://foxglove.dev/studio')
        self.get_logger().info('')
        self.get_logger().info('  2. Open connection 클릭')
        self.get_logger().info('     → Rosbridge (ROS 1 & 2)')
        self.get_logger().info('     → WebSocket URL: ws://192.168.0.3:9090')
        self.get_logger().info('')
        self.get_logger().info('⚠️  ROS_DOMAIN_ID=5 설정 필수!')
        self.get_logger().info('')
        self.get_logger().info('  3. 패널 추가:')
        self.get_logger().info('     - 3D: /map, /scan, /odom, /tf')
        self.get_logger().info('     - Plot: /odom (속도 그래프)')
        self.get_logger().info('     - Image: /camera/image/compressed')
        self.get_logger().info('')
        self.get_logger().info('='*60)
    
    def map_cb(self, msg):
        self.topic_status['/map'] = True
    
    def scan_cb(self, msg):
        self.topic_status['/scan'] = True
    
    def odom_cb(self, msg):
        self.topic_status['/odom'] = True
    
    def plan_cb(self, msg):
        self.topic_status['/plan'] = True
    
    def publish_status(self):
        """토픽 상태 발행"""
        status = {
            'topics': self.topic_status,
            'websocket_port': 9090,
            'foxglove_url': 'https://foxglove.dev/studio'
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WebRvizBridge()
    
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
