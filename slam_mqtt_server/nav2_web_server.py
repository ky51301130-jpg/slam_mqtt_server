#!/usr/bin/env python3
"""
Nav2 Web Server - 웹 기반 SLAM/Nav2 콘솔
Based on PinkLab's pinky_pro: https://github.com/pinklab-art/pinky_pro

브라우저에서 맵 시각화, Goal 설정, SLAM 저장/리셋 가능
"""
import threading
import time
import math
import os

from flask import Flask, jsonify, request, send_from_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from tf2_ros import Buffer, TransformListener

# SLAM Toolbox services (optional)
try:
    from slam_toolbox.srv import SaveMap, Reset
    SLAM_TOOLBOX_OK = True
except ImportError:
    SLAM_TOOLBOX_OK = False

from std_msgs.msg import String

############################################################
# Flask 설정
############################################################
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(os.path.dirname(BASE_DIR), 'scripts', 'web')

app = Flask(
    __name__,
    static_folder=STATIC_DIR,
    static_url_path=""
)

ros_node = None


def quat_to_yaw(q):
    """Quaternion → yaw"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Nav2WebBridge(Node):
    """Nav2/SLAM 웹 브릿지 노드"""
    
    def __init__(self):
        super().__init__("nav2_web_bridge")
        
        self.declare_parameter("ip", "0.0.0.0")
        self.declare_parameter("port", 8080)
        
        # ROS 데이터
        self.map_msg = None
        self.path_msg = None
        self.local_costmap_msg = None
        self.global_costmap_msg = None
        self.tf_pose = None
        
        self.lock = threading.Lock()
        
        # Map (TRANSIENT_LOCAL QoS)
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "map", self.map_callback, map_qos)
        
        # Path
        self.create_subscription(Path, "plan", self.path_callback, 10)
        
        # Costmaps
        self.local_costmap_seen = False
        self.global_costmap_seen = False
        self.create_subscription(Costmap, "local_costmap/costmap", self.local_costmap_callback, 10)
        self.create_subscription(Costmap, "local_costmap/costmap_raw", self.local_costmap_callback, 10)
        self.create_subscription(Costmap, "global_costmap/costmap", self.global_costmap_callback, 10)
        self.create_subscription(Costmap, "global_costmap/costmap_raw", self.global_costmap_callback, 10)
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.1, self.update_pose_from_tf)
        
        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # SLAM Toolbox 서비스 클라이언트 (optional)
        if SLAM_TOOLBOX_OK:
            self.save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
            self.reset_client = self.create_client(Reset, "/slam_toolbox/reset")
        
        self.get_logger().info("Nav2 Web Bridge 시작")
    
    def map_callback(self, msg):
        with self.lock:
            self.map_msg = msg
    
    def path_callback(self, msg):
        with self.lock:
            self.path_msg = msg
    
    def local_costmap_callback(self, msg):
        with self.lock:
            self.local_costmap_msg = msg
        if not self.local_costmap_seen:
            self.local_costmap_seen = True
            self.get_logger().info(f"Local costmap 수신: {msg.metadata.size_x}x{msg.metadata.size_y}")
    
    def global_costmap_callback(self, msg):
        with self.lock:
            self.global_costmap_msg = msg
        if not self.global_costmap_seen:
            self.global_costmap_seen = True
            self.get_logger().info(f"Global costmap 수신: {msg.metadata.size_x}x{msg.metadata.size_y}")
    
    def update_pose_from_tf(self):
        """map → base_link TF에서 pose 추출"""
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", Time())
            t = trans.transform
            x, y = t.translation.x, t.translation.y
            yaw = quat_to_yaw(t.rotation)
            with self.lock:
                self.tf_pose = (x, y, yaw)
        except Exception:
            pass
    
    def get_state_snapshot(self):
        """현재 상태 JSON 반환"""
        with self.lock:
            map_msg = self.map_msg
            path_msg = self.path_msg
            local_costmap_msg = self.local_costmap_msg
            global_costmap_msg = self.global_costmap_msg
            tf_pose = self.tf_pose
        
        # Map
        map_json = None
        if map_msg:
            info = map_msg.info
            map_json = {
                "width": info.width,
                "height": info.height,
                "resolution": info.resolution,
                "origin": {
                    "x": info.origin.position.x,
                    "y": info.origin.position.y,
                    "yaw": quat_to_yaw(info.origin.orientation)
                },
                "data": list(map_msg.data),
            }
        
        # Pose
        pose_json = None
        if tf_pose:
            pose_json = {"x": tf_pose[0], "y": tf_pose[1], "yaw": tf_pose[2]}
        
        # Path
        path_json = []
        if path_msg:
            path_json = [{"x": ps.pose.position.x, "y": ps.pose.position.y} for ps in path_msg.poses]
        
        # Costmaps
        local_costmap_json = self._costmap_to_json(local_costmap_msg)
        global_costmap_json = self._costmap_to_json(global_costmap_msg)
        
        return {
            "map": map_json,
            "pose": pose_json,
            "path": path_json,
            "local_costmap": local_costmap_json,
            "global_costmap": global_costmap_json,
        }
    
    def _costmap_to_json(self, msg):
        if not msg or not msg.data:
            return None
        meta = msg.metadata
        return {
            "width": meta.size_x,
            "height": meta.size_y,
            "resolution": meta.resolution,
            "origin": {
                "x": meta.origin.position.x,
                "y": meta.origin.position.y,
                "yaw": quat_to_yaw(meta.origin.orientation),
            },
            "data": list(msg.data),
        }
    
    def send_goal(self, x, y, yaw):
        """Nav2 Goal 전송"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("navigate_to_pose 서버 없음")
            return False
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f"[WEB] Goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        self.nav_client.send_goal_async(goal)
        return True
    
    def slam_reset(self) -> bool:
        """SLAM 리셋"""
        if not SLAM_TOOLBOX_OK:
            return False
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/slam_toolbox/reset 서비스 없음")
            return False
        
        req = Reset.Request()
        self.reset_client.call_async(req)
        self.get_logger().info("[WEB] SLAM 리셋 요청")
        return True
    
    def slam_save_map(self, name: str) -> bool:
        """SLAM 맵 저장"""
        if not SLAM_TOOLBOX_OK:
            return False
        if not self.save_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/slam_toolbox/save_map 서비스 없음")
            return False
        
        req = SaveMap.Request()
        req.name = String(data=name)
        self.save_map_client.call_async(req)
        self.get_logger().info(f"[WEB] 맵 저장 요청: {name}")
        return True


############################################################
# Flask 라우트
############################################################

@app.route("/")
def serve_index():
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/pinklab_logo.png")
def serve_logo():
    return send_from_directory(STATIC_DIR, "pinklab_logo.png")


@app.route("/api/state")
def api_state():
    global ros_node
    if ros_node is None:
        return jsonify({"error": "ROS not ready"}), 500
    return jsonify(ros_node.get_state_snapshot())


@app.route("/api/goal", methods=["POST"])
def api_goal():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500
    
    data = request.get_json()
    x = float(data["x"])
    y = float(data["y"])
    yaw = float(data.get("yaw", 0.0))
    
    ok = ros_node.send_goal(x, y, yaw)
    return jsonify({"success": ok})


@app.route("/api/slam/reset", methods=["POST"])
def api_slam_reset():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500
    
    ok = ros_node.slam_reset()
    return jsonify({"success": ok})


@app.route("/api/slam/save_map", methods=["POST"])
def api_slam_save_map():
    global ros_node
    if ros_node is None:
        return jsonify({"success": False, "msg": "ROS not ready"}), 500
    
    data = request.get_json() or {}
    name = data.get("name", "").strip()
    if not name:
        name = time.strftime("pinky_map_%Y%m%d_%H%M%S")
    
    ok = ros_node.slam_save_map(name)
    return jsonify({"success": ok, "name": name})


def ros_spin_thread():
    try:
        rclpy.spin(ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    ros_node = Nav2WebBridge()
    
    ip = ros_node.get_parameter("ip").value
    port = ros_node.get_parameter("port").value
    
    # ROS 스레드 시작
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()
    
    time.sleep(1.0)
    
    print(f"🌐 Web Server: http://{ip}:{port}")
    print("   브라우저에서 접속하여 SLAM/Nav2 제어 가능")
    
    app.run(host=ip, port=int(port), debug=False)


if __name__ == "__main__":
    main()
