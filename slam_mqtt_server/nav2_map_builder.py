#!/usr/bin/env python3
"""
Nav2 맵 빌더: ICP 기반 회전 정렬 + 과반수 투표
- 첫 번째 맵 기준 ICP 회전 정렬
- 과반수 이상 벽이면 벽으로 처리 (노이즈 제거)
"""
from __future__ import annotations
import glob
import json
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml as pyyaml
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


@dataclass
class MapData:
    """단일 맵 데이터 컨테이너"""
    img: np.ndarray
    origin_x: float = 0.0
    origin_y: float = 0.0
    yaw: float = 0.0
    resolution: float = 0.05
    initial_pose: dict = field(default_factory=dict)
    ports: dict = field(default_factory=dict)  # PORT 좌표 {name: {x, y, yaw}}


class Nav2MapBuilderWithAlignment(Node):
    """ICP 회전 정렬 + 과반수 투표 맵 빌더"""
    
    # 픽셀 및 임계값 상수
    PX_WALL, PX_FREE, PX_UNKNOWN = 0, 254, 205
    TH_WALL, TH_FREE = 50, 201
    OCC_FREE, OCC_OCCUPIED, OCC_UNKNOWN = 0, 100, -1
    
    # 설정
    SAVE_PATH = "/home/kim1/save/map"
    OUTPUT_PATH = "/home/kim1/save/renewed_map"
    MARGIN = 50
    OVERLAP_THRESH = 2.0  # ICP 겹침 판정 픽셀 거리
    
    def __init__(self):
        super().__init__('nav2_map_builder')
        
        os.makedirs(self.SAVE_PATH, exist_ok=True)
        os.makedirs(self.OUTPUT_PATH, exist_ok=True)
        
        self.cycle_maps: list[MapData] = []
        self.current_map: Optional[OccupancyGrid] = None
        self.publish_timer = None
        self._publish_count = 0
        self._origin_offset = np.array([0.0, 0.0])
        
        self._init_ros()
        self.get_logger().info('Nav2 Map Builder (ICP + 과반수 투표) 시작')
    
    def _init_ros(self):
        """ROS 퍼블리셔/서브스크라이버 초기화"""
        self.create_subscription(String, '/map_saver/cycle_complete', 
                                 self._on_cycle_complete, 10)
        # 로봇에서 오는 토픽도 구독
        self.create_subscription(String, '/ros/map_saver/cycle_complete', 
                                 self._on_cycle_complete, 10)
        self.map_ready_pub = self.create_publisher(String, '/nav2/map_ready', 10)
        
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.map_metadata_pub = self.create_publisher(MapMetaData, '/map_metadata', qos)

    # =========================================================================
    # 유틸리티 메서드
    # =========================================================================
    
    def _crop_to_content(self, m: np.ndarray) -> np.ndarray:
        """의미있는 영역만 크롭"""
        mask = m != self.PX_UNKNOWN
        if not mask.any():
            return m
        rows, cols = np.any(mask, axis=1), np.any(mask, axis=0)
        y1, y2 = np.where(rows)[0][[0, -1]]
        x1, x2 = np.where(cols)[0][[0, -1]]
        return m[y1:y2+1, x1:x2+1]
    
    def _get_wall_points(self, m: np.ndarray) -> np.ndarray:
        """벽 픽셀 좌표 (x, y)"""
        return np.argwhere(m == self.PX_WALL)[:, ::-1].astype(np.float32)
    
    def _get_wall_centroid(self, m: np.ndarray) -> tuple[float, float]:
        """벽 중심점 (x, y)"""
        pts = np.argwhere(m == self.PX_WALL)
        if len(pts) == 0:
            return m.shape[1] / 2, m.shape[0] / 2
        return float(np.mean(pts[:, 1])), float(np.mean(pts[:, 0]))
    
    def _rotate_image(self, img: np.ndarray, angle_deg: float, 
                      center: tuple = None, out_size: tuple = None) -> np.ndarray:
        """이미지 회전 (각도: 도)"""
        h, w = img.shape[:2]
        cx, cy = center or (w / 2, h / 2)
        
        if out_size is None:
            rad = np.radians(abs(angle_deg))
            cos_a, sin_a = np.cos(rad), np.sin(rad)
            out_w = int(w * cos_a + h * sin_a) + 10
            out_h = int(h * cos_a + w * sin_a) + 10
        else:
            out_w, out_h = out_size
        
        mat = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)
        mat[0, 2] += (out_w - w) / 2
        mat[1, 2] += (out_h - h) / 2
        
        return cv2.warpAffine(img, mat, (out_w, out_h), flags=cv2.INTER_NEAREST,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=self.PX_UNKNOWN)
    
    # =========================================================================
    # ICP 회전 정렬
    # =========================================================================

    def _compute_overlap(self, ref_centered: np.ndarray, tgt_centered: np.ndarray, 
                         angle_deg: int) -> float:
        """주어진 각도에서 겹침 비율 계산 (벡터화)"""
        rad = np.radians(angle_deg)
        cos_a, sin_a = np.cos(rad), np.sin(rad)
        rot = np.array([[cos_a, sin_a], [-sin_a, cos_a]])
        rotated = tgt_centered @ rot.T
        
        # KD-Tree 대신 브로드캐스팅 사용 (소규모 데이터에 적합)
        # 샘플링으로 속도 향상 (최대 500점)
        if len(rotated) > 500:
            idx = np.random.choice(len(rotated), 500, replace=False)
            sample = rotated[idx]
        else:
            sample = rotated
        
        dists = np.linalg.norm(ref_centered[None, :, :] - sample[:, None, :], axis=2)
        return np.sum(np.min(dists, axis=1) <= self.OVERLAP_THRESH) / len(sample)

    def _icp_find_rotation(self, ref_map: np.ndarray, tgt_map: np.ndarray) -> tuple[float, float]:
        """ICP 기반 최적 회전각 탐색"""
        ref_pts, tgt_pts = self._get_wall_points(ref_map), self._get_wall_points(tgt_map)
        if len(ref_pts) < 10 or len(tgt_pts) < 10:
            return 0.0, 0.0
        
        ref_centered = ref_pts - np.mean(ref_pts, axis=0)
        tgt_centered = tgt_pts - np.mean(tgt_pts, axis=0)
        
        np.random.seed(42)
        
        # 거친 탐색 (5도 단위)
        best_angle, best_overlap = 0, 0.0
        for angle in range(-180, 180, 5):
            overlap = self._compute_overlap(ref_centered, tgt_centered, angle)
            if overlap > best_overlap:
                best_overlap, best_angle = overlap, angle
        
        # 정밀 탐색 (1도 단위)
        for angle in range(best_angle - 5, best_angle + 6):
            overlap = self._compute_overlap(ref_centered, tgt_centered, angle)
            if overlap > best_overlap:
                best_overlap, best_angle = overlap, angle
        
        return float(best_angle), best_overlap
    
    # =========================================================================
    # Cycle 콜백 및 맵 로딩
    # =========================================================================
    
    def _on_cycle_complete(self, msg: String):
        """Cycle 완료 시 맵 로드 및 병합"""
        try:
            data = json.loads(msg.data)
            pgm_path = data.get('pgm_path', '')
            yaml_path = data.get('yaml_path', '')
            cycle_num = data.get('cycle_number', 0)
            
            if not (pgm_path and yaml_path and os.path.exists(pgm_path) and os.path.exists(yaml_path)):
                self.get_logger().info('Cycle 완료 신호 수신 (경로 없음), 디렉토리에서 검색')
                if len(self.cycle_maps) >= 8:
                    self._build_nav2_map()
                return
            
            map_data = self._load_map(pgm_path, yaml_path)
            if map_data:
                self.cycle_maps.append(map_data)
                self.get_logger().info(f'[Cycle {cycle_num}] 저장됨 ({len(self.cycle_maps)}/8)')
            
            if len(self.cycle_maps) >= 8:
                self.get_logger().info('=== 8 Cycles 완료! Nav2 맵 빌드 시작 ===')
                self._build_nav2_map()
                self.cycle_maps.clear()
        except Exception as e:
            self.get_logger().error(f'Cycle 처리 오류: {e}')
    
    def _load_map(self, pgm_path: str, yaml_path: str) -> Optional[MapData]:
        """단일 맵 로드 (ports 정보 포함)"""
        try:
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                return None
            
            with open(yaml_path, 'r') as f:
                cfg = pyyaml.safe_load(f)
            
            origin = cfg.get('origin', [0.0, 0.0, 0.0])
            ports = cfg.get('ports', {})  # PORT 좌표 로드
            
            return MapData(
                img=img,
                origin_x=origin[0],
                origin_y=origin[1],
                yaw=origin[2],
                resolution=cfg.get('resolution', 0.05),
                ports=ports
            )
        except Exception as e:
            self.get_logger().error(f'맵 로드 실패 {pgm_path}: {e}')
            return None
    
    def _load_maps_from_dir(self):
        """디렉토리에서 맵 파일 로드"""
        pgm_files = sorted(glob.glob(os.path.join(self.SAVE_PATH, 'map_*.pgm')))[:8]
        for idx, pgm in enumerate(pgm_files):
            yaml_path = pgm.replace('.pgm', '.yaml')
            if not os.path.exists(yaml_path):
                continue
            map_data = self._load_map(pgm, yaml_path)
            if map_data:
                # yaw가 0이면 8-사이클 기준 45도씩 자동 할당
                if map_data.yaw == 0.0 and len(pgm_files) == 8:
                    map_data.yaw = np.radians(idx * 45)
                self.cycle_maps.append(map_data)
                self.get_logger().info(f'로드: {os.path.basename(pgm)}')
    
    # =========================================================================
    # 맵 빌드 (핵심 로직)
    # =========================================================================
    
    def _build_nav2_map(self):
        """8개 맵을 ICP 정렬 + 과반수 투표로 병합"""
        try:
            if not self.cycle_maps:
                self._load_maps_from_dir()
            
            if not self.cycle_maps:
                self.get_logger().error('맵 데이터 없음!')
                return
            
            base = self.cycle_maps[0]
            resolution = base.resolution
            
            # ICP 정렬 및 병합 (ports 좌표 포함)
            final_map, origin_x, origin_y, final_ports = self._align_and_merge(resolution)
            
            # OccupancyGrid 변환 및 발행
            h, w = final_map.shape
            self.current_map = self._to_occupancy_grid(final_map, resolution, origin_x, origin_y, w, h)
            
            if self.publish_timer is None:
                self.publish_timer = self.create_timer(1.0, self._publish_map)
            
            # 파일 저장 (ports 포함)
            ts = self._save_files(final_map, resolution, origin_x, origin_y, final_ports)
            self.get_logger().info(f'✅ Nav2 맵 생성 완료: nav2_final_map_{ts}.pgm')
            
            self.map_ready_pub.publish(String(data=json.dumps({
                'pgm': f'nav2_final_map_{ts}.pgm', 'yaml': f'nav2_final_map_{ts}.yaml'
            })))
        except Exception as e:
            self.get_logger().error(f'맵 빌드 오류: {e}')
    
    def _align_and_merge(self, resolution: float) -> tuple[np.ndarray, float, float, dict]:
        """ICP 정렬 + 과반수 투표 병합 + PORT 좌표 변환"""
        num_maps = len(self.cycle_maps)
        majority = (num_maps // 2) + 1
        
        ref = self.cycle_maps[0]
        ref_img = ref.img
        ref_h, ref_w = ref_img.shape
        ref_cx, ref_cy = self._get_wall_centroid(ref_img)
        
        # 캔버스 크기 (margin 포함)
        canvas_h, canvas_w = ref_h + self.MARGIN * 2, ref_w + self.MARGIN * 2
        
        # ICP로 각 맵의 회전각 계산
        ref_cropped = self._crop_to_content(ref_img)
        rotations = [0.0]
        for i in range(1, num_maps):
            tgt_cropped = self._crop_to_content(self.cycle_maps[i].img)
            angle, score = self._icp_find_rotation(ref_cropped, tgt_cropped)
            rotations.append(angle)
            self.get_logger().info(f'[맵 {i+1}] ICP 회전: {angle:.0f}°, 겹침율: {score:.1%}')
        
        # 투표 배열
        wall_votes = np.zeros((canvas_h, canvas_w), dtype=np.int32)
        free_votes = np.zeros((canvas_h, canvas_w), dtype=np.int32)
        
        # PORT 좌표 수집 (모든 맵에서)
        all_ports = {}  # {port_name: [(x, y, yaw), ...]}
        
        for idx, (map_data, angle) in enumerate(zip(self.cycle_maps, rotations)):
            img = map_data.img
            h, w = img.shape
            cx, cy = self._get_wall_centroid(img)
            
            if angle != 0:
                # 회전 + 중심 정렬
                mat = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
                mat[0, 2] += (ref_cx - cx) + self.MARGIN
                mat[1, 2] += (ref_cy - cy) + self.MARGIN
                aligned = cv2.warpAffine(img, mat, (canvas_w, canvas_h),
                                          borderValue=self.PX_UNKNOWN, flags=cv2.INTER_NEAREST)
            else:
                aligned = np.full((canvas_h, canvas_w), self.PX_UNKNOWN, dtype=np.uint8)
                aligned[self.MARGIN:self.MARGIN+h, self.MARGIN:self.MARGIN+w] = img
            
            wall_votes[aligned == self.PX_WALL] += 1
            free_votes[aligned == self.PX_FREE] += 1
            
            # PORT 좌표 변환 (ICP 회전 + 중심점 이동 적용)
            if map_data.ports:
                # 현재 맵의 벽 중심점 (픽셀 좌표)
                curr_cx, curr_cy = self._get_wall_centroid(img)
                
                for port_name, port_data in map_data.ports.items():
                    px, py = port_data.get('x', 0), port_data.get('y', 0)
                    port_yaw = port_data.get('yaw', 0)
                    
                    # 1. 월드 좌표 → 픽셀 좌표 (현재 맵 기준)
                    px_pixel = (px - map_data.origin_x) / resolution
                    py_pixel = (py - map_data.origin_y) / resolution
                    
                    # 2. 중심점 기준으로 이동
                    dx = px_pixel - curr_cx
                    dy = py_pixel - curr_cy
                    
                    # 3. ICP 회전 적용
                    rad = np.radians(angle)
                    cos_a, sin_a = np.cos(rad), np.sin(rad)
                    rotated_dx = dx * cos_a - dy * sin_a
                    rotated_dy = dx * sin_a + dy * cos_a
                    
                    # 4. 기준 맵(ref) 중심점으로 이동 + MARGIN 오프셋
                    new_px_pixel = ref_cx + rotated_dx + self.MARGIN
                    new_py_pixel = ref_cy + rotated_dy + self.MARGIN
                    
                    # 5. 픽셀 → 월드 좌표 (최종 맵 기준, origin은 나중에 계산됨)
                    # 여기서는 ref 맵 origin 기준으로 저장
                    new_x = new_px_pixel * resolution + ref.origin_x - self.MARGIN * resolution
                    new_y = new_py_pixel * resolution + ref.origin_y - self.MARGIN * resolution
                    new_yaw = port_yaw + rad
                    
                    if port_name not in all_ports:
                        all_ports[port_name] = []
                    all_ports[port_name].append((new_x, new_y, new_yaw))
                    
                    if idx == 0:
                        self.get_logger().info(f'   [맵 {idx+1}] {port_name}: ({px:.2f},{py:.2f}) → ({new_x:.2f},{new_y:.2f})')
        
        # 과반수 투표
        final = np.full((canvas_h, canvas_w), self.PX_UNKNOWN, dtype=np.uint8)
        wall_mask = wall_votes >= majority
        free_mask = (free_votes > 0) & ~wall_mask
        final[wall_mask] = self.PX_WALL
        final[free_mask] = self.PX_FREE
        
        wall_px = np.sum(wall_mask)
        self.get_logger().info(f'과반수 벽: {wall_px}px (임계: {majority}/{num_maps})')
        
        # origin 계산
        origin_x = ref.origin_x - self.MARGIN * resolution
        origin_y = ref.origin_y - self.MARGIN * resolution
        
        # PORT 좌표 평균 (모든 맵에서 감지된 것들의 ICP 변환 후 평균)
        final_ports = {}
        for port_name, coords_list in all_ports.items():
            if coords_list:
                avg_x = np.mean([c[0] for c in coords_list])
                avg_y = np.mean([c[1] for c in coords_list])
                avg_yaw = np.mean([c[2] for c in coords_list])
                
                # yaw 정규화 (-pi ~ pi)
                while avg_yaw > np.pi:
                    avg_yaw -= 2 * np.pi
                while avg_yaw < -np.pi:
                    avg_yaw += 2 * np.pi
                
                final_ports[port_name] = {
                    'x': round(float(avg_x), 3),
                    'y': round(float(avg_y), 3),
                    'yaw': round(float(avg_yaw), 3)
                }
                self.get_logger().info(f'📍 {port_name} 최종: ({avg_x:.3f}, {avg_y:.3f}, yaw={avg_yaw:.3f}) [{len(coords_list)}개 평균]')
        
        return final, origin_x, origin_y, final_ports
    
    # =========================================================================
    # 파일 저장 및 OccupancyGrid 변환
    # =========================================================================
    
    def _save_files(self, final_map: np.ndarray, resolution: float, 
                    origin_x: float, origin_y: float, ports: dict = None) -> str:
        """최종 맵을 PGM/YAML로 저장 (ports 정보 포함)"""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        pgm = os.path.join(self.OUTPUT_PATH, f"nav2_final_map_{ts}.pgm")
        yaml_path = os.path.join(self.OUTPUT_PATH, f"nav2_final_map_{ts}.yaml")
        
        cv2.imwrite(pgm, final_map)
        
        cfg = {
            'image': os.path.basename(pgm),
            'resolution': float(resolution),
            'origin': [float(origin_x), float(origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        # PORT 좌표 추가
        if ports:
            cfg['ports'] = ports
        
        with open(yaml_path, 'w') as f:
            pyyaml.dump(cfg, f, default_flow_style=False)
        
        # qr_positions.yaml도 업데이트 (Nav2에서 사용)
        if ports:
            self._update_qr_positions(ports)
        
        return ts
    
    def _update_qr_positions(self, ports: dict):
        """qr_positions.yaml 파일 업데이트"""
        qr_path = "/home/kim1/nav2_maps/qr_positions.yaml"
        os.makedirs(os.path.dirname(qr_path), exist_ok=True)
        
        # ArUco ID 매핑
        aruco_ids = {'PORT_A': 0, 'PORT_B': 1, 'HOME': 2}
        
        qr_data = {}
        for port_name, port_info in ports.items():
            x = float(port_info.get('x', 0))
            y = float(port_info.get('y', 0))
            yaw = float(port_info.get('yaw', 0))
            
            # yaw → quaternion (z축 회전)
            qw = float(np.cos(yaw / 2))
            qz = float(np.sin(yaw / 2))
            
            qr_data[port_name] = {
                'aruco_id': aruco_ids.get(port_name, -1),
                'position': {'x': round(x, 4), 'y': round(y, 4), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': round(qz, 4), 'w': round(qw, 4)}
            }
        
        # HOME이 없으면 PORT_A와 동일하게
        if 'HOME' not in qr_data and 'PORT_A' in qr_data:
            import copy
            qr_data['HOME'] = copy.deepcopy(qr_data['PORT_A'])
            qr_data['HOME']['aruco_id'] = 2
        
        with open(qr_path, 'w') as f:
            pyyaml.dump(qr_data, f, default_flow_style=False)
        
        self.get_logger().info(f'✅ qr_positions.yaml 업데이트: {list(qr_data.keys())}')
    
    def _to_occupancy_grid(self, img: np.ndarray, resolution: float,
                           origin_x: float, origin_y: float, 
                           width: int, height: int) -> OccupancyGrid:
        """이미지를 OccupancyGrid로 변환"""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        
        # 픽셀 변환: 상하반전 + 값 변환
        flipped = np.flipud(img).flatten()
        data = np.full(len(flipped), self.OCC_UNKNOWN, dtype=np.int8)
        data[flipped == self.PX_WALL] = self.OCC_OCCUPIED
        data[flipped == self.PX_FREE] = self.OCC_FREE
        grid.data = data.tolist()
        
        return grid
    
    def _publish_map(self):
        """맵 주기적 발행 (1Hz)"""
        if self.current_map is None:
            return
        
        self.current_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.current_map)
        
        meta = MapMetaData()
        meta.map_load_time = self.current_map.header.stamp
        meta.resolution = self.current_map.info.resolution
        meta.width = self.current_map.info.width
        meta.height = self.current_map.info.height
        meta.origin = self.current_map.info.origin
        self.map_metadata_pub.publish(meta)
        
        self._publish_count += 1
        if self._publish_count % 10 == 0:
            w, h = self.current_map.info.width, self.current_map.info.height
            self.get_logger().info(f'맵 발행 중: {w}x{h} @ {self.current_map.info.resolution}m/px')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2MapBuilderWithAlignment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
