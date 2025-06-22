#!/usr/bin/env python3
# ws_px4_control/src/px4_ros_com/src/examples/map_visualization/publish_map_info.py
# Publishes ArUco ground-truth markers in RViz2, loading pose data from CSV.

import csv
import math
import pathlib
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray


# ───── util: RPY → quaternion (x, y, z, w) ─────
def quat_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def _find_default_csv(script_dir: pathlib.Path) -> pathlib.Path:
    """
    현재 스크립트 위치에서 위로 1~4단계 올라가며
    <candidate>/data/aruco_markers.csv 가 있으면 반환.
    """
    for up in range(5):
        candidate = script_dir
        for _ in range(up):
            candidate = candidate.parent
        csv_path = candidate / 'data' / 'aruco_markers.csv'
        if csv_path.exists():
            return csv_path
    # 못 찾으면 마지막 후보 경로 반환(없는 파일), 나중에 FileNotFoundError
    return script_dir.parent.parent / 'data' / 'aruco_markers.csv'


class MapInfoPublisher(Node):
    """CSV 기반 ArUco Marker Publisher"""

    def __init__(self):
        super().__init__('publish_map_info')

        # QoS: RViz 재연결 시 과거 Marker 유지(Transient Local)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_ground_truth', qos)

        # ───── 파라미터 선언 ─────
        script_dir = pathlib.Path(__file__).resolve().parent
        default_csv = _find_default_csv(script_dir)

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('csv_path', str(default_csv))
        self.declare_parameter('pitch_offset', -math.pi / 2)   # 세워주기용 +90°
        self.declare_parameter('yaw_offset',   -math.pi / 2)   # 가로→세로 맞추기용 +90°
        self.declare_parameter('marker_size',       1.0)       # 1 m × 1 m 평면
        self.declare_parameter('marker_thickness',  0.005)     # 5 mm 두께
        self.declare_parameter('text_height',       0.3)
        self.declare_parameter('text_z_offset',     1.5)       # 텍스트를 마커 위 1.5 m

        self.frame_id         = self.get_parameter('frame_id').value
        self.pitch_offset     = float(self.get_parameter('pitch_offset').value)
        self.yaw_offset       = float(self.get_parameter('yaw_offset').value)
        self.marker_size      = float(self.get_parameter('marker_size').value)
        self.marker_thickness = float(self.get_parameter('marker_thickness').value)
        self.text_height      = float(self.get_parameter('text_height').value)
        self.text_z_offset    = float(self.get_parameter('text_z_offset').value)

        # ───── CSV 로드 ─────
        csv_path = pathlib.Path(self.get_parameter('csv_path').value).expanduser()
        if not csv_path.exists():
            # 후보 경로들을 가시적으로 나열
            tried = "\n  - ".join(
                str(_find_default_csv(script_dir.parent / ('..' * i))) for i in range(5)
            )
            self.get_logger().error(
                f'CSV not found: {csv_path}\n'
                f'자동 탐색 시도 경로:\n  - {tried}\n'
                f'다른 위치라면 ‟--ros-args -p csv_path:=<절대경로>” 로 지정하세요.'
            )
            raise FileNotFoundError(csv_path)
        self.markers = self._load_csv(csv_path)

        # 2 Hz 주기로 MarkerArray 발행
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.get_logger().info(
            f'publish_map_info started → /aruco_ground_truth '
            f'({len(self.markers)} markers, csv="{csv_path}")'
        )

    # --------------------------------------------------
    def _load_csv(self, path: pathlib.Path) -> List[Tuple[str, Tuple[float, ...]]]:
        """CSV → [(name, (x, y, z, roll, pitch, yaw)), …]"""
        markers = []
        with path.open(newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                name  = row['name']
                x     = float(row['e'])   # ENU: East → x
                y     = float(row['n'])   #          North → y
                z     = float(row['u'])   #          Up   → z
                roll  = float(row['roll'])
                pitch = float(row['pitch'])
                yaw   = float(row['yaw'])
                markers.append((name, (x, y, z, roll, pitch, yaw)))
        return markers

    # --------------------------------------------------
    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        for idx, (name, pose) in enumerate(self.markers):
            x, y, z, roll, pitch, yaw = pose

            # Gazebo 모델 ↔ Marker.CUBE 차이 보정
            pitch_adj = pitch + self.pitch_offset
            yaw_adj   = yaw   + self.yaw_offset
            qx, qy, qz, qw = quat_from_euler(roll, pitch_adj, yaw_adj)

            # ───── 마커 평면(CUBE) ─────
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp    = now
            m.ns   = 'aruco'
            m.id   = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            m.scale.x = m.scale.y = self.marker_size
            m.scale.z = self.marker_thickness
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            out.markers.append(m)

            # ───── 텍스트 ─────
            t = Marker()
            t.header = m.header
            t.ns     = 'aruco_text'
            t.id     = idx + 100
            t.type   = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = z + self.text_z_offset
            t.scale.z = self.text_height
            t.color.r, t.color.g, t.color.b, t.color.a = 0.0, 1.0, 0.0, 1.0
            
            # ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
            # ★★        여기가 수정된 부분입니다        ★★
            # ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
            # 텍스트에 이름과 (E, N, U) 좌표를 함께 표시 (소수점 2자리까지)
            t.text = f"{name}\n({x:.2f}, {y:.2f}, {z:.2f})"
            out.markers.append(t)

        self.marker_pub.publish(out)


# ───── main ─────
def main():
    rclpy.init()
    node = MapInfoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()