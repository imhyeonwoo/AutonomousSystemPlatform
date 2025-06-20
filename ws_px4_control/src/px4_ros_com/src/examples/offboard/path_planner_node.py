#!/usr/bin/env python3
# path_planner_node.py – CSV (data/offboard_waypoints.csv) 로드 버전

import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

ENU = Tuple[float, float, float]

class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.path_pub_   = self.create_publisher(Path,        'planned_path',     qos)
        self.marker_pub_ = self.create_publisher(MarkerArray, 'waypoint_markers', qos)

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wp_csv',   '')                # 파라미터로 CSV 경로 덮어쓰기 가능
        self.frame_id_ = self.get_parameter('frame_id').value

        csv_path = self.get_parameter('wp_csv').value or (
            get_package_share_directory('px4_ros_com') + '/data/offboard_waypoints.csv'
        )
        self.enu_wps_ = self.load_csv(csv_path)
        self.get_logger().info(f'로드된 웨이포인트: {len(self.enu_wps_)}  (from {csv_path})')

        self.timer_ = self.create_timer(0.5, self.timer_cb)

    # ────────── CSV → list[ENU] ──────────
    @staticmethod
    def load_csv(path: str) -> List[ENU]:
        wps: List[ENU] = []
        with open(path, newline='') as f:
            reader = csv.reader(f)
            next(reader, None)        # header skip
            for row in reader:
                if len(row) < 3:
                    continue
                wps.append(tuple(map(float, row[:3])))  # (E,N,U)
        return wps

    # ────────── 주기 콜백 ──────────
    def timer_cb(self):
        now = self.get_clock().now().to_msg()

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id_
        path_msg.header.stamp    = now

        marker_arr = MarkerArray()

        for idx, (e, n, u) in enumerate(self.enu_wps_):
            # Path
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id_
            pose.header.stamp    = now
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = e, n, u
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            # Marker (sphere)
            sphere = Marker()
            sphere.header = pose.header
            sphere.ns, sphere.id = 'wp', idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5
            sphere.color.r = sphere.color.b = 1.0
            sphere.color.a = 1.0
            sphere.pose = pose.pose
            marker_arr.markers.append(sphere)

            # 텍스트
            text = Marker()
            text.header = pose.header
            text.ns, text.id = 'wp_text', idx + len(self.enu_wps_)
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.7
            text.color.r = text.color.b = 1.0
            text.color.a = 1.0
            text.pose.position.x, text.pose.position.y, text.pose.position.z = e, n, u + 1.0
            text.pose.orientation.w = 1.0
            text.text = f'waypoint {idx}'
            marker_arr.markers.append(text)

        # Line strip
        line = Marker()
        line.header.frame_id = self.frame_id_
        line.header.stamp    = now
        line.ns, line.id = 'path_line', 999
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.15
        line.color.r = line.color.b = 1.0
        line.color.a = 1.0
        for e, n, u in self.enu_wps_:
            pt = PoseStamped().pose.position.__class__()  # geometry_msgs/Point
            pt.x, pt.y, pt.z = e, n, u
            line.points.append(pt)
        marker_arr.markers.append(line)

        self.path_pub_.publish(path_msg)
        self.marker_pub_.publish(marker_arr)


def main():
    rclpy.init()
    rclpy.spin(PathPlannerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
