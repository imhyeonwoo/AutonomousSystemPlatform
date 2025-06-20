#!/usr/bin/env python3
"""
CSV 파일(test2.csv)의 (x, y, z) 점들을 frame=map-ENU 그대로 nav_msgs/Path 로 퍼블리시
+ Gazebo(world) ↔ RViz(map) 원점을 일치시키는 static TF 송출
"""

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSReliabilityPolicy, QoSDurabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as NavPath
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


class PathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("path_planner")

        # ── ① world → map 정체(static) TF ------------------------------------------------
        self._static_tf_pub = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world"   # Gazebo root
        tf.child_frame_id  = "map"     # RViz Fixed Frame
        tf.transform.rotation.w = 1.0  # (0,0,0,1)
        self._static_tf_pub.sendTransform(tf)

        # ── ② CSV 경로 & 토픽 파라미터 ----------------------------------------------------
        pkg_share = Path(get_package_share_directory("px4_ros_com"))
        default_csv = pkg_share / "test2.csv"
        self.declare_parameter("csv_path",  str(default_csv))
        self.declare_parameter("path_topic", "uav_planned_path")

        csv_path  = Path(self.get_parameter("csv_path").value)
        topic     = self.get_parameter("path_topic").value
        if not csv_path.exists():
            raise FileNotFoundError(csv_path)

        # ── ③ QoS (latched) --------------------------------------------------------------
        qos = QoSProfile(depth=1,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self._path_pub = self.create_publisher(NavPath, topic, qos)

        # ── ④ Path 메시지 만들기 ----------------------------------------------------------
        self._path = self._load_csv(csv_path)
        self.create_timer(1.0, self._timer_cb)          # 1 Hz 재송출
        self.get_logger().info(f"{len(self._path.poses)} WP → '{topic}' 퍼블리시")

    # ----------------------------------------------------------------------------
    def _load_csv(self, p: Path) -> NavPath:
        msg = NavPath(); msg.header.frame_id = "map"
        with p.open() as f:
            reader = csv.reader(f)
            for row in reader:
                x, y, z = map(float, row[:3])
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
        return msg

    def _timer_cb(self):
        now = self.get_clock().now().to_msg()
        self._path.header.stamp = now
        for p in self._path.poses:
            p.header.stamp = now
        self._path_pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPlanner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
