#!/usr/bin/env python3
# publish_map_info.py  –  Fix: plane lays flat in RViz
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray


# ───── util: RPY → quaternion (returns x,y,z,w) ─────
def quat_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class MapInfoPublisher(Node):

    def __init__(self):
        super().__init__('publish_map_info')

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_ground_truth', qos)

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        # name,  (x,y,z, roll,pitch,yaw)
        self.markers: List[Tuple[str, Tuple[float, ...]]] = [
            ('aruco_marker_5',  (-97.3988342285, 68.4287948608, 3.8531584740,  2.7093537970, 1.5707900273, -3.1408861898)),
            ('aruco_marker_4_1',(-72.5414657593, 74.8781433105, 23.4892425537, 0.3495391902, 1.4420900290,  0.7867001886)),
            ('aruco_marker_3',  (-61.8448905945, 80.1436080933,  7.8816661835,-2.9840699856, 1.2723399928, -2.5439699729)),
            ('aruco_marker_1',  (-78.8555068970,109.8188781738,  3.8079309463, 2.7093537970, 1.5707900273, -3.1408861898)),
            ('aruco_marker_4_2',(-96.9238967896,104.2799987793,  8.5504398346,-2.9952591423, 1.2871588988, -2.5560711177)),
            ('aruco_marker_0',  (-105.072784424,100.772109985 , 23.5399646759, 0.3378409862, 1.4445500016,  0.7744779858)),
            ('aruco_marker_2_1',(-103.5649871836,99.7337265015, 19.8078727722, 1.4788500379, 0.2464029939,  3.1121099963)),
            ('aruco_marker_2_2',(-101.258003235 ,100.0        , 12.2202997208, 1.4610009702, 0.6129622287,  3.0646405630)),
            ('aruco_marker_2_3',(-71.1409912109, 74.0306396484, 20.1738681793, 1.5346099484, 0.1174869930,  3.0243400012)),
            ('aruco_marker_2_4',(-70.5141525269, 74.3680419922, 15.4188995361, 1.4769700260, 0.3216460094,  3.1065600054)),
        ]

        self.timer = self.create_timer(0.5, self.timer_cb)
        self.get_logger().info('publish_map_info started → /aruco_ground_truth')

    # --------------------------------------------------
    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        for idx, (name, pose) in enumerate(self.markers):
            x, y, z, roll, pitch, yaw = pose

            # Gazebo 모델 ↔ Marker 차이를 보정 (plane lays flat)
            pitch_adj = pitch - math.pi/2.0
            yaw_adj = yaw - math.pi/2.0

            qx, qy, qz, qw = quat_from_euler(roll, pitch_adj, yaw_adj)

            # 본체 (CUBE)
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
            m.scale.x = m.scale.y = 2.0       # 20 cm 정사각형
            m.scale.z = 0.005                  # 5 mm 두께
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            out.markers.append(m)

            # 텍스트
            t = Marker()
            t.header = m.header
            t.ns = 'aruco_text'
            t.id = idx + 100
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = z + 1.5        # 위로 40 cm
            t.scale.z = 0.3
            t.color.r = 0.0
            t.color.g = 1.0
            t.color.b = 0.0
            t.color.a = 1.0
            t.text = name
            out.markers.append(t)

        self.marker_pub.publish(out)


def main():
    rclpy.init()
    node = MapInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
