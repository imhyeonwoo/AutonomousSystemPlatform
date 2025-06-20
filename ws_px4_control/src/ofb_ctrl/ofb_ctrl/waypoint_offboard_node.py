#!/usr/bin/env python3
"""
pose_control_offboard_node.py – PX4 오프보드, 동적 Pose 추종
  - /target_pose (geometry_msgs/msg/PoseStamped) 토픽을 구독하여
    map(ENU) 좌표계의 목표 위치/자세를 local(NED) 좌표계로 변환하여 추종함.
  - TF 트리 구조에 맞춰 drone_frame 파라미터 수정.
  - 사용 예:
    * ros2 run your_package_name pose_control_offboard_node
    * (Rviz2 연동 시) ros2 run your_package_name pose_control_offboard_node --remap /goal_pose:=/target_pose
"""
from __future__ import annotations
import math
import numpy as np
import rclpy
import tf2_ros
import tf_transformations as tft
from typing import List, Optional, Tuple

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy,
)
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)

# ───────── 상수 ─────────
MAP_FRAME   = "map"
LOCAL_FRAME = "local_ned"

TAKEOFF_ALT_ENU = 5.0
CMD_HZ    = 20.0

R_ENU2NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
R_NED2ENU = R_ENU2NED.T

# ────────────────────────────────────────────
class PoseControlNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_control_offboard_node")

        # ❶ 파라미터: 'map' 프레임에 직접 연결된 드론의 프레임 이름 (TF 트리 기반 수정)
        self.declare_parameter("drone_frame", "x500_gimbal_0")
        self.drone_frame = self.get_parameter("drone_frame").value
        self.get_logger().info(f"Using '{self.drone_frame}' as the drone's root frame in the map.")

        # QoS 프로파일
        cmd_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                durability=QoSDurabilityPolicy.VOLATILE)

        # 퍼블리셔
        self.pub_ctrl = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", cmd_qos)
        self.pub_sp   = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint",   cmd_qos)
        self.pub_cmd  = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command",       cmd_qos)

        # 서브스크라이버
        self.create_subscription(PoseStamped, "/target_pose", self._target_pose_cb, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._pos_cb, sensor_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._vs_cb,  sensor_qos)

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tfl    = tf2_ros.TransformListener(self.tf_buf, self)
        self.tf_br  = TransformBroadcaster(self)

        # 상태 변수
        self.state: str = "INIT"
        self.arm_timer_count: int = 0
        self.curr_ned: Optional[np.ndarray] = None
        self.curr_enu: Optional[np.ndarray] = None
        self.vs: Optional[VehicleStatus] = None
        self.home_enu: Optional[np.ndarray] = None
        self.home_set: bool = False
        self.target_pose_enu: Optional[PoseStamped] = None

        # 메인 루프 타이머
        self.create_timer(1.0 / CMD_HZ, self._loop_cmd)

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != MAP_FRAME:
            self.get_logger().warn(
                f"Received target_pose in '{msg.header.frame_id}' frame, but expected '{MAP_FRAME}' frame. Ignoring.",
                throttle_duration_sec=5
            )
            return
        self.target_pose_enu = msg

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.curr_ned = np.array([msg.x, msg.y, msg.z])
        if self.home_set:
            local_enu_offset = R_NED2ENU @ self.curr_ned
            self.curr_enu = self.home_enu + local_enu_offset

    def _vs_cb(self, msg: VehicleStatus) -> None:
        self.vs = msg

    def _ensure_home_and_tf(self) -> bool:
        if self.home_set:
            return True
        try:
            # ❷ TF 조회 수정: 'map'에서 'drone_frame'으로의 변환을 찾음
            tf = self.tf_buf.lookup_transform(
                target_frame=MAP_FRAME,
                source_frame=self.drone_frame, # 수정한 파라미터 사용
                time=rclpy.time.Time()
            )
            self.home_enu = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ])
            self.get_logger().info(f"Home position set at ENU {self.home_enu.round(2)}")

            # map(ENU) -> local_ned(NED) 변환 관계 설정
            T = np.eye(4)
            T[0:3, 0:3] = R_ENU2NED
            T[0:3, 3]   = -R_ENU2NED @ self.home_enu
            quat = tft.quaternion_from_matrix(T)

            tf_msg = TransformStamped()
            tf_msg.header.frame_id = MAP_FRAME
            tf_msg.child_frame_id = LOCAL_FRAME
            tf_msg.transform.translation.x = T[0, 3]
            tf_msg.transform.translation.y = T[1, 3]
            tf_msg.transform.translation.z = T[2, 3]
            tf_msg.transform.rotation.x = quat[0]
            tf_msg.transform.rotation.y = quat[1]
            tf_msg.transform.rotation.z = quat[2]
            tf_msg.transform.rotation.w = quat[3]

            def _pub_tf():
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                self.tf_br.sendTransform(tf_msg)
            self.create_timer(0.1, _pub_tf)

            self.home_set = True
            return True

        except TransformException as e:
            self.get_logger().warn(f"Waiting for TF '{MAP_FRAME}' -> '{self.drone_frame}': {e}", throttle_duration_sec=2)
            return False

    def _calculate_ned_setpoint(self, target_pose: PoseStamped) -> Tuple[np.ndarray, float]:
        target_pos_enu = np.array([
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        ])
        setpoint_pos_ned = R_ENU2NED @ (target_pos_enu - self.home_enu)

        q_enu = [
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        ]
        _, _, yaw_enu = tft.euler_from_quaternion(q_enu)
        yaw_ned = -yaw_enu + math.pi / 2.0
        yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))

        return setpoint_pos_ned, yaw_ned

    def _loop_cmd(self) -> None:
        self._publish_offboard_control_mode()

        prev_state = self.state
        if self.state == "INIT":
            if self.vs is None or self.curr_ned is None:
                self.get_logger().warn("Waiting for vehicle status and position...", throttle_duration_sec=2)
                return
            if self._ensure_home_and_tf():
                self.arm_timer_count += 1
                if self.arm_timer_count > (CMD_HZ * 2):
                    self.state = "ARMING"

        elif self.state == "ARMING":
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.vs and self.vs.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.state = "CONTROLLING"

        elif self.state == "CONTROLLING":
            if self.target_pose_enu:
                pos, yaw = self._calculate_ned_setpoint(self.target_pose_enu)
                self._publish_trajectory_setpoint(pos, yaw)
            else:
                hover_pos_enu = self.home_enu + np.array([0, 0, TAKEOFF_ALT_ENU])
                hover_pose = PoseStamped()
                hover_pose.header.frame_id = MAP_FRAME
                hover_pose.pose.position.x = hover_pos_enu[0]
                hover_pose.pose.position.y = hover_pos_enu[1]
                hover_pose.pose.position.z = hover_pos_enu[2]
                hover_pose.pose.orientation.w = 1.0
                
                pos, yaw = self._calculate_ned_setpoint(hover_pose)
                self._publish_trajectory_setpoint(pos, yaw)
                self.get_logger().info("No target received. Hovering at takeoff altitude.", throttle_duration_sec=5)

        if self.state != prev_state:
            self.get_logger().info(f"State changed: {prev_state} -> {self.state}")

    def _get_timestamp(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode(
            position=True, velocity=False, acceleration=False, attitude=False, body_rate=False,
            timestamp=self._get_timestamp()
        )
        self.pub_ctrl.publish(msg)

    def _publish_trajectory_setpoint(self, pos: np.ndarray, yaw: float) -> None:
        msg = TrajectorySetpoint(
            position=[float(pos[0]), float(pos[1]), float(pos[2])],
            yaw=float(yaw),
            timestamp=self._get_timestamp()
        )
        self.pub_sp.publish(msg)

    def _publish_vehicle_command(self, command: int, p1: float = 0.0, p2: float = 0.0) -> None:
        msg = VehicleCommand(
            command=command, param1=p1, param2=p2,
            target_system=1, target_component=1, source_system=1, source_component=1,
            from_external=True, timestamp=self._get_timestamp()
        )
        self.pub_cmd.publish(msg)

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PoseControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()