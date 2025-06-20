#!/usr/bin/env python3
"""
Enhanced PX4 Offboard controller.

• /mission_path(nav_msgs/Path, ENU) 를 구독해 경로를 받아서
  – ENU → NED 변환 후 TrajectorySetpoint 발행
• 차량 위치(/fmu/out/vehicle_odometry, NED) 를 구독해
  – 목표점 도달 여부 판단, 다음 포인트로 진행
• 마지막 포인트에 0.8 m 이내 도달 시 자동 착륙(VEHICLE_CMD_NAV_LAND)
"""

from __future__ import annotations
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
)

from nav_msgs.msg import Path
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class OffboardControl(Node):
    """Waypoint-following offboard controller (position set-point mode)."""

    def __init__(self) -> None:  # noqa: D401
        super().__init__("offboard_control")

        # QoS ----------------------------------------------------------------
        be_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers ---------------------------------------------------------
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", be_qos
        )
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", be_qos
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", be_qos
        )

        # Subscribers --------------------------------------------------------
        self.create_subscription(Path, "/mission_path", self._path_cb, latched_qos)
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, be_qos
        )

        # Internal state -----------------------------------------------------
        self.path_enu: List[List[float]] = []
        self.target_idx = 0
        self.current_ned: List[float] | None = None
        self.setpoint_counter = 0

        self.create_timer(0.1, self._timer_cb)  # 10 Hz
        self.get_logger().info("OffboardControl node initialised")

    # ------------------------------------------------------------------ CBs
    def _path_cb(self, msg: Path) -> None:
        self.path_enu = [
            [p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses
        ]
        self.target_idx = 0
        self.get_logger().info(f"Received path with {len(self.path_enu)} points")

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        self.current_ned = [msg.position[0], msg.position[1], msg.position[2]]

    # ------------------------------------------------------ publishers + util
    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_cmd(self, cmd: int, p1: float = 0.0, p2: float = 0.0) -> None:
        m = VehicleCommand()
        m.command, m.param1, m.param2 = cmd, float(p1), float(p2)
        m.target_system = m.source_system = 1
        m.target_component = m.source_component = 1
        m.from_external = True
        m.timestamp = self._timestamp_us()
        self.pub_cmd.publish(m)

    def _arm(self) -> None:
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def _land(self) -> None:
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")

    # ------------------------------------------------------ main timer (10 Hz)
    def _timer_cb(self) -> None:
        if self.setpoint_counter == 10:           # 모드 전환 + Arm
            self._publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self._arm()
        if self.setpoint_counter < 11:
            self.setpoint_counter += 1

        # 항상 offboard_control_mode + trajectory_setpoint 페어로 전송
        self._send_offboard_mode()
        self._send_setpoint()

    # -------------------------------------------------- Offboard control mode
    def _send_offboard_mode(self) -> None:
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self._timestamp_us()
        self.pub_offboard.publish(m)

    # ------------------------------------------------------------- Set-point
    def _send_setpoint(self) -> None:
        if not self.path_enu:
            return
        e, n, u = self.path_enu[self.target_idx]       # ENU
        ned = [n, e, -u]                               # ENU→NED

        # yaw (NED frame, rad) = atan2(East, North)
        if self.target_idx < len(self.path_enu) - 1:
            e_next, n_next, _ = self.path_enu[self.target_idx + 1]
            yaw = math.atan2(e_next - e, n_next - n)
        else:
            yaw = 0.0

        sp = TrajectorySetpoint()
        sp.position, sp.yaw, sp.timestamp = ned, yaw, self._timestamp_us()
        self.pub_sp.publish(sp)

        # ---------- 도달 판정 ----------
        if self.current_ned is None:
            return
        dx, dy, dz = (self.current_ned[i] - ned[i] for i in range(3))
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if self.target_idx < len(self.path_enu) - 1:
            if dist < 1.0:              # waypoint tolerance
                self.target_idx += 1
                self.get_logger().info(
                    f"→ Waypoint {self.target_idx}/{len(self.path_enu)-1}"
                )
        else:
            if dist < 0.8:
                self._land()
                self.get_logger().info("Final waypoint reached – landing")

# --------------------------------------------------------------------- main
def main(args: list[str] | None = None) -> None:  # noqa: D401
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
