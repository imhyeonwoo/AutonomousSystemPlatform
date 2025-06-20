#!/usr/bin/env python3
"""
PX4 Offboard – Ground-Truth TF 기반 제어
======================================

● Arm  
● 현재 위치 +10 m 수직 이륙  
● 3 s 호버링  
● map-ENU (0 m, 0 m, 10 m) 으로 이동 → 호버 지속

* 좌표는 `map → x500_gimbal_0` TF(ENU)를 **항상 lookup**해서
  Ground-Truth 를 얻고, 즉석에서 NED(PX4)로 변환해 사용합니다.
* TF 가 잠시 끊기면 직전 위치 세트포인트를 보내 “Freeze” 합니다.
* SITL 테스트 편의를 위해 **강제 Arm**(`FORCE_ARM=True`) 옵션 포함
  (실기체 사용 시 반드시 False 로 변경).

ROS 2 Humble / Python 3.10
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleStatus
)

# ─────────── 사용자 파라미터 ───────────
DRONE_FRAME = 'x500_gimbal_0'   # ↔ tf child_frame_id  (필요시 수정)
TAKEOFF_ALT = 10.0              # [m] +Up  (현재 위치에서)
HOVER_SEC   = 3.0               # [s]
TARGET_ENU  = (0.0, 0.0, 10.0)  # map-좌표 최종지점
POS_EPS     = 0.6               # [m] 위치 도달 오차
ALT_EPS     = 0.3               # [m] 고도 오차
SP_HZ       = 20                # TrajectorySetpoint 전송 주기
HB_PREROLL  = 2.0               # [s] Arm/Offboard 전 하트비트
FORCE_ARM   = True              # SITL 전용 (실기체 False!)
STALE_TF_MAX = 0.2              # [s] TF 허용 지연
# ──────────────────────────────────────


class OffboardTF(Node):
    """Ground-Truth TF 기반 상태-머신 비행 노드"""

    PREPARE, TAKEOFF, HOVER, MOVE_TARGET, DONE = range(5)

    def __init__(self):
        super().__init__('offboard_tf_takeoff_origin')

        # PX4 FMU(uORB) QoS
        fmu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # 퍼블리셔
        self.ctrl_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', fmu_qos)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',   fmu_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command',           fmu_qos)
        # 상태 서브스크라이버(Arm 상태 체크용)
        self.create_subscription(VehicleStatus,
                                 '/fmu/out/vehicle_status',
                                 self.cb_status, fmu_qos)

        # TF listener
        self.tf_buf = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buf, self)

        # 내부 상태
        self.status = VehicleStatus()
        self.state = self.PREPARE
        self.hb_needed = int(SP_HZ * HB_PREROLL)
        self.hb_cnt = 0

        self.takeoff_n = self.takeoff_e = self.takeoff_d = None
        self.hover_end = None
        self.target_ned = (TARGET_ENU[1], TARGET_ENU[0], -TARGET_ENU[2])  # (N,E,D)

        # 주기 타이머
        self.timer = self.create_timer(1.0 / SP_HZ, self.loop)
        self.get_logger().info("▶︎ TF-기반 Offboard 노드 준비 완료")

    # ────────────── 콜백 ──────────────
    def cb_status(self, msg):   self.status = msg

    # ────────────── 헬퍼 ──────────────
    def lookup_drone_ned(self):
        """map→드론 최신 변환을 NED 로 변환하여 반환. 실패 시 None"""
        try:
            tr = self.tf_buf.lookup_transform(
                target_frame='map',
                source_frame=DRONE_FRAME,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            # ENU
            x_e = tr.transform.translation.x
            y_n = tr.transform.translation.y
            z_u = tr.transform.translation.z
            # 시간 지연 체크
            stamp = (tr.header.stamp.sec +
                     tr.header.stamp.nanosec * 1e-9)
            now   = self.get_clock().now().nanoseconds * 1e-9
            age   = now - stamp
            if age > STALE_TF_MAX:
                self.get_logger().warn(
                    f"Stale TF ({age:.3f}s) – maintaining current setpoint")
            # ENU → NED
            return y_n, x_e, -z_u
        except (LookupException, ExtrapolationException, ConnectivityException):
            return None

    def send_cmd(self, cmd, **p):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = cmd
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        for i in range(1, 8):
            setattr(msg, f'param{i}', float(p.get(f'param{i}', 0.0)))
        self.cmd_pub.publish(msg)

    def arm(self, force=False):
        self.send_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=(21196 if force else 0.0))

    def set_offboard(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                      param1=1, param2=6)  # CUSTOM, OFFBOARD

    def send_ctrl_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True
        self.ctrl_pub.publish(msg)

    def send_sp(self, n, e, d):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [n, e, d]
        self.traj_pub.publish(msg)

    # ────────────── 메인 루프 ──────────────
    def loop(self):
        # 1) 최신 TF 기반 위치 획득
        ned = self.lookup_drone_ned()
        if ned is None:
            # 아직 TF 불가 → 하트비트만 전송
            self.send_ctrl_mode()
            return
        cur_n, cur_e, cur_d = ned

        # 2) Heartbeat
        self.send_ctrl_mode()

        # 3) 상태-머신
        if self.state == self.PREPARE:
            self.send_sp(cur_n, cur_e, cur_d)
            if self.hb_cnt < self.hb_needed:
                self.hb_cnt += 1
                if self.hb_cnt == self.hb_needed:
                    self.set_offboard()
                    self.arm(FORCE_ARM)
                    self.takeoff_n, self.takeoff_e = cur_n, cur_e
                    self.takeoff_d = cur_d - TAKEOFF_ALT
                    self.state = self.TAKEOFF
                    self.get_logger().info("▶︎ STATE → TAKEOFF")
            return

        if self.state == self.TAKEOFF:
            self.send_sp(self.takeoff_n, self.takeoff_e, self.takeoff_d)
            if abs(cur_d - self.takeoff_d) < ALT_EPS:
                self.hover_end = self.get_clock().now() + Duration(seconds=HOVER_SEC)
                self.state = self.HOVER
                self.get_logger().info("▶︎ STATE → HOVER")
            return

        if self.state == self.HOVER:
            self.send_sp(self.takeoff_n, self.takeoff_e, self.takeoff_d)
            if self.get_clock().now() >= self.hover_end:
                self.state = self.MOVE_TARGET
                self.get_logger().info(
                    f"▶︎ STATE → MOVE_TARGET  NED {self.target_ned}")
            return

        if self.state == self.MOVE_TARGET:
            tgt_n, tgt_e, tgt_d = self.target_ned
            self.send_sp(tgt_n, tgt_e, tgt_d)
            dist = math.sqrt((tgt_n - cur_n) ** 2 +
                             (tgt_e - cur_e) ** 2 +
                             (tgt_d - cur_d) ** 2)
            if dist < POS_EPS:
                self.state = self.DONE
                self.get_logger().info("▶︎ STATE → DONE (hover at map origin)")
            return

        if self.state == self.DONE:
            tgt_n, tgt_e, tgt_d = self.target_ned
            self.send_sp(tgt_n, tgt_e, tgt_d)  # 계속 호버 유지


# ─────────────── main ───────────────
def main(args=None):
    rclpy.init(args=args)
    node = OffboardTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt – 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
