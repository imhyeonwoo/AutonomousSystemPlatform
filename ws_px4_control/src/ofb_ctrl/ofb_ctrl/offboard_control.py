#!/usr/bin/env python3
"""A direct Python port of the PX4 C++ offboard_control example.

This node publishes OffboardControlMode, TrajectorySetpoint and VehicleCommand
messages at 10 Hz.  After 10 set‑points it switches the vehicle to OFFBOARD mode
and arms it, commanding a hover at (0,0,−5 m) with yaw −π rad.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)


class OffboardControl(Node):
    """Node that replicates the PX4 C++ offboard_control example in Python."""

    def __init__(self) -> None:  # noqa: D401 (ignore docstring style warning)
        super().__init__("offboard_control")

        # --- QoS: keep last 10 samples, BEST_EFFORT, TRANSIENT_LOCAL ---
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers ---------------------------------------------------------
        self._offboard_ctrl_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self._traj_sp_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self._vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        # Counter used to send the first 10 set‑points before switching modes
        self._setpoint_counter = 0

        # Timer: 100 ms (10 Hz)
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info("OffboardControl node initialised")

    # --------------------------------------------------------------------- #
    # Helper publishers                                                       #
    # --------------------------------------------------------------------- #

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True  # enable position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp_us()
        self._offboard_ctrl_pub.publish(msg)

    def _publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -5.0]  # NED frame: 5 m above origin
        msg.yaw = -3.14  # −π rad (180°)
        msg.timestamp = self._timestamp_us()
        self._traj_sp_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self._vehicle_cmd_pub.publish(msg)

    # Convenience wrappers --------------------------------------------------
    def _arm(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def _disarm(self) -> None:  # not used in the demo, but handy
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    # --------------------------------------------------------------------- #
    # Timer callback                                                         #
    # --------------------------------------------------------------------- #

    def _timer_cb(self) -> None:
        """Publish control messages at 10 Hz and switch to OFFBOARD after 1 s."""

        # After sending 10 set‑points, switch to OFFBOARD mode and arm.
        if self._setpoint_counter == 10:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6  # main_mode=1, sub_mode=6 (OFFBOARD)
            )
            self._arm()

        # Offboard control must always be paired with trajectory set‑points
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        if self._setpoint_counter < 11:
            self._setpoint_counter += 1

    # --------------------------------------------------------------------- #
    def _timestamp_us(self) -> int:
        """Return current time in micro‑seconds (int)."""
        return int(self.get_clock().now().nanoseconds / 1000)


# ------------------------------------------------------------------------- #
# Main entry point                                                          #
# ------------------------------------------------------------------------- #


def main(args=None) -> None:  # noqa: D401
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
