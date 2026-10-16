#!/usr/bin/env python3
"""ROS adapter for motion supervision decisions."""

import math
import signal
import time
from typing import Any, Optional

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import HostMotionCommand, MotionSupervisionState, WheelObservation
from sensor_msgs.msg import Imu
from robot_chassis_model.wheel_layout import WheelLayout, WheelLayoutError

from robot_supervision.motion_supervisor import (
    MotionSupervisor,
    SupervisorConfig,
    SupervisorLevel,
    observation_requires_release,
)


class MotionSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_supervisor")
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        defaults = SupervisorConfig()
        self.declare_parameter("config_sha256", "development-uncompiled")
        self.declare_parameter("wheel_observation_topic", "wheel/observation")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("host_motion_command_topic", "chassis/host_motion_command")
        self.declare_parameter("motion_supervision_topic", "motion/supervision_state")
        self.declare_parameter("wheel_track_width", 0.176)
        self.declare_parameter("observation_timeout_sec", 0.25)
        self.declare_parameter("imu_timeout_sec", 0.20)
        self.declare_parameter("command_timeout_sec", 0.25)
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        values = {name: self.get_parameter(name).value for name in defaults.__dataclass_fields__}
        self.supervisor = MotionSupervisor(SupervisorConfig(**values))
        self.track_width = max(float(self.get_parameter("wheel_track_width").value), 1e-6)
        self.observation_timeout = float(self.get_parameter("observation_timeout_sec").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout_sec").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.last_command: Optional[Any] = None
        self.last_command_at: Optional[float] = None
        self.last_command_identity: Optional[tuple[int, int]] = None
        self.last_gyro_z: Optional[float] = None
        self.last_imu_at: Optional[float] = None
        self.last_observation_at: Optional[float] = None
        self.last_observation: Optional[Any] = None
        self.last_sample_identity: Optional[tuple[int, int]] = None
        self.publisher = self.create_publisher(
            MotionSupervisionState,
            str(self.get_parameter("motion_supervision_topic").value),
            10,
        )
        self.create_subscription(
            HostMotionCommand,
            str(self.get_parameter("host_motion_command_topic").value),
            self._on_command,
            10,
        )
        self.create_subscription(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            self._on_wheel,
            20,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._health_tick, clock=self.steady_clock)

    def _on_command(self, msg: HostMotionCommand) -> None:
        identity = (int(msg.command_epoch), int(msg.sequence))
        if self.last_command_identity is not None:
            old_epoch, old_sequence = self.last_command_identity
            delta = (identity[1] - old_sequence) & 0xFFFFFFFF
            if identity[0] == old_epoch and not (0 < delta < 0x80000000):
                return
        self.last_command_identity = identity
        self.last_command = msg
        self.last_command_at = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        gyro = float(msg.angular_velocity.z)
        covariance = float(msg.angular_velocity_covariance[0])
        sample_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - sample_sec
        if (
            not math.isfinite(gyro)
            or not math.isfinite(covariance)
            or covariance < 0.0
            or not -0.02 <= age <= self.imu_timeout
        ):
            self.last_gyro_z = None
            self.last_imu_at = None
            return
        self.last_gyro_z = gyro
        # Include sample age at admission, then use only the monotonic clock.
        self.last_imu_at = time.monotonic() - max(age, 0.0)

    def _on_wheel(self, msg: WheelObservation) -> None:
        now = time.monotonic()
        identity = (int(msg.transport_session_id), int(msg.status_sequence))
        if self.last_sample_identity is not None:
            old_session, old_sequence = self.last_sample_identity
            delta = (identity[1] - old_sequence) & 0xFFFFFFFF
            if identity[0] == old_session and not (0 < delta < 0x80000000):
                return
        self.last_sample_identity = identity
        self.last_observation_at = now
        self.last_observation = msg
        if observation_requires_release(
            schema_version=msg.schema_version,
            expected_schema_version=WheelObservation.SCHEMA_VERSION,
            encoder_anomaly_mask=msg.encoder_anomaly_mask,
            enabled_mask=msg.motor_enabled_mask,
            speed_valid_mask=msg.speed_valid_mask,
        ):
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_OBSERVATION_INVALID,
            )
            return
        speeds = tuple(float(value) for value in msg.wheel_speed_mps)
        targets = tuple(float(value) for value in msg.wheel_target_mps)
        if len(speeds) != 4 or len(targets) != 4:
            return
        wheel_speeds = (speeds[0], speeds[1], speeds[2], speeds[3])
        wheel_targets = (targets[0], targets[1], targets[2], targets[3])
        layout = WheelLayout(msg.motor_enabled_mask, msg.speed_valid_mask, msg.encoder_anomaly_mask)
        try:
            left, right = layout.aggregate(wheel_speeds)
        except WheelLayoutError:
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_OBSERVATION_INVALID,
            )
            return
        command_vx = 0.0
        command_wz = 0.0
        command_fresh = self.last_command is not None and self._fresh(
            self.last_command_at, self.command_timeout, now
        )
        command = self.last_command
        if command_fresh and command is not None and command.enable:
            command_vx = float(command.twist.linear.x)
            command_wz = float(command.twist.angular.z)
        gyro = self.last_gyro_z if self._fresh(self.last_imu_at, self.imu_timeout, now) else None
        result = self.supervisor.update(
            now_sec=now,
            command_vx=command_vx,
            command_wz=command_wz,
            wheel_speeds=wheel_speeds,
            wheel_targets=wheel_targets,
            enabled_mask=msg.motor_enabled_mask,
            speed_valid_mask=msg.speed_valid_mask,
            anomaly_mask=msg.encoder_anomaly_mask,
            feedback_vx=0.5 * (left + right),
            wheel_wz=(right - left) / self.track_width,
            gyro_z=gyro,
            command_valid=command_fresh,
        )
        reason_map = {
            "wheel_pair": MotionSupervisionState.REASON_WHEEL_PAIR,
            "tracking": MotionSupervisionState.REASON_TRACKING,
            "yaw": MotionSupervisionState.REASON_YAW_DISAGREEMENT,
            "unexpected_motion": MotionSupervisionState.REASON_UNEXPECTED_MOTION,
        }
        reason_flags = reason_map.get(result.reason, MotionSupervisionState.REASON_NONE)
        if gyro is None:
            reason_flags |= MotionSupervisionState.REASON_IMU_UNAVAILABLE
        if len(layout.left_indices) < 2 or len(layout.right_indices) < 2:
            reason_flags |= MotionSupervisionState.REASON_NO_WHEEL_REDUNDANCY
        self._publish(
            result.level,
            result.score,
            result.command_scale,
            result.release_host_candidate,
            reason_flags,
            components=dict(result.components),
        )

    def _health_tick(self) -> None:
        now = time.monotonic()
        if (
            self.last_observation_at is None
            or now - self.last_observation_at > self.observation_timeout
        ):
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_OBSERVATION_STALE,
            )
            return
        if (
            self.last_command is not None
            and self.last_command.enable
            and (self.last_command_at is None or now - self.last_command_at > self.command_timeout)
        ):
            self._publish(
                SupervisorLevel.CRITICAL,
                1.0,
                0.0,
                True,
                MotionSupervisionState.REASON_COMMAND_STALE,
            )

    def _publish(self, level, score, scale, release, reason_flags, components=None) -> None:
        msg = MotionSupervisionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.level = int(level) + 1
        msg.reason_flags = int(reason_flags)
        msg.score = float(score)
        msg.command_scale = float(scale)
        msg.release_host_candidate = bool(release)
        components = components or {}
        msg.wheel_pair_risk = float(components.get("wheel_pair", 0.0))
        msg.tracking_risk = float(components.get("tracking", 0.0))
        msg.yaw_consistency_risk = float(components.get("yaw", 0.0))
        msg.unexpected_motion_risk = float(components.get("unexpected_motion", 0.0))
        if self.last_command is not None:
            msg.command_epoch = self.last_command.command_epoch
            msg.command_sequence = self.last_command.sequence
        if self.last_observation is not None:
            msg.observation_session_id = self.last_observation.transport_session_id
            msg.observation_sequence = self.last_observation.status_sequence
            layout = WheelLayout(
                self.last_observation.motor_enabled_mask,
                self.last_observation.speed_valid_mask,
                self.last_observation.encoder_anomaly_mask,
            )
            msg.left_valid_wheel_count = len(layout.left_indices)
            msg.right_valid_wheel_count = len(layout.right_indices)
            msg.wheel_valid_mask = layout.eligible_mask
            msg.motor_enabled_mask = self.last_observation.motor_enabled_mask
        msg.observation_age_ms = self._age_ms(self.last_observation_at)
        msg.gyro_age_ms = self._age_ms(self.last_imu_at)
        msg.command_age_ms = self._age_ms(self.last_command_at)
        now = time.monotonic()
        msg.gyro_valid = self._fresh(self.last_imu_at, self.imu_timeout, now)
        msg.command_valid = self._fresh(self.last_command_at, self.command_timeout, now)
        msg.config_sha256 = self.config_sha256
        self.publisher.publish(msg)

    @staticmethod
    def _age_ms(timestamp: Optional[float]) -> int:
        if timestamp is None:
            return 0xFFFFFFFF
        return min(int(max(time.monotonic() - timestamp, 0.0) * 1000), 0xFFFFFFFF)

    @staticmethod
    def _fresh(timestamp: Optional[float], timeout: float, now: float) -> bool:
        return timestamp is not None and 0.0 <= now - timestamp <= timeout


def main(args=None) -> None:
    # Unwind spin before context teardown so DDS message conversion stays valid.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = MotionSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
