#!/usr/bin/env python3
"""Publish formal odometry and TF only from fresh current-session wheel evidence."""

import signal
import time
from typing import Optional
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import PlatformCompatibilityState, WheelObservation
from tf2_ros import TransformBroadcaster

from robot_state_estimation.formal_odometry_policy import evidence_matches_odometry


class FormalOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("formal_odometry")
        self.declare_parameter("input_topic", "odometry/filtered_internal")
        self.declare_parameter("output_topic", "odom")
        self.declare_parameter("compatibility_state_topic", "platform/compatibility_state")
        self.declare_parameter("wheel_observation_topic", "wheel/observation")
        self.declare_parameter("evidence_max_age_sec", 0.25)
        self.declare_parameter("compatibility_timeout_sec", 0.25)
        self.permitted = False
        self.permitted_session = 0
        self.fresh_evidence_at: Optional[float] = None
        self.fresh_evidence_stamp_ns: Optional[int] = None
        self.evidence_max_age_sec = float(self.get_parameter("evidence_max_age_sec").value)
        self.compatibility_timeout_sec = float(
            self.get_parameter("compatibility_timeout_sec").value
        )
        if min(self.evidence_max_age_sec, self.compatibility_timeout_sec) <= 0.0:
            raise ValueError("formal odometry freshness limits must be positive")
        self.compatibility_received_at: Optional[float] = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        compatibility_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            self._on_wheel_observation,
            20,
        )
        self.create_subscription(
            PlatformCompatibilityState,
            str(self.get_parameter("compatibility_state_topic").value),
            self._on_compatibility,
            compatibility_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._on_odometry,
            20,
        )
        self.create_timer(0.05, self._health_tick, clock=self.steady_clock)

    def _on_compatibility(self, msg: PlatformCompatibilityState) -> None:
        self.compatibility_received_at = time.monotonic()
        permitted = bool(msg.permit_formal_odometry and msg.observation_ready)
        session = int(msg.observation_session_id) if permitted else 0
        if not permitted or session != self.permitted_session:
            self.fresh_evidence_at = None
            self.fresh_evidence_stamp_ns = None
        self.permitted = permitted
        self.permitted_session = session

    def _health_tick(self) -> None:
        if (
            self.compatibility_received_at is None
            or time.monotonic() - self.compatibility_received_at > self.compatibility_timeout_sec
        ):
            self.permitted = False
            self.permitted_session = 0
            self.fresh_evidence_at = None
            self.fresh_evidence_stamp_ns = None

    def _on_wheel_observation(self, msg: WheelObservation) -> None:
        if (
            self.permitted
            and self.permitted_session != 0
            and int(msg.transport_session_id) == self.permitted_session
            and int(msg.schema_version) == WheelObservation.SCHEMA_VERSION
        ):
            self.fresh_evidence_at = time.monotonic()
            self.fresh_evidence_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )

    def _on_odometry(self, msg: Odometry) -> None:
        self._health_tick()
        input_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if (
            not self.permitted
            or self.permitted_session == 0
            or self.fresh_evidence_at is None
            or self.fresh_evidence_stamp_ns is None
            or time.monotonic() - self.fresh_evidence_at > self.evidence_max_age_sec
            or not evidence_matches_odometry(
                input_stamp_ns,
                self.fresh_evidence_stamp_ns,
                self.evidence_max_age_sec,
            )
        ):
            return
        self.publisher.publish(msg)
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = FormalOdometryNode()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while not stop_requested and node.context.ok():
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if node.context.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
