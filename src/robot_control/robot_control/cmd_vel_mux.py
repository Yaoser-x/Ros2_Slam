#!/usr/bin/env python3
import secrets
import math
import signal
import time
from typing import Dict, Optional

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks
from robot_interfaces.msg import (
    ChassisLinkState,
    HostControlState,
    HostMotionCommand,
    MotionSupervisionState,
    NavigationGuardState,
    PlatformCompatibilityState,
)

from robot_control.control_policy import (
    Command,
    CommandMux,
    MotionLimiter,
    SelectedCommand,
    SourceConfig,
    SourceUpdateDisposition,
)


def _candidate_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _command_qos(deadline_sec: float, lifespan_sec: float) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=deadline_sec),
        lifespan=Duration(seconds=lifespan_sec),
    )


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class CmdVelMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_mux")
        self.monotonic_clock = time.monotonic
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.declare_parameter("config_sha256", "development-uncompiled")
        self.declare_parameter("unsafe_development_mode", True)
        self.declare_parameter("research_sources", [])
        self.declare_parameter("linear_limit", 0.4)
        self.declare_parameter("angular_limit", 1.5)
        self.declare_parameter("input_linear_abs_max", 5.0)
        self.declare_parameter("input_angular_abs_max", 20.0)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.5)
        self.declare_parameter("max_linear_jerk", 2.0)
        self.declare_parameter("max_angular_jerk", 6.0)
        self.declare_parameter("motion_limiter_max_dt", 0.1)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("command_deadline_sec", 0.10)
        self.declare_parameter("command_lifespan_sec", 0.12)
        self.declare_parameter("host_motion_command_topic", "chassis/host_motion_command")
        self.declare_parameter("motion_supervision_topic", "motion/supervision_state")
        self.declare_parameter("chassis_link_state_topic", "chassis/link_state")
        self.declare_parameter("host_control_state_topic", "chassis/host_control_state")
        self.declare_parameter("navigation_guard_state_topic", "navigation/guard_state")
        self.declare_parameter("nav_guard_timeout_sec", 0.25)
        self.declare_parameter("require_motion_supervision", True)
        self.declare_parameter("supervision_timeout_sec", 0.25)
        self.declare_parameter("compatibility_state_topic", "platform/compatibility_state")
        self.declare_parameter("compatibility_timeout_sec", 0.25)
        self.declare_parameter("rearm_quiet_sec", 0.25)
        self.declare_parameter("rearm_disable_retry_sec", 0.05)

        research_sources = [str(value) for value in self.get_parameter("research_sources").value]
        self.source_config = SourceConfig(research_sources=research_sources)
        self.mux = CommandMux(
            source_config=self.source_config,
            linear_limit=float(self.get_parameter("linear_limit").value),
            angular_limit=float(self.get_parameter("angular_limit").value),
            timeout_sec=float(self.get_parameter("timeout_sec").value),
            input_linear_abs_max=float(self.get_parameter("input_linear_abs_max").value),
            input_angular_abs_max=float(self.get_parameter("input_angular_abs_max").value),
        )

        self.chassis_command_topic = str(self.get_parameter("host_motion_command_topic").value)
        self.dds_deadline_miss_count = 0
        self.publisher_liveliness_lost_count = 0
        self.command_queue_drop_count = 0
        deadline_sec = float(self.get_parameter("command_deadline_sec").value)
        lifespan_sec = float(self.get_parameter("command_lifespan_sec").value)
        publish_period_sec = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)
        if (
            not 0.0
            < publish_period_sec
            < deadline_sec
            <= lifespan_sec
            < float(self.get_parameter("timeout_sec").value)
        ):
            raise RuntimeError(
                "command timing must satisfy publish period < deadline <= lifespan < source lease"
            )
        self.publisher = self.create_publisher(
            HostMotionCommand,
            self.chassis_command_topic,
            _command_qos(deadline_sec, lifespan_sec),
            event_callbacks=PublisherEventCallbacks(
                deadline=self._on_publish_deadline,
                liveliness=self._on_publish_liveliness,
                use_default_callbacks=False,
            ),
        )
        self.state_publisher = self.create_publisher(
            HostControlState,
            str(self.get_parameter("host_control_state_topic").value),
            STATE_QOS,
        )
        self.sequence = 0
        self.command_epoch = 0
        self.motion_limiter = MotionLimiter(
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            max_linear_jerk=float(self.get_parameter("max_linear_jerk").value),
            max_angular_jerk=float(self.get_parameter("max_angular_jerk").value),
            max_dt_sec=float(self.get_parameter("motion_limiter_max_dt").value),
            max_linear_velocity=float(self.get_parameter("linear_limit").value),
            max_angular_velocity=float(self.get_parameter("angular_limit").value),
        )
        self.last_active = False
        self.last_source = "idle"
        self.invalid_command_count = 0
        self.invalid_command_count_by_source = {
            source: 0 for source in self.source_config.topic_map()
        }
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.require_supervision = bool(self.get_parameter("require_motion_supervision").value)
        self.supervision_timeout = float(self.get_parameter("supervision_timeout_sec").value)
        self.compatibility_timeout = float(self.get_parameter("compatibility_timeout_sec").value)
        if not math.isfinite(self.compatibility_timeout) or self.compatibility_timeout <= 0.0:
            raise ValueError("compatibility_timeout_sec must be finite and positive")
        self.compatibility_at: Optional[float] = None
        self.compatibility = None
        self.rearm_quiet_sec = float(self.get_parameter("rearm_quiet_sec").value)
        self.rearm_disable_retry_sec = float(self.get_parameter("rearm_disable_retry_sec").value)
        if not math.isfinite(self.rearm_disable_retry_sec) or self.rearm_disable_retry_sec <= 0.0:
            raise ValueError("rearm_disable_retry_sec must be finite and positive")
        self.last_rearm_disable_at: Optional[float] = None
        self.motion_state = None
        self.motion_state_at: Optional[float] = None
        self.motion_scale = 1.0
        self.supervision_seen = False
        self.rearm_required = True
        self.rearm_reason_flags = HostControlState.REARM_TRANSPORT
        self.gate_state = HostControlState.STATE_HOST_CLEARED
        self.last_reject_reason = HostControlState.REJECT_NONE
        self.last_wire_session_id = 0
        self.wire_ready = False
        self.quiet_started_at: Optional[float] = self.monotonic_clock()
        self.release_sent_wire_session: Optional[int] = None
        self.nav_active = False
        self.nav_guard_at: Optional[float] = None
        self.nav_guard_timeout = float(self.get_parameter("nav_guard_timeout_sec").value)
        if self.nav_guard_timeout <= 0.0:
            raise ValueError("nav_guard_timeout_sec must be positive")
        self.nav_goal_generation = 0
        self.required_nav_generation = 0
        self.subscriptions_by_source: Dict[str, object] = {}
        for source, topic in self.source_config.topic_map().items():
            self.subscriptions_by_source[source] = self.create_subscription(
                Twist,
                topic,
                self._make_callback(source),
                _candidate_qos(),
                event_callbacks=SubscriptionEventCallbacks(
                    liveliness=self._on_candidate_liveliness,
                    message_lost=self._on_candidate_message_lost,
                    use_default_callbacks=False,
                ),
            )
        self.create_subscription(
            PlatformCompatibilityState,
            str(self.get_parameter("compatibility_state_topic").value),
            self._on_compatibility,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            MotionSupervisionState,
            str(self.get_parameter("motion_supervision_topic").value),
            self._on_motion_state,
            STATE_QOS,
        )
        self.create_subscription(
            ChassisLinkState,
            str(self.get_parameter("chassis_link_state_topic").value),
            self._on_chassis_state,
            STATE_QOS,
        )
        self.create_subscription(
            NavigationGuardState,
            str(self.get_parameter("navigation_guard_state_topic").value),
            self._on_navigation_guard,
            STATE_QOS,
        )

        publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / publish_hz, self._publish_selected, clock=self.steady_clock)
        self.get_logger().info(
            "cmd_vel_mux active: "
            f"sources={','.join(self.source_config.topic_map().keys())} "
            f"chassis_command_topic={self.chassis_command_topic} "
            "platform_api=5"
        )

    def _make_callback(self, source: str):
        def _callback(msg: Twist) -> None:
            if source == "nav" and not self._navigation_guard_fresh(self.monotonic_clock()):
                return
            decision = self.mux.update(
                source,
                Command(linear_x=msg.linear.x, angular_z=msg.angular.z),
                self.monotonic_clock(),
            )
            if decision.disposition is not SourceUpdateDisposition.ACCEPTED:
                self.invalid_command_count += 1
                self.invalid_command_count_by_source[source] += 1
                self.last_reject_reason = HostControlState.REJECT_INVALID
                self.get_logger().error(f"Rejected {source} command: {decision.reason}")
                if decision.disposition is SourceUpdateDisposition.REJECTED_ACTIVE_WITH_FALLBACK:
                    self._publish_selected()
                elif (
                    decision.disposition is SourceUpdateDisposition.REJECTED_ACTIVE_RELEASE
                    and self.last_active
                ):
                    self._publish_release()
                return

            if self.rearm_required:
                if self.gate_state == HostControlState.STATE_WAIT_FRESH_HOST_INTENT:
                    if not self.wire_ready:
                        return
                    if source == "nav" and self.nav_goal_generation <= self.required_nav_generation:
                        return
                    self.rearm_required = False
                    self.rearm_reason_flags = 0
                    self.gate_state = HostControlState.STATE_WAIT_FRESH_HOST_INTENT
                    self.last_active = False
                    self.command_epoch = 0
                    self.sequence = 0
                    self.motion_limiter.reset()
                else:
                    if self.gate_state == HostControlState.STATE_WAIT_SOURCE_QUIET:
                        self.quiet_started_at = None
                    return
            self.last_reject_reason = HostControlState.REJECT_NONE
            if not self.last_active:
                self._publish_selected()

        return _callback

    def _on_publish_deadline(self, event) -> None:
        self.dds_deadline_miss_count += max(int(event.total_count_change), 0)

    def _on_publish_liveliness(self, event) -> None:
        self.publisher_liveliness_lost_count += max(int(event.total_count_change), 0)

    def _on_candidate_message_lost(self, event) -> None:
        self.command_queue_drop_count += max(int(event.total_count_change), 0)

    def _on_candidate_liveliness(self, event) -> None:
        self.publisher_liveliness_lost_count += max(-int(event.alive_count_change), 0)

    def _publish_selected(self) -> None:
        now_sec = self.monotonic_clock()
        if not self._compatibility_fresh(now_sec):
            self._latch_rearm(
                HostControlState.REARM_PROTOCOL_INCOMPATIBLE,
                HostControlState.REJECT_REARM_REQUIRED,
            )
            self._retry_rearm_disable(now_sec)
            self._publish_control_state(now_sec)
            return
        if not self._navigation_guard_fresh(now_sec):
            self.nav_active = False
            self.mux.clear_source("nav")
        if self.require_supervision:
            supervision_fresh = (
                self.motion_state_at is not None
                and 0.0 <= now_sec - self.motion_state_at <= self.supervision_timeout
            )
            if not supervision_fresh:
                if self.supervision_seen:
                    self._latch_rearm(
                        HostControlState.REARM_SUPERVISION_STALE,
                        HostControlState.REJECT_SUPERVISION_STALE,
                    )
                elif not self.rearm_required:
                    self.gate_state = HostControlState.STATE_HOST_CLEARED
                self._retry_rearm_disable(now_sec)
                self._publish_control_state(now_sec)
                return
        if self.rearm_required:
            if self.gate_state == HostControlState.STATE_HOST_CLEARED:
                # Establish an explicit Host-side disabled epoch before any
                # future enabled epoch.  Queue admission is not rearm proof;
                # the following gates still wait for wire disable ACK.
                if self.release_sent_wire_session != self.last_wire_session_id:
                    self._publish_release()
                    self.release_sent_wire_session = self.last_wire_session_id
                self.gate_state = HostControlState.STATE_WAIT_SOURCE_QUIET
                self.quiet_started_at = now_sec
            elif self.gate_state == HostControlState.STATE_WAIT_SOURCE_QUIET:
                if self.mux.has_recent_input(now_sec, self.rearm_quiet_sec):
                    self.quiet_started_at = None
                else:
                    if self.quiet_started_at is None:
                        self.quiet_started_at = now_sec
                    elif now_sec - self.quiet_started_at >= self.rearm_quiet_sec:
                        self.mux.clear()
                        self.gate_state = HostControlState.STATE_WAIT_WIRE_READY
            elif self.gate_state == HostControlState.STATE_WAIT_WIRE_READY and self.wire_ready:
                self.gate_state = HostControlState.STATE_WAIT_FRESH_HOST_INTENT
            self._retry_rearm_disable(now_sec)
            self._publish_control_state(now_sec)
            return
        selected = self.mux.select(now_sec)
        if not selected.active:
            if self.last_active:
                self._publish_release()
            self.gate_state = HostControlState.STATE_WAIT_FRESH_HOST_INTENT
            self._publish_control_state(now_sec)
            return

        if not self.last_active:
            self.command_epoch = secrets.randbits(64) or 1
            self.sequence = 0

        scaled = Command(
            selected.command.linear_x * self.motion_scale,
            selected.command.angular_z * self.motion_scale,
        )
        limited = self.motion_limiter.limit(scaled, now_sec=now_sec)
        self._publish_command(SelectedCommand(source=selected.source, command=limited, active=True))
        self.last_active = True
        self.last_source = selected.source
        self.gate_state = HostControlState.STATE_HOST_ACTIVE
        self._publish_control_state(now_sec)

    def _retry_rearm_disable(self, now_sec: float) -> None:
        if (
            self.rearm_required
            and not self.wire_ready
            and (
                self.last_rearm_disable_at is None
                or now_sec - self.last_rearm_disable_at >= self.rearm_disable_retry_sec
            )
        ):
            # A volatile first sample can miss a late DDS reader. Retry the
            # same disabled Host epoch until current-session wire evidence
            # confirms readiness; publishing is never an ACK substitute.
            self._publish_release()

    def _publish_release(self) -> None:
        if self.command_epoch == 0:
            self.command_epoch = secrets.randbits(64) or 1
        self.motion_limiter.reset()
        selected = SelectedCommand(source="idle", command=Command(0.0, 0.0), active=False)
        self._publish_command(selected)
        self.last_rearm_disable_at = self.monotonic_clock()
        self.last_active = False
        self.last_source = "idle"

    def _on_motion_state(self, msg: MotionSupervisionState) -> None:
        now_sec = self.monotonic_clock()
        self.motion_state = msg
        self.motion_state_at = now_sec
        self.supervision_seen = True
        self.motion_scale = max(0.0, min(float(msg.command_scale), 1.0))
        if msg.release_host_candidate or msg.level == MotionSupervisionState.LEVEL_CRITICAL:
            self._latch_rearm(
                HostControlState.REARM_MOTION_CRITICAL,
                HostControlState.REJECT_MOTION_CRITICAL,
            )
        elif not self.rearm_required and not self.last_active:
            self.gate_state = HostControlState.STATE_WAIT_FRESH_HOST_INTENT

    def _on_compatibility(self, msg: PlatformCompatibilityState) -> None:
        sample_sec = int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - sample_sec
        self.compatibility = msg
        self.compatibility_at = (
            self.monotonic_clock() - max(age, 0.0)
            if math.isfinite(age) and -0.02 <= age <= self.compatibility_timeout
            else None
        )
        if not self._compatibility_fresh(self.monotonic_clock()):
            self._latch_rearm(
                HostControlState.REARM_PROTOCOL_INCOMPATIBLE,
                HostControlState.REJECT_REARM_REQUIRED,
            )

    def _compatibility_fresh(self, now_sec: float) -> bool:
        state = self.compatibility
        return bool(
            state is not None
            and self.compatibility_at is not None
            and 0.0 <= now_sec - self.compatibility_at <= self.compatibility_timeout
            and state.observation_ready
            and self.last_wire_session_id != 0
            and int(state.wire_session_id) == self.last_wire_session_id
            and int(state.observation_session_id) == self.last_wire_session_id
            and (
                state.state == PlatformCompatibilityState.STATE_SIMULATION
                or (
                    state.state == PlatformCompatibilityState.STATE_COMPATIBLE
                    and state.permit_formal_odometry
                )
            )
        )

    def _on_chassis_state(self, msg: ChassisLinkState) -> None:
        wire_session = int(msg.wire_session_id)
        session_changed = wire_session != 0 and wire_session != self.last_wire_session_id
        self.last_wire_session_id = wire_session
        self.wire_ready = bool(
            msg.link_state
            in (ChassisLinkState.STATE_WIRE_REARM_READY, ChassisLinkState.STATE_DRIVE_ACTIVE)
            and not msg.wire_rearm_required
            and msg.protocol_compatible
        )
        if msg.wire_rearm_required or not self.wire_ready:
            self._latch_rearm(
                int(msg.wire_rearm_reason_flags) or HostControlState.REARM_TRANSPORT,
                HostControlState.REJECT_REARM_REQUIRED,
                force_release=session_changed,
            )

    def _on_navigation_guard(self, msg: NavigationGuardState) -> None:
        self.nav_guard_at = self.monotonic_clock()
        self.nav_goal_generation = int(msg.goal_generation)
        self.nav_active = msg.state == NavigationGuardState.STATE_ACTIVE
        if not self.nav_active:
            self.mux.clear_source("nav")
            if self.last_source == "nav":
                self._publish_selected()

    def _navigation_guard_fresh(self, now_sec: float) -> bool:
        return bool(
            self.nav_active
            and self.nav_guard_at is not None
            and 0.0 <= now_sec - self.nav_guard_at <= self.nav_guard_timeout
        )

    def _latch_rearm(self, reason_flags, reject_reason, *, force_release=False) -> None:
        first = not self.rearm_required
        self.rearm_required = True
        self.rearm_reason_flags |= int(reason_flags)
        self.last_reject_reason = int(reject_reason)
        restart = (
            first
            or force_release
            or self.gate_state
            in (
                HostControlState.STATE_WAIT_FRESH_HOST_INTENT,
                HostControlState.STATE_HOST_ACTIVE,
            )
        )
        if restart:
            self.mux.clear()
            self.motion_limiter.reset()
            self.gate_state = HostControlState.STATE_HOST_CLEARED
            self.quiet_started_at = self.monotonic_clock()
            self.required_nav_generation = self.nav_goal_generation
        if first or force_release or self.last_active:
            self._publish_release()
            self.release_sent_wire_session = self.last_wire_session_id

    def _publish_control_state(self, now_sec: float) -> None:
        msg = HostControlState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.control_state = self.gate_state
        msg.selected_host_subsource = self._source_id(self.last_source)
        age = self.mux.newest_age(now_sec)
        msg.command_age_ms = 0xFFFFFFFF if age is None else min(int(age * 1000.0), 0xFFFFFFFF)
        msg.selected_command_age_ms = msg.command_age_ms
        msg.dds_deadline_miss_count = self.dds_deadline_miss_count
        msg.publisher_liveliness_lost_count = self.publisher_liveliness_lost_count
        msg.command_queue_drop_count = self.command_queue_drop_count
        msg.command_reject_reason = self.last_reject_reason
        msg.enable_intent = self.last_active and not self.rearm_required
        msg.rearm_required = self.rearm_required
        msg.rearm_reason_flags = self.rearm_reason_flags
        msg.command_epoch = self.command_epoch
        msg.command_sequence = self.sequence
        msg.config_sha256 = self.config_sha256
        self.state_publisher.publish(msg)

    def _publish_command(self, selected) -> None:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        msg = HostMotionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = selected.command.linear_x
        msg.twist.angular.z = selected.command.angular_z
        msg.enable = selected.active
        msg.host_subsource = self._source_id(selected.source)
        msg.command_epoch = self.command_epoch
        msg.sequence = self.sequence
        self.publisher.publish(msg)

    @staticmethod
    def _source_id(source: str) -> int:
        if source == "teleop":
            return HostMotionCommand.HOST_SUBSOURCE_TELEOP
        if source == "test":
            return HostMotionCommand.HOST_SUBSOURCE_TEST
        if source == "nav":
            return HostMotionCommand.HOST_SUBSOURCE_NAV
        if source.startswith("research/"):
            return HostMotionCommand.HOST_SUBSOURCE_RESEARCH
        return HostMotionCommand.HOST_SUBSOURCE_NONE


def main() -> None:
    # Unwind spin before context teardown so DDS message conversion stays valid.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = CmdVelMuxNode()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while not stop_requested and node.context.ok():
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
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
