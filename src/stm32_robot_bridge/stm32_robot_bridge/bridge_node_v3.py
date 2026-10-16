#!/usr/bin/env python3
"""ROS adapter for the fail-closed upper protocol v3 bridge."""

from __future__ import annotations

import secrets
import time
from typing import Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.qos_event import SubscriptionEventCallbacks
from robot_interfaces.msg import (
    ChassisLinkState,
    FirmwareInfo,
    FirmwareControlState,
    HostMotionCommand,
    ImuObservation,
    WheelObservation,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, UInt32
from std_srvs.srv import SetBool, Trigger

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

from stm32_robot_bridge.bridge_core import (
    REARM_ACK_TIMEOUT,
    REARM_ESTOP,
    REARM_FIRMWARE_REJECT,
    REARM_TRANSPORT,
    BridgeCore,
    CommandDisposition,
    StatusDisposition,
)
from stm32_robot_bridge.bridge_state import BridgeState
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_REJECTED,
    ACK_SESSION_VALID,
    CMD_CLEAR_FAULT,
    CMD_DIAGNOSTIC,
    CMD_ESTOP,
    CMD_GET_INFO,
    CMD_HELLO,
    CMD_IMU_STATUS,
    CMD_LINE_CTRL,
    CMD_SET_VELOCITY,
    CMD_STATUS,
    PROTOCOL_VERSION,
    CommandStream,
    StatusPayload,
    decode_diagnostic_payload,
    decode_hello_payload,
    decode_imu_status_payload,
    decode_status_payload,
    encode_clear_fault_payload,
    encode_estop_payload,
    encode_get_info_payload,
    encode_line_ctrl_payload,
    imu_identity_is_new,
    sequence_is_forward,
)
from stm32_robot_bridge.transport_supervisor import OutboundFrame, SerialTransportSupervisor


def _qos_command(deadline_sec: float, lifespan_sec: float) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=deadline_sec),
        lifespan=Duration(seconds=lifespan_sec),
    )


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE
)
STATIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class STM32BridgeV3(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge")
        defaults = {
            "config_sha256": "development-uncompiled",
            "unsafe_development_mode": True,
            "port": "/dev/serial0",
            "baudrate": 115200,
            "protocol_version": 3,
            "host_motion_command_topic": "chassis/host_motion_command",
            "chassis_link_state_topic": "chassis/link_state",
            "firmware_control_state_topic": "chassis/firmware_control_state",
            "firmware_info_topic": "chassis/firmware_info",
            "wheel_observation_topic": "wheel/observation",
            "imu_observation_topic": "imu/observation",
            "diagnostics_topic": "diagnostics",
            "base_frame_id": "base_link",
            "imu_frame_id": "imu_link",
            "publish_tf": False,
            "cmd_timeout": 0.15,
            "drive_keepalive_sec": 0.05,
            "status_timeout": 0.25,
            "command_ack_timeout_sec": 0.15,
            "max_command_age_sec": 0.15,
            "command_lifespan_sec": 0.12,
            "hard_max_linear_mps": 0.45,
            "hard_max_angular_radps": 1.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if int(self.get_parameter("protocol_version").value) != 3:
            raise RuntimeError("STM32 bridge Platform API 5 only supports upper protocol v3")
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.ack_timeout = float(self.get_parameter("command_ack_timeout_sec").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        if bool(self.get_parameter("publish_tf").value):
            raise RuntimeError("bridge v3 never owns odom->base_link TF")

        self.core = BridgeCore(
            hard_max_linear_mps=float(self.get_parameter("hard_max_linear_mps").value),
            hard_max_angular_radps=float(self.get_parameter("hard_max_angular_radps").value),
            command_timeout_sec=float(self.get_parameter("cmd_timeout").value),
            status_timeout_sec=self.status_timeout,
            max_command_age_sec=float(self.get_parameter("max_command_age_sec").value),
        )
        self.command_stream = CommandStream(
            float(self.get_parameter("cmd_timeout").value),
            float(self.get_parameter("drive_keepalive_sec").value),
            secrets.randbits(64) or 1,
        )
        self.last_ack_evidence_at = time.monotonic()
        self.pending_disable_started_at: Optional[float] = None
        self.pending_active_ack_started_at: Optional[float] = None
        self.last_applied_sequence: Optional[int] = None
        self.release_count = 0
        self.invalid_command_count = 0
        self.command_deadline_miss_count = 0
        self.command_publisher_lost_count = 0
        self.command_queue_drop_count = 0
        self.warning_suppressed_count = 0
        self._warning_next_at: dict[str, float] = {}
        self.last_status: Optional[StatusPayload] = None
        self.enabled_mask: Optional[int] = None

        if serial is None:
            raise RuntimeError("python3-serial is required")
        self.transport = SerialTransportSupervisor(
            serial,
            str(self.get_parameter("port").value),
            int(self.get_parameter("baudrate").value),
            # IMU is a sampled observation: keep the newest pending sample while
            # preserving every STATUS/ACK and safety transition in FIFO order.
            coalesce_inbound_commands=(CMD_IMU_STATUS,),
        )

        self.wheel_observation_pub = self.create_publisher(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            STATE_QOS,
        )
        self.imu_observation_pub = self.create_publisher(
            ImuObservation,
            str(self.get_parameter("imu_observation_topic").value),
            qos_profile_sensor_data,
        )
        self.link_state_pub = self.create_publisher(
            ChassisLinkState,
            str(self.get_parameter("chassis_link_state_topic").value),
            STATIC_QOS,
        )
        self.firmware_control_pub = self.create_publisher(
            FirmwareControlState,
            str(self.get_parameter("firmware_control_state_topic").value),
            STATE_QOS,
        )
        self.firmware_pub = self.create_publisher(
            FirmwareInfo, str(self.get_parameter("firmware_info_topic").value), STATIC_QOS
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), STATE_QOS
        )
        self.chassis_status_pub = self.create_publisher(UInt32, "chassis/status", STATE_QOS)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", STATE_QOS)
        self.left_current_pub = self.create_publisher(Float32, "motor/left_current", STATE_QOS)
        self.right_current_pub = self.create_publisher(Float32, "motor/right_current", STATE_QOS)
        self.create_subscription(
            HostMotionCommand,
            str(self.get_parameter("host_motion_command_topic").value),
            self.on_chassis_command,
            _qos_command(
                float(self.get_parameter("max_command_age_sec").value),
                float(self.get_parameter("command_lifespan_sec").value),
            ),
            event_callbacks=SubscriptionEventCallbacks(
                deadline=self._on_command_deadline,
                liveliness=self._on_command_liveliness,
                message_lost=self._on_command_message_lost,
                use_default_callbacks=False,
            ),
        )
        self.create_service(SetBool, "~/wire_estop", self.on_estop)
        self.create_service(Trigger, "~/wire_clear_fault", self.on_clear_fault)
        self.create_service(SetBool, "~/wire_line_ctrl", self.on_line_ctrl)
        self.create_service(Trigger, "~/wire_get_info", self.on_get_info)
        self.create_timer(0.01, self.control_tick, clock=self.steady_clock)
        self.create_timer(1.0, self.publish_diagnostics, clock=self.steady_clock)
        self.transport.start()
        self.get_logger().warn(
            "upper protocol v3 only; beta4/v2 firmware is intentionally incompatible"
        )

        self.last_imu_identity: Optional[Tuple[int, int, int]] = None

    def on_chassis_command(self, msg: HostMotionCommand) -> None:
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now_mono = time.monotonic()
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        decision = self.core.accept_command(
            vx=msg.twist.linear.x,
            wz=msg.twist.angular.z,
            enable=bool(msg.enable),
            source=int(msg.host_subsource),
            session_id=int(msg.command_epoch),
            sequence=int(msg.sequence),
            command_stamp_sec=stamp,
            now_ros_sec=now_ros,
            now_monotonic=now_mono,
        )
        if decision.disposition is CommandDisposition.GLOBAL_RELEASE:
            self.invalid_command_count += 1
            self._release(f"rejected command: {decision.reason}")
            return
        if decision.disposition not in (
            CommandDisposition.ACCEPT_ENABLE,
            CommandDisposition.ACCEPT_DISABLE,
        ):
            if decision.disposition is not CommandDisposition.IGNORE_DUPLICATE:
                self.invalid_command_count += 1
                self._warn_bounded("ignored_command", f"ignored command: {decision.reason}")
            return
        command = decision.validated_command
        if command is None:  # defensive: accepted dispositions always carry a command
            self.invalid_command_count += 1
            self._release("accepted command decision has no validated command")
            return
        if not command.enable:
            self._release("")
            return
        self.command_stream.update_command(command.vx, command.wz, now_mono)
        if (
            self.pending_active_ack_started_at is None
            and self.command_stream.sequence != self.last_applied_sequence
        ):
            # Start the first unresolved command's budget.  New targets while
            # waiting are transmissions, not evidence that firmware applied them.
            self.pending_active_ack_started_at = now_mono

    def _on_command_deadline(self, event) -> None:
        self.command_deadline_miss_count += max(int(event.total_count_change), 0)

    def _on_command_liveliness(self, event) -> None:
        self.command_publisher_lost_count += max(-int(event.alive_count_change), 0)

    def _on_command_message_lost(self, event) -> None:
        self.command_queue_drop_count += max(int(event.total_count_change), 0)

    def control_tick(self) -> None:
        began = time.monotonic()
        # Publishing a batch of sensor messages must not monopolize the ROS
        # callback group and starve fresh Host commands. Never dequeue work
        # beyond this tick's budget; remaining frames retain their order.
        for _ in range(4):
            events = self.transport.drain_events(limit=1)
            if not events:
                break
            event = events[0]
            if event.kind == "connected":
                self._on_connected()
            elif event.kind == "disconnected":
                self.core.on_disconnected()
                self.last_status = None
                self.enabled_mask = None
                self._publish_link_state()
            elif event.kind == "frame":
                self._handle_frame(event.cmd, event.payload, event.received_at)
            if time.monotonic() - began >= 0.004:
                break
        now = time.monotonic()
        before = self.core.snapshot
        release, reason = self.core.tick(now)
        if release:
            command_age = None if before.last_command_at is None else now - before.last_command_at
            status_age = None if before.last_status_at is None else now - before.last_status_at
            self._warn_bounded(
                "timing",
                f"timing reason={reason} host_sequence={before.command_sequence} "
                f"command_age={command_age} status_age={status_age} drain_sec={now-began:.6f}"
            )
            self._release(reason)
            self._publish_link_state()
        if self.core.can_drive and self.command_stream.enabled:
            sent = self.command_stream.tick(
                now, lambda payload: self.transport.submit(CMD_SET_VELOCITY, payload, coalesce=True)
            )
            if sent:
                self.core.record_wire_sequence(self.command_stream.sequence)
        if self.core.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK:
            self.command_stream.tick_disabled(
                now,
                lambda payload: self.transport.submit(
                    CMD_SET_VELOCITY, payload, critical=True, coalesce=True
                ),
            )
        if (
            self.command_stream.enabled
            and self.pending_active_ack_started_at is not None
            and now - self.pending_active_ack_started_at > self.ack_timeout
        ):
            self.core.require_rearm(REARM_ACK_TIMEOUT)
            self._release("command ACK timeout")
            self._publish_link_state()
        if (
            self.core.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK
            and self.pending_disable_started_at is not None
            and now - self.pending_disable_started_at > self.ack_timeout
        ):
            self.core.require_rearm(REARM_ACK_TIMEOUT)
            self._begin_wire_session(request_info=True)

    def _on_connected(self) -> None:
        self._begin_wire_session(request_info=True)

    def _begin_wire_session(self, *, request_info: bool) -> None:
        self.transport.clear_pending_observations()
        session = secrets.randbits(64) or 1
        self.core.on_connected(session)
        self.command_stream = CommandStream(
            float(self.get_parameter("cmd_timeout").value),
            float(self.get_parameter("drive_keepalive_sec").value),
            session,
        )
        self.last_ack_evidence_at = time.monotonic()
        self.pending_disable_started_at = None
        self.pending_active_ack_started_at = None
        self.last_applied_sequence = None
        self.last_status = None
        self.enabled_mask = None
        self.last_imu_identity = None
        if request_info:
            self.transport.submit(CMD_GET_INFO, encode_get_info_payload(), critical=True)
        for _ in range(3):
            self.command_stream.release(
                lambda payload: self.transport.submit(CMD_SET_VELOCITY, payload, critical=True)
            )
        self.core.record_wire_sequence(self.command_stream.sequence)
        self._publish_link_state()

    def _handle_frame(self, cmd: int, payload: bytes, received_at: float) -> None:
        if cmd == CMD_HELLO:
            hello = decode_hello_payload(payload)
            if hello:
                previous_hello = self.core.snapshot.hello
                compatible = self.core.on_hello(hello)
                if previous_hello is not None and previous_hello != hello:
                    self._release("firmware identity changed", force_fresh=True)
                    self._begin_wire_session(request_info=True)
                    return
                if not compatible:
                    self._release("incompatible Upper-v3 HELLO")
                msg = FirmwareInfo()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.wire_session_id = self.core.snapshot.wire_session_id
                msg.protocol_version, msg.schema_version = hello.version, hello.schema_version
                msg.capabilities, msg.firmware_commit = hello.capabilities, hello.firmware_commit
                msg.hardware_revision, msg.parameter_crc32 = (
                    hello.hardware_revision,
                    hello.parameter_crc32,
                )
                msg.simulated = False
                self.firmware_pub.publish(msg)
                self._publish_link_state()
                if not compatible:
                    self.get_logger().error("incompatible Upper-v3 HELLO")
            return
        if cmd == CMD_STATUS:
            status = decode_status_payload(payload)
            if status is not None:
                if self.enabled_mask is not None and status.motor_enabled_mask != self.enabled_mask:
                    self.core.require_rearm(REARM_TRANSPORT)
                    self._begin_wire_session(request_info=True)
                    return
                disposition = self.core.on_status(status, received_at)
                if disposition is StatusDisposition.NEW:
                    self.enabled_mask = status.motor_enabled_mask
                    self.last_status = status
                    self._record_ack_evidence(status, received_at)
                    self._publish_status(status, disposition, received_at)
                elif disposition is StatusDisposition.OUT_OF_ORDER:
                    self.core.require_rearm(REARM_ACK_TIMEOUT)
                    self._begin_wire_session(request_info=True)
            return
        if cmd == CMD_IMU_STATUS:
            imu = decode_imu_status_payload(payload)
            if imu is not None:
                identity = (
                    self.core.snapshot.wire_session_id,
                    int(imu.sample_count),
                    int(imu.sensor_time),
                )
                if imu_identity_is_new(identity, self.last_imu_identity):
                    self.last_imu_identity = identity
                    self._publish_imu(imu, received_at)
            return
        if cmd == CMD_DIAGNOSTIC:
            decode_diagnostic_payload(payload)

    def _record_ack_evidence(self, status, received_at: float) -> None:
        required = ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED
        applied = status.last_applied_sequence
        sent = self.core.snapshot.wire_sent_sequence
        if (
            status.last_received_session_id == self.command_stream.session_id
            and self.core.snapshot.wire_session_id == self.command_stream.session_id
            and status.last_received_sequence == applied
            and (applied == sent or sequence_is_forward(sent, applied))
            and status.command_ack_flags & required == required
            and not status.command_ack_flags & ACK_REJECTED
            and status.last_reject_reason == 0
        ):
            if (
                self.core.state is BridgeState.WIRE_REARM_READY
                and applied == self.command_stream.sequence
            ):
                self.pending_disable_started_at = None
            if self.last_applied_sequence is None:
                if applied != sent:
                    return
            elif not sequence_is_forward(applied, self.last_applied_sequence):
                return
            self.last_applied_sequence = applied
            self.last_ack_evidence_at = float(received_at)
            if self.command_stream.enabled and self.pending_active_ack_started_at is not None:
                # A forward ACK may lag a changing target; let subsequent ACKs
                # catch up.  Repeated ACKs cannot renew this waiting budget.
                self.pending_active_ack_started_at = (
                    None if applied == self.command_stream.sequence else float(received_at)
                )

    def _receive_stamp(self, received_at: float):
        # Queue residence must remain visible to downstream freshness checks.
        now = self.get_clock().now()
        age_ns = round(max(time.monotonic() - received_at, 0.0) * 1e9)
        return (now - Duration(nanoseconds=age_ns)).to_msg()

    def _publish_status(self, status, disposition: StatusDisposition, received_at: float) -> None:
        receive_stamp = self._receive_stamp(received_at)
        observation = WheelObservation()
        observation.header.stamp = receive_stamp
        observation.header.frame_id = self.base_frame_id
        observation.schema_version = WheelObservation.SCHEMA_VERSION
        observation.transport_session_id = self.core.snapshot.wire_session_id
        observation.reset_generation = self.core.snapshot.reset_generation
        observation.status_sequence = status.status_sequence
        observation.mcu_sample_time_ms = status.sample_timestamp_ms
        observation.encoder_count = list(status.encoder_count)
        observation.wheel_speed_mps = list(status.motor_speed_mps)
        observation.wheel_target_mps = list(status.motor_target_mps)
        observation.motor_current_a = list(status.motor_current_a)
        observation.motor_output_permille = list(status.motor_output_permille)
        observation.motor_enabled_mask = status.motor_enabled_mask
        observation.speed_valid_mask = status.motor_speed_valid_mask
        observation.encoder_anomaly_mask = status.encoder_anomaly_mask
        observation.side_consistency_flags = status.side_consistency_flags
        observation.comm_health_flags = status.comm_health_flags
        observation.status_flags = status.status_flags
        observation.error_flags = status.error_flags
        observation.latched_error_flags = status.latched_error_flags
        self.wheel_observation_pub.publish(observation)
        if status.status_flags & 0x03:
            self._release("firmware ESTOP/fault-stop")
        if (
            self.core.snapshot.rearm_required
            and not status.status_flags & 0x03
            and self.core.state in (BridgeState.WAIT_STATUS, BridgeState.WAIT_FAULT_CLEAR)
        ):
            self._release("post-clear wire disable", force_fresh=True)
        if self.command_stream.enabled and (
            status.last_reject_reason != 0 or status.command_ack_flags & ACK_REJECTED
        ):
            self.core.require_rearm(REARM_FIRMWARE_REJECT)
            self._release(f"firmware rejected command: {status.last_reject_reason}")
        self.chassis_status_pub.publish(UInt32(data=status.error_flags))
        battery = BatteryState()
        battery.header.stamp = receive_stamp
        battery.voltage, battery.present = status.battery_voltage, status.battery_voltage > 1.0
        self.battery_pub.publish(battery)
        self.left_current_pub.publish(Float32(data=sum(status.motor_current_a[:2])))
        self.right_current_pub.publish(Float32(data=sum(status.motor_current_a[2:])))
        self._publish_chassis_states(status)

    def _publish_imu(self, imu, received_at: float) -> None:
        msg = ImuObservation()
        msg.header.stamp = self._receive_stamp(received_at)
        msg.header.frame_id = self.imu_frame_id
        msg.schema_version = ImuObservation.SCHEMA_VERSION
        msg.transport_session_id = self.core.snapshot.wire_session_id
        msg.mcu_sample_time_ms = imu.timestamp_ms
        msg.sensor_time = imu.sensor_time
        msg.sample_sequence = imu.sample_count
        msg.acceleration_g = list(imu.accel_g)
        msg.angular_velocity_dps = list(imu.gyro_corrected_dps)
        msg.euler_deg = list(imu.euler_deg)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = imu.quaternion
        msg.quality_flags = imu.quality_flags
        msg.quality_counters = list(imu.quality_counters)
        msg.status_flags = imu.status_flags
        msg.temperature_c = imu.temperature_c
        self.imu_observation_pub.publish(msg)

    def _release(
        self, reason: str, *, force_fresh: bool = False, publish_state: bool = True
    ) -> None:
        was_pending = self.core.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK
        pending_identity = (
            self.core.snapshot.wire_session_id,
            self.core.snapshot.wire_sent_sequence,
        )
        now = time.monotonic()
        if self.command_stream.release(
            lambda payload: self.transport.submit(CMD_SET_VELOCITY, payload, critical=True),
            now,
            force_new_sequence=force_fresh,
        ):
            self.release_count += 1
            self.pending_active_ack_started_at = None
            self.core.on_disable_sent(self.command_stream.sequence)
            if self.core.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK:
                # DDS disable retries reuse one wire command.  Their arrival
                # must not extend the ACK budget for that same identity.
                identity = (self.command_stream.session_id, self.command_stream.sequence)
                if (
                    not was_pending
                    or identity != pending_identity
                    or self.pending_disable_started_at is None
                ):
                    self.pending_disable_started_at = now
            if publish_state:
                self._publish_link_state()
        if reason:
            self._warn_bounded("release", reason)

    def _warn_bounded(self, category: str, message: str) -> None:
        # Humble's logger discovers the Python caller before applying its own
        # throttle. Bound calls before entering that expensive path; safety
        # decisions, state publication and counters above are never throttled.
        now = time.monotonic()
        if now < self._warning_next_at.get(category, 0.0):
            self.warning_suppressed_count += 1
            return
        self._warning_next_at[category] = now + 1.0
        self.get_logger().warn(message)

    def on_estop(self, request, response):
        if not request.data:
            response.success, response.message = False, "remote ESTOP release is forbidden"
            return response
        self.core.require_rearm(REARM_ESTOP)
        self._release("remote ESTOP requested")
        response.success = self.transport.submit(
            CMD_ESTOP, encode_estop_payload(True), critical=True
        )
        response.message = "estop queued" if response.success else "queue full"
        return response

    def on_clear_fault(self, _request, response):
        response.success = self.transport.submit(
            CMD_CLEAR_FAULT, encode_clear_fault_payload(), critical=True
        )
        if response.success:
            self.core.on_clear_fault_requested()
            self._publish_link_state()
        response.message = "clear fault queued" if response.success else "queue full"
        return response

    def on_line_ctrl(self, request, response):
        response.success = self.transport.submit(
            CMD_LINE_CTRL, encode_line_ctrl_payload(request.data), critical=True
        )
        response.message = "line control queued" if response.success else "queue full"
        return response

    def on_get_info(self, _request, response):
        response.success = self.transport.submit(
            CMD_GET_INFO, encode_get_info_payload(), critical=True
        )
        response.message = "get info queued" if response.success else "queue full"
        return response

    def _publish_link_state(self) -> None:
        s = self.core.snapshot
        status = self.last_status
        msg = ChassisLinkState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.link_state = int(s.state)
        msg.state_generation = self.core.state_generation
        msg.protocol_version = 3
        msg.protocol_compatible = s.protocol_compatible
        msg.incompatibility_flags = s.incompatibility_flags
        msg.wire_session_id, msg.wire_sent_sequence = (
            s.wire_session_id,
            s.wire_sent_sequence,
        )
        msg.firmware_ack_available = bool(
            status is not None
            and s.state is not BridgeState.DISCONNECTED
            and status.last_received_session_id == s.wire_session_id
        )
        msg.firmware_session_id, msg.firmware_received_sequence = (
            (status.last_received_session_id if status is not None else 0),
            (status.last_received_sequence if status is not None else 0),
        )
        msg.firmware_applied_sequence, msg.firmware_reject_reason = (
            (status.last_applied_sequence if status is not None else 0),
            (status.last_reject_reason if status is not None else 0),
        )
        msg.wire_rearm_required = s.rearm_required
        msg.wire_rearm_reason_flags = s.rearm_reason_flags
        now = time.monotonic()
        msg.status_age_ms = (
            ChassisLinkState.AGE_INVALID
            if s.last_status_at is None or s.state is BridgeState.DISCONNECTED
            else min(int(max(now - s.last_status_at, 0.0) * 1000.0), 0xFFFFFFFF)
        )
        msg.command_ack_age_ms = (
            min(int(max(now - self.last_ack_evidence_at, 0.0) * 1000.0), 0xFFFFFFFF)
            if msg.firmware_ack_available and self.last_applied_sequence is not None
            else ChassisLinkState.AGE_INVALID
        )
        msg.config_sha256 = self.config_sha256
        self.link_state_pub.publish(msg)

    def _publish_chassis_states(self, status) -> None:
        self._publish_link_state()
        s = self.core.snapshot

        control = FirmwareControlState()
        control.header.stamp = self.get_clock().now().to_msg()
        control.firmware_control_source = status.control_source
        control.status_flags = status.status_flags
        control.error_flags = status.error_flags
        control.latched_error_flags = status.latched_error_flags
        control.estop_active = bool(status.status_flags & 0x01)
        control.fault_stop_active = bool(status.status_flags & 0x02)
        control.line_control_enabled = bool(status.status_flags & 0x04)
        control.status_sequence = status.status_sequence
        control.wire_session_id = s.wire_session_id
        self.firmware_control_pub.publish(control)

    def publish_diagnostics(self) -> None:
        snapshot = self.core.snapshot
        now = time.monotonic()
        status_age = None if snapshot.last_status_at is None else now - snapshot.last_status_at
        level = DiagnosticStatus.OK
        message = snapshot.state.name.lower()
        if (
            snapshot.state in (BridgeState.DISCONNECTED, BridgeState.WAIT_HELLO)
            or snapshot.hello is None
            or not snapshot.protocol_compatible
        ):
            level, message = DiagnosticStatus.ERROR, "v3 HELLO/STATUS/ACK unavailable"
        elif snapshot.rearm_required:
            level, message = DiagnosticStatus.WARN, "transport rearm required"
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            level=level,
            name="stm32_bridge/v3",
            message=message,
            hardware_id=(snapshot.hello.firmware_commit if snapshot.hello else "unknown"),
        )
        values = {
            "protocol": 3,
            "config_sha256": self.config_sha256,
            "state": snapshot.state.name,
            "status_age": "none" if status_age is None else f"{status_age:.3f}",
            "wire_session": snapshot.wire_session_id,
            "wire_sequence": self.command_stream.sequence,
            "rearm_required": snapshot.rearm_required,
            "rearm_reason_flags": snapshot.rearm_reason_flags,
            "release_count": self.release_count,
            "invalid_command_count": self.invalid_command_count,
            "dds_deadline_miss_count": self.command_deadline_miss_count,
            "publisher_liveliness_lost_count": self.command_publisher_lost_count,
            "command_queue_drop_count": self.command_queue_drop_count,
            "rate_limited_warning_count": self.warning_suppressed_count,
            "command_timeout_sec": self.get_parameter("cmd_timeout").value,
            "status_timeout_sec": self.status_timeout,
            "serial_rx_frames": self.transport.stats.rx_frames,
            "serial_tx_frames": self.transport.stats.tx_frames,
            "serial_crc_errors": self.transport.stats.rx_crc_errors,
            "serial_rx_queue_drops": self.transport.stats.rx_queue_drops,
            "serial_rx_coalesced_frames": self.transport.stats.rx_coalesced_frames,
            "serial_tx_queue_drops": self.transport.stats.tx_queue_drops,
        }
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def shutdown(self) -> None:
        try:
            # rclpy's signal handler may already have invalidated the context.
            self._release("", force_fresh=True, publish_state=False)
        finally:
            payload = self.command_stream.last_payload
            final_frame = (
                OutboundFrame(CMD_SET_VELOCITY, payload, critical=True)
                if payload is not None and not self.command_stream.enabled
                else None
            )
            self.shutdown_disable_written = self.transport.close(
                timeout_sec=0.2, final_frame=final_frame
            )
            # The ROS context can already be invalid; retain host write evidence
            # in the process log without publishing or claiming a firmware ACK.
            print(
                f"SERIAL_SHUTDOWN session={self.command_stream.session_id} "
                f"sequence={self.command_stream.sequence} "
                f"disable_written={self.shutdown_disable_written} applied=UNCONFIRMED",
                flush=True,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STM32BridgeV3()
    # All ROS callbacks use the mutually exclusive default group, and serial
    # I/O already has its own owner. Keep state-machine callbacks on one thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
