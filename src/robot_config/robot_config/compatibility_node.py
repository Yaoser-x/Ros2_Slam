#!/usr/bin/env python3
"""Publish the machine-readable runtime compatibility gate."""

import signal
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import (
    ChassisLinkState,
    FirmwareInfo,
    PlatformCompatibilityState,
    WheelObservation,
)

from robot_config.compatibility import evaluate_compatibility


class PlatformCompatibilityNode(Node):
    def __init__(self) -> None:
        super().__init__("platform_compatibility")
        defaults = {
            "firmware_info_topic": "chassis/firmware_info",
            "wheel_observation_topic": "wheel/observation",
            "chassis_link_state_topic": "chassis/link_state",
            "compatibility_state_topic": "platform/compatibility_state",
            "expected_firmware_commit": "366a0385290d526009e6cd3bbdaa7b74b2fecad6",
            "expected_hardware_revision": 0x00020000,
            "required_capabilities": 0x1F,
            "expected_parameter_crc32": 0,
            "expected_enabled_mask": 0b0110,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PlatformCompatibilityState,
            str(self.get_parameter("compatibility_state_topic").value),
            qos,
        )
        self.firmware: Optional[FirmwareInfo] = None
        self.link: Optional[ChassisLinkState] = None
        self.observation: Optional[WheelObservation] = None
        self.create_subscription(
            FirmwareInfo,
            str(self.get_parameter("firmware_info_topic").value),
            self._on_firmware,
            qos,
        )
        self.create_subscription(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            self._on_wheel,
            10,
        )
        self.create_subscription(
            ChassisLinkState,
            str(self.get_parameter("chassis_link_state_topic").value),
            self._on_link,
            qos,
        )

    def _on_firmware(self, msg) -> None:
        self.firmware = msg
        self._publish()

    def _on_wheel(self, msg) -> None:
        if self.link is None or int(msg.transport_session_id) != int(self.link.wire_session_id):
            return
        if int(msg.schema_version) != WheelObservation.SCHEMA_VERSION:
            self.observation = None
        else:
            self.observation = msg
        self._publish()

    def _on_link(self, msg) -> None:
        old_session = 0 if self.link is None else int(self.link.wire_session_id)
        self.link = msg
        disconnected = msg.link_state == ChassisLinkState.STATE_DISCONNECTED
        if disconnected or int(msg.wire_session_id) != old_session:
            self.observation = None
            if self.firmware is not None and (
                disconnected or int(self.firmware.wire_session_id) != int(msg.wire_session_id)
            ):
                self.firmware = None
        self._publish()

    def _publish(self) -> None:
        output = PlatformCompatibilityState()
        output.header.stamp = self.get_clock().now().to_msg()
        link = self.link
        firmware = self.firmware
        observation = self.observation
        wire_session = 0 if link is None else int(link.wire_session_id)
        output.wire_session_id = wire_session
        output.observation_session_id = (
            0 if observation is None else int(observation.transport_session_id)
        )
        output.observation_ready = observation is not None
        link_ready = bool(
            link is not None
            and link.link_state
            in (ChassisLinkState.STATE_WIRE_REARM_READY, ChassisLinkState.STATE_DRIVE_ACTIVE)
            and link.protocol_compatible
            and not link.wire_rearm_required
        )
        identity_ready = bool(
            firmware is not None and int(firmware.wire_session_id) == wire_session
        )
        if not link_ready or not identity_ready or observation is None or firmware is None:
            output.state = PlatformCompatibilityState.STATE_UNKNOWN
            output.reason = "waiting for current-session link, HELLO, and wheel observation"
        else:
            result = evaluate_compatibility(
                simulated=firmware.simulated,
                firmware_commit=firmware.firmware_commit,
                expected_firmware_commit=self.get_parameter("expected_firmware_commit").value,
                hardware_revision=firmware.hardware_revision,
                expected_hardware_revision=self.get_parameter("expected_hardware_revision").value,
                capabilities=firmware.capabilities,
                required_capabilities=self.get_parameter("required_capabilities").value,
                parameter_crc32=firmware.parameter_crc32,
                expected_parameter_crc32=self.get_parameter("expected_parameter_crc32").value,
                enabled_mask=observation.motor_enabled_mask,
                expected_enabled_mask=self.get_parameter("expected_enabled_mask").value,
                protocol_version=firmware.protocol_version,
                expected_protocol_version=3,
                schema_version=firmware.schema_version,
                expected_schema_version=1,
            )
            output.state = (
                PlatformCompatibilityState.STATE_SIMULATION
                if result.simulated
                else (
                    PlatformCompatibilityState.STATE_COMPATIBLE
                    if result.compatible
                    else PlatformCompatibilityState.STATE_INCOMPATIBLE
                )
            )
            output.mismatch_flags = result.mismatch_flags
            output.reason = result.reason
            output.permit_formal_odometry = result.compatible
            output.permit_navigation = result.compatible
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = PlatformCompatibilityNode()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        # Idle DDS waits must return so Python signal handlers can run.
        while node.context.ok():
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
