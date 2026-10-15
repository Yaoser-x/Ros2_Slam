"""Normative upper protocol v3 codecs and command-session stream."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from .framing import FrameParser, build_frame, crc8

PROTOCOL_VERSION = 3
CMD_SET_VELOCITY, CMD_ESTOP, CMD_LINE_CTRL, CMD_CLEAR_FAULT, CMD_GET_INFO = range(1, 6)
CMD_HELLO, CMD_STATUS, CMD_DIAGNOSTIC, CMD_IMU_STATUS = 0x80, 0x81, 0x82, 0x83
VELOCITY_PAYLOAD_SIZE, HELLO_PAYLOAD_SIZE = 23, 34
STATUS_PAYLOAD_SIZE, DIAGNOSTIC_PAYLOAD_SIZE, IMU_STATUS_PAYLOAD_SIZE = 92, 28, 99
STATUS_FLAG_ESTOP, STATUS_FLAG_FAULT_STOP = 1 << 0, 1 << 1
IMU_FLAG_ONLINE, IMU_FLAG_CALIBRATED, IMU_FLAG_ERROR, IMU_FLAG_SENSOR_TIME = 1, 2, 4, 8
CAP_COMMAND_SESSION_ACK, CAP_STATUS_SAMPLE_TIME = 1, 2
CAP_IMU_FIELD_QUALITY, CAP_SIDE_CONSISTENCY, CAP_BUILD_IDENTITY = 4, 8, 16
REQUIRED_CAPABILITIES = 0x1F
ACK_SESSION_VALID, ACK_RECEIVED, ACK_APPLIED = 1, 2, 4
ACK_DUPLICATE_KEEPALIVE, ACK_REJECTED = 8, 16
REJECT_NONE, REJECT_MALFORMED, REJECT_UNSUPPORTED_VERSION = 0, 1, 2
REJECT_NON_FINITE, REJECT_INVALID_MODE, REJECT_STALE_SESSION = 3, 4, 5
REJECT_OUT_OF_ORDER, REJECT_FAULT_ACTIVE, REJECT_SOURCE_NOT_PERMITTED = 6, 7, 8


@dataclass(frozen=True)
class HelloPayload:
    version: int
    schema_version: int
    capabilities: int
    firmware_commit: str
    hardware_revision: int
    parameter_crc32: int


@dataclass(frozen=True)
class StatusPayload:
    version: int
    status_flags: int
    control_source: int
    motor_enabled_mask: int
    error_flags: int
    latched_error_flags: int
    battery_voltage: float
    motor_speed_mps: Tuple[float, float, float, float]
    encoder_count: Tuple[int, int, int, int]
    motor_current_a: Tuple[float, float, float, float]
    motor_target_mps: Tuple[float, float, float, float]
    motor_output_permille: Tuple[int, int, int, int]
    motor_speed_valid_mask: int
    encoder_anomaly_mask: int
    comm_health_flags: int
    status_sequence: int
    sample_timestamp_ms: int
    last_received_session_id: int
    last_received_sequence: int
    last_applied_sequence: int
    last_reject_reason: int
    side_consistency_flags: int
    command_ack_flags: int


@dataclass(frozen=True)
class ImuStatusPayload:
    version: int
    accel_g: Tuple[float, float, float]
    gyro_corrected_dps: Tuple[float, float, float]
    euler_deg: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    timestamp_ms: int
    sensor_time: int
    sample_count: int
    quality_flags: int
    quality_counters: Tuple[int, int, int, int, int, int, int]
    status_flags: int
    temperature_c: int


@dataclass(frozen=True)
class DiagnosticPayload:
    protocol_version: int
    schema_version: int
    post_done: int
    imu_status_flags: int
    post_error_flags: int
    adc_invalid_reason_flags: int
    task_timeout_mask: int
    imu_quality_flags: int
    reset_reason_flags: int
    uptime_ms: int


def encode_velocity_payload(
    vx: float, wz: float, enable: bool, mode: int, session_id: int, sequence: int
) -> bytes:
    if not math.isfinite(float(vx)) or not math.isfinite(float(wz)):
        raise ValueError("velocity must be finite")
    return struct.pack(
        "<BffBBQI",
        PROTOCOL_VERSION,
        float(vx),
        float(wz),
        bool(enable),
        int(mode) & 0xFF,
        int(session_id) & 0xFFFFFFFFFFFFFFFF,
        int(sequence) & 0xFFFFFFFF,
    )


def encode_estop_payload(enabled: bool) -> bytes:
    if not enabled:
        raise ValueError("remote ESTOP release is forbidden")
    return bytes((PROTOCOL_VERSION, 1))


def encode_line_ctrl_payload(enabled: bool) -> bytes:
    return bytes((PROTOCOL_VERSION, bool(enabled)))


def encode_clear_fault_payload() -> bytes:
    return bytes((PROTOCOL_VERSION,))


def encode_get_info_payload() -> bytes:
    return bytes((PROTOCOL_VERSION,))


def decode_hello_payload(payload: bytes) -> Optional[HelloPayload]:
    if len(payload) != HELLO_PAYLOAD_SIZE or payload[0] != PROTOCOL_VERSION:
        return None
    return HelloPayload(
        payload[0],
        payload[1],
        struct.unpack_from("<I", payload, 2)[0],
        payload[6:26].hex(),
        struct.unpack_from("<I", payload, 26)[0],
        struct.unpack_from("<I", payload, 30)[0],
    )


def decode_status_payload(payload: bytes) -> Optional[StatusPayload]:
    if len(payload) != STATUS_PAYLOAD_SIZE or payload[0] != PROTOCOL_VERSION:
        return None
    return StatusPayload(
        payload[0],
        payload[1],
        payload[2],
        payload[3],
        struct.unpack_from("<I", payload, 4)[0],
        struct.unpack_from("<I", payload, 8)[0],
        struct.unpack_from("<H", payload, 12)[0] / 1000.0,
        tuple(v / 1000.0 for v in struct.unpack_from("<4h", payload, 14)),
        tuple(struct.unpack_from("<4i", payload, 22)),
        tuple(v / 1000.0 for v in struct.unpack_from("<4H", payload, 38)),
        tuple(v / 1000.0 for v in struct.unpack_from("<4h", payload, 46)),
        tuple(struct.unpack_from("<4h", payload, 54)),
        payload[62],
        payload[63],
        payload[64],
        struct.unpack_from("<I", payload, 65)[0],
        struct.unpack_from("<I", payload, 69)[0],
        struct.unpack_from("<Q", payload, 73)[0],
        struct.unpack_from("<I", payload, 81)[0],
        struct.unpack_from("<I", payload, 85)[0],
        payload[89],
        payload[90],
        payload[91],
    )


def decode_imu_status_payload(payload: bytes) -> Optional[ImuStatusPayload]:
    if len(payload) != IMU_STATUS_PAYLOAD_SIZE or payload[0] != PROTOCOL_VERSION:
        return None
    w, x, y, z = struct.unpack_from("<4f", payload, 37)
    return ImuStatusPayload(
        payload[0],
        tuple(struct.unpack_from("<3f", payload, 1)),
        tuple(struct.unpack_from("<3f", payload, 13)),
        tuple(struct.unpack_from("<3f", payload, 25)),
        (x, y, z, w),
        struct.unpack_from("<I", payload, 53)[0],
        struct.unpack_from("<I", payload, 57)[0],
        struct.unpack_from("<I", payload, 61)[0],
        struct.unpack_from("<I", payload, 65)[0],
        tuple(struct.unpack_from("<7I", payload, 69)),
        payload[97],
        struct.unpack_from("<b", payload, 98)[0],
    )


def decode_diagnostic_payload(payload: bytes) -> Optional[DiagnosticPayload]:
    if len(payload) != DIAGNOSTIC_PAYLOAD_SIZE or payload[0] != PROTOCOL_VERSION:
        return None
    return DiagnosticPayload(
        payload[0],
        payload[1],
        payload[2],
        payload[3],
        struct.unpack_from("<I", payload, 4)[0],
        struct.unpack_from("<I", payload, 8)[0],
        struct.unpack_from("<H", payload, 12)[0],
        struct.unpack_from("<I", payload, 16)[0],
        struct.unpack_from("<I", payload, 20)[0],
        struct.unpack_from("<I", payload, 24)[0],
    )


def sequence_is_forward(new: int, old: int) -> bool:
    delta = (int(new) - int(old)) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


def imu_identity_is_new(current, previous) -> bool:
    """Order samples by their u32 count, independently of sensor-time quality.

    BMI270 sensor_time is a raw 24-bit clock and can also be invalid.  Its
    wrap or absence must not discard a fresh sample with usable gyro fields.
    """
    if previous is None or int(current[0]) != int(previous[0]):
        return True
    return sequence_is_forward(current[1], previous[1])


class CommandStream:
    """Keepalives preserve identity; fresh host commands renew it every second."""

    def __init__(
        self, timeout_sec: float, keepalive_sec: float, session_id: int, mode: int = 2
    ) -> None:
        self.timeout_sec, self.keepalive_sec = max(float(timeout_sec), 0.0), max(
            float(keepalive_sec), 0.001
        )
        self.session_id, self.mode = int(session_id) & 0xFFFFFFFFFFFFFFFF or 1, int(mode) & 0xFF
        self.sequence, self.target, self.enabled = 0, (0.0, 0.0), False
        self.last_command_time: Optional[float] = None
        self.last_send_time: Optional[float] = None
        self.last_payload: Optional[bytes] = None
        self.last_sequence_time: Optional[float] = None

    def update_command(self, vx: float, wz: float, now_sec: float) -> None:
        # Sequence identity follows the values that actually cross the wire.
        target = struct.unpack("<ff", struct.pack("<ff", float(vx), float(wz)))
        if (
            target != self.target
            or not self.enabled
            or self.last_sequence_time is None
            or float(now_sec) - self.last_sequence_time >= 1.0
        ):
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            self.last_sequence_time = float(now_sec)
        self.target, self.enabled, self.last_command_time = target, True, float(now_sec)

    def clear_command(self) -> None:
        self.target, self.enabled, self.last_command_time = (0.0, 0.0), False, None
        self.last_sequence_time = None

    def tick(self, now_sec: float, send_payload: Callable[[bytes], object]) -> bool:
        if self.last_command_time is None:
            return False
        if float(now_sec) - self.last_command_time > self.timeout_sec:
            return self.release(send_payload, now_sec)
        payload = encode_velocity_payload(
            *self.target, True, self.mode, self.session_id, self.sequence
        )
        if (
            self.last_send_time is not None
            and payload == self.last_payload
            and float(now_sec) - self.last_send_time < self.keepalive_sec
        ):
            return False
        if send_payload(payload) is False:
            return False
        self.last_payload, self.last_send_time = payload, float(now_sec)
        return True

    def tick_disabled(self, now_sec: float, send_payload: Callable[[bytes], object]) -> bool:
        """Retransmit one ordered disable without changing its sequence."""
        payload = encode_velocity_payload(
            0.0, 0.0, False, self.mode, self.session_id, self.sequence
        )
        if (
            self.last_send_time is not None
            and payload == self.last_payload
            and float(now_sec) - self.last_send_time < self.keepalive_sec
        ):
            return False
        if send_payload(payload) is False:
            return False
        self.last_payload, self.last_send_time = payload, float(now_sec)
        return True

    def release(
        self,
        send_payload: Callable[[bytes], object],
        now_sec: Optional[float] = None,
        *,
        force_new_sequence: bool = False,
    ) -> bool:
        if force_new_sequence or self.enabled or self.sequence == 0:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        payload = encode_velocity_payload(
            0.0, 0.0, False, self.mode, self.session_id, self.sequence
        )
        if send_payload(payload) is False:
            return False
        self.target, self.enabled, self.last_command_time = (0.0, 0.0), False, None
        self.last_sequence_time = None
        self.last_payload, self.last_send_time = payload, (
            float(now_sec) if now_sec is not None else None
        )
        return True
