import struct

import pytest

from stm32_robot_bridge.framing import FrameParser, build_frame, crc8
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    CMD_STATUS,
    DIAGNOSTIC_PAYLOAD_SIZE,
    HELLO_PAYLOAD_SIZE,
    IMU_STATUS_PAYLOAD_SIZE,
    PROTOCOL_VERSION,
    STATUS_PAYLOAD_SIZE,
    CommandStream,
    decode_diagnostic_payload,
    decode_hello_payload,
    decode_imu_status_payload,
    decode_status_payload,
    encode_clear_fault_payload,
    encode_estop_payload,
    encode_get_info_payload,
    encode_line_ctrl_payload,
    encode_velocity_payload,
    imu_identity_is_new,
    sequence_is_forward,
)


def make_status():
    payload = bytearray(STATUS_PAYLOAD_SIZE)
    payload[0:4] = bytes((3, 0, 2, 0x0F))
    struct.pack_into("<I", payload, 4, 0)
    struct.pack_into("<I", payload, 8, 0)
    struct.pack_into("<H", payload, 12, 12000)
    struct.pack_into("<4h", payload, 14, 100, 100, 120, 120)
    struct.pack_into("<4i", payload, 22, 10, 10, 12, 12)
    struct.pack_into("<4H", payload, 38, 100, 100, 100, 100)
    struct.pack_into("<4h", payload, 46, 100, 100, 120, 120)
    struct.pack_into("<4h", payload, 54, 1, 1, 1, 1)
    payload[62:65] = bytes((0x0F, 0, 0))
    struct.pack_into("<I", payload, 65, 7)
    struct.pack_into("<I", payload, 69, 1234)
    struct.pack_into("<Q", payload, 73, 99)
    struct.pack_into("<I", payload, 81, 5)
    struct.pack_into("<I", payload, 85, 5)
    payload[91] = ACK_APPLIED
    return bytes(payload)


def test_crc_matches_existing_golden_algorithm():
    assert crc8(bytes.fromhex("0b0100000000000000000002")) == 0xBA


def test_v3_velocity_layout_and_sequence_order():
    payload = encode_velocity_payload(0.1, -0.2, True, 2, 0x1122334455667788, 9)
    assert len(payload) == 23 and payload[0] == PROTOCOL_VERSION
    assert struct.unpack_from("<Q", payload, 11)[0] == 0x1122334455667788
    assert sequence_is_forward(0, 0xFFFFFFFF)
    assert not sequence_is_forward(4, 5)


def test_imu_identity_orders_sample_count_independently_of_sensor_clock():
    previous = (7, 10, 100)
    assert not imu_identity_is_new(previous, previous)
    assert imu_identity_is_new((7, 11, 99), previous)
    assert not imu_identity_is_new((7, 9, 101), previous)
    assert imu_identity_is_new((7, 11, 101), previous)
    assert imu_identity_is_new((8, 0, 0), previous)
    assert imu_identity_is_new((7, 11, 0), previous)
    assert imu_identity_is_new((7, 11, 0), (7, 10, 0))
    assert imu_identity_is_new((7, 11, 0), (7, 10, 0x00FFFF00))
    assert imu_identity_is_new((7, 0, 0), (7, 0xFFFFFFFF, 0x00FFFF00))


def test_v3_status_and_framing_fragmentation():
    frame = build_frame(CMD_STATUS, make_status())
    parser = FrameParser()
    assert parser.feed(frame[:9]) == []
    frames = parser.feed(frame[9:])
    assert frames[0][0] == CMD_STATUS
    status = decode_status_payload(frames[0][1])
    assert status.status_sequence == 7
    assert status.last_received_session_id == 99
    assert status.encoder_count == (10, 10, 12, 12)


def test_hello_layout():
    payload = (
        bytes((3, 1)) + struct.pack("<I", 31) + bytes.fromhex("11" * 20) + struct.pack("<II", 2, 3)
    )
    assert len(payload) == HELLO_PAYLOAD_SIZE
    hello = decode_hello_payload(payload)
    assert hello.capabilities == 31 and hello.firmware_commit == "11" * 20


def test_v3_small_commands_and_invalid_payloads_fail_closed():
    assert encode_estop_payload(True) == b"\x03\x01"
    with pytest.raises(ValueError, match="ESTOP"):
        encode_estop_payload(False)
    assert encode_line_ctrl_payload(False) == b"\x03\x00"
    assert encode_clear_fault_payload() == encode_get_info_payload() == b"\x03"
    with pytest.raises(ValueError, match="finite"):
        encode_velocity_payload(float("nan"), 0, True, 2, 1, 1)
    assert decode_hello_payload(b"") is None
    assert decode_status_payload(bytes(STATUS_PAYLOAD_SIZE)) is None
    assert decode_imu_status_payload(bytes(IMU_STATUS_PAYLOAD_SIZE)) is None
    assert decode_diagnostic_payload(bytes(DIAGNOSTIC_PAYLOAD_SIZE)) is None


def test_v3_imu_and_diagnostic_layouts():
    imu = bytearray(IMU_STATUS_PAYLOAD_SIZE)
    imu[0] = 3
    struct.pack_into("<3f", imu, 1, 1.0, 2.0, 3.0)
    struct.pack_into("<3f", imu, 13, 4.0, 5.0, 6.0)
    struct.pack_into("<3f", imu, 25, 7.0, 8.0, 9.0)
    struct.pack_into("<4f", imu, 37, 10.0, 11.0, 12.0, 13.0)
    struct.pack_into("<4I", imu, 53, 14, 15, 16, 17)
    struct.pack_into("<7I", imu, 69, *range(18, 25))
    imu[97], imu[98] = 9, 250
    decoded_imu = decode_imu_status_payload(bytes(imu))
    assert decoded_imu.accel_g == pytest.approx((1, 2, 3))
    assert decoded_imu.quaternion == pytest.approx((11, 12, 13, 10))
    assert decoded_imu.temperature_c == -6

    diagnostic = bytearray(DIAGNOSTIC_PAYLOAD_SIZE)
    diagnostic[0:4] = bytes((3, 1, 1, 9))
    struct.pack_into("<IIH", diagnostic, 4, 2, 3, 4)
    struct.pack_into("<III", diagnostic, 16, 5, 6, 7)
    decoded_diagnostic = decode_diagnostic_payload(bytes(diagnostic))
    assert decoded_diagnostic.task_timeout_mask == 4
    assert decoded_diagnostic.uptime_ms == 7


def test_command_stream_keepalive_timeout_and_failed_send():
    stream = CommandStream(timeout_sec=0.15, keepalive_sec=0.05, session_id=0)
    sent = []
    assert stream.session_id == 1
    assert not stream.tick(0.0, sent.append)
    stream.update_command(0.1, 0.2, 1.0)
    sequence = stream.sequence
    stream.update_command(0.1, 0.2, 1.01)
    assert stream.sequence == sequence
    assert stream.tick(1.01, sent.append)
    assert not stream.tick(1.02, sent.append)
    assert stream.tick(1.07, sent.append)
    assert stream.tick(1.30, sent.append)
    assert not stream.enabled
    stream.update_command(0.3, 0.0, 2.0)
    assert not stream.tick(2.0, lambda _payload: False)
    assert not stream.release(lambda _payload: False)
    assert stream.release(sent.append)
    stream.clear_command()
    assert stream.last_command_time is None


def test_fresh_identical_commands_renew_wire_sequence_once_per_second():
    stream = CommandStream(timeout_sec=0.15, keepalive_sec=0.05, session_id=7)
    sent = []
    previous_sequence = 0
    renewals = []
    for step in range(101):
        now = step * 0.05
        stream.update_command(0.2, 0.0, now)
        if stream.sequence != previous_sequence:
            renewals.append(now)
            previous_sequence = stream.sequence
        stream.tick(now, sent.append)
        assert stream.enabled
    assert stream.last_command_time == pytest.approx(5.0)
    assert renewals == pytest.approx([0, 1, 2, 3, 4, 5])
    # Binary floating point can occasionally place an exact 50 ms tick just
    # below the keepalive boundary; the stream must nevertheless remain live.
    assert len(sent) >= 50


def test_keepalive_without_fresh_command_cannot_renew_sequence_or_lease():
    stream = CommandStream(timeout_sec=0.15, keepalive_sec=0.05, session_id=7)
    sent = []
    stream.update_command(0.2, 0, 0)
    assert stream.tick(0, sent.append)
    sequence = stream.sequence
    assert stream.tick(0.05, sent.append)
    assert stream.sequence == sequence
    assert stream.tick(0.16, sent.append)
    assert not stream.enabled
    assert stream.sequence == sequence + 1
