import math
import os
import select
import struct

import pytest
import serial

from stm32_robot_bridge.bridge_core import (
    BridgeCore,
    CommandDisposition,
    StatusDisposition,
)
from stm32_robot_bridge.bridge_state import BridgeState
from stm32_robot_bridge.framing import FrameParser, build_frame
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_SESSION_VALID,
    CMD_HELLO,
    CMD_SET_VELOCITY,
    CMD_STATUS,
    HELLO_PAYLOAD_SIZE,
    REQUIRED_CAPABILITIES,
    STATUS_PAYLOAD_SIZE,
    CommandStream,
    decode_hello_payload,
    decode_status_payload,
)
from stm32_robot_bridge.serial_transport import TransportStats, write_all


def hello_payload(*, capabilities=REQUIRED_CAPABILITIES):
    payload = (
        bytes((3, 1))
        + struct.pack("<I", capabilities)
        + bytes.fromhex("11" * 20)
        + struct.pack("<II", 2, 0x12345678)
    )
    assert len(payload) == HELLO_PAYLOAD_SIZE
    return payload


def status_payload(
    sequence,
    wire_session,
    *,
    flags=0,
    received=1,
    applied=1,
    ack_flags=ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED,
):
    payload = bytearray(STATUS_PAYLOAD_SIZE)
    payload[:4] = bytes((3, flags, 2, 0x0F))
    struct.pack_into("<H", payload, 12, 12000)
    payload[62:65] = bytes((0x0F, 0, 0))
    struct.pack_into(
        "<IIQII", payload, 65, sequence, sequence * 20, wire_session, received, applied
    )
    payload[91] = ack_flags
    return bytes(payload)


class FakeSTM32V3:
    def __init__(self, master_fd):
        self.master_fd = master_fd
        self.parser = FrameParser()

    def send(self, command, payload):
        os.write(self.master_fd, build_frame(command, payload))

    def receive_frames(self):
        ready, _, _ = select.select([self.master_fd], [], [], 0.5)
        assert ready, "timed out waiting for upper serial frame"
        return self.parser.feed(os.read(self.master_fd, 4096))


def host_receive(device, parser):
    ready, _, _ = select.select([device.fileno()], [], [], 0.5)
    assert ready, "timed out waiting for fake STM32 frame"
    return parser.feed(device.read(device.in_waiting or 1))


def decode_velocity(frames):
    assert len(frames) == 1 and frames[0][0] == CMD_SET_VELOCITY
    version, vx, wz, enable, mode, session, sequence = struct.unpack("<BffBBQI", frames[0][1])
    assert version == 3 and math.isfinite(vx) and math.isfinite(wz)
    return vx, wz, enable, mode, session, sequence


@pytest.mark.serial
@pytest.mark.skipif(not hasattr(os, "openpty"), reason="PTY requires real POSIX os.openpty")
def test_pty_upper_v3_hello_drive_timeout_fault_and_reconnect_rearm():
    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    os.close(slave_fd)
    device = serial.Serial(slave_name, 115200, timeout=0.0, write_timeout=0.2)
    try:
        fake = FakeSTM32V3(master_fd)
        host_parser = FrameParser()
        stats = TransportStats()
        core = BridgeCore(
            hard_max_linear_mps=0.45,
            hard_max_angular_radps=1.5,
            command_timeout_sec=0.15,
            status_timeout_sec=0.25,
            max_command_age_sec=0.15,
        )
        core.on_connected(7)

        fake.send(CMD_HELLO, hello_payload())
        frames = host_receive(device, host_parser)
        assert core.on_hello(decode_hello_payload(frames[0][1]))
        fake.send(CMD_STATUS, status_payload(1, 7, received=0, applied=0))
        frames = host_receive(device, host_parser)
        status = decode_status_payload(frames[0][1])
        assert core.on_status(status, 1.0) is StatusDisposition.NEW
        assert core.snapshot.state is BridgeState.WAIT_STATUS
        core.on_disable_sent(1)
        fake.send(CMD_STATUS, status_payload(2, 7))
        status = decode_status_payload(host_receive(device, host_parser)[0][1])
        assert core.on_status(status, 1.005) is StatusDisposition.NEW
        assert core.snapshot.state is BridgeState.WIRE_REARM_READY

        core.accept_command(
            vx=0.0,
            wz=0.0,
            enable=False,
            source=0,
            session_id=99,
            sequence=1,
            command_stamp_sec=0.0,
            now_ros_sec=0.0,
            now_monotonic=1.006,
        )

        decision = core.accept_command(
            vx=0.8,
            wz=-2.0,
            enable=True,
            source=3,
            session_id=100,
            sequence=1,
            command_stamp_sec=10.0,
            now_ros_sec=10.01,
            now_monotonic=1.01,
        )
        assert decision.disposition is CommandDisposition.ACCEPT_ENABLE
        command = decision.validated_command
        assert command is not None
        stream = CommandStream(0.15, 0.05, core.snapshot.wire_session_id)
        stream.update_command(command.vx, command.wz, 1.01)
        sender = lambda payload: write_all(device, build_frame(CMD_SET_VELOCITY, payload), stats)
        assert stream.tick(1.01, sender)
        velocity = decode_velocity(fake.receive_frames())
        assert velocity[:3] == (pytest.approx(0.45), -1.5, 1)
        assert velocity[4] == 7

        assert stream.tick(1.20, sender)
        assert decode_velocity(fake.receive_frames())[:3] == (0.0, 0.0, 0)
        assert core.tick(1.20) == (True, "command_timeout")
        blocked = core.accept_command(
            vx=0.1,
            wz=0.0,
            enable=True,
            source=3,
            session_id=100,
            sequence=2,
            command_stamp_sec=10.02,
            now_ros_sec=10.03,
            now_monotonic=1.21,
        )
        assert blocked.disposition is CommandDisposition.REQUIRE_REARM

        core.on_disconnected()
        core.on_connected(8)
        fake.send(CMD_HELLO, hello_payload())
        assert core.on_hello(decode_hello_payload(host_receive(device, host_parser)[0][1]))
        fake.send(CMD_STATUS, status_payload(1, 8, flags=1))
        status = decode_status_payload(host_receive(device, host_parser)[0][1])
        assert core.on_status(status, 2.0) is StatusDisposition.NEW
        assert core.snapshot.state is BridgeState.FAULT_LATCHED
        assert core.snapshot.rearm_required
    finally:
        device.close()
        os.close(master_fd)
