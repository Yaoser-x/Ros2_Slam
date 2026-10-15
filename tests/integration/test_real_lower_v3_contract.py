"""Cross-language contract tests against the production lower command owner.

Requires the sibling lower repository and a host C compiler.  No UART or ROS
runtime is used; parser, dispatcher, session tracker and lease owner are real C.
"""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from stm32_robot_bridge.bridge_core import BridgeCore, CommandDisposition
from stm32_robot_bridge.framing import FrameParser, build_frame
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    ACK_REJECTED,
    CMD_GET_INFO,
    CMD_SET_VELOCITY,
    CommandStream,
    decode_hello_payload,
    decode_imu_status_payload,
    decode_status_payload,
    encode_get_info_payload,
    encode_velocity_payload,
    imu_identity_is_new,
)


@pytest.fixture(scope="session")
def lower_contract_binary(tmp_path_factory):
    lower = Path(
        os.environ.get(
            "FIRMWARE_CONTRACT_DIR", Path(__file__).resolve().parents[2].parent / "lower"
        )
    )
    compiler = shutil.which("clang") or shutil.which("gcc")
    if not lower.is_dir() or compiler is None:
        pytest.fail("real-lower contract tests require sibling lower/ and clang or gcc")
    output = tmp_path_factory.mktemp("real-lower-v3") / "upper_v3_contract_harness"
    if os.name == "nt":
        output = output.with_suffix(".exe")
    include_dirs = (
        "Platform",
        "Service/communication",
        "Service/communication/internal",
        "Service/command_management",
        "Service/parameter_management",
        "Algorithm/calibration",
        "BSP/imu",
    )
    sources = (
        "tests/host/upper_v3_contract_harness.c",
        "tests/host/host_platform.c",
        "Service/communication/internal/frame_stream_parser.c",
        "Service/communication/internal/robot_link_protocol.c",
        "Service/communication/internal/remote_command_dispatcher.c",
        "Service/communication/internal/communication_session_tracker.c",
        "Service/communication/internal/communication_operation_mailbox.c",
        "Service/command_management/command_management_service.c",
    )
    result = subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
            *[f"-I{lower / directory}" for directory in include_dirs],
            *[str(lower / source) for source in sources],
            "-o",
            str(output),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return output


class LowerContract:
    def __init__(self, executable):
        self.process = subprocess.Popen(
            [str(executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    def request(self, command):
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline()
        assert response, "lower contract harness terminated before responding"
        result = json.loads(response)
        if result.get("status"):
            frames = FrameParser().feed(bytes.fromhex(result["status"]))
            assert len(frames) == 1
            result["decoded_status"] = decode_status_payload(frames[0][1])
        if result.get("hello"):
            frames = FrameParser().feed(bytes.fromhex(result["hello"]))
            result["decoded_hello"] = decode_hello_payload(frames[0][1])
        return result

    def wire(self, payload, now_ms, link=0, command=CMD_SET_VELOCITY):
        return self.request(f"wire {link} {now_ms} {build_frame(command, payload).hex()}")

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=5)
            assert self.process.returncode == 0, self.process.stderr.read()
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            self.process.stdout.close()
            self.process.stderr.close()


@pytest.fixture
def lower(lower_contract_binary):
    contract = LowerContract(lower_contract_binary)
    try:
        yield contract
    finally:
        contract.close()


def command(core, enable, epoch, sequence, now):
    return core.accept_command(
        vx=0.2 if enable else 0,
        wz=0,
        enable=enable,
        source=3 if enable else 0,
        session_id=epoch,
        sequence=sequence,
        command_stamp_sec=now,
        now_ros_sec=now,
        now_monotonic=now,
    )


def ready_pair(lower, session=7):
    core = BridgeCore(
        hard_max_linear_mps=0.45,
        hard_max_angular_radps=1.5,
        command_timeout_sec=0.15,
        status_timeout_sec=0.25,
        max_command_age_sec=0.15,
    )
    core.on_connected(session)
    hello = lower.wire(encode_get_info_payload(), 0, command=CMD_GET_INFO)
    assert core.on_hello(hello["decoded_hello"])
    core.on_status(hello["decoded_status"], 0)
    stream = CommandStream(0.15, 0.05, session)
    responses = []
    stream.release(lambda payload: responses.append(lower.wire(payload, 1)), 0.001)
    core.on_disable_sent(stream.sequence)
    core.on_status(responses[-1]["decoded_status"], 0.001)
    assert core.snapshot.rearm_required  # Wire ACK alone does not qualify host.
    assert command(core, False, 99, 1, 0.002).disposition is CommandDisposition.ACCEPT_DISABLE
    assert core.can_drive
    return core, stream


def test_ten_second_fresh_constant_command_survives_and_input_loss_stops(lower):
    core, stream = ready_pair(lower)
    responses = []
    sequence_times = []
    previous = stream.sequence
    for step in range(1, 202):
        now = step * 0.05
        accepted = command(core, True, 100, step, now)
        assert accepted.disposition is CommandDisposition.ACCEPT_ENABLE
        stream.update_command(accepted.validated_command.vx, 0, now)
        if stream.sequence != previous:
            sequence_times.append(now)
            previous = stream.sequence
        stream.tick(now, lambda payload: responses.append(lower.wire(payload, round(now * 1000))))
        response = lower.request(f"poll 0 {round(now * 1000)}")
        assert response["active"] == 1
        assert response["decoded_status"].last_reject_reason == 0
        core.on_status(response["decoded_status"], now)
    assert len(sequence_times) >= 10
    last_sequence = stream.sequence
    # No more fresh ROS commands: duplicates cannot renew the host lease.
    assert command(core, True, 100, 201, 10.06).disposition is CommandDisposition.IGNORE_DUPLICATE
    stream.tick(10.11, lambda payload: responses.append(lower.wire(payload, 10110)))
    assert stream.sequence == last_sequence
    assert core.tick(10.21) == (True, "command_timeout")
    stream.release(lambda payload: responses.append(lower.wire(payload, 10210)), 10.21)
    core.on_disable_sent(stream.sequence)
    core.on_status(responses[-1]["decoded_status"], 10.21)
    assert responses[-1]["active"] == 0
    assert command(core, True, 100, 202, 10.22).disposition is CommandDisposition.REQUIRE_REARM
    # Firmware watchdog also protects against losing the release frame.
    assert lower.request("poll 0 10450")["active"] == 0


@pytest.mark.parametrize("link", [0, 1])
@pytest.mark.parametrize("event", ["gate", "manual"])
def test_old_disable_cannot_rearm_after_safety_or_mode_recovery(lower, link, event):
    disable = encode_velocity_payload(0, 0, False, 2, 55, 1)
    accepted = lower.wire(disable, 10, link)
    assert accepted["decoded_status"].command_ack_flags & ACK_APPLIED
    if event == "gate":
        lower.request("gate 0 2 20")
        lower.request("gate 1 3 30")
    else:
        lower.request("policy 4 62 1 20")  # MANUAL: only PS2 permitted.
        lower.request("policy 62 62 2 30")  # AUTO restored, qualification revoked.
    replay = lower.wire(disable, 40, link)
    assert replay["decoded_status"].command_ack_flags & ACK_REJECTED
    enabled = lower.wire(encode_velocity_payload(0.2, 0, True, 2, 55, 2), 50, link)
    assert enabled["active"] == 0
    assert enabled["decoded_status"].command_ack_flags & ACK_REJECTED
    fresh = lower.wire(encode_velocity_payload(0, 0, False, 2, 55, 3), 60, link)
    assert fresh["decoded_status"].command_ack_flags & ACK_APPLIED
    enabled = lower.wire(encode_velocity_payload(0.2, 0, True, 2, 55, 4), 70, link)
    assert enabled["active"] == 1
    assert enabled["decoded_status"].command_ack_flags & ACK_APPLIED


def test_get_info_during_drive_is_idempotent_and_new_identity_closes_drive(lower):
    core, stream = ready_pair(lower)
    accepted = command(core, True, 100, 1, 0.05)
    stream.update_command(accepted.validated_command.vx, 0, 0.05)
    stream.tick(0.05, lambda payload: lower.wire(payload, 50))
    hello = lower.wire(encode_get_info_payload(), 60, command=CMD_GET_INFO)
    assert hello["active"] == 1
    assert core.on_hello(hello["decoded_hello"])
    core.on_status(hello["decoded_status"], 0.06)
    assert core.can_drive
    assert command(core, True, 100, 2, 0.07).disposition is CommandDisposition.ACCEPT_ENABLE
    changed = lower.request("identity 34 131072 305419896 80")
    assert core.on_hello(changed["decoded_hello"])
    assert not core.can_drive
    core.on_connected(8)
    # Old wire session disable cannot qualify the newly constructed bridge.
    assert core.snapshot.rearm_required


def test_real_lower_imu_encoding_preserves_fresh_samples_across_24bit_wrap(lower):
    previous = None
    for sample, clock, quality in [
        (10, 0x00FFFF00, 0),
        (11, 0, 0),
        (12, 256, 0),
        (13, 0, 8),
        (14, 0, 8),
    ]:
        response = lower.request(f"imu {sample} {clock} {quality} {sample * 10}")
        frames = FrameParser().feed(bytes.fromhex(response["imu"]))
        imu = decode_imu_status_payload(frames[0][1])
        current = (7, imu.sample_count, imu.sensor_time)
        assert imu_identity_is_new(current, previous)
        assert imu.quality_flags == quality
        assert imu.gyro_corrected_dps[2] == 5
        previous = current
    assert not imu_identity_is_new(previous, previous)


def test_firmware_watchdog_and_parser_remain_fail_closed(lower):
    lower.wire(encode_velocity_payload(0, 0, False, 2, 55, 1), 0)
    lower.wire(encode_velocity_payload(0.2, 0, True, 2, 55, 2), 1)
    assert lower.request("poll 0 200")["active"] == 1
    assert lower.request("poll 0 202")["active"] == 0
    frame = bytearray(
        build_frame(CMD_SET_VELOCITY, encode_velocity_payload(0.3, 0, True, 2, 55, 3))
    )
    frame[-1] ^= 1
    bad = lower.request(f"wire 0 203 {frame.hex()}")
    assert bad["active"] == 0 and bad["frames"] == 0
    good = build_frame(CMD_SET_VELOCITY, encode_velocity_payload(0.3, 0, True, 2, 55, 3))
    partial = lower.request(f"wire 0 204 {good[:10].hex()}")
    assert partial["active"] == 0 and partial["frames"] == 0
    complete = lower.request(f"wire 0 205 {good[10:].hex()}")
    assert complete["active"] == 1 and complete["frames"] == 1
