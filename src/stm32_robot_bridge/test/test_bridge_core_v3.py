import pytest
from dataclasses import replace
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

from stm32_robot_bridge.bridge_core import (
    BridgeCore,
    CommandDisposition,
    REARM_ACK_TIMEOUT,
    REARM_FIRMWARE_REJECT,
    StatusDisposition,
)
from stm32_robot_bridge.bridge_state import BridgeState
from stm32_robot_bridge.protocol_v3 import (
    ACK_APPLIED,
    ACK_RECEIVED,
    ACK_REJECTED,
    ACK_SESSION_VALID,
    HelloPayload,
    CommandStream,
    CMD_GET_INFO,
    CMD_HELLO,
    CMD_SET_VELOCITY,
    CMD_STATUS,
    STATUS_PAYLOAD_SIZE,
    decode_status_payload,
    encode_get_info_payload,
    sequence_is_forward,
    StatusPayload,
)
from stm32_robot_bridge.transport_supervisor import OutboundFrame, SerialTransportSupervisor, TransportEvent


def status(sequence=1, session=7, received=1, applied=1, flags=0):
    return StatusPayload(
        3,
        flags,
        2,
        0x0F,
        0,
        0,
        12.0,
        (0.0,) * 4,
        (0,) * 4,
        (0.0,) * 4,
        (0.0,) * 4,
        (0,) * 4,
        0x0F,
        0,
        0,
        sequence,
        sequence * 50,
        session,
        received,
        applied,
        0,
        0,
        ACK_SESSION_VALID | ACK_RECEIVED | ACK_APPLIED,
    )


def ready_core():
    core = BridgeCore(
        hard_max_linear_mps=0.45,
        hard_max_angular_radps=1.5,
        command_timeout_sec=0.15,
        status_timeout_sec=0.25,
        max_command_age_sec=0.15,
    )
    core.on_connected(7)
    assert core.on_hello(HelloPayload(3, 1, 0x1F, "11" * 20, 1, 2))
    assert core.on_status(status(received=0, applied=0), 1.0)
    assert core.snapshot.state is BridgeState.WAIT_STATUS
    core.on_disable_sent(1)
    assert core.on_status(status(sequence=2), 1.01)
    assert core.snapshot.state is BridgeState.WIRE_REARM_READY
    core.accept_command(
        vx=0,
        wz=0,
        enable=False,
        source=0,
        session_id=99,
        sequence=1,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.01,
    )
    return core


def test_connection_generation_always_requires_upper_layer_disable():
    core = ready_core()
    assert not core.snapshot.rearm_required
    core.on_disconnected()
    core.on_connected(8)
    assert core.snapshot.rearm_required
    assert core.snapshot.rearm_reason_flags


def test_bridge_core_rejects_missing_capability_and_duplicate_status():
    core = BridgeCore(
        hard_max_linear_mps=1,
        hard_max_angular_radps=1,
        command_timeout_sec=0.1,
        status_timeout_sec=0.2,
        max_command_age_sec=0.1,
    )
    core.on_connected(7)
    assert not core.on_hello(HelloPayload(3, 1, 0x0F, "00" * 20, 1, 0))
    assert core.on_hello(HelloPayload(3, 1, 0x1F, "00" * 20, 1, 0))
    assert core.on_status(status(), 1.0)
    assert core.on_status(status(), 1.01) is StatusDisposition.DUPLICATE


def test_bridge_core_sessions_order_limits_and_timeout_rearm():
    core = ready_core()
    decision = core.accept_command(
        vx=4.0,
        wz=-4.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.0,
        now_ros_sec=10.01,
        now_monotonic=1.01,
    )
    command = decision.validated_command
    assert decision.disposition is CommandDisposition.ACCEPT_ENABLE
    assert command is not None
    assert command.vx == pytest.approx(0.45) and command.wz == -1.5
    conflicting = core.accept_command(
        vx=0,
        wz=0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.02,
        now_ros_sec=10.03,
        now_monotonic=1.02,
    )
    assert conflicting.disposition is CommandDisposition.GLOBAL_RELEASE
    assert core.snapshot.rearm_required
    rearm = core.accept_command(
        vx=0,
        wz=0,
        enable=True,
        source=3,
        session_id=100,
        sequence=2,
        command_stamp_sec=10.04,
        now_ros_sec=10.05,
        now_monotonic=1.21,
    )
    assert rearm.disposition is CommandDisposition.REQUIRE_REARM


def test_command_and_status_timeouts_release():
    command_core = ready_core()
    command_core.accept_command(
        vx=0.2,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.02,
    )
    assert command_core.tick(1.20) == (True, "command_timeout")
    assert command_core.snapshot.rearm_required

    status_core = ready_core()
    assert status_core.tick(1.30) == (True, "status_timeout")
    assert status_core.snapshot.rearm_required


def test_duplicate_is_idempotent_and_does_not_refresh_lease():
    core = ready_core()
    accepted = core.accept_command(
        vx=0.2,
        wz=0.1,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.0,
        now_ros_sec=10.0,
        now_monotonic=1.02,
    )
    duplicate = core.accept_command(
        vx=0.2,
        wz=0.1,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=999.0,
        now_ros_sec=-999.0,
        now_monotonic=1.10,
    )

    assert accepted.disposition is CommandDisposition.ACCEPT_ENABLE
    assert duplicate.disposition is CommandDisposition.IGNORE_DUPLICATE
    assert core.snapshot.last_command_at == 1.02


def test_stale_new_epoch_is_ignored_but_stale_active_epoch_releases():
    inactive = ready_core()
    before = inactive.snapshot
    stale = inactive.accept_command(
        vx=0.2,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=10.0,
        now_ros_sec=10.5,
        now_monotonic=1.02,
    )
    assert stale.disposition is CommandDisposition.REJECT_STALE
    assert inactive.snapshot == before

    active = ready_core()
    assert (
        active.accept_command(
            vx=0.2,
            wz=0.0,
            enable=True,
            source=3,
            session_id=100,
            sequence=1,
            command_stamp_sec=10.0,
            now_ros_sec=10.01,
            now_monotonic=1.02,
        ).disposition
        is CommandDisposition.ACCEPT_ENABLE
    )
    stale_active = active.accept_command(
        vx=0.3,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=2,
        command_stamp_sec=10.0,
        now_ros_sec=10.5,
        now_monotonic=1.03,
    )
    assert stale_active.disposition is CommandDisposition.GLOBAL_RELEASE
    assert active.snapshot.rearm_required
    assert active.snapshot.target_vx == 0.0


def test_old_session_and_out_of_order_do_not_mutate_current_target():
    core = ready_core()
    core.accept_command(
        vx=0.2,
        wz=0.1,
        enable=True,
        source=3,
        session_id=100,
        sequence=10,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.02,
    )
    before = core.snapshot
    stale = core.accept_command(
        vx=0.3,
        wz=0.0,
        enable=True,
        source=3,
        session_id=99,
        sequence=2,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.03,
    )
    out_of_order = core.accept_command(
        vx=0.4,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=9,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.04,
    )

    assert stale.disposition is CommandDisposition.REJECT_OLD_SESSION
    assert out_of_order.disposition is CommandDisposition.REJECT_OUT_OF_ORDER
    assert core.snapshot == before


def test_active_nonfinite_command_requires_release_and_rearm():
    core = ready_core()
    core.accept_command(
        vx=0.2,
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=1,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.02,
    )
    decision = core.accept_command(
        vx=float("nan"),
        wz=0.0,
        enable=True,
        source=3,
        session_id=100,
        sequence=2,
        command_stamp_sec=0.0,
        now_ros_sec=0.0,
        now_monotonic=1.03,
    )

    assert decision.disposition is CommandDisposition.GLOBAL_RELEASE
    assert core.snapshot.rearm_required
    assert core.snapshot.target_vx == 0.0


def test_bridge_core_disable_enable_edge_recovers_after_fault():
    core = ready_core()
    assert core.on_status(status(sequence=3, flags=1), 1.05)
    core.on_disable_sent(2)
    assert core.snapshot.state is BridgeState.FAULT_LATCHED
    assert core.snapshot.rearm_required
    core.accept_command(
        vx=0,
        wz=0,
        enable=False,
        source=0,
        session_id=101,
        sequence=1,
        command_stamp_sec=2.0,
        now_ros_sec=2.01,
        now_monotonic=1.06,
    )
    assert core.on_status(status(sequence=4), 1.07)
    assert core.snapshot.state is BridgeState.WAIT_FAULT_CLEAR
    core.on_disable_sent(3)
    assert core.on_status(status(sequence=5, received=3, applied=3), 1.08)
    assert core.snapshot.state is BridgeState.WIRE_REARM_READY
    assert core.snapshot.rearm_required  # The fault-time host disable cannot qualify recovery.


def host_command(core, *, enabled, epoch, sequence, now):
    return core.accept_command(
        vx=0.2 if enabled else 0,
        wz=0,
        enable=enabled,
        source=3 if enabled else 0,
        session_id=epoch,
        sequence=sequence,
        command_stamp_sec=now,
        now_ros_sec=now,
        now_monotonic=now,
    )


@pytest.mark.parametrize("reason", [REARM_ACK_TIMEOUT, REARM_FIRMWARE_REJECT])
def test_wire_ack_does_not_rearm_the_host_after_safety_release(reason):
    core = ready_core()
    assert (
        host_command(core, enabled=True, epoch=100, sequence=1, now=1.02).disposition
        is CommandDisposition.ACCEPT_ENABLE
    )
    core.require_rearm(reason)
    core.on_disable_sent(2)
    core.on_status(status(sequence=3, received=2, applied=2), 1.03)
    assert core.snapshot.rearm_required and not core.can_drive
    assert (
        host_command(core, enabled=True, epoch=100, sequence=2, now=1.04).disposition
        is CommandDisposition.REQUIRE_REARM
    )
    assert (
        host_command(core, enabled=False, epoch=100, sequence=2, now=1.05).disposition
        is CommandDisposition.ACCEPT_DISABLE
    )
    assert (
        host_command(core, enabled=True, epoch=101, sequence=1, now=1.06).disposition
        is CommandDisposition.ACCEPT_ENABLE
    )


@pytest.mark.parametrize("timeout", ["command", "status"])
def test_timeout_requires_fresh_host_disable_even_after_wire_ack(timeout):
    core = ready_core()
    host_command(core, enabled=True, epoch=100, sequence=1, now=1.02)
    now = 1.18 if timeout == "command" else 1.30
    assert core.tick(now) == (True, f"{timeout}_timeout")
    core.on_disable_sent(2)
    core.on_status(status(sequence=3, received=2, applied=2), now + 0.01)
    assert (
        host_command(core, enabled=True, epoch=100, sequence=2, now=now + 0.02).disposition
        is CommandDisposition.REQUIRE_REARM
    )
    assert core.snapshot.rearm_required


def test_identical_hello_is_idempotent_while_driving_but_changed_identity_closes_drive():
    core = ready_core()
    host_command(core, enabled=True, epoch=100, sequence=1, now=1.02)
    before = core.snapshot
    assert core.on_hello(before.hello)
    assert core.snapshot == before
    assert (
        host_command(core, enabled=True, epoch=100, sequence=2, now=1.03).disposition
        is CommandDisposition.ACCEPT_ENABLE
    )
    assert core.on_hello(HelloPayload(3, 1, 0x1F, "22" * 20, 1, 2))
    assert not core.can_drive and core.snapshot.rearm_required
    assert core.snapshot.target_vx == 0
    assert core.snapshot.last_status_at is None


def test_incompatible_hello_revokes_drive_immediately():
    core = ready_core()
    host_command(core, enabled=True, epoch=100, sequence=1, now=1.02)
    assert not core.on_hello(HelloPayload(3, 1, 0x0F, "11" * 20, 1, 2))
    assert not core.can_drive and core.snapshot.rearm_required
    assert core.state is BridgeState.WAIT_HELLO


class _Serial:
    pass


def test_transport_coalesces_velocity_and_preserves_critical_priority():
    transport = SerialTransportSupervisor(_Serial(), "/dev/null", 115200, outbound_capacity=3)
    assert transport.submit(1, b"old", coalesce=True)
    assert transport.submit(2, b"critical", critical=True)
    assert transport.submit(1, b"new", coalesce=True)
    assert [(item.cmd, item.payload) for item in transport._outbound] == [
        (2, b"critical"),
        (1, b"new"),
    ]


def test_ros_adapter_has_no_external_state_machine_write():
    node_source = (
        Path(__file__).resolve().parents[1] / "stm32_robot_bridge" / "bridge_node_v3.py"
    ).read_text(encoding="utf-8")

    assert "core.snapshot =" not in node_source
    assert "core.machine." not in node_source


def bridge_timer_adapter():
    # Run the unchanged adapter methods with the real core, wire stream and
    # transport queue.  Only the ROS publisher shell and monotonic clock are
    # injected; no serial worker or hardware device is started.
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tests"))
    from ros_adapter_helpers import adapter_classes

    class Clock:
        value = 1.0

        def monotonic(self):
            return self.value

    class SessionIds:
        next_session = 8

        def randbits(self, _bits):
            session = self.next_session
            self.next_session += 1
            return session

    class LinkState:
        AGE_INVALID = 0xFFFFFFFF

        def __init__(self):
            self.header = SimpleNamespace()

    clock = Clock()
    classes = adapter_classes(
        Path(__file__).resolve().parents[1] / "stm32_robot_bridge/bridge_node_v3.py",
        time=clock,
        secrets=SessionIds(),
        BridgeCore=BridgeCore,
        BridgeState=BridgeState,
        CommandStream=CommandStream,
        CommandDisposition=CommandDisposition,
        StatusDisposition=StatusDisposition,
        HostMotionCommand=object,
        QoSProfile=object,
        ChassisLinkState=LinkState,
        CMD_GET_INFO=CMD_GET_INFO,
        CMD_HELLO=CMD_HELLO,
        CMD_SET_VELOCITY=CMD_SET_VELOCITY,
        CMD_STATUS=CMD_STATUS,
        decode_status_payload=decode_status_payload,
        encode_get_info_payload=encode_get_info_payload,
        sequence_is_forward=sequence_is_forward,
        ACK_SESSION_VALID=ACK_SESSION_VALID,
        ACK_RECEIVED=ACK_RECEIVED,
        ACK_APPLIED=ACK_APPLIED,
        ACK_REJECTED=ACK_REJECTED,
        OutboundFrame=OutboundFrame,
        REARM_ACK_TIMEOUT=REARM_ACK_TIMEOUT,
    )
    node = classes.STM32BridgeV3.__new__(classes.STM32BridgeV3)
    node.core = BridgeCore(
        hard_max_linear_mps=0.45,
        hard_max_angular_radps=1.5,
        command_timeout_sec=0.15,
        status_timeout_sec=0.25,
        max_command_age_sec=0.15,
    )
    node.core.on_connected(7)
    node.core.on_hello(HelloPayload(3, 1, 0x1F, "11" * 20, 1, 2))
    node.core.on_status(status(sequence=1, received=0, applied=0), clock.value)
    node.command_stream = CommandStream(0.15, 0.05, 7)
    node.transport = SerialTransportSupervisor(_Serial(), "/dev/not-opened", 115200)
    node.pending_disable_started_at = None
    node.pending_active_ack_started_at = None
    node.last_applied_sequence = None
    node.last_ack_evidence_at = clock.value
    node.last_status = None
    node.enabled_mask = None
    node.ack_timeout = 0.15
    node.release_count = 0
    node.invalid_command_count = 0
    node._publish_link_state = lambda: None
    node._publish_status = lambda _status, _disposition: None
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock.value * 1e9), to_msg=lambda: None)
    )
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

    def get_parameter(name):
        if name == "cmd_timeout":
            return SimpleNamespace(value=node.core.command_timeout_sec)
        if name == "drive_keepalive_sec":
            return SimpleNamespace(value=node.command_stream.keepalive_sec)
        raise KeyError(name)

    node.get_parameter = get_parameter
    return node, clock


def test_disable_retries_do_not_postpone_the_original_wire_ack_deadline():
    node, clock = bridge_timer_adapter()
    node._release("first disable")
    assert node.pending_disable_started_at == 1.0
    sequence = node.command_stream.sequence
    for index, now in enumerate((1.04, 1.08, 1.12, 1.16), 2):
        clock.value = now
        node.core.on_status(status(sequence=index, received=0, applied=0), now)
        node._release("fresh forward host disable retry")
        assert node.command_stream.sequence == sequence
        assert node.pending_disable_started_at == 1.0
        node.control_tick()
    assert node.core.state is BridgeState.WAIT_HELLO
    assert node.core.snapshot.wire_session_id == 8
    assert not node.core.can_drive and not node.command_stream.enabled
    assert node.pending_disable_started_at is None
    assert node.transport._thread is None


def test_new_wire_disable_sequence_starts_its_own_ack_budget():
    node, clock = bridge_timer_adapter()
    node._release("first disable")
    old_sequence = node.command_stream.sequence
    clock.value = 1.10
    node._release("new disable", force_fresh=True)
    assert node.command_stream.sequence == (old_sequence + 1) & 0xFFFFFFFF
    assert node.pending_disable_started_at == 1.10
    clock.value = 1.21
    node.core.on_status(status(sequence=2, received=0, applied=0), clock.value)
    node._release("same new disable retry")
    node.control_tick()
    assert node.core.snapshot.wire_session_id == 7
    assert node.pending_disable_started_at == 1.10
    clock.value = 1.27
    node.control_tick()
    assert node.core.snapshot.wire_session_id == 8
    assert not node.core.can_drive


def test_new_wire_session_discards_the_previous_pending_ack_deadline():
    node, clock = bridge_timer_adapter()
    node._release("first session disable")
    clock.value = 1.10
    node._begin_wire_session(request_info=True)
    assert node.core.snapshot.wire_session_id == 8
    assert node.pending_disable_started_at is None
    node.core.on_hello(HelloPayload(3, 1, 0x1F, "11" * 20, 1, 2))
    clock.value = 1.11
    node.core.on_status(status(sequence=1, session=8, received=0, applied=0), clock.value)
    node._release("first disable pending in new session")
    assert node.pending_disable_started_at == 1.11
    clock.value = 1.22
    node.core.on_status(status(sequence=2, session=8, received=0, applied=0), clock.value)
    node._release("new session same disable retry")
    node.control_tick()
    assert node.core.snapshot.wire_session_id == 8
    clock.value = 1.27
    node.control_tick()
    assert node.core.snapshot.wire_session_id == 9
    assert not node.core.can_drive and not node.command_stream.enabled


def adapter_host_command(node, clock, sequence, *, target=0.2, enable=True, epoch=100):
    nanos = round(clock.value * 1e9)
    node.on_chassis_command(
        SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=nanos // 10**9, nanosec=nanos % 10**9)
            ),
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=target if enable else 0), angular=SimpleNamespace(z=0)
            ),
            enable=enable,
            host_subsource=3 if enable else 0,
            command_epoch=epoch,
            sequence=sequence,
        )
    )


def queue_adapter_status(node, clock, sample):
    # Inject one encoded transport event, so production frame decoding, Core
    # STATUS ordering and the adapter's ACK bookkeeping all execute together.
    payload = bytearray(STATUS_PAYLOAD_SIZE)
    payload[:4] = bytes((sample.version, sample.status_flags, 2, sample.motor_enabled_mask))
    struct.pack_into("<H", payload, 12, 12000)
    payload[62] = sample.motor_speed_valid_mask
    struct.pack_into(
        "<IIQII",
        payload,
        65,
        sample.status_sequence,
        sample.sample_timestamp_ms,
        sample.last_received_session_id,
        sample.last_received_sequence,
        sample.last_applied_sequence,
    )
    payload[89:92] = bytes(
        (sample.last_reject_reason, sample.comm_health_flags, sample.command_ack_flags)
    )
    node.transport.events.put_nowait(
        TransportEvent("frame", CMD_STATUS, bytes(payload), clock.value)
    )


def ready_timer_adapter():
    node, clock = bridge_timer_adapter()
    adapter_host_command(node, clock, 1, enable=False, epoch=99)
    assert node.pending_disable_started_at == 1.0
    clock.value = 1.01
    queue_adapter_status(node, clock, status(sequence=2))
    node.control_tick()
    assert node.core.can_drive and node.core.state is BridgeState.WIRE_REARM_READY
    assert node.last_applied_sequence == 1
    assert node.pending_disable_started_at is None
    return node, clock


def test_changing_targets_and_fresh_status_without_ack_cannot_extend_active_budget():
    node, clock = ready_timer_adapter()
    for index in range(9):
        clock.value = 1.02 + index * 0.02
        adapter_host_command(node, clock, index + 1, target=0.1 + index * 0.001)
        queue_adapter_status(node, clock, status(sequence=index + 3))
        node.control_tick()
        assert node.last_ack_evidence_at == 1.01
        if index < 8:
            assert node.core.can_drive and node.command_stream.enabled
            assert node.pending_active_ack_started_at == 1.02
    assert not node.core.can_drive and not node.command_stream.enabled
    assert node.core.snapshot.rearm_reason_flags & REARM_ACK_TIMEOUT
    assert node.core.snapshot.last_status_at == clock.value
    assert node.pending_active_ack_started_at is None
    assert node.transport._thread is None
    assert node.invalid_command_count == 0


@pytest.mark.parametrize("changing_target", [False, True])
def test_forward_applied_acks_support_ten_seconds_of_fresh_commands(changing_target):
    node, clock = ready_timer_adapter()
    sent = 1
    for index in range(501):
        clock.value = 1.02 + index * 0.02
        target = 0.1 + (index % 11) * 0.001 if changing_target else 0.2
        adapter_host_command(node, clock, index + 1, target=target)
        queue_adapter_status(node, clock, status(sequence=index + 3, received=sent, applied=sent))
        node.control_tick()
        assert node.core.can_drive and node.command_stream.enabled
        assert node.last_applied_sequence == sent
        sent = node.core.snapshot.wire_sent_sequence
        assert sent == node.command_stream.sequence
    assert clock.value == pytest.approx(11.02)
    assert node.release_count == 1
    assert node.invalid_command_count == 0
    assert node.transport._thread is None
    if changing_target:
        assert sent > 500
        assert node.pending_active_ack_started_at == clock.value
    else:
        assert 10 <= sent <= 12
    clock.value += 0.02
    queue_adapter_status(node, clock, status(sequence=504, received=sent, applied=sent))
    node.control_tick()
    assert node.pending_active_ack_started_at is None
    assert node.core.can_drive and node.command_stream.enabled


def test_repeated_applied_sequence_in_new_status_does_not_extend_active_budget():
    node, clock = ready_timer_adapter()
    clock.value = 1.02
    adapter_host_command(node, clock, 1, target=0.1)
    node.control_tick()
    acked = node.core.snapshot.wire_sent_sequence
    clock.value = 1.04
    adapter_host_command(node, clock, 2, target=0.11)
    queue_adapter_status(node, clock, status(sequence=3, received=acked, applied=acked))
    node.control_tick()
    assert node.pending_active_ack_started_at == 1.04
    for index, now in enumerate((1.08, 1.12, 1.16, 1.20), 3):
        clock.value = now
        adapter_host_command(node, clock, index, target=0.1 + index * 0.01)
        queue_adapter_status(node, clock, status(sequence=index + 1, received=acked, applied=acked))
        node.control_tick()
        assert node.last_ack_evidence_at == 1.04
        if now < 1.20:
            assert node.pending_active_ack_started_at == 1.04
            assert node.core.can_drive
    assert not node.core.can_drive and not node.command_stream.enabled
    assert node.core.snapshot.rearm_reason_flags & REARM_ACK_TIMEOUT


@pytest.mark.parametrize(
    "invalid_ack",
    ["other_session", "unsent", "not_applied", "received_differs", "rejected", "old_applied"],
)
def test_invalid_ack_cannot_extend_active_budget(invalid_ack):
    node, clock = ready_timer_adapter()
    clock.value = 1.02
    adapter_host_command(node, clock, 1, target=0.1)
    node.control_tick()
    for index in range(8):
        clock.value = 1.04 + index * 0.02
        adapter_host_command(node, clock, index + 2, target=0.11 + index * 0.001)
        sequence = (
            node.command_stream.sequence
            if invalid_ack == "unsent"
            else node.core.snapshot.wire_sent_sequence
        )
        sample = status(sequence=index + 3, received=sequence, applied=sequence)
        if invalid_ack == "other_session":
            sample = replace(sample, last_received_session_id=8)
        elif invalid_ack == "not_applied":
            sample = replace(sample, command_ack_flags=ACK_SESSION_VALID | ACK_RECEIVED)
        elif invalid_ack == "received_differs":
            sample = replace(sample, last_received_sequence=sequence + 1)
        elif invalid_ack == "rejected":
            sample = replace(sample, command_ack_flags=sample.command_ack_flags | ACK_REJECTED)
        elif invalid_ack == "old_applied":
            sample = replace(sample, last_received_sequence=0, last_applied_sequence=0)
        # "unsent" acknowledges the target updated above before its first tick.
        queue_adapter_status(node, clock, sample)
        node.control_tick()
        assert node.last_ack_evidence_at == 1.01
    assert not node.core.can_drive and not node.command_stream.enabled
    assert node.core.snapshot.rearm_reason_flags & REARM_ACK_TIMEOUT


def test_new_wire_session_resets_active_ack_progress_and_rejects_previous_session():
    node, clock = ready_timer_adapter()
    clock.value = 1.02
    adapter_host_command(node, clock, 1)
    node.control_tick()
    assert node.pending_active_ack_started_at == 1.02
    clock.value = 1.04
    node._begin_wire_session(request_info=True)
    assert node.command_stream.session_id == 8
    assert node.pending_active_ack_started_at is None
    assert node.last_applied_sequence is None
    node.core.on_hello(HelloPayload(3, 1, 0x1F, "11" * 20, 1, 2))
    adapter_host_command(node, clock, 1, enable=False, epoch=101)
    clock.value = 1.06
    queue_adapter_status(node, clock, status(sequence=1, session=7))
    node.control_tick()
    assert not node.core.can_drive
    assert node.core.state is BridgeState.WAIT_POST_CLEAR_DISABLE_ACK
    assert node.pending_disable_started_at == 1.04
    assert node.last_applied_sequence is None
    clock.value = 1.08
    queue_adapter_status(node, clock, status(sequence=2, session=8))
    node.control_tick()
    assert node.core.can_drive
    assert node.last_applied_sequence == 1
    assert node.pending_disable_started_at is None
    clock.value = 1.10
    adapter_host_command(node, clock, 1, epoch=102)
    node.control_tick()
    assert node.core.can_drive and node.pending_active_ack_started_at == 1.10


def test_queue_rejection_does_not_publish_unsent_target_or_accept_its_ack():
    node, clock = ready_timer_adapter()
    # The initial critical disable occupies the only queue slot.  Exercise the
    # real queue's failure path, then model its consumer taking that frame.
    node.transport._outbound_capacity = 1
    assert len(node.transport._outbound) == 1
    clock.value = 1.02
    adapter_host_command(node, clock, 1)
    node.control_tick()
    assert node.command_stream.sequence == 2
    assert node.core.snapshot.wire_sent_sequence == 1
    assert node.transport.stats.tx_queue_drops == 1
    messages = []
    node.link_state_pub = SimpleNamespace(publish=messages.append)
    node.config_sha256 = "test-source"
    type(node)._publish_link_state(node)
    assert messages[-1].wire_sent_sequence == 1
    clock.value = 1.04
    queue_adapter_status(node, clock, status(sequence=3, received=2, applied=2))
    node.control_tick()
    assert node.last_applied_sequence == 1
    assert node.last_ack_evidence_at == 1.01
    assert node.pending_active_ack_started_at == 1.02
    node.transport._outbound.pop(0)
    clock.value = 1.06
    node.control_tick()
    assert node.core.snapshot.wire_sent_sequence == 2
    type(node)._publish_link_state(node)
    assert messages[-1].wire_sent_sequence == 2
    clock.value = 1.08
    queue_adapter_status(node, clock, status(sequence=4, received=2, applied=2))
    node.control_tick()
    assert node.core.can_drive and node.command_stream.enabled
    assert node.pending_active_ack_started_at is None
    assert node.last_applied_sequence == 2
    assert node.last_ack_evidence_at == 1.08


def published_adapter_link(node):
    messages = []
    node.link_state_pub = SimpleNamespace(publish=messages.append)
    node.config_sha256 = "test-source"
    type(node)._publish_link_state(node)
    return messages[-1]


@pytest.mark.parametrize("new_session", [False, True])
def test_ack_age_is_unavailable_without_actual_applied_evidence(new_session):
    if new_session:
        node, clock = ready_timer_adapter()
        node._begin_wire_session(request_info=True)
        node.core.on_hello(HelloPayload(3, 1, 0x1F, "11" * 20, 1, 2))
    else:
        node, clock = bridge_timer_adapter()
    queue_adapter_status(
        node,
        clock,
        replace(
            status(sequence=2, session=node.command_stream.session_id, received=0, applied=0),
            command_ack_flags=ACK_SESSION_VALID | ACK_RECEIVED,
        ),
    )
    node.control_tick()
    link = published_adapter_link(node)
    assert link.firmware_ack_available  # ACK fields exist; APPLIED evidence does not.
    assert node.last_applied_sequence is None
    assert link.command_ack_age_ms == 0xFFFFFFFF
    clock.value += 0.05
    assert published_adapter_link(node).command_ack_age_ms == 0xFFFFFFFF


def test_ack_age_for_real_sequence_zero_remains_valid_and_duplicate_ack_does_not_refresh():
    node, clock = bridge_timer_adapter()
    # Seed the end of the u32 range, then use a real release to wrap to zero.
    node.command_stream.sequence = 0xFFFFFFFF
    node._release("wrapped wire disable", force_fresh=True)
    assert node.command_stream.sequence == 0
    clock.value = 1.01
    queue_adapter_status(node, clock, status(sequence=2, received=0, applied=0))
    node.control_tick()
    assert node.last_applied_sequence == 0
    link = published_adapter_link(node)
    assert link.firmware_ack_available and link.command_ack_age_ms == 0
    clock.value = 1.07
    queue_adapter_status(node, clock, status(sequence=3, received=0, applied=0))
    node.control_tick()
    assert node.last_ack_evidence_at == 1.01
    assert 59 <= published_adapter_link(node).command_ack_age_ms <= 60


def test_shutdown_releases_and_closes_transport_without_using_invalid_ros_context(capsys):
    node, clock = ready_timer_adapter()
    adapter_host_command(node, clock, 1)
    node.control_tick()
    assert node.command_stream.enabled

    def invalid_context(*_args, **_kwargs):
        raise RuntimeError("publisher's context is invalid")

    node._publish_link_state = invalid_context
    node.get_logger = invalid_context
    node.shutdown()
    trace = capsys.readouterr().out
    assert "SERIAL_SHUTDOWN session=" in trace
    assert "disable_written=False applied=UNCONFIRMED" in trace
    assert not node.command_stream.enabled
    assert node.transport._stop
    final = node.transport._outbound[0]
    assert final.critical and final.cmd == CMD_SET_VELOCITY
    assert final.payload[9] == 0  # v3 enable flag; the final intent is disabled.


def test_shutdown_closes_transport_even_when_release_raises():
    node, _clock = bridge_timer_adapter()

    def failed_release(*_args, **_kwargs):
        raise RuntimeError('release failure')

    node._release = failed_release
    with pytest.raises(RuntimeError, match='release failure'):
        node.shutdown()
    assert node.transport._stop
