from stm32_robot_bridge.protocol_v3 import CMD_IMU_STATUS, CMD_STATUS
from stm32_robot_bridge.transport_supervisor import SerialTransportSupervisor, TransportEvent


def make_transport():
    return SerialTransportSupervisor(
        object(), "/dev/not-opened", 115200, inbound_capacity=4,
        coalesce_inbound_commands=(CMD_IMU_STATUS,),
    )


def test_sensor_burst_preserves_all_ack_and_fault_transitions_before_latest_sample():
    transport = make_transport()
    statuses = [TransportEvent("frame", CMD_STATUS, bytes([i]), float(i)) for i in range(4)]
    for i in range(100):
        transport._emit(TransportEvent("frame", CMD_IMU_STATUS, bytes([i]), float(i)))
        if i % 25 == 0:
            transport._emit(statuses[i // 25])
    # Limited drains must not remove work that the callback cannot process yet.
    assert transport.drain_events(limit=0) == []
    assert [transport.drain_events(limit=1)[0] for _ in range(4)] == statuses
    latest = transport.drain_events(limit=1)
    assert latest == [TransportEvent("frame", CMD_IMU_STATUS, b"\x63", 99.0)]
    assert transport.drain_events() == []
    assert transport.stats.rx_queue_drops == 0
    assert transport.stats.rx_coalesced_frames == 99


def test_new_connection_and_session_discard_pending_sensor_from_previous_identity():
    transport = make_transport()
    for kind in ("disconnected", "connected"):
        transport._emit(TransportEvent("frame", CMD_IMU_STATUS, b"old", 1.0))
        boundary = TransportEvent(kind, received_at=2.0)
        transport._emit(boundary)
        assert transport.drain_events() == [boundary]
    transport._emit(TransportEvent("frame", CMD_IMU_STATUS, b"old-session", 3.0))
    transport.clear_pending_observations()
    assert transport.drain_events() == []


def test_default_transport_retains_every_sensor_frame_in_order():
    transport = SerialTransportSupervisor(object(), "/dev/not-opened", 115200)
    frames = [TransportEvent("frame", CMD_IMU_STATUS, bytes([i]), float(i)) for i in range(3)]
    for event in frames:
        transport._emit(event)
    assert transport.drain_events() == frames
    assert transport.stats.rx_coalesced_frames == 0
