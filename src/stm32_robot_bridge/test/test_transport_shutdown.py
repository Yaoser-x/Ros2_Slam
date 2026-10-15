import threading
import time

from stm32_robot_bridge.framing import build_frame
from stm32_robot_bridge.transport_supervisor import OutboundFrame, SerialTransportSupervisor
import stm32_robot_bridge.transport_supervisor as supervisor_module


class Device:
    in_waiting = 0

    def __init__(self):
        self.connected = threading.Event()
        self.entered_write = threading.Event()
        self.release_write = threading.Event()
        self.block_first = False
        self.closed = False
        self.writes = []
        self.writers = []
        self.partial = False

    def write(self, value):
        self.entered_write.set()
        if self.block_first and not self.writes:
            assert self.release_write.wait(1.0)
        if self.closed:
            raise IOError("device closed")
        size = min(3, len(value)) if self.partial else len(value)
        self.writes.append(bytes(value[:size]))
        self.writers.append(threading.current_thread().name)
        return size

    def close(self):
        self.closed = True
        self.release_write.set()


def transport_with_device(monkeypatch, device):
    def open_device(*_args):
        device.connected.set()
        return device

    monkeypatch.setattr(supervisor_module, "open_serial_port", open_device)
    transport = SerialTransportSupervisor(object(), "/dev/test-only", 115200)
    return transport


def test_shutdown_drops_queued_motion_and_only_io_owner_writes_final_disable(monkeypatch):
    device = Device()
    device.block_first = True
    transport = transport_with_device(monkeypatch, device)
    transport.submit(1, b"in-flight")
    transport.start()
    assert device.entered_write.wait(1.0)
    transport.submit(1, b"queued-motion")
    transport.submit(4, b"queued-operation", critical=True)
    final = OutboundFrame(1, b"fresh-disable", critical=True)
    result = []
    closer = threading.Thread(target=lambda: result.append(transport.close(final_frame=final)))
    try:
        closer.start()
        with transport._condition:
            assert transport._condition.wait_for(lambda: transport._closing, timeout=1.0)
        assert not transport.submit(1, b"late-motion", critical=True)
        device.release_write.set()
        closer.join(timeout=1.0)
        assert result == [True]
        assert device.writes == [build_frame(1, b"in-flight"), build_frame(1, b"fresh-disable")]
        assert device.writers == ["stm32-serial", "stm32-serial"]
        assert device.closed and not transport._thread.is_alive()
        assert any(event.kind == "shutdown_disable_written" for event in transport.drain_events())
    finally:
        device.release_write.set()
        transport.close()
        closer.join(timeout=1.0)


def test_final_short_writes_must_complete_before_shutdown_is_successful(monkeypatch):
    device = Device()
    device.partial = True
    transport = transport_with_device(monkeypatch, device)
    transport.start()
    assert device.connected.wait(1.0)
    final = OutboundFrame(1, b"disable", critical=True)
    assert transport.close(final_frame=final)
    assert b"".join(device.writes) == build_frame(1, b"disable")
    assert transport.stats.tx_short_writes > 0
    assert device.closed


def test_unavailable_link_does_not_reconnect_during_shutdown(monkeypatch):
    attempted = threading.Event()
    attempts = []

    def unavailable(*_args):
        attempts.append(1)
        attempted.set()
        raise IOError("offline")

    monkeypatch.setattr(supervisor_module, "open_serial_port", unavailable)
    transport = SerialTransportSupervisor(object(), "/dev/test-only", 115200, retry_sec=10)
    transport.start()
    assert attempted.wait(1.0)
    assert not transport.close(final_frame=OutboundFrame(1, b"disable", critical=True))
    assert len(attempts) == 1
    assert not transport._thread.is_alive()


def test_shutdown_timeout_reports_failure_and_closes_blocked_device(monkeypatch):
    device = Device()
    device.block_first = True
    transport = transport_with_device(monkeypatch, device)
    transport.start()
    assert device.connected.wait(1.0)
    started = time.monotonic()
    assert not transport.close(
        timeout_sec=0.05, final_frame=OutboundFrame(1, b"disable", critical=True)
    )
    assert time.monotonic() - started < 0.3
    transport._thread.join(timeout=1.0)
    assert device.closed and not device.writes
    assert not transport._thread.is_alive()
    assert not any(event.kind == "shutdown_disable_written" for event in transport.drain_events())


def test_never_started_transport_cannot_claim_a_stop_was_written():
    transport = SerialTransportSupervisor(object(), "/dev/not-opened", 115200)
    assert not transport.close(final_frame=OutboundFrame(1, b"disable", critical=True))
    assert transport._stop and not transport.submit(1, b"late-motion")
