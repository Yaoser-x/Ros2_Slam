"""Dedicated bounded serial I/O supervisor."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from .framing import FrameParser, build_frame
from .serial_transport import TransportStats, open_serial_port, set_modem_lines_low, write_all


@dataclass(frozen=True)
class TransportEvent:
    kind: str
    cmd: int = 0
    payload: bytes = b""
    received_at: float = 0.0
    detail: str = ""


@dataclass(frozen=True)
class OutboundFrame:
    cmd: int
    payload: bytes
    critical: bool = False
    coalesce: bool = False


class SerialTransportSupervisor:
    def __init__(
        self,
        serial_module,
        port: str,
        baudrate: int,
        *,
        retry_sec: float = 2.0,
        inbound_capacity: int = 256,
        outbound_capacity: int = 32,
        coalesce_inbound_commands: tuple[int, ...] = (),
    ) -> None:
        self.serial_module, self.port, self.baudrate = serial_module, str(port), int(baudrate)
        self.retry_sec = float(retry_sec)
        self.events: queue.Queue[TransportEvent] = queue.Queue(maxsize=inbound_capacity)
        self._coalesce_inbound_commands = frozenset(coalesce_inbound_commands)
        self._latest_observations: dict[int, TransportEvent] = {}
        self._inbound_lock = threading.Lock()
        self._outbound_capacity = int(outbound_capacity)
        self._outbound: list[OutboundFrame] = []
        self._condition = threading.Condition()
        self._stop = False
        self._closing = False
        self._shutdown_frame: Optional[OutboundFrame] = None
        self._shutdown_done = threading.Event()
        self._shutdown_written = False
        self._shutdown_deadline = 0.0
        self._thread: Optional[threading.Thread] = None
        self.device: Optional[Any] = None
        self.parser = FrameParser()
        self.stats = TransportStats()

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="stm32-serial", daemon=True)
            self._thread.start()

    def close(
        self, timeout_sec: float = 0.2, *, final_frame: Optional[OutboundFrame] = None
    ) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        with self._condition:
            if not self._closing:
                self._closing = True
                self._shutdown_frame = final_frame
                self._shutdown_deadline = deadline
                # A shutdown disable replaces queued work, including stale critical
                # commands. Only the I/O owner may write it to the serial device.
                self._outbound = [final_frame] if final_frame is not None else []
            if final_frame is None or self._thread is None:
                self._stop = True
            self._condition.notify_all()
        if final_frame is not None and self._thread is not None:
            self._shutdown_done.wait(timeout=max(deadline - time.monotonic(), 0.0))
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=max(deadline - time.monotonic(), 0.0))
        self._close_device()
        return self._shutdown_written

    def submit(
        self, cmd: int, payload: bytes, *, critical: bool = False, coalesce: bool = False
    ) -> bool:
        frame = OutboundFrame(int(cmd) & 0xFF, bytes(payload), bool(critical), bool(coalesce))
        with self._condition:
            if self._closing or self._stop:
                self.stats.tx_queue_drops += 1
                return False
            if coalesce:
                self._outbound = [
                    item for item in self._outbound if item.cmd != frame.cmd or item.critical
                ]
            if len(self._outbound) >= self._outbound_capacity:
                if not critical:
                    self.stats.tx_queue_drops += 1
                    return False
                candidates = [
                    index for index, item in enumerate(self._outbound) if not item.critical
                ]
                if not candidates:
                    self.stats.tx_queue_drops += 1
                    return False
                del self._outbound[candidates[-1]]
            if critical:
                self._outbound.insert(0, frame)
            else:
                self._outbound.append(frame)
            self._condition.notify_all()
        return True

    def drain_events(self, limit: int = 64) -> list[TransportEvent]:
        result = []
        with self._inbound_lock:
            for _ in range(max(int(limit), 0)):
                try:
                    result.append(self.events.get_nowait())
                except queue.Empty:
                    if not self._latest_observations:
                        break
                    cmd = next(iter(self._latest_observations))
                    result.append(self._latest_observations.pop(cmd))
        return result

    def clear_pending_observations(self) -> None:
        with self._inbound_lock:
            self._latest_observations.clear()

    def _emit(self, event: TransportEvent) -> None:
        with self._inbound_lock:
            if event.kind in ("connected", "disconnected"):
                self._latest_observations.clear()
            if event.kind == "frame" and event.cmd in self._coalesce_inbound_commands:
                if event.cmd in self._latest_observations:
                    self.stats.rx_coalesced_frames += 1
                self._latest_observations[event.cmd] = event
                return
            try:
                self.events.put_nowait(event)
            except queue.Full:
                self.stats.rx_queue_drops += 1
                try:
                    self.events.get_nowait()
                    self.events.put_nowait(event)
                except queue.Empty:
                    pass

    def _run(self) -> None:
        while not self._stop:
            if self.device is None:
                if self._closing:
                    self._shutdown_done.set()
                    break
                try:
                    self.device = open_serial_port(self.serial_module, self.port, self.baudrate)
                    set_modem_lines_low(self.device)
                    self.parser = FrameParser()
                    self.stats.serial_reconnects += 1
                    self._emit(TransportEvent("connected", received_at=time.monotonic()))
                except Exception as exc:
                    self._emit(
                        TransportEvent(
                            "connect_error", detail=str(exc), received_at=time.monotonic()
                        )
                    )
                    with self._condition:
                        self._condition.wait(timeout=self.retry_sec)
                    continue
            try:
                outbound = None
                with self._condition:
                    if self._outbound:
                        outbound = self._outbound.pop(0)
                if outbound is not None:
                    with self._condition:
                        if self._closing and outbound is not self._shutdown_frame:
                            continue
                    frame = build_frame(outbound.cmd, outbound.payload)
                    write_all(self.device, frame, self.stats)
                    self.stats.tx_frames += 1
                    self.stats.tx_bytes += len(frame)
                    if outbound is self._shutdown_frame:
                        self._shutdown_written = time.monotonic() <= self._shutdown_deadline
                        self._shutdown_done.set()
                        self._stop = True
                        self._emit(
                            TransportEvent(
                                (
                                    "shutdown_disable_written"
                                    if self._shutdown_written
                                    else "shutdown_write_timeout"
                                ),
                                outbound.cmd,
                                outbound.payload,
                                time.monotonic(),
                            )
                        )
                        break
                device = self.device
                if device is None:
                    continue
                waiting = int(getattr(device, "in_waiting", 0))
                if waiting > 0:
                    chunk = device.read(waiting)
                    self.stats.rx_bytes += len(chunk)
                    frames = self.parser.feed(chunk)
                    self.stats.rx_crc_errors = self.parser.stats.crc_errors
                    self.stats.rx_bad_length = self.parser.stats.bad_length
                    self.stats.rx_resync_bytes = self.parser.stats.resync_bytes
                    self.stats.rx_frames += len(frames)
                    for cmd, payload in frames:
                        self._emit(TransportEvent("frame", cmd, payload, time.monotonic()))
                if outbound is None and waiting <= 0:
                    with self._condition:
                        self._condition.wait(timeout=0.005)
            except Exception as exc:
                self.stats.tx_errors += 1
                self._emit(
                    TransportEvent("disconnected", detail=str(exc), received_at=time.monotonic())
                )
                self._close_device()
                if self._closing:
                    self._shutdown_done.set()
                    break

    def _close_device(self) -> None:
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None
