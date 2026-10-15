"""Serial-device operations with no ROS dependency."""

from dataclasses import dataclass
from typing import Optional


class SerialShortWrite(IOError):
    pass


@dataclass
class TransportStats:
    tx_frames: int = 0
    tx_bytes: int = 0
    tx_errors: int = 0
    tx_short_writes: int = 0
    rx_frames: int = 0
    rx_bytes: int = 0
    rx_crc_errors: int = 0
    rx_bad_length: int = 0
    rx_bad_version: int = 0
    rx_unknown_cmd: int = 0
    rx_resync_bytes: int = 0
    serial_reconnects: int = 0
    rx_queue_drops: int = 0
    tx_queue_drops: int = 0


def open_serial_port(serial_module, resolved_port: str, baudrate: int):
    device = serial_module.Serial()
    device.port = resolved_port
    device.baudrate = baudrate
    device.timeout = 0.0
    device.write_timeout = 0.2
    device.rtscts = False
    device.dsrdtr = False
    if hasattr(device, "exclusive"):
        device.exclusive = True
    set_modem_lines_low(device)
    device.open()
    set_modem_lines_low(device)
    return device


def set_modem_lines_low(device) -> None:
    if device is None:
        return
    for attr in ("dtr", "rts"):
        try:
            setattr(device, attr, False)
        except Exception:
            pass


def reset_serial_buffers(device) -> None:
    if device is None:
        return
    for method_name in ("reset_input_buffer", "reset_output_buffer"):
        try:
            getattr(device, method_name)()
        except Exception:
            pass


def write_all(device, frame: bytes, stats: Optional[TransportStats] = None) -> int:
    """Write a complete frame, continuing only while each call makes progress."""
    offset = 0
    while offset < len(frame):
        remaining = frame[offset:]
        written = device.write(remaining)
        if written is None:
            if stats is not None:
                stats.tx_short_writes += 1
            raise SerialShortWrite(f"expected={len(frame)}, written={offset}, last_write=None")
        try:
            written = int(written)
        except (TypeError, ValueError) as exc:
            if stats is not None:
                stats.tx_short_writes += 1
            raise SerialShortWrite(
                f"expected={len(frame)}, written={offset}, invalid_write_count={written!r}"
            ) from exc
        if written <= 0 or written > len(remaining):
            if stats is not None:
                stats.tx_short_writes += 1
            raise SerialShortWrite(f"expected={len(frame)}, written={offset}, last_write={written}")
        if written < len(remaining) and stats is not None:
            stats.tx_short_writes += 1
        offset += written
    return offset
