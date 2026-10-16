"""Bounded handoff from ROS receipt to the sole bridge state owner."""

from collections import deque
from dataclasses import dataclass
from threading import Lock


@dataclass(frozen=True)
class ReceivedCommand:
    vx: float
    wz: float
    enable: bool
    source: int
    session_id: int
    sequence: int
    stamp_sec: float
    received_ros_sec: float
    received_monotonic: float


class CommandInbox:
    def __init__(self, capacity: int = 32):
        self._capacity = capacity
        self._lock = Lock()
        self._queue = deque()
        self._overflow = False
        self._accepting = True

    def put(self, command: ReceivedCommand) -> None:
        with self._lock:
            if not self._accepting or self._overflow:
                return
            if len(self._queue) >= self._capacity:
                self._queue.clear()
                self._overflow = True
                return
            # Preserve disables, duplicate conflicts and epoch transitions.
            self._queue.append(command)

    def take(self):
        with self._lock:
            if self._overflow:
                self._overflow = False
                self._queue.clear()
                return True, None
            return False, self._queue.popleft() if self._queue else None

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()

    def stop(self) -> None:
        with self._lock:
            self._accepting = False
            self._queue.clear()
