"""Bounded JSONL handoff so disk stalls cannot block the HIL command loop."""

from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Thread


class EvidenceWriteError(RuntimeError):
    pass


class EvidenceWriter:
    def __init__(self, path: Path, *, capacity=4096):
        self._stream = path.open("x", encoding="utf-8")
        self._queue = Queue(maxsize=capacity)
        self._closing = Event()
        self._failure = None
        self.submitted = 0
        self.written = 0
        self.max_write_seconds = 0.0
        self._worker = Thread(target=self._run, name="hil-evidence", daemon=True)
        self._worker.start()

    def check(self):
        if self._failure is not None:
            raise EvidenceWriteError(self._failure)

    def write(self, record: str):
        self.check()
        if self._closing.is_set():
            raise EvidenceWriteError("evidence writer is closing")
        try:
            self._queue.put_nowait(record)
        except Full:
            self._failure = "evidence queue overflow; capture is incomplete"
            raise EvidenceWriteError(self._failure) from None
        self.submitted += 1

    def _run(self):
        import time

        try:
            while not self._closing.is_set() or not self._queue.empty():
                try:
                    record = self._queue.get(timeout=0.05)
                except Empty:
                    continue
                began = time.monotonic()
                self._stream.write(record)
                self.max_write_seconds = max(self.max_write_seconds, time.monotonic() - began)
                self.written += 1
            self._stream.close()
        except Exception as exc:
            self._failure = f"evidence write failed: {type(exc).__name__}: {exc}"
            try:
                self._stream.close()
            except Exception:
                pass

    def close(self, timeout_sec=2.0):
        self._closing.set()
        self._worker.join(timeout_sec)
        if self._worker.is_alive():
            self._failure = "evidence flush did not finish within its bounded wait"
        self.check()
        if self.submitted != self.written:
            raise EvidenceWriteError("evidence record count mismatch")

    def summary(self):
        return {
            "submitted": self.submitted,
            "written": self.written,
            "complete": not self._worker.is_alive()
            and self._failure is None
            and self.submitted == self.written,
            "failure": self._failure,
            "max_disk_write_seconds": self.max_write_seconds,
        }
