import sys
from pathlib import Path
from threading import Event

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "verification/runners/hil"))
from evidence_writer import EvidenceWriter, EvidenceWriteError


class BlockingStream:
    def __init__(self, *, fail=False):
        self.entered, self.release = Event(), Event()
        self.records = []
        self.closed = False
        self.fail = fail

    def write(self, record):
        self.entered.set()
        assert self.release.wait(3)
        if self.fail:
            raise OSError("simulated disk failure")
        self.records.append(record)

    def close(self):
        self.closed = True


def test_disk_stall_does_not_block_capture_or_lose_order(monkeypatch, tmp_path):
    stream = BlockingStream()
    monkeypatch.setattr(Path, "open", lambda *_args, **_kwargs: stream)
    writer = EvidenceWriter(tmp_path / "events.jsonl")
    try:
        writer.write("first\n")
        assert stream.entered.wait(1)
        # The disk worker is still blocked; command/ROS callbacks can return.
        writer.write("second\n")
        assert writer.submitted == 2 and writer.written == 0
    finally:
        stream.release.set()
        writer.close()
    assert stream.records == ["first\n", "second\n"] and stream.closed
    assert writer.summary()["complete"]


def test_overflow_is_sticky_and_capture_cannot_be_reported_complete(monkeypatch, tmp_path):
    stream = BlockingStream()
    monkeypatch.setattr(Path, "open", lambda *_args, **_kwargs: stream)
    writer = EvidenceWriter(tmp_path / "events.jsonl", capacity=1)
    try:
        writer.write("first\n")
        assert stream.entered.wait(1)
        writer.write("second\n")
        with pytest.raises(EvidenceWriteError, match="overflow"):
            writer.write("third\n")
    finally:
        stream.release.set()
        with pytest.raises(EvidenceWriteError, match="overflow"):
            writer.close()
    assert stream.records == ["first\n", "second\n"]
    assert not writer.summary()["complete"]


def test_disk_failure_and_flush_timeout_are_failures(monkeypatch, tmp_path):
    stream = BlockingStream(fail=True)
    monkeypatch.setattr(Path, "open", lambda *_args, **_kwargs: stream)
    writer = EvidenceWriter(tmp_path / "events.jsonl")
    writer.write("first\n")
    assert stream.entered.wait(1)
    with pytest.raises(EvidenceWriteError, match="bounded wait"):
        writer.close(timeout_sec=0.01)
    stream.release.set()
    writer._worker.join(1)
    with pytest.raises(EvidenceWriteError, match="simulated disk failure"):
        writer.check()
    assert not writer.summary()["complete"]


def test_close_persists_all_records_and_refuses_overwrite(tmp_path):
    path = tmp_path / "events.jsonl"
    writer = EvidenceWriter(path)
    for index in range(50):
        writer.write(f"{index}\n")
    writer.close()
    assert path.read_text().splitlines() == list(map(str, range(50)))
    assert writer.summary()["complete"]
    with pytest.raises(EvidenceWriteError, match="closing"):
        writer.write("late\n")
    with pytest.raises(FileExistsError):
        EvidenceWriter(path)
