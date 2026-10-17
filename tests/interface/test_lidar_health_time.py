"""Replay scan streams through the real health observer without opening hardware."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]


def observe(monkeypatch, capsys, *, clock_jump=False, fault=None):
    source = (ROOT / "launch_scripts/check_lidar_health.sh").read_text(encoding="utf-8")
    observer = source.split("<<'PY'\n", 1)[1].split("\nPY\n", 1)[0]
    clock, subscription = NS(tick=0), []
    node = NS(destroy_node=lambda: None)
    node.create_subscription = lambda _kind, _topic, callback, _qos: subscription.append(callback)

    def spin_once(_node, timeout_sec):
        clock.tick += 1
        assert clock.tick <= 70, "health sampling must finish despite wall-clock changes"
        stamp = 100_000_000_000 + clock.tick * 100_000_000
        if fault == "stamp" and clock.tick == 10:
            stamp -= 200_000_000
        message = NS(
            header=NS(stamp=NS(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)),
            ranges=[math.inf if fault == "empty" else 1.0] * 425,
            range_min=0.1,
            range_max=12.0,
            scan_time=math.nan if fault == "period" else 0.1,
            time_increment=1e12 if fault == "increment" else 0.1 / 424,
        )
        subscription[0](message)

    monkeypatch.setitem(
        sys.modules,
        "time",
        NS(
            monotonic=lambda: clock.tick / 10,
            time=lambda: 1000 + clock.tick / 10 + (100 if clock_jump and clock.tick >= 3 else 0),
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        NS(
            init=lambda: None,
            create_node=lambda _name: node,
            spin_once=spin_once,
            shutdown=lambda: None,
        ),
    )
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(qos_profile_sensor_data=None))
    monkeypatch.setitem(sys.modules, "sensor_msgs", NS())
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", NS(LaserScan=object))
    monkeypatch.setattr(sys, "argv", ["-", "6"])
    exec(compile(observer, "lidar_health_observer", "exec"), {})
    return dict(line.split("=", 1) for line in capsys.readouterr().out.splitlines() if "=" in line)


def test_receiving_frequency_and_sampling_window_use_monotonic_clock(monkeypatch, capsys):
    result = observe(monkeypatch, capsys, clock_jump=True)
    assert int(result["COUNT"]) == 60
    assert float(result["HZ"]) == pytest.approx(10.0)


@pytest.mark.parametrize("fault", ["stamp", "period", "increment"])
def test_bad_scan_timing_is_reported_even_when_message_rate_is_normal(monkeypatch, capsys, fault):
    result = observe(monkeypatch, capsys, fault=fault)
    assert float(result["HZ"]) == pytest.approx(10.0)
    assert int(result.get("SCAN_METADATA_ERRORS", "0")) > 0


def test_scan_without_valid_distance_is_reported(monkeypatch, capsys):
    result = observe(monkeypatch, capsys, fault="empty")
    assert int(result["VALID_RANGE_COUNT"]) == 0


def test_normal_scan_has_no_metadata_errors_and_retains_size_statistics(monkeypatch, capsys):
    result = observe(monkeypatch, capsys)
    assert int(result["SCAN_METADATA_ERRORS"]) == 0
    assert int(result["VALID_RANGE_COUNT"]) == 60 * 425
    assert result["RANGE_SIZE_SPAN"] == "0"
