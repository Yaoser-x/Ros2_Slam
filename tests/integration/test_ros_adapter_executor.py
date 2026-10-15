"""Real ROS executor checks run in clean processes, isolated from host-test mocks."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.ros
@pytest.mark.parametrize(
    "case",
    [
        "navigation",
        "navigation_cancel_pending",
        "navigation_pose",
        "rearm_late_subscriber",
        "operations",
        "estimation",
    ],
)
def test_adapters_in_single_threaded_ros_executor(case):
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(path) for path in (ROOT / "src").iterdir() if (path / path.name).is_dir()]
        + [environment.get("PYTHONPATH", "")]
    )
    probe = subprocess.run(
        [sys.executable, "-c", "from rclpy.executors import SingleThreadedExecutor"],
        env=environment,
        capture_output=True,
        text=True,
    )
    if probe.returncode and "No module named 'rclpy'" in probe.stderr:
        pytest.skip("real ROS 2 rclpy runtime unavailable; executor acceptance remains pending")
    assert probe.returncode == 0, probe.stderr
    result = subprocess.run(
        [sys.executable, str(ROOT / "tests/ros_adapter_executor_cases.py"), case],
        env=environment,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
