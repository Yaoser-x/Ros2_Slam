import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests"))
from ros_adapter_helpers import adapter_classes


@pytest.mark.parametrize(
    "gyro,covariance,age,valid",
    [
        (1.2, 0.001, 0.0, True),
        (0.0, -1.0, 0.0, False),
        (float("nan"), 0.001, 0.0, False),
        (0.0, float("nan"), 0.0, False),
        (1.2, 0.001, 0.3, False),
        (1.2, 0.001, -0.1, False),
        (1.2, 0.001, 0.15, True),
    ],
)
def test_gyro_field_and_sample_freshness_admission(gyro, covariance, age, valid):
    classes = adapter_classes(
        ROOT / "src/robot_supervision/robot_supervision/motion_supervisor_node.py",
        time=NS(monotonic=lambda: 20.0),
        math=math,
        Imu=object,
        HostMotionCommand=object,
        WheelObservation=object,
    )
    node = classes.MotionSupervisorNode.__new__(classes.MotionSupervisorNode)
    node.imu_timeout = 0.20
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    sample_ns = int(round((10.0 - age) * 1e9))
    message = NS(
        angular_velocity=NS(z=gyro),
        angular_velocity_covariance=[covariance] * 9,
        header=NS(stamp=NS(sec=sample_ns // 1_000_000_000, nanosec=sample_ns % 1_000_000_000)),
    )
    node._on_imu(message)
    assert (node.last_gyro_z is not None) is valid
    if valid:
        assert node.last_imu_at == pytest.approx(20.0 - max(age, 0.0))
    else:
        assert node.last_imu_at is None
