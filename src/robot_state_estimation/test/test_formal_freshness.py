from pathlib import Path
import sys
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests"))
from ros_adapter_helpers import adapter_classes


def test_stale_compatibility_closes_formal_gate_and_requires_new_wheel_evidence():
    clock = NS(value=10.0)
    classes = adapter_classes(
        ROOT / "src/robot_state_estimation/robot_state_estimation/formal_odometry_node.py",
        time=NS(monotonic=lambda: clock.value),
        PlatformCompatibilityState=object,
        WheelObservation=object,
        Odometry=object,
    )
    node = classes.FormalOdometryNode.__new__(classes.FormalOdometryNode)
    node.permitted, node.permitted_session = False, 0
    node.fresh_evidence_at = node.fresh_evidence_stamp_ns = None
    node.compatibility_timeout_sec = 0.25
    node._on_compatibility(
        NS(permit_formal_odometry=True, observation_ready=True, observation_session_id=7)
    )
    node.fresh_evidence_at, node.fresh_evidence_stamp_ns = clock.value, 100
    clock.value += 0.30
    node._health_tick()
    assert not node.permitted and node.permitted_session == 0
    assert node.fresh_evidence_at is None
    node._on_compatibility(
        NS(permit_formal_odometry=True, observation_ready=True, observation_session_id=7)
    )
    assert node.permitted
    assert node.fresh_evidence_at is None
