import importlib.util
from pathlib import Path

import pytest

from robot_config.compiler import ConfigError
from robot_config.runtime_options import runtime_overrides


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "suspended_evidence_test", ROOT / "verification/runners/hil/suspended_wheel_evidence.py"
)
evidence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evidence)


def wheel(sequence, *, session=7, counts=None, speed=None, target=None, output=None):
    return {
        "status_sequence": sequence,
        "transport_session_id": session,
        "motor_enabled_mask": 6,
        "speed_valid_mask": 6,
        "encoder_anomaly_mask": 0,
        "error_flags": 0,
        "latched_error_flags": 0,
        "status_flags": 0,
        "encoder_count": counts or [0, 10, 10, 0],
        "wheel_speed_mps": speed or [0.0] * 4,
        "wheel_target_mps": target or [0.0] * 4,
        "motor_output_permille": output or [0] * 4,
    }


def test_stationary_evidence_needs_distinct_current_session_samples_and_actual_counts():
    values = [wheel(index) for index in range(3)]
    assert evidence.three_sample_stop(values)
    assert not evidence.three_sample_stop(values[:2])
    assert not evidence.three_sample_stop([wheel(1)] * 3)
    assert not evidence.three_sample_stop([wheel(1, session=8), wheel(2), wheel(3)])
    assert not evidence.three_sample_stop([wheel(1, counts=[0, 9, 10, 0]), wheel(2), wheel(3)])
    for key, value in (
        ("wheel_speed_mps", [0, 0.006, 0, 0]),
        ("wheel_target_mps", [0, 0.001, 0, 0]),
        ("motor_output_permille", [0, 1, 0, 0]),
    ):
        changed = [wheel(index) for index in range(3)]
        changed[-1][key] = value
        assert not evidence.three_sample_stop(changed)


def test_stop_cannot_use_queued_samples_from_before_stop_or_stale_samples():
    samples = [wheel(index) for index in range(3)]
    for index, sample in enumerate(samples):
        sample["header"] = {"stamp": {"sec": 10, "nanosec": index * 50_000_000}}
    assert evidence.three_sample_stop(samples, not_before_ns=10_000_000_000, now_ns=10_200_000_000)
    assert not evidence.three_sample_stop(
        samples, not_before_ns=10_050_000_000, now_ns=10_200_000_000
    )
    assert not evidence.three_sample_stop(
        samples, not_before_ns=10_000_000_000, now_ns=10_400_000_000
    )
    assert not evidence.three_sample_stop(samples, now_ns=9_999_999_999)
    assert not evidence.three_sample_stop([wheel(index) for index in range(3)], now_ns=10_200_000_000)


@pytest.mark.parametrize(
    "key,value",
    [
        ("wheel_speed_mps", [0, 0.151, 0, 0]),
        ("motor_output_permille", [0, 401, 0, 0]),
        ("motor_output_permille", [1, 0, 0, 0]),
        ("speed_valid_mask", 2),
        ("error_flags", 1),
        ("encoder_anomaly_mask", 2),
        ("wheel_speed_mps", [0, float("nan"), 0, 0]),
        ("wheel_target_mps", [0, float("inf"), 0, 0]),
    ],
)
def test_wheel_guards_reject_unsafe_or_unobservable_feedback(key, value):
    sample = wheel(1)
    sample[key] = value
    assert evidence.wheel_guard(sample)


def test_direction_is_proven_by_both_wheels_and_includes_reverse_and_turn():
    samples = [wheel(index, speed=[0, -0.02, 0.02, 0]) for index in range(3)]
    assert evidence.direction_observed(samples, 0.0, 0.2)
    assert not evidence.direction_observed(samples, 0.0, -0.2)
    assert not evidence.direction_observed(samples, 0.05, 0.0)
    reverse = [wheel(index, speed=[0, -0.05, -0.05, 0]) for index in range(3)]
    assert evidence.direction_observed(reverse, -0.05, 0.0)
    assert not evidence.direction_observed([wheel(index) for index in range(3)], 0.05, 0.0)
    assert evidence.opposite_feedback(samples[0], 0.05, 0.0)
    assert not evidence.opposite_feedback(reverse[0], -0.05, 0.0)
    assert not evidence.opposite_feedback(reverse[0], 0.0, 0.0)


def test_suspended_environment_is_hashed_before_launch_and_cannot_enable_fake_or_imu_fusion():
    config = {"runtime": {"fusion_mode": "wheel"}}
    result = runtime_overrides(["base-test", "--suspended-wheel"], config, {})
    assert result["runtime"]["test_environment"] == "suspended_wheels"
    assert "--suspended-wheel" in result["runtime"]["cli_args"]
    for arguments in (
        ["base-test", "--suspended-wheel", "--fake-base"],
        ["base-test", "--suspended-wheel", "--ekf-base"],
        ["mapping", "--suspended-wheel"],
    ):
        with pytest.raises(ConfigError):
            runtime_overrides(arguments, config, {})
