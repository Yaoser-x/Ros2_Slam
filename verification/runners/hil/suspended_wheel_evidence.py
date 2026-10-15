"""Physical suspended-wheel assertions, independent of ROS and serial I/O."""

import math

PHASES = (
    ("forward", 0.05, 0.0, 2.0),
    ("backward", -0.05, 0.0, 2.0),
    ("left", 0.0, 0.20, 2.0),
    ("right", 0.0, -0.20, 2.0),
    ("constant", 0.05, 0.0, 12.0),
    ("creep", 0.01, 0.0, 1.0),
)


def wheel_guard(message):
    for field in ("wheel_speed_mps", "wheel_target_mps", "motor_output_permille"):
        if len(message[field]) != 4 or not all(math.isfinite(value) for value in message[field]):
            return "nonfinite or incomplete wheel feedback"
    if message["error_flags"] or message["latched_error_flags"] or message["status_flags"] & 3:
        return "firmware safety gate closed"
    if message["encoder_anomaly_mask"]:
        return "encoder anomaly"
    if message["motor_enabled_mask"] != 6 or message["speed_valid_mask"] & 6 != 6:
        return "unexpected wheel layout or invalid enabled encoder"
    if any(abs(value) > 0.15 for value in message["wheel_speed_mps"]):
        return "wheel speed exceeded 0.15m/s guard"
    if any(abs(value) > 400 for value in message["motor_output_permille"]):
        return "motor output exceeded 40% guard"
    if any(
        message["motor_output_permille"][index] or message["wheel_target_mps"][index]
        for index in (0, 3)
    ):
        return "disabled motor received output"
    return ""


def opposite_feedback(message, vx, wz):
    expected = (vx - wz * 0.178 / 2.0, vx + wz * 0.178 / 2.0)
    return any(
        abs(message["wheel_speed_mps"][index]) > 0.005
        and message["wheel_speed_mps"][index] * target < 0.0
        for index, target in zip((1, 2), expected)
    )


def three_sample_stop(samples):
    if len(samples) < 3:
        return False
    values = samples[-3:]
    if len({sample["status_sequence"] for sample in values}) != 3:
        return False
    if len({sample["transport_session_id"] for sample in values}) != 1:
        return False
    return all(
        not wheel_guard(sample)
        and not any(sample["wheel_target_mps"])
        and not any(sample["motor_output_permille"])
        and all(abs(value) <= 0.005 for value in sample["wheel_speed_mps"])
        and sample["encoder_count"] == values[-1]["encoder_count"]
        for sample in values
    )


def direction_observed(samples, vx, wz):
    expected = (vx - wz * 0.178 / 2.0, vx + wz * 0.178 / 2.0)
    return (
        sum(
            all(
                sample["wheel_speed_mps"][index] * sign > 0.00001
                for index, sign in zip((1, 2), expected)
            )
            for sample in samples
        )
        >= 3
    )
