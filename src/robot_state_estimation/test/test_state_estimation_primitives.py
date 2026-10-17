import math
import pytest

from robot_state_estimation.imu_processing import classify_quality
from robot_state_estimation.formal_odometry_policy import evidence_matches_odometry
from robot_state_estimation.time_mapper import (
    McuClockMapper,
    SampleDisposition,
    SampleOrderTracker,
)
from robot_state_estimation.wheel_odometry import (
    EncoderOdometry,
    covariance_multiplier,
    se2_increment,
    signed_int32_delta,
)


def make_odom():
    return EncoderOdometry(
        wheel_radius_m=0.05,
        track_width_m=0.20,
        counts_per_revolution=1000,
        max_dt_sec=0.25,
    )


def test_encoder_odometry_validates_geometry_and_count_shape():
    with pytest.raises(ValueError):
        EncoderOdometry(wheel_radius_m=0.0, track_width_m=0.2, counts_per_revolution=1000)
    odom = make_odom()
    with pytest.raises(ValueError):
        odom.update((0, 0, 0), sample_time_sec=0.0)


def test_order_tracker_separates_sessions_duplicates_and_old_samples():
    tracker = SampleOrderTracker()
    assert tracker.update(1, 10) is SampleDisposition.FIRST
    assert tracker.update(1, 10) is SampleDisposition.DUPLICATE
    assert tracker.update(1, 9) is SampleDisposition.OUT_OF_ORDER
    assert tracker.update(1, 11) is SampleDisposition.FORWARD
    assert tracker.update(2, 0) is SampleDisposition.FIRST


def test_sensor_reset_allows_sequence_restart_in_same_transport_session():
    tracker = SampleOrderTracker()
    assert tracker.update(7, 100) is SampleDisposition.FIRST
    assert tracker.update(7, 0) is SampleDisposition.OUT_OF_ORDER
    tracker.reset()
    assert tracker.update(7, 0) is SampleDisposition.FIRST


def test_clock_mapper_uses_sensor_delta_and_handles_wrap():
    mapper = McuClockMapper(min_samples=2)
    first = mapper.update(0xFFFFFFF0, 10.0)
    second = mapper.update(0x00000004, 10.02)
    assert second.sensor_time_sec - first.sensor_time_sec == pytest.approx(0.020)
    assert second.sample_ros_sec > first.sample_ros_sec


def test_encoder_odometry_integrates_straight_and_degrades_single_wheel_anomaly():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    update = odom.update((100, 100, 100, 100), sample_time_sec=0.1)
    assert update.integrated and update.pose.x > 0.0
    before = update.pose
    degraded = odom.update((200, 200, 200, 200), sample_time_sec=0.2, anomaly_mask=1)
    assert degraded.integrated and degraded.degraded
    assert degraded.pose.x > before.x


def test_speed_invalid_does_not_invalidate_encoder_counts():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0, speed_valid_mask=0)
    update = odom.update((100, 100, 100, 100), sample_time_sec=0.1, speed_valid_mask=0)

    assert update.integrated
    assert not update.degraded


def test_whole_side_failure_does_not_integrate_pose():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    failed = odom.update((100, 100, 100, 100), sample_time_sec=0.1, anomaly_mask=0b0011)

    assert not failed.integrated
    assert failed.pose.x == 0.0
    recovered = odom.update((200, 200, 200, 200), sample_time_sec=0.2)
    assert recovered.integrated
    assert recovered.dt == pytest.approx(0.2)
    assert recovered.pose.x == pytest.approx(200 * odom.meters_per_count)


def test_formal_odometry_accepts_bounded_estimator_latency_only():
    latest_wheel_ns = 10_200_000_000
    assert evidence_matches_odometry(10_150_000_000, latest_wheel_ns, 0.25)
    assert evidence_matches_odometry(10_250_000_000, latest_wheel_ns, 0.25)
    assert not evidence_matches_odometry(9_900_000_000, latest_wheel_ns, 0.25)
    assert not evidence_matches_odometry(10_500_000_000, latest_wheel_ns, 0.25)


def test_one_bad_sample_preserves_other_wheels_and_recovers_baseline():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    first = odom.update((100000, 100, 100, 100), sample_time_sec=0.1)
    second = odom.update((200, 200, 200, 200), sample_time_sec=0.2)
    third = odom.update((300, 300, 300, 300), sample_time_sec=0.3)

    assert first.integrated and first.degraded
    assert second.integrated and second.degraded
    assert third.integrated and not third.degraded


def test_encoder_odometry_integrates_rotation_and_arc():
    rotation = make_odom()
    rotation.update((0, 0, 0, 0), sample_time_sec=0.0)
    turned = rotation.update((-100, -100, 100, 100), sample_time_sec=0.1)
    assert turned.integrated
    assert turned.pose.yaw > 0.0
    assert abs(turned.pose.x) < 1e-9

    arc = make_odom()
    arc.update((0, 0, 0, 0), sample_time_sec=0.0)
    curved = arc.update((50, 50, 100, 100), sample_time_sec=0.1)
    assert curved.integrated
    assert curved.pose.x > 0.0 and curved.pose.y > 0.0 and curved.pose.yaw > 0.0


def test_se2_exponential_matches_exact_arc_and_is_stable_at_small_angle():
    distance, angle = 0.8, 0.6
    dx, dy = se2_increment(distance, angle, 0.0)
    radius = distance / angle
    assert dx == pytest.approx(radius * math.sin(angle))
    assert dy == pytest.approx(radius * (1.0 - math.cos(angle)))

    tiny_dx, tiny_dy = se2_increment(1.0, 1.0e-12, 0.0)
    assert tiny_dx == pytest.approx(1.0, abs=1.0e-15)
    assert tiny_dy == pytest.approx(0.5e-12, abs=1.0e-18)


def test_correlated_wheel_noise_produces_symmetric_positive_covariance():
    odom = EncoderOdometry(
        wheel_radius_m=0.05,
        track_width_m=0.20,
        counts_per_revolution=1000,
        left_right_correlation=0.4,
        turn_noise_gain=0.5,
    )
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    update = odom.update((50, 50, 100, 100), sample_time_sec=0.1)
    covariance = update.pose_covariance

    assert covariance[1] == pytest.approx(covariance[3])
    assert covariance[2] == pytest.approx(covariance[6])
    assert covariance[5] == pytest.approx(covariance[7])
    assert covariance[0] >= 0.0
    assert covariance[4] >= 0.0
    assert covariance[8] >= 0.0


def test_invalid_wheel_correlation_is_rejected():
    with pytest.raises(ValueError, match="correlation"):
        EncoderOdometry(
            wheel_radius_m=0.05,
            track_width_m=0.20,
            counts_per_revolution=1000,
            left_right_correlation=1.0,
        )


def test_default_m2_m3_layout_does_not_halve_distance():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0, enabled_mask=0b0110, speed_valid_mask=0b0110)
    update = odom.update(
        (0, 100, 100, 0),
        sample_time_sec=0.1,
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
    )
    expected = 100 * odom.meters_per_count
    assert update.integrated
    assert update.left_distance == pytest.approx(expected)
    assert update.right_distance == pytest.approx(expected)


def test_layout_change_resets_integration_baseline():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    changed = odom.update(
        (0, 100, 100, 0),
        sample_time_sec=0.1,
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
    )
    assert not changed.integrated

    odom.reset_sample_baseline()
    missing_side = odom.update(
        (0, 0, 0, 0),
        sample_time_sec=0.2,
        enabled_mask=0b0010,
        speed_valid_mask=0b0010,
    )
    assert not missing_side.integrated


def test_reset_generation_resets_baseline_even_with_same_wire_session():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0, transport_session_id=7, reset_generation=1)
    reset = odom.update(
        (100, 100, 100, 100),
        sample_time_sec=0.1,
        transport_session_id=7,
        reset_generation=2,
    )
    assert not reset.integrated


@pytest.mark.parametrize(
    "enabled,counts",
    [
        (0b0110, (0, 100, 100, 0)),
        (0b1111, (100, 100, 100, 100)),
        (0b1110, (0, 100, 100, 100)),
        (0b0111, (100, 100, 100, 0)),
    ],
)
def test_two_and_four_drive_layouts_share_one_distance_algorithm(enabled, counts):
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0, enabled_mask=enabled, speed_valid_mask=enabled)
    update = odom.update(
        counts, sample_time_sec=0.1, enabled_mask=enabled, speed_valid_mask=enabled
    )
    assert update.integrated
    assert update.pose.x == pytest.approx(100 * odom.meters_per_count)


def test_implausible_enabled_wheel_jump_is_rejected():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0, enabled_mask=0b0110)
    jump = odom.update((0, 100000, 100, 0), sample_time_sec=0.1, enabled_mask=0b0110)
    assert not jump.integrated


def test_time_regression_and_long_gap_do_not_move_pose():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=1.0)
    regressed = odom.update((10, 10, 10, 10), sample_time_sec=0.9)
    assert not regressed.integrated and regressed.pose.x == 0.0
    long_gap = odom.update((20, 20, 20, 20), sample_time_sec=2.0)
    assert not long_gap.integrated and long_gap.pose.x == 0.0


def test_covariance_multiplier_increases_for_age_turn_disagreement_and_quality():
    nominal = covariance_multiplier(
        wheel_speeds=(0.1, 0.1, 0.1, 0.1),
        speed_valid_mask=0x0F,
        sample_age_sec=0.0,
        turn_rate=0.0,
    )
    degraded = covariance_multiplier(
        wheel_speeds=(0.1, 0.3, 0.1, -0.1),
        speed_valid_mask=0x07,
        sample_age_sec=0.1,
        turn_rate=0.5,
        quality_flags=3,
    )
    assert nominal == 1.0
    assert degraded > nominal

    two_wheel = covariance_multiplier(
        wheel_speeds=(0.0, 0.1, 0.1, 0.0),
        enabled_mask=0b0110,
        speed_valid_mask=0b0110,
        sample_age_sec=0.0,
        turn_rate=0.0,
    )
    assert two_wheel == 3.0


def test_signed_int32_delta_wraps():
    assert signed_int32_delta(-2147483648, 2147483647) == 1
    assert signed_int32_delta(2147483647, -2147483648) == -1


def test_twist_covariance_is_per_sample_not_accumulated_pose_uncertainty():
    odom = make_odom()
    odom.update((0, 0, 0, 0), sample_time_sec=0.0)
    updates = []
    for index in range(1, 51):
        updates.append(
            odom.update(
                (index * 10,) * 4,
                sample_time_sec=index * 0.1,
                hard_max_wheel_peripheral_speed_mps=2.0,
            )
        )
    assert updates[-1].pose_covariance[0] > updates[0].pose_covariance[0]
    assert updates[-1].twist_covariance == pytest.approx(updates[0].twist_covariance)


def test_imu_field_quality_degrades_orientation_independently():
    validity = classify_quality(1 << 2, status_flags=1)
    assert validity.gyro_valid
    assert validity.accel_valid
    assert not validity.orientation_valid


@pytest.mark.parametrize(
    "quality_flags,status_flags,expected",
    [
        (1 << 3, 7, (True, True, True, False)),
        (1 << 0, 5, (False, True, True, True)),
        (1 << 1, 5, (True, False, True, True)),
        (1 << 2, 5, (True, True, False, True)),
        ((1 << 0) | (1 << 3), 7, (False, True, True, False)),
        (0x0F, 7, (False, False, False, False)),
        (1 << 8, 1, (False, False, False, False)),
        ((1 << 8) | (1 << 3), 7, (False, False, False, False)),
        (1 << 3, 0, (False, False, False, False)),
        (0, 5, (False, False, False, False)),
        (1 << 7, 5, (False, False, False, False)),
        (1 << 12, 5, (False, False, False, False)),
    ],
)
def test_imu_summary_error_preserves_field_validity(quality_flags, status_flags, expected):
    # The real v3 encoder also sets the summary ERROR bit for timestamp-only
    # invalidity. It must not discard gyro/accel when their own fields are valid.
    validity = classify_quality(quality_flags, status_flags)
    assert (
        validity.gyro_valid,
        validity.accel_valid,
        validity.orientation_valid,
        validity.timestamp_valid,
    ) == expected
