"""Field-level IMU validation and unit conversion."""

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


GRAVITY_MPS2 = 9.80665
IMU_FLAG_ONLINE = 1
IMU_FLAG_ERROR = 4
IMU_QUALITY_GYRO_INVALID = 1 << 0
IMU_QUALITY_ACCEL_INVALID = 1 << 1
IMU_QUALITY_ORIENTATION_INVALID = 1 << 2
IMU_QUALITY_TIMESTAMP_INVALID = 1 << 3
IMU_QUALITY_GYRO_WARNING = 1 << 4
IMU_QUALITY_ACCEL_WARNING = 1 << 5
IMU_QUALITY_ORIENTATION_WARNING = 1 << 6
IMU_QUALITY_TIMESTAMP_WARNING = 1 << 7
IMU_QUALITY_FATAL = 1 << 8
IMU_QUALITY_SENSOR_RESET = 1 << 9


@dataclass(frozen=True)
class ImuFieldValidity:
    gyro_valid: bool
    accel_valid: bool
    orientation_valid: bool
    timestamp_valid: bool
    warning: bool


def classify_quality(quality_flags: int, status_flags: int) -> ImuFieldValidity:
    flags = int(quality_flags)
    field_invalid_mask = (
        IMU_QUALITY_GYRO_INVALID
        | IMU_QUALITY_ACCEL_INVALID
        | IMU_QUALITY_ORIENTATION_INVALID
        | IMU_QUALITY_TIMESTAMP_INVALID
    )
    # ERROR summarizes any invalid field in Upper v3, including sensor time.
    # Keep other fields usable; an error without field detail remains invalid.
    unexplained_error = bool(int(status_flags) & IMU_FLAG_ERROR) and not bool(
        flags & field_invalid_mask
    )
    base_valid = (
        bool(int(status_flags) & IMU_FLAG_ONLINE)
        and not unexplained_error
        and not bool(flags & IMU_QUALITY_FATAL)
    )
    warning_mask = (
        IMU_QUALITY_GYRO_WARNING
        | IMU_QUALITY_ACCEL_WARNING
        | IMU_QUALITY_ORIENTATION_WARNING
        | IMU_QUALITY_TIMESTAMP_WARNING
        | IMU_QUALITY_SENSOR_RESET
    )
    return ImuFieldValidity(
        base_valid and not bool(flags & IMU_QUALITY_GYRO_INVALID),
        base_valid and not bool(flags & IMU_QUALITY_ACCEL_INVALID),
        base_valid and not bool(flags & IMU_QUALITY_ORIENTATION_INVALID),
        base_valid and not bool(flags & IMU_QUALITY_TIMESTAMP_INVALID),
        bool(flags & warning_mask),
    )


def finite_vector(values: Iterable[float], length: int) -> Optional[Tuple[float, ...]]:
    converted = tuple(float(value) for value in values)
    if len(converted) != length or not all(math.isfinite(value) for value in converted):
        return None
    return converted


def normalize_quaternion(values: Iterable[float]) -> Optional[Tuple[float, float, float, float]]:
    converted = finite_vector(values, 4)
    if converted is None:
        return None
    norm = math.sqrt(sum(value * value for value in converted))
    if norm <= 1e-9:
        return None
    normalized = tuple(value / norm for value in converted)
    return normalized[0], normalized[1], normalized[2], normalized[3]
