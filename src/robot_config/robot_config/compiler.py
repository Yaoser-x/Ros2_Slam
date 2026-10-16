"""Compile canonical YAML fragments into one validated effective configuration."""

from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import subprocess
import tempfile
import shlex
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import yaml

try:
    import jsonschema
except ImportError:  # pragma: no cover - packaging/rosdep supplies this at runtime
    jsonschema = None


class ConfigError(ValueError):
    pass


class StrictSafeLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate key: {key}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


StrictSafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _load(path: Path) -> dict[str, Any]:
    try:
        value = yaml.load(path.read_text(encoding="utf-8"), Loader=StrictSafeLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"cannot load config {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigError(f"config root must be a mapping: {path}")
    _reject_nonfinite(value, path.as_posix())
    return value


def _reject_nonfinite(value: Any, path: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ConfigError(f"non-finite value at {path}")
    if isinstance(value, Mapping):
        for key, child in value.items():
            _reject_nonfinite(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_nonfinite(child, f"{path}[{index}]")


def _merge(base: dict[str, Any], overlay: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = _merge(dict(result[key]), value)
        else:
            result[key] = deepcopy(value)
    return result


def _canonical_hash(config: Mapping[str, Any]) -> str:
    payload = json.dumps(config, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _positive(config: Mapping[str, Any], path: str) -> float:
    value: Any = config
    for part in path.split("."):
        if not isinstance(value, Mapping) or part not in value:
            raise ConfigError(f"missing required config: {path}")
        value = value[part]
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{path} must be numeric") from exc
    if converted <= 0.0:
        raise ConfigError(f"{path} must be positive")
    return converted


def validate_effective_config(config: Mapping[str, Any]) -> None:
    _reject_nonfinite(config, "effective_config")
    if int(config.get("schema_version", 0)) != 1:
        raise ConfigError("schema_version must be 1")
    if int(config.get("platform_api_version", 0)) != 5:
        raise ConfigError("platform_api_version must be 5")
    if int(config.get("protocol", {}).get("version", 0)) != 3:
        raise ConfigError("only upper protocol v3 is supported")
    topics = config.get("platform", {}).get("topics", {})
    if not isinstance(topics, Mapping) or not topics:
        raise ConfigError("platform.topics must be a non-empty mapping")
    if any(
        not isinstance(value, str) or not value or value.startswith("/")
        for value in topics.values()
    ):
        raise ConfigError("platform topics must be non-empty relative names")
    if len(set(topics.values())) != len(topics):
        raise ConfigError("platform topics must be unique")

    positive_paths = (
        "motion.soft_max_linear_mps",
        "motion.soft_max_angular_radps",
        "motion.hard_max_linear_mps",
        "motion.hard_max_angular_radps",
        "motion.max_linear_accel_mps2",
        "motion.max_angular_accel_radps2",
        "motion.max_linear_jerk_mps3",
        "motion.max_angular_jerk_radps3",
        "calibration.drive.wheel_radius_m",
        "calibration.drive.wheel_track_width_m",
        "calibration.drive.encoder_counts_per_revolution",
        "calibration.limits.hard_max_wheel_peripheral_speed_mps",
    )
    for path in positive_paths:
        _positive(config, path)

    keepalive = _positive(config, "safety.timing.drive_keepalive_sec")
    publish_period = _positive(config, "safety.timing.host_publish_period_sec")
    command_deadline = _positive(config, "safety.timing.host_command_deadline_sec")
    command_lifespan = _positive(config, "safety.timing.host_command_lifespan_sec")
    bridge_timeout = _positive(config, "safety.timing.bridge_command_timeout_sec")
    firmware_timeout = _positive(config, "safety.timing.firmware_upper_timeout_sec")
    if not keepalive < bridge_timeout < firmware_timeout:
        raise ConfigError("require keepalive < bridge timeout < firmware timeout")
    if not publish_period < command_deadline <= command_lifespan < bridge_timeout:
        raise ConfigError("require host publish period < deadline <= lifespan < bridge timeout")
    if _positive(config, "safety.timing.command_ack_timeout_sec") > bridge_timeout:
        raise ConfigError("command ACK timeout must not exceed bridge command timeout")

    motion = config.get("motion", {})
    if float(motion.get("soft_max_linear_mps", 0.0)) > float(
        motion.get("hard_max_linear_mps", 0.0)
    ):
        raise ConfigError("soft linear limit exceeds hard limit")
    if float(motion.get("soft_max_angular_radps", 0.0)) > float(
        motion.get("hard_max_angular_radps", 0.0)
    ):
        raise ConfigError("soft angular limit exceeds hard limit")
    drive = config.get("calibration", {}).get("drive", {})
    odometry_limits = config.get("calibration", {}).get("limits", {})
    theoretical_wheel_speed = (
        abs(float(motion.get("hard_max_linear_mps", 0.0)))
        + abs(float(motion.get("hard_max_angular_radps", 0.0)))
        * float(drive.get("wheel_track_width_m", 0.0))
        / 2.0
    )
    if float(odometry_limits.get("hard_max_wheel_peripheral_speed_mps", 0.0)) < (
        theoretical_wheel_speed
    ):
        raise ConfigError("single-wheel speed limit is below the hard chassis envelope")
    noise = config.get("calibration", {}).get("odom_covariance", {})
    nonnegative_noise = (
        "left_variance_floor_m2",
        "left_variance_per_meter",
        "right_variance_floor_m2",
        "right_variance_per_meter",
        "turn_noise_gain",
        "slip_noise_gain",
        "degraded_single_wheel_multiplier",
        "unobserved_pose_variance",
        "unobserved_twist_variance",
    )
    for name in nonnegative_noise:
        value = float(noise.get(name, float("nan")))
        if not math.isfinite(value) or value < 0.0:
            raise ConfigError(f"calibration.odom_covariance.{name} must be finite and nonnegative")
    for name in ("unobserved_pose_variance", "unobserved_twist_variance"):
        if float(noise[name]) <= 0.0:
            raise ConfigError(f"calibration.odom_covariance.{name} must be positive")
    correlation = float(noise.get("left_right_correlation", float("nan")))
    if not math.isfinite(correlation) or not -1.0 < correlation < 1.0:
        raise ConfigError("left_right_correlation must be finite and strictly between -1 and 1")

    footprint = config.get("hardware", {}).get("footprint_m", [])
    if not isinstance(footprint, list) or len(footprint) < 3:
        raise ConfigError("hardware.footprint_m must contain at least three points")
    if int(config.get("hardware", {}).get("baudrate", 0)) <= 0:
        raise ConfigError("hardware.baudrate must be positive")
    if config.get("release_mode") and config.get("unsafe_development_mode"):
        raise ConfigError("release mode forbids unsafe_development_mode")
    calibration = config.get("calibration", {})
    if config.get("release_mode") and calibration.get("status") != "final":
        raise ConfigError("release mode requires final calibration")
    if config.get("release_mode") and calibration.get("imu", {}).get("status") != "final":
        raise ConfigError("release mode requires final IMU calibration")
    if config.get("release_mode") and calibration.get("lidar", {}).get("status") != "final":
        raise ConfigError("release mode requires final lidar calibration")
    bundle = config.get("calibration_bundle", {})
    if bundle.get("wheel_layout", {}).get("order") != ["LF", "LR", "RF", "RR"]:
        raise ConfigError("calibration_bundle wheel order must be LF/LR/RF/RR")
    expected_mask = int(bundle.get("wheel_layout", {}).get("expected_enabled_mask", 0))
    if expected_mask & ~0x0F or not (expected_mask & 0x03) or not (expected_mask & 0x0C):
        raise ConfigError("expected wheel mask must contain at least one wheel on each side")
    geometry = bundle.get("geometry", {})
    geometry_pairs = (
        ("wheel_radius_m", "wheel_radius_m"),
        ("effective_track_width_m", "wheel_track_width_m"),
        ("counts_per_revolution", "encoder_counts_per_revolution"),
    )
    for bundle_name, drive_name in geometry_pairs:
        if not math.isclose(
            float(geometry.get(bundle_name, float("nan"))),
            float(drive.get(drive_name, float("nan"))),
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ConfigError(f"calibration bundle geometry differs at {bundle_name}")
    lidar = calibration["lidar"]
    if (
        bundle["lidar"]["translation"] != lidar["translation_m"]
        or bundle["lidar"]["rotation"] != lidar["rotation_rpy_rad"]
        or bundle["lidar"]["frame"] != config["platform"]["frames"]["laser"]
        or lidar["frame"] != config["platform"]["frames"]["laser"]
    ):
        raise ConfigError(
            "calibration bundle lidar differs from canonical lidar calibration/frames"
        )
    imu = calibration.get("imu", {})
    for name in ("angular_velocity_stddev", "linear_acceleration_stddev"):
        values = imu.get(name, ())
        if len(values) != 3 or any(float(value) <= 0.0 for value in values):
            raise ConfigError(f"calibration.imu.{name} must contain three positive values")
    if float(imu.get("orientation_stddev", 0.0)) <= 0.0:
        raise ConfigError("calibration.imu.orientation_stddev must be positive")
    supervisor = config.get("motion_supervisor", {})
    threshold_pairs = (
        ("wheel_pair_warn_mps", "wheel_pair_critical_mps"),
        ("tracking_warn_mps", "tracking_critical_mps"),
        ("yaw_warn_radps", "yaw_critical_radps"),
    )
    for warn_name, critical_name in threshold_pairs:
        if (
            not 0.0
            <= float(supervisor.get(warn_name, -1.0))
            < float(supervisor.get(critical_name, -1.0))
        ):
            raise ConfigError(f"motion supervisor requires {warn_name} < {critical_name}")
    clear = float(supervisor.get("clear_score", -1.0))
    warn = float(supervisor.get("warn_score", -1.0))
    degraded = float(supervisor.get("degraded_score", -1.0))
    critical = float(supervisor.get("critical_score", -1.0))
    if not 0.0 <= clear < warn <= degraded < critical <= 1.0:
        raise ConfigError("motion supervisor score thresholds are inconsistent")
    profile = str(config.get("profile", ""))
    if config.get("runtime", {}).get("mode") != profile:
        raise ConfigError("runtime.mode must match the selected profile")
    if (
        config.get("release_mode")
        and int(bundle.get("identity", {}).get("firmware_parameter_crc32", 0)) == 0
    ):
        raise ConfigError("release mode requires a measured firmware parameter CRC")


def _source_files(config_root: Path, robot_id: str, profile: str) -> tuple[Path, ...]:
    return (
        config_root / "platform.yaml",
        config_root / "safety.yaml",
        config_root / "hardware" / f"{robot_id}.yaml",
        config_root / "calibration" / robot_id / "chassis_v1.yaml",
        config_root / "calibration" / robot_id / "imu_v1.yaml",
        config_root / "calibration" / robot_id / "lidar_v1.yaml",
        config_root / "profiles" / f"{profile}.yaml",
    )


def _component_files(config_root: Path) -> tuple[Path, ...]:
    root = config_root / "components"
    return (
        root / "lidar.yaml",
        root / "slam" / "base.yaml",
        root / "slam" / "quality.yaml",
        root / "slam" / "high_resolution_experimental.yaml",
        root / "slam" / "legacy_fast_experimental.yaml",
        root / "nav2" / "base.yaml",
        root / "nav2" / "navigation.yaml",
        root / "ekf" / "wheel.yaml",
        root / "ekf" / "wheel_imu.yaml",
    )


def _record_provenance(value: Any, source: str, result: dict[str, str], prefix="") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            _record_provenance(child, source, result, path)
    elif isinstance(value, list):
        result[prefix] = source
    else:
        result[prefix] = source


def _validate_schema(config_root: Path, effective: Mapping[str, Any]) -> None:
    if jsonschema is None:
        raise ConfigError("python3-jsonschema is required for strict config validation")
    schema_path = config_root / "schemas" / "effective-config-v1.schema.yaml"
    schema = _load(schema_path)
    try:
        # ROS 2 Humble on Ubuntu 22.04 ships jsonschema 3.x.  The effective
        # configuration schema intentionally uses only Draft 7 keywords, so
        # validate with the oldest explicitly supported runtime contract
        # instead of depending on Draft202012Validator from jsonschema 4.x.
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(effective)
    except jsonschema.SchemaError as exc:
        raise ConfigError(f"invalid effective config schema: {exc.message}") from exc
    except jsonschema.ValidationError as exc:
        location = ".".join(str(part) for part in exc.absolute_path) or "<root>"
        raise ConfigError(f"schema validation failed at {location}: {exc.message}") from exc


def load_effective_config(
    config_root: Path,
    *,
    robot_id: str = "robot_001",
    profile: str = "mapping",
    overrides: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    files = _source_files(config_root, robot_id, profile)
    effective: dict[str, Any] = {}
    for path in files:
        effective = _merge(effective, _load(path))
    if overrides:
        effective = _merge(effective, overrides)
    effective["robot_id"] = robot_id
    effective["profile"] = profile
    effective["component_sha256"] = {
        path.relative_to(config_root).as_posix(): _sha256(path)
        for path in _component_files(config_root)
    }
    _validate_schema(config_root, effective)
    validate_effective_config(effective)
    without_hash = deepcopy(effective)
    without_hash.pop("config_sha256", None)
    without_hash["calibration_bundle"]["identity"].pop("upper_config_sha256", None)
    effective["config_sha256"] = _canonical_hash(without_hash)
    effective["calibration_bundle"]["identity"]["upper_config_sha256"] = effective["config_sha256"]
    _validate_schema(config_root, effective)
    return effective


def ros_parameters(config: Mapping[str, Any]) -> dict[str, Any]:
    timing = config["safety"]["timing"]
    motion = config["motion"]
    drive = config["calibration"]["drive"]
    covariance = config["calibration"]["odom_covariance"]
    odometry_limits = config["calibration"]["limits"]
    imu = config["calibration"]["imu"]
    supervisor = config["motion_supervisor"]
    topics = config["platform"]["topics"]
    frames = config["platform"]["frames"]
    bundle = config["calibration_bundle"]
    identity = bundle["identity"]
    expected_mask = int(bundle["wheel_layout"]["expected_enabled_mask"])
    common = {
        "config_sha256": config["config_sha256"],
        "unsafe_development_mode": bool(config.get("unsafe_development_mode", False)),
    }
    return {
        "stm32_bridge": {
            "ros__parameters": {
                **common,
                "port": config["hardware"]["base_port"],
                "baudrate": config["hardware"]["baudrate"],
                "dds_publication_mode": config["hardware"]["bridge_publication_mode"],
                "protocol_version": 3,
                "host_motion_command_topic": topics["host_motion_command"],
                "chassis_link_state_topic": topics["chassis_link_state"],
                "firmware_control_state_topic": topics["firmware_control_state"],
                "firmware_info_topic": topics["firmware_info"],
                "wheel_observation_topic": topics["wheel_observation"],
                "imu_observation_topic": topics["imu_observation"],
                "diagnostics_topic": topics["diagnostics"],
                "base_frame_id": frames["base"],
                "imu_frame_id": frames["imu"],
                "publish_tf": False,
                "cmd_timeout": timing["bridge_command_timeout_sec"],
                "drive_keepalive_sec": timing["drive_keepalive_sec"],
                "status_timeout": timing["status_timeout_sec"],
                "command_ack_timeout_sec": timing["command_ack_timeout_sec"],
                "max_command_age_sec": timing["max_command_age_sec"],
                "command_lifespan_sec": timing["host_command_lifespan_sec"],
                "hard_max_linear_mps": motion["hard_max_linear_mps"],
                "hard_max_angular_radps": motion["hard_max_angular_radps"],
            }
        },
        "wheel_odometry": {
            "ros__parameters": {
                **common,
                "observation_topic": topics["wheel_observation"],
                "odom_topic": topics["wheel_odom"],
                "frame_id": frames["odom"],
                "child_frame_id": frames["base"],
                "wheel_radius": drive["wheel_radius_m"],
                "wheel_track_width": drive["wheel_track_width_m"],
                "encoder_counts_per_revolution": drive["encoder_counts_per_revolution"],
                "odom_linear_scale": drive["odom_linear_scale"],
                "odom_angular_scale": drive["odom_angular_scale"],
                "odom_angular_sign": drive["odom_angular_sign"],
                "max_dt_sec": timing["status_timeout_sec"],
                "hard_max_wheel_peripheral_speed_mps": odometry_limits[
                    "hard_max_wheel_peripheral_speed_mps"
                ],
                "left_variance_floor_m2": covariance["left_variance_floor_m2"],
                "left_variance_per_meter": covariance["left_variance_per_meter"],
                "right_variance_floor_m2": covariance["right_variance_floor_m2"],
                "right_variance_per_meter": covariance["right_variance_per_meter"],
                "left_right_correlation": covariance["left_right_correlation"],
                "turn_noise_gain": covariance["turn_noise_gain"],
                "slip_noise_gain": covariance["slip_noise_gain"],
                "degraded_single_wheel_multiplier": covariance["degraded_single_wheel_multiplier"],
                "unobserved_pose_variance": covariance["unobserved_pose_variance"],
                "unobserved_twist_variance": covariance["unobserved_twist_variance"],
                "compatibility_state_topic": topics["platform_compatibility_state"],
            }
        },
        "platform_compatibility": {
            "ros__parameters": {
                "firmware_info_topic": topics["firmware_info"],
                "chassis_link_state_topic": topics["chassis_link_state"],
                "wheel_observation_topic": topics["wheel_observation"],
                "compatibility_state_topic": topics["platform_compatibility_state"],
                "expected_firmware_commit": identity["firmware_commit"],
                "expected_hardware_revision": identity["hardware_revision"],
                "required_capabilities": 31,
                "expected_parameter_crc32": identity["firmware_parameter_crc32"],
                "expected_enabled_mask": expected_mask,
            }
        },
        "imu_adapter": {
            "ros__parameters": {
                **common,
                "observation_topic": topics["imu_observation"],
                "imu_topic": topics["imu"],
                "frame_id": frames["imu"],
                "use_orientation": imu["use_orientation"],
                "orientation_stddev": imu["orientation_stddev"],
                "angular_velocity_stddev": imu["angular_velocity_stddev"],
                "linear_acceleration_stddev": imu["linear_acceleration_stddev"],
            }
        },
        "formal_odometry": {
            "ros__parameters": {
                "input_topic": "odometry/filtered_internal",
                "output_topic": topics["odom"],
                "compatibility_state_topic": topics["platform_compatibility_state"],
                "wheel_observation_topic": topics["wheel_observation"],
                "evidence_max_age_sec": timing["status_timeout_sec"],
                "compatibility_timeout_sec": timing["status_timeout_sec"],
            }
        },
        "navigation_guard": {
            "ros__parameters": {
                "state_timeout_sec": timing["status_timeout_sec"],
                "backend_response_timeout_sec": 1.0,
                "goal_pose_topic": "goal_pose",
                "chassis_link_state_topic": topics["chassis_link_state"],
                "firmware_control_state_topic": topics["firmware_control_state"],
                "motion_supervision_topic": topics["motion_supervision_state"],
                "compatibility_state_topic": topics["platform_compatibility_state"],
                "host_control_state_topic": topics["host_control_state"],
                "navigation_guard_state_topic": topics["navigation_guard_state"],
            }
        },
        "chassis_ops": {
            "ros__parameters": {
                "operation_timeout_sec": 1.0,
                "state_timeout_sec": timing["status_timeout_sec"],
                "firmware_control_state_topic": topics["firmware_control_state"],
                "firmware_info_topic": topics["firmware_info"],
                "chassis_link_state_topic": topics["chassis_link_state"],
            }
        },
        "motion_supervisor": {
            "ros__parameters": {
                **common,
                "wheel_observation_topic": topics["wheel_observation"],
                "imu_topic": topics["imu"],
                "host_motion_command_topic": topics["host_motion_command"],
                "motion_supervision_topic": topics["motion_supervision_state"],
                "wheel_track_width": drive["wheel_track_width_m"],
                "observation_timeout_sec": timing["status_timeout_sec"],
                "imu_timeout_sec": 0.20,
                "command_timeout_sec": timing["mux_source_timeout_sec"],
                **{
                    key: value
                    for key, value in supervisor.items()
                    if key != "covariance_max_multiplier"
                },
            }
        },
        "fake_base": {
            "ros__parameters": {
                **common,
                "host_motion_command_topic": topics["host_motion_command"],
                "wheel_observation_topic": topics["wheel_observation"],
                "imu_observation_topic": topics["imu_observation"],
                "chassis_link_state_topic": topics["chassis_link_state"],
                "firmware_control_state_topic": topics["firmware_control_state"],
                "firmware_info_topic": topics["firmware_info"],
                "diagnostics_topic": topics["diagnostics"],
                "base_frame_id": frames["base"],
                "imu_frame_id": frames["imu"],
                "wheel_radius": drive["wheel_radius_m"],
                "wheel_track_width": drive["wheel_track_width_m"],
                "encoder_counts_per_revolution": drive["encoder_counts_per_revolution"],
                "expected_enabled_mask": expected_mask,
                "cmd_timeout": timing["bridge_command_timeout_sec"],
                "publish_hz": 50.0,
            }
        },
        "cmd_vel_mux": {
            "ros__parameters": {
                **common,
                "host_motion_command_topic": topics["host_motion_command"],
                "linear_limit": motion["soft_max_linear_mps"],
                "angular_limit": motion["soft_max_angular_radps"],
                "max_linear_accel": motion["max_linear_accel_mps2"],
                "max_angular_accel": motion["max_angular_accel_radps2"],
                "max_linear_jerk": motion["max_linear_jerk_mps3"],
                "max_angular_jerk": motion["max_angular_jerk_radps3"],
                "timeout_sec": timing["mux_source_timeout_sec"],
                "nav_guard_timeout_sec": timing["status_timeout_sec"],
                "publish_hz": 1.0 / timing["host_publish_period_sec"],
                "command_deadline_sec": timing["host_command_deadline_sec"],
                "command_lifespan_sec": timing["host_command_lifespan_sec"],
                "motion_supervision_topic": topics["motion_supervision_state"],
                "chassis_link_state_topic": topics["chassis_link_state"],
                "host_control_state_topic": topics["host_control_state"],
                "navigation_guard_state_topic": topics["navigation_guard_state"],
                "require_motion_supervision": True,
                "supervision_timeout_sec": timing["status_timeout_sec"],
                "compatibility_state_topic": topics["platform_compatibility_state"],
                "compatibility_timeout_sec": timing["status_timeout_sec"],
                "rearm_quiet_sec": timing["mux_source_timeout_sec"],
                "rearm_disable_retry_sec": timing["host_publish_period_sec"],
            }
        },
    }


def compile_effective_config(
    config_root: Path,
    output_dir: Path,
    *,
    robot_id: str = "robot_001",
    profile: str = "mapping",
    overrides: Mapping[str, Any] | None = None,
) -> dict[str, Path]:
    if output_dir.exists():
        raise ConfigError(f"output directory already exists: {output_dir}")
    effective = load_effective_config(
        config_root, robot_id=robot_id, profile=profile, overrides=overrides
    )
    upper_commit, upper_dirty = _git_identity(config_root)
    if effective.get("release_mode") and (upper_commit is None or upper_dirty is not False):
        raise ConfigError("release mode requires a clean, traceable Git worktree")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.", dir=output_dir.parent))
    try:
        effective_path = temporary / "effective-config.yaml"
        ros_path = temporary / "ros-params.yaml"
        manifest_path = temporary / "manifest.yaml"
        _write_fsynced(effective_path, yaml.safe_dump(effective, sort_keys=True))
        _write_fsynced(ros_path, yaml.safe_dump(ros_parameters(effective), sort_keys=False))
        _render_component_artifacts(config_root, temporary, effective)

        provenance: dict[str, str] = {}
        source_hashes = {}
        for source in (
            *_source_files(config_root, robot_id, profile),
            *_component_files(config_root),
        ):
            relative = source.relative_to(config_root).as_posix()
            _record_provenance(_load(source), relative, provenance)
            source_hashes[relative] = _sha256(source)
        if overrides:
            _record_provenance(overrides, "<runtime-overrides>", provenance)
        safety_override_paths = sorted(
            path
            for path, source in provenance.items()
            if source == "<runtime-overrides>"
            and path.split(".", 1)[0]
            in {
                "safety",
                "motion",
                "motion_supervisor",
                "release_mode",
                "unsafe_development_mode",
            }
        )
        artifacts = {
            path.relative_to(temporary).as_posix(): _sha256(path)
            for path in sorted(temporary.rglob("*"))
            if path.is_file() and path != manifest_path
        }
        manifest = {
            "schema_version": 2,
            "robot_id": robot_id,
            "profile": profile,
            "config_sha256": effective["config_sha256"],
            "upper_commit": upper_commit,
            "upper_dirty": upper_dirty,
            "source_sha256": source_hashes,
            "artifact_sha256": artifacts,
            "provenance": dict(sorted(provenance.items())),
            "safety_override_audit": {
                "present": bool(safety_override_paths),
                "paths": safety_override_paths,
            },
        }
        _write_fsynced(manifest_path, yaml.safe_dump(manifest, sort_keys=False))
        os.replace(temporary, output_dir)
        _fsync_directory(output_dir.parent)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return {
        "effective": output_dir / "effective-config.yaml",
        "ros_params": output_dir / "ros-params.yaml",
        "manifest": output_dir / "manifest.yaml",
    }


def _write_fsynced(path: Path, content: str) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())


def _fsync_directory(path: Path) -> None:
    if os.name == "nt":
        # Windows does not expose directory fsync; files are still flushed
        # before the atomic rename.  Linux deployment retains directory fsync.
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _render_component_artifacts(
    config_root: Path, output_dir: Path, effective: Mapping[str, Any]
) -> None:
    components = config_root / "components"
    frames = effective["platform"]["frames"]
    topics = effective["platform"]["topics"]
    runtime = effective["runtime"]

    def render_platform_fields(value):
        if not isinstance(value, dict):
            return
        for key, child in value.items():
            if isinstance(child, dict):
                render_platform_fields(child)
            elif key in ("base_frame", "base_frame_id", "robot_base_frame", "base_link_frame"):
                value[key] = frames["base"]
            elif key in ("odom_frame", "odom_frame_id"):
                value[key] = frames["odom"]
            elif key in ("map_frame", "global_frame", "global_frame_id"):
                value[key] = frames["odom"] if child == "odom" else frames["map"]
            elif key == "odom_topic":
                value[key] = topics["odom"]
            elif key == "footprint":
                value[key] = json.dumps(effective["hardware"]["footprint_m"])
            elif key in ("scan_topic", "topic") and child == "/scan":
                value[key] = "scan"

    lidar = _load(components / "lidar.yaml")
    lidar_params = lidar["ydlidar_ros2_driver_node"]["ros__parameters"]
    lidar_params["port"] = effective["hardware"]["lidar_port"]
    lidar_params["frame_id"] = frames["laser"]
    for name in ("inverted", "reversion"):
        if f"lidar_{name}" in runtime:
            lidar_params[name] = runtime[f"lidar_{name}"]
    _write_fsynced(
        output_dir / "lidar-params.yaml",
        yaml.safe_dump(lidar, sort_keys=False),
    )
    slam_base = _load(components / "slam" / "base.yaml")
    nav2 = _merge(
        _load(components / "nav2" / "base.yaml"),
        _load(components / "nav2" / "navigation.yaml").get("overlay", {}),
    )
    render_platform_fields(nav2)
    for profile, output_name in (
        ("quality.yaml", "slam-params.yaml"),
        ("high_resolution_experimental.yaml", "slam-precision-params.yaml"),
        ("legacy_fast_experimental.yaml", "slam-fast-params.yaml"),
    ):
        slam = _merge(slam_base, _load(components / "slam" / profile).get("overlay", {}))
        render_platform_fields(slam)
        _write_fsynced(output_dir / output_name, yaml.safe_dump(slam, sort_keys=False))
    _write_fsynced(output_dir / "nav2-params.yaml", yaml.safe_dump(nav2, sort_keys=False))
    for source, artifact in (
        ("wheel.yaml", "ekf-wheel.yaml"),
        ("wheel_imu.yaml", "ekf-wheel-imu.yaml"),
    ):
        ekf = _load(components / "ekf" / source)
        params = ekf["/**"]["ros__parameters"]
        if params.get("publish_tf") is not False:
            raise ConfigError("EKF must not publish the formal odometry TF")
        params.update(
            map_frame=frames["map"],
            odom_frame=frames["odom"],
            base_link_frame=frames["base"],
            world_frame=frames["odom"],
            odom0=topics["wheel_odom"],
        )
        if "imu0" in params:
            params["imu0"] = topics["imu"]
        _write_fsynced(output_dir / artifact, yaml.safe_dump(ekf, sort_keys=False))
    calibration = effective["calibration_bundle"]
    launch_args = {
        "schema_version": 1,
        "frames": effective["platform"]["frames"],
        "geometry": calibration["geometry"],
        "imu": calibration["imu"],
        "lidar": calibration["lidar"],
    }
    _write_fsynced(
        output_dir / "launch-calibration.yaml",
        yaml.safe_dump(launch_args, sort_keys=False),
    )
    launch_environment = {
        "BASE_PORT": effective["hardware"]["base_port"],
        "LIDAR_TF_YAW": calibration["lidar"]["rotation"][2],
        "ROBOT_BASE_MODE": runtime.get("base_mode", "real"),
        "ROBOT_BASE_FUSION_MODE": runtime["fusion_mode"],
    }
    _write_fsynced(
        output_dir / "launch-env.sh",
        "".join(
            f"export {name}={shlex.quote(str(value))}\n"
            for name, value in launch_environment.items()
        ),
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_identity(config_root: Path) -> tuple[str | None, bool | None]:
    try:
        root = subprocess.check_output(
            ["git", "-C", str(config_root), "rev-parse", "--show-toplevel"], text=True
        ).strip()
        commit = subprocess.check_output(
            ["git", "-C", root, "rev-parse", "HEAD"], text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "-C", root, "status", "--porcelain", "--untracked-files=normal"],
                text=True,
            ).strip()
        )
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return None, None
