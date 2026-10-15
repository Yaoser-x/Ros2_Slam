"""Resolve supported CLI overrides before creating the immutable run config."""

from copy import deepcopy
import math
import os

from robot_config.compiler import ConfigError


def runtime_overrides(arguments, effective, environment=None):
    environment = os.environ if environment is None else environment
    command = arguments[0] if arguments else "help"
    command = {
        "nav": "navigation",
        "bridge": "base",
        "test-base": "base-test",
        "sensors": "sensor",
        "system": "full",
    }.get(command, command)
    runtime = {
        "cli_args": list(arguments),
        "base_mode": "none" if command in ("mapping", "sensor") else "real",
    }
    overrides = {"runtime": runtime}
    for name in ("BASE_WHEEL_TRACK_WIDTH", "BASE_ODOM_ANGULAR_SCALE", "BASE_ODOM_ANGULAR_SIGN"):
        if name in environment:
            raise ConfigError(f"{name} is deprecated; update the canonical calibration instead")
    timing = {}
    for name, key in (
        ("BASE_CMD_TIMEOUT", "bridge_command_timeout_sec"),
        ("BASE_DRIVE_KEEPALIVE_SEC", "drive_keepalive_sec"),
    ):
        if name in environment:
            try:
                timing[key] = float(environment[name])
            except ValueError as exc:
                raise ConfigError(f"{name} must be numeric") from exc
            if not math.isfinite(timing[key]):
                raise ConfigError(f"{name} must be finite")
    if timing:
        overrides["safety"] = {"timing": timing}
    for name in ("BASE_STATUS_LOG_INTERVAL_SEC", "BASE_CMD_LOG_INTERVAL_SEC"):
        if name in environment:
            raise ConfigError(f"{name} is unsupported by the v3 bridge")
    hardware = {}
    if environment.get("BASE_PORT"):
        hardware["base_port"] = environment["BASE_PORT"]
    yaw = environment.get("LIDAR_TF_YAW")
    value_options = {
        "--base-port",
        "--port",
        "--lidar-port",
        "--lidar-yaw-rad",
        "--lidar-yaw-deg",
        "--namespace",
    }
    index = 1
    while index < len(arguments):
        arg = arguments[index]
        if arg in value_options:
            index += 1
            if index >= len(arguments):
                raise ConfigError(f"{arg} requires a value")
            value = arguments[index]
            if arg == "--port" and command in ("sensor", "sensors", "check-lidar"):
                hardware["lidar_port"] = value
            elif arg in ("--base-port", "--port"):
                hardware["base_port"] = value
            elif arg == "--lidar-port":
                hardware["lidar_port"] = value
            elif arg == "--namespace":
                if value.strip("/"):
                    raise ConfigError("nonempty namespace is not supported by the current platform")
            else:
                try:
                    yaw = float(value)
                except ValueError as exc:
                    raise ConfigError(f"{arg} must be numeric") from exc
                if arg == "--lidar-yaw-deg":
                    yaw = math.radians(yaw)
        elif arg in ("--fake-base", "--real-base"):
            runtime["base_mode"] = "fake" if arg == "--fake-base" else "real"
        elif arg == "--ekf-base":
            runtime["fusion_mode"] = "wheel_imu"
        elif arg in ("--lidar-inverted", "--no-lidar-inverted"):
            runtime["lidar_inverted"] = arg == "--lidar-inverted"
        elif arg in ("--lidar-reversion", "--no-lidar-reversion"):
            runtime["lidar_reversion"] = arg == "--lidar-reversion"
        elif command == "mapping" and arg in ("quality", "precision", "fast"):
            runtime["slam_profile"] = arg
        index += 1
    if hardware:
        overrides["hardware"] = hardware
    if yaw is not None:
        try:
            yaw = float(yaw)
        except ValueError as exc:
            raise ConfigError("LIDAR_TF_YAW must be numeric") from exc
        if not math.isfinite(yaw):
            raise ConfigError("lidar yaw must be finite")
        bundle_rotation = deepcopy(effective["calibration_bundle"]["lidar"]["rotation"])
        calibration_rotation = deepcopy(effective["calibration"]["lidar"]["rotation_rpy_rad"])
        bundle_rotation[2] = calibration_rotation[2] = yaw
        overrides["calibration_bundle"] = {"lidar": {"rotation": bundle_rotation}}
        overrides["calibration"] = {"lidar": {"rotation_rpy_rad": calibration_rotation}}
        overrides.setdefault("hardware", {})["lidar_mount_yaw_rad"] = yaw
    return overrides
