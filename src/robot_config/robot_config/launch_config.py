"""Read and verify the compiled artifacts consumed by launch composition."""

import hashlib
import json
from copy import deepcopy
from pathlib import Path

import yaml


def load_launch_config(params_file):
    params = Path(params_file)
    directory = params.parent
    if params.name != "ros-params.yaml":
        raise RuntimeError("launch requires the compiled ros-params.yaml artifact")
    try:
        manifest = yaml.safe_load((directory / "manifest.yaml").read_text(encoding="utf-8"))
        artifacts = manifest["artifact_sha256"]
        for name, digest in artifacts.items():
            path = (directory / name).resolve()
            if not path.is_relative_to(directory.resolve()):
                raise RuntimeError("compiled artifact escapes its config directory")
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"compiled artifact hash mismatch: {name}")
        for required in (
            "ros-params.yaml",
            "effective-config.yaml",
            "launch-calibration.yaml",
            "ekf-wheel-imu.yaml",
            "ekf-wheel.yaml",
            "lidar-params.yaml",
            "slam-params.yaml",
            "slam-precision-params.yaml",
            "slam-fast-params.yaml",
            "nav2-params.yaml",
            "launch-env.sh",
        ):
            if required not in artifacts:
                raise RuntimeError(f"missing compiled artifact: {required}")
        effective = yaml.safe_load(
            (directory / "effective-config.yaml").read_text(encoding="utf-8")
        )
        calibration = yaml.safe_load(
            (directory / "launch-calibration.yaml").read_text(encoding="utf-8")
        )
        if effective["config_sha256"] != manifest["config_sha256"]:
            raise RuntimeError("effective config identity differs from its manifest")
        payload = deepcopy(effective)
        payload.pop("config_sha256", None)
        payload["calibration_bundle"]["identity"].pop("upper_config_sha256", None)
        actual = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode(
                "utf-8"
            )
        ).hexdigest()
        if actual != manifest["config_sha256"]:
            raise RuntimeError("effective config hash does not describe its content")
        return effective, calibration
    except (OSError, KeyError, TypeError, AttributeError, yaml.YAMLError) as exc:
        raise RuntimeError(f"cannot load compiled launch artifacts: {exc}") from exc
