#!/usr/bin/env python3
"""Fail on release, Platform API, firmware candidate or report drift."""

from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[2]
RELEASE = "v0.6.0-rc2"
PLATFORM_API = 5
FIRMWARE = "cf6cbde82e4750d848c9b2745f2dba4ba3ed40c3"


def main() -> int:
    compatibility = yaml.safe_load(
        (ROOT / "compatibility/firmware.yaml").read_text(encoding="utf-8")
    )
    release = yaml.safe_load(
        (ROOT / f"verification/configs/release/{RELEASE}.yaml").read_text(encoding="utf-8")
    )
    chassis = yaml.safe_load(
        (ROOT / "src/robot_config/config/calibration/robot_001/chassis_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    cross_repo = yaml.safe_load(
        (ROOT / "docs/contracts/cross_repo_compatibility.yaml").read_text(encoding="utf-8")
    )
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    ci = (ROOT / ".github/workflows/ci.yml").read_text(encoding="utf-8")

    assert (ROOT / "PLATFORM_API_VERSION").read_text().strip() == str(PLATFORM_API)
    assert compatibility["platform_api_version"] == PLATFORM_API
    assert compatibility["upper"]["release"] == RELEASE
    assert compatibility["firmware"]["candidate_commit"] == FIRMWARE
    assert compatibility["firmware"]["tested_commit"] is None
    assert compatibility["firmware"]["compatible_commit"] is None
    assert compatibility["release_compatible"] is False
    assert release["release"] == RELEASE
    assert release["platform_api_version"] == PLATFORM_API
    assert release["firmware_candidate_commit"] == FIRMWARE
    assert chassis["calibration_bundle"]["identity"]["firmware_commit"] == FIRMWARE
    for key in ("hardware_revision", "parameter_crc32"):
        identity_key = "firmware_parameter_crc32" if key == "parameter_crc32" else key
        assert (
            chassis["calibration_bundle"]["identity"][identity_key]
            == compatibility["firmware"][key]
        )
    assert cross_repo["firmware_candidate_commit"] == FIRMWARE
    assert FIRMWARE in ci
    assert "Platform API 5" in readme and "ba0f430" in readme
    assert not re.search(r"Platform API 4\s*/\s*Upper Protocol v3", readme)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
