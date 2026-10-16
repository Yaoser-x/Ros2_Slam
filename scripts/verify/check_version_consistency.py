#!/usr/bin/env python3
"""Fail on release, Platform API, firmware candidate or report drift."""

from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[2]
RELEASE = "v0.6.0-rc2"
PLATFORM_API = 5
FIRMWARE = "ba670e6454c4c46fff8d1e26bff57d58b5e08cac"


def main() -> int:
    compatibility = yaml.safe_load((ROOT / "compatibility/firmware.yaml").read_text())
    release = yaml.safe_load((ROOT / f"verification/configs/release/{RELEASE}.yaml").read_text())
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
    assert FIRMWARE in ci
    assert "Platform API 5" in readme and "ba670e6" in readme
    assert not re.search(r"Platform API 4\s*/\s*Upper Protocol v3", readme)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
