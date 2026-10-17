"""Keep Humble trinary PGM unknown pixels unknown after Nav2 reload."""

from pathlib import Path
import re

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def assert_trinary_classes(free_threshold: float, occupied_threshold: float) -> None:
    # Humble map_io writes free/unknown/occupied as 254/205/0 in 8-bit PGM.
    assert 1.0 - 254.0 / 255.0 < free_threshold
    assert free_threshold <= 1.0 - 205.0 / 255.0 <= occupied_threshold
    assert occupied_threshold < 1.0


@pytest.mark.parametrize(
    "relative_path",
    (
        "src/robot_config/config/components/nav2/base.yaml",
        "src/robot_bringup/config/nav2_params_robot.yaml",
    ),
)
def test_map_saver_defaults_preserve_unknown_pixels(relative_path: str) -> None:
    config = yaml.safe_load((ROOT / relative_path).read_text(encoding="utf-8"))
    params = config["map_saver"]["ros__parameters"]
    assert_trinary_classes(params["free_thresh_default"], params["occupied_thresh_default"])


def test_save_entry_passes_thresholds_to_standalone_map_saver() -> None:
    script = (ROOT / "launch_scripts/save_map.sh").read_text(encoding="utf-8")
    # map_saver_cli does not inherit the compiled navigation node parameters.
    free = re.search(r"--free\s+([0-9.]+)", script)
    occupied = re.search(r"--occ\s+([0-9.]+)", script)
    assert free and occupied, "standalone map saver needs explicit compatible thresholds"
    assert_trinary_classes(float(free[1]), float(occupied[1]))
