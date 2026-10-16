import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "verification" / "configs" / "benchmarks" / "rosbag-datasets-v0.6.0-rc2.yaml"
MODULE_PATH = ROOT / "tools" / "datasets" / "benchmark_bags.py"


def load_module():
    spec = importlib.util.spec_from_file_location("benchmark_bags", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dataset_catalog_covers_all_ten_benchmarks_and_required_topics():
    catalog = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

    assert catalog["schema_version"] == 1
    assert list(catalog["datasets"]) == [
        "bag_00_static",
        "bag_01_straight",
        "bag_02_rotation_cw",
        "bag_03_rotation_ccw",
        "bag_04_square",
        "bag_05_figure8",
        "bag_06_mapping_train",
        "bag_07_mapping_validation",
        "bag_08_navigation_validation",
        "bag_09_fault_injection",
    ]
    assert set(catalog["required_topics"]) == {
        "/scan",
        "/wheel/observation",
        "/imu/observation",
        "/wheel/odom",
        "/imu/data",
        "/odom",
        "/tf",
        "/tf_static",
        "/cmd_vel/nav",
        "/cmd_vel/teleop",
        "/cmd_vel/test",
        "/chassis/host_motion_command",
        "/chassis/link_state",
        "/chassis/firmware_control_state",
        "/chassis/host_control_state",
        "/motion/supervision_state",
        "/navigation/guard_state",
        "/platform/compatibility_state",
        "/diagnostics",
    }
    for dataset in catalog["datasets"].values():
        assert dataset["minimum_duration_sec"] > 0
        assert dataset["procedure"]
    assert set(catalog["required_run_metadata"]) == {
        "upper_commit",
        "firmware_commit",
        "hardware_revision",
        "firmware_parameter_crc32",
        "effective_config_sha256",
        "artifact_sha256",
        "calibration_version",
    }


def test_record_plan_is_deterministic_and_rejects_unknown_dataset(tmp_path):
    module = load_module()
    catalog = module.load_catalog(CONFIG_PATH)

    plan = module.build_record_plan(
        catalog,
        "bag_04_square",
        output_root=tmp_path,
        run_id="run-001",
    )

    assert plan.output_dir == tmp_path / "bag_04_square" / "run-001"
    assert plan.command[:4] == ("ros2", "bag", "record", "--output")
    required = tuple(catalog["required_topics"])
    assert plan.command[-len(required) :] == required
    with pytest.raises(ValueError, match="unknown dataset"):
        module.build_record_plan(catalog, "bag_99", output_root=tmp_path, run_id="x")


def test_metadata_validation_reports_missing_topics_duration_and_empty_streams():
    module = load_module()
    catalog = module.load_catalog(CONFIG_PATH)
    metadata = {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 5_000_000_000},
            "topics_with_message_count": [
                {
                    "topic_metadata": {"name": topic},
                    "message_count": 10 if topic != "/imu/data" else 0,
                }
                for topic in catalog["required_topics"]
                if topic != "/diagnostics"
            ],
        }
    }

    result = module.validate_metadata(catalog, "bag_00_static", metadata)

    assert result.valid is False
    assert "/diagnostics" in result.missing_topics
    assert "/imu/data" not in result.empty_topics  # Wheel-only IMU observation is event evidence.
    assert result.duration_ok is False


def test_empty_event_topics_are_allowed_but_required_phase_facts_are_not():
    module = load_module()
    catalog = module.load_catalog(CONFIG_PATH)
    facts, events = module.topic_requirements(catalog, "bag_00_static")
    metadata = {
        "rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 600_000_000_000},
            "topics_with_message_count": [
                {"topic_metadata": {"name": topic}, "message_count": 10} for topic in facts
            ]
            + [{"topic_metadata": {"name": topic}, "message_count": 0} for topic in events],
        }
    }
    assert module.validate_metadata(catalog, "bag_00_static", metadata).valid
    navigation = module.validate_metadata(catalog, "bag_08_navigation_validation", metadata)
    assert not navigation.valid
    assert "/scan" in navigation.empty_topics and "/cmd_vel/nav" in navigation.empty_topics
    metadata["rosbag2_bagfile_information"]["topics_with_message_count"][0]["message_count"] = 0
    assert not module.validate_metadata(catalog, "bag_00_static", metadata).valid


def test_run_metadata_fails_closed_when_traceability_is_incomplete():
    module = load_module()
    catalog = module.load_catalog(CONFIG_PATH)
    complete = {name: "recorded" for name in catalog["required_run_metadata"]}
    assert module.validate_run_metadata(catalog, complete) == ()
    complete["firmware_parameter_crc32"] = None
    assert module.validate_run_metadata(catalog, complete) == ("firmware_parameter_crc32",)
