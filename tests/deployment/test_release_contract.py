from pathlib import Path
import xml.etree.ElementTree as ET
import importlib.util
import hashlib
import os
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
FIRMWARE_COMMIT = "158254bd5c21f74a64f1d7c8609475a4c7caa1c7"


def test_firmware_contract_pins_v3_candidate_and_remains_hil_blocked() -> None:
    contract = yaml.safe_load(
        (ROOT / "compatibility" / "firmware.yaml").read_text(encoding="utf-8")
    )

    assert (
        contract["upper"]["architecture_baseline_commit"]
        == "2f16c8b9127a9534bade4fe9f3abb33643b4bd0b"
    )
    assert contract["upper"]["release"] == "v0.6.0-rc2"
    assert contract["upper"]["wire_protocols_supported"] == [3]
    assert contract["firmware"]["candidate_commit"] == FIRMWARE_COMMIT
    assert contract["firmware"]["candidate_tag"] is None
    assert contract["firmware"]["parameter_crc32"] == 0x0D663E2A
    assert contract["firmware"]["hardware_revision"] == 0x20000
    assert contract["firmware"]["tested_commit"] is None
    assert contract["firmware"]["compatible_commit"] is None
    vector_path = ROOT / contract["firmware"]["golden_vector_file"]
    assert (
        hashlib.sha256(vector_path.read_bytes()).hexdigest()
        == contract["firmware"]["golden_vector_sha256"]
    )
    assert contract["release_compatible"] is False
    assert contract["incompatible_firmware"][0]["commit"] != FIRMWARE_COMMIT
    assert contract["timing"]["required_firmware_upper_timeout_ms"] == 200
    assert contract["timing"]["bridge_keepalive_ms"] == 50
    assert contract["timing"]["bridge_command_timeout_ms"] == 150
    assert contract["timing"]["bridge_status_timeout_ms"] == 250
    assert (
        contract["timing"]["host_publish_period_ms"]
        < contract["timing"]["host_command_deadline_ms"]
        <= contract["timing"]["host_command_lifespan_ms"]
        < contract["timing"]["bridge_command_timeout_ms"]
    )
    assert (
        contract["timing"]["bridge_keepalive_ms"] < contract["timing"]["bridge_command_timeout_ms"]
    )
    assert (
        contract["timing"]["bridge_command_timeout_ms"]
        < contract["timing"]["required_firmware_upper_timeout_ms"]
    )


def test_unified_verify_entrypoints_exist_and_use_python_module_pytest() -> None:
    for name in ("verify_upper.sh", "verify_protocol.sh", "verify_runtime.sh"):
        path = ROOT / "scripts" / "verify" / name
        assert path.is_file(), name
        if os.name == "nt":
            mode = subprocess.check_output(
                ["git", "-C", str(ROOT), "ls-files", "-s", str(path.relative_to(ROOT))],
                text=True,
            ).split()[0]
            assert mode == "100755", name
        else:
            assert path.stat().st_mode & 0o111, name

    upper_script = (ROOT / "scripts" / "verify" / "verify_upper.sh").read_text(encoding="utf-8")
    assert "python3 -m pytest" in upper_script


def test_release_documentation_entrypoints_exist() -> None:
    assert (ROOT / "CHANGELOG.md").is_file()
    assert (ROOT / "docs" / "releases" / "v0.5.0-rc1.md").is_file()
    assert (ROOT / "docs" / "releases" / "v0.6.0-rc2.md").is_file()
    assert (ROOT / "verification" / "reports" / "hil" / "v0.6.0-rc2.yaml").is_file()
    assert (ROOT / "verification" / "reports" / "vehicle" / "v0.6.0-rc2.yaml").is_file()


def test_ci_enforces_quality_and_module_coverage_thresholds() -> None:
    ci = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")

    assert "ruff check" in ci
    assert "black --check" in ci
    assert "yamllint" in ci
    assert "shellcheck" in ci
    assert "coverage json -o coverage.json" in ci
    assert "scripts/verify/check_coverage.py" in ci
    assert "run_software_evidence.py" in ci and "--with-coverage" in ci
    runner = (ROOT / "scripts/verify/run_software_evidence.py").read_text(encoding="utf-8")
    assert "--cov=src/robot_supervision" in runner
    assert "--cov=src/robot_verification" in runner


def test_all_owned_ros_packages_are_versioned_v060() -> None:
    packages = sorted((ROOT / "src").glob("*/package.xml"))
    assert packages
    assert {ET.parse(package).getroot().findtext("version") for package in packages} == {"0.6.0"}


def test_hil_contract_lists_all_fault_injections_and_is_fail_closed() -> None:
    contract = yaml.safe_load(
        (ROOT / "verification/configs/hil/fault-injection-v0.4.0.yaml").read_text(encoding="utf-8")
    )
    assert len(contract["scenarios"]) == 16
    assert len({scenario["id"] for scenario in contract["scenarios"]}) == 16
    hil = yaml.safe_load(
        (ROOT / "verification/reports/hil/v0.6.0-rc2.yaml").read_text(encoding="utf-8")
    )
    vehicle = yaml.safe_load(
        (ROOT / "verification/reports/vehicle/v0.6.0-rc2.yaml").read_text(encoding="utf-8")
    )
    assert hil["result"] == vehicle["result"] == "NOT_RUN"


def test_bridge_executor_path_contains_no_blocking_sleep() -> None:
    bridge = (
        ROOT / "src" / "stm32_robot_bridge" / "stm32_robot_bridge" / "bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    assert "time.sleep(" not in bridge


def test_v060_rc2_release_gate_is_machine_readable_and_fail_closed() -> None:
    path = ROOT / "scripts/verify/verify_release.py"
    spec = importlib.util.spec_from_file_location("verify_release", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    failures = module.release_failures(ROOT)
    assert any("hil_report is not PASS: NOT_RUN" in failure for failure in failures)
    assert any(
        "firmware compatibility is not release-compatible" in failure for failure in failures
    )


def test_release_result_validation_rejects_boolean_identity_and_bad_artifact_hash() -> None:
    path = ROOT / "scripts/verify/verify_release.py"
    spec = importlib.util.spec_from_file_location("verify_release_validation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    report = {
        "schema_version": 1,
        "release": "v0.6.0-rc2",
        "result": "PASS",
        "upper_commit": "a" * 40,
        "firmware_commit": "b" * 40,
        "hardware_revision": True,
        "config_sha256": "c" * 64,
        "artifact_sha256": {"ros-params.yaml": "bad"},
        "parameter_crc32": True,
        "calibration_version": "v1",
        "metrics": {"sample_count": 1},
        "failures": [],
    }

    failures = module._validate_result("audit", report, "v0.6.0-rc2")

    assert any("positive hardware" in failure for failure in failures)
    assert any("invalid artifact" in failure for failure in failures)
    assert any("parameter CRC" in failure for failure in failures)
