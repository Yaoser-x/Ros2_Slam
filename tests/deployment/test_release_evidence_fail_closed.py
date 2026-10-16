import copy
import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def load_module(relative):
    spec = importlib.util.spec_from_file_location(Path(relative).stem, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GATE = load_module("scripts/verify/verify_release.py")
RUNNER = load_module("scripts/verify/run_software_evidence.py")
HIL = load_module("verification/runners/hil/run_uart_hil.py")


def software_report(name):
    pair = {
        "upper": {"commit": "a" * 40, "dirty": False, "source_sha256": "c" * 64},
        "firmware": {
            "commit": "cf012dda6f4a83c8ff96a54f3623565df0fbd72f",
            "dirty": False,
            "source_sha256": "d" * 64,
        },
    }
    checks = {
        check: {"exit_code": 0, "result": "PASS", "log_sha256": "e" * 64}
        for check in GATE.SOFTWARE_CHECKS[name]
    }
    return RUNNER.build_report(
        name, pair, pair, checks, {"tests_executed": 8, "skipped": 0, "failed": 0}, [], "test"
    )


@pytest.fixture
def valid_release(tmp_path):
    manifest = yaml.safe_load((ROOT / "verification/configs/release/v0.6.0-rc2.yaml").read_text())

    def write(relative, value):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(value), encoding="utf-8")

    write("verification/configs/release/v0.6.0-rc2.yaml", manifest)
    for relative in manifest["release_materials"].values():
        write(relative, {"material": "test"})
    write(
        manifest["release_materials"]["compatibility"],
        {
            "schema_version": 3,
            "platform_api_version": 5,
            "release_compatible": True,
            "upper": {
                "release": GATE.RELEASE,
                "tested_commit": "a" * 40,
                "release_candidate_commit": "a" * 40,
            },
            "firmware": {
                "candidate_commit": manifest["firmware_candidate_commit"],
                "tested_commit": manifest["firmware_candidate_commit"],
                "compatible_commit": manifest["firmware_candidate_commit"],
                "parameter_crc32": 123,
                "hardware_revision": 131072,
            },
        },
    )
    hardware = {
        "schema_version": 1,
        "release": GATE.RELEASE,
        "result": "PASS",
        "failures": [],
        "upper_commit": "a" * 40,
        "firmware_commit": manifest["firmware_candidate_commit"],
        "hardware_revision": 131072,
        "parameter_crc32": 123,
        "config_sha256": "c" * 64,
        "artifact_sha256": {"params.yaml": "d" * 64},
        "calibration_version": "test-v1",
        "metrics": {"sample_count": 1},
        "status": "final",
    }
    for relative in manifest["hardware_gates"].values():
        write(relative, hardware)
    for name, relative in manifest["software_gates"].items():
        write(relative, software_report(name))
    assert GATE.release_failures(tmp_path) == []
    return tmp_path, manifest, write


@pytest.mark.parametrize("section", ["software_gates", "hardware_gates", "release_materials"])
@pytest.mark.parametrize("mutation", ["remove", "empty", "wrong_type"])
def test_required_manifest_sets_cannot_be_removed_to_bypass_release(
    valid_release, section, mutation
):
    root, manifest, write = valid_release
    if mutation == "remove":
        del manifest[section][next(iter(manifest[section]))]
    else:
        manifest[section] = {} if mutation == "empty" else []
    write("verification/configs/release/v0.6.0-rc2.yaml", manifest)
    assert any(section in failure for failure in GATE.release_failures(root))


@pytest.mark.parametrize(
    "field,value", [("schema_version", True), ("release", "wrong"), ("platform_api_version", 4)]
)
def test_manifest_schema_release_and_api_are_required(valid_release, field, value):
    root, manifest, write = valid_release
    manifest[field] = value
    write("verification/configs/release/v0.6.0-rc2.yaml", manifest)
    assert GATE.release_failures(root)


@pytest.mark.parametrize("name", sorted(GATE.SOFTWARE_GATES))
def test_each_software_gate_is_consumed_and_cannot_claim_pass_with_failures(valid_release, name):
    root, manifest, write = valid_release
    report = software_report(name)
    report["failures"] = ["actual failure"]
    write(manifest["software_gates"][name], report)
    assert any(
        name in failure and "non-empty failures" in failure
        for failure in GATE.release_failures(root)
    )


@pytest.mark.parametrize(
    "mutation", ["dirty", "zero_sha", "changed", "empty_checks", "skipped", "not_run"]
)
def test_software_identity_and_executed_evidence_are_fail_closed(valid_release, mutation):
    root, manifest, write = valid_release
    report = software_report("protocol_cross_test")
    if mutation == "dirty":
        report["tested_pair"]["firmware"]["dirty"] = True
    elif mutation == "zero_sha":
        report["upper_commit"] = "0" * 40
        report["tested_pair"]["upper"]["commit"] = "0" * 40
    elif mutation == "changed":
        report["source_pair_after"] = copy.deepcopy(report["tested_pair"])
        report["source_pair_after"]["upper"]["source_sha256"] = "f" * 64
    elif mutation == "empty_checks":
        report["checks"] = {}
    elif mutation == "skipped":
        report["metrics"]["skipped"] = 1
    else:
        report["result"] = "NOT_RUN"
    write(manifest["software_gates"]["protocol_cross_test"], report)
    assert GATE.release_failures(root)


def test_external_software_reports_do_not_modify_default_report_templates(valid_release, tmp_path):
    root, manifest, write = valid_release
    external = root / "external-software"
    for name, relative in manifest["software_gates"].items():
        write(f"external-software/{name}.yaml", software_report(name))
        report = software_report(name)
        report["result"] = "NOT_RUN"
        write(relative, report)
    assert GATE.release_failures(root)
    assert GATE.release_failures(root, external) == []


def test_invalid_hash_value_is_rejected_without_crashing_consistency_checks(valid_release):
    root, manifest, write = valid_release
    path = root / manifest["hardware_gates"]["hil_report"]
    report = yaml.safe_load(path.read_text())
    report["config_sha256"] = ["invalid"]
    write(manifest["hardware_gates"]["hil_report"], report)
    assert any("config SHA-256" in failure for failure in GATE.release_failures(root))


@pytest.mark.parametrize("field", ["parameter_crc32", "hardware_revision"])
def test_compatibility_boolean_identity_is_rejected(valid_release, field):
    root, manifest, write = valid_release
    relative = manifest["release_materials"]["compatibility"]
    compatibility = yaml.safe_load((root / relative).read_text())
    compatibility["firmware"][field] = True
    write(relative, compatibility)
    assert any("u32" in failure for failure in GATE.release_failures(root))


def test_hil_metadata_cannot_override_missing_events_to_pass():
    contract = {
        "release": GATE.RELEASE,
        "firmware_candidate_commit": "b" * 40,
        "scenarios": [
            {"id": "stop", "required_events": ["fault_injected_at", "motor_target_zero_at"]}
        ],
    }
    metadata = {
        "upper_commit": "a" * 40,
        "firmware_commit": "b" * 40,
        "hardware_revision": 1,
        "parameter_crc32": 1,
        "config_sha256": "c" * 64,
        "artifact_sha256": {"params.yaml": "d" * 64},
        "calibration_version": "test",
        "result": "PASS",
        "release": "fake",
        "schema_version": 99,
        "failures": [],
        "metrics": {"fake": True},
    }
    report = HIL.build_report(contract, metadata, {}, b"", b"trace")
    assert report["result"] == "FAIL"
    assert report["release"] == GATE.RELEASE and report["schema_version"] == 1
    assert "fake" not in report["metrics"]
    assert any("missing events" in failure for failure in report["failures"])
    assert any("override computed field result" in failure for failure in report["failures"])


def test_wheel_stop_event_name_does_not_claim_a_measured_physical_threshold():
    contract = {
        "firmware_candidate_commit": "b" * 40,
        "scenarios": [
            {
                "id": "stop",
                "required_events": [
                    "fault_injected_at",
                    "motor_target_zero_at",
                    "wheel_speed_below_threshold_at",
                ],
                "max_motor_target_zero_ms": 200,
            }
        ],
    }
    failures, metrics = HIL.evaluate(
        contract,
        {},
        {
            "stop": {
                "fault_injected_at": 0,
                "motor_target_zero_at": 100000000,
                "wheel_speed_below_threshold_at": 150000000,
            }
        },
    )
    assert metrics["wheel_stop_acceptance"]["stop"] == "NOT_EVALUATED"
    assert any("wheel-stop threshold" in failure for failure in failures)


def test_software_runner_never_reports_pass_for_changed_sources_or_skipped_tests():
    report = software_report("pty_report")
    pair = report["tested_pair"]
    after = copy.deepcopy(pair)
    after["upper"]["source_sha256"] = "f" * 64
    changed = RUNNER.build_report(
        "pty_report", pair, after, report["checks"], report["metrics"], [], "test"
    )
    assert changed["result"] == "FAIL"
    skipped = RUNNER.build_report(
        "pty_report",
        pair,
        pair,
        report["checks"],
        {"tests_executed": 1, "skipped": 1, "failed": 0},
        [],
        "test",
    )
    assert skipped["result"] == "FAIL"


def test_ci_runner_preserves_all_existing_module_coverage_arguments(tmp_path):
    commands, _ = RUNNER.commands("ros_ci", tmp_path, with_coverage=True)
    expected = {
        "--cov=src/stm32_robot_bridge",
        "--cov=src/robot_control",
        "--cov=src/robot_sensing",
        "--cov=src/robot_state_estimation",
        "--cov=src/robot_supervision",
        "--cov=src/robot_verification",
        "--cov-report=term",
        "--cov-report=xml",
    }
    assert expected <= set(commands["pytest"])
    assert set(commands) == GATE.SOFTWARE_CHECKS["ros_ci"]
    assert "install/setup.bash" in commands["colcon_test"][2]
    default, _ = RUNNER.commands("ros_ci", tmp_path)
    assert not any(argument.startswith("--cov") for argument in default["pytest"])
    for gate in ("protocol_cross_test", "pty_report"):
        separate, _ = RUNNER.commands(gate, tmp_path, with_coverage=True)
        assert not any(
            argument.startswith("--cov") for argv in separate.values() for argument in argv
        )


def test_ci_has_one_owner_for_build_test_and_pytest_coverage():
    workflow = (ROOT / ".github/workflows/ci.yml").read_text()
    assert workflow.count("run_software_evidence.py") == 1
    assert "--with-coverage" in workflow
    assert "colcon build" not in workflow and "colcon test" not in workflow
    assert "python3 -m pytest tests/ src/" not in workflow
    assert "coverage json -o coverage.json" in workflow
    assert "scripts/verify/check_coverage.py coverage.json" in workflow
    assert "ref: cf012dda6f4a83c8ff96a54f3623565df0fbd72f" in workflow
