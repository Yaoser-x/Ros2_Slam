#!/usr/bin/env python3
"""Fail-closed v0.6.0-rc2 release gate over machine-readable evidence."""

from __future__ import annotations

import json
import argparse
import re
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
RELEASE = "v0.6.0-rc2"
MANIFEST = ROOT / "verification" / "configs" / "release" / f"{RELEASE}.yaml"
SHA256 = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA = re.compile(r"^[0-9a-f]{40}$")
SOFTWARE_GATES = frozenset({"ros_ci", "protocol_cross_test", "pty_report"})
HARDWARE_GATES = frozenset(
    {
        "protocol_report",
        "hil_report",
        "calibration_report",
        "dynamics_report",
        "wheel_odom_report",
        "imu_report",
        "ekf_report",
        "slam_report",
        "nav2_report",
        "fault_injection_report",
        "long_run_report",
        "vehicle_report",
    }
)
RELEASE_MATERIALS = frozenset(
    {"changelog", "compatibility", "protocol", "authority", "upgrade_and_rollback"}
)
SOFTWARE_CHECKS = {
    "ros_ci": {"colcon_build", "colcon_test", "colcon_test_result", "ros_runtime", "pytest"},
    "protocol_cross_test": {"protocol_cross"},
    "pty_report": {"pty"},
}


def _git_sha(value):
    return isinstance(value, str) and GIT_SHA.fullmatch(value) is not None and value != "0" * 40


def _failed_result(name, report):
    failures = []
    if report.get("result") != "PASS":
        failures.append(f"{name} is not PASS: {report.get('result')}")
    if not isinstance(report.get("failures"), list):
        failures.append(f"{name} has no explicit failures list")
    elif report["failures"]:
        failures.append(f"{name} records non-empty failures")
    return failures


def _mapping(path: Path) -> dict:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"document is not a mapping: {path}")
    return value


def _validate_result(name: str, report: dict, release: str) -> list[str]:
    failures = _failed_result(name, report)
    required = {
        "schema_version",
        "release",
        "result",
        "upper_commit",
        "firmware_commit",
        "hardware_revision",
        "config_sha256",
        "artifact_sha256",
        "parameter_crc32",
        "calibration_version",
        "metrics",
        "failures",
    }
    missing = sorted(required - set(report))
    if missing:
        failures.append(f"{name} missing fields: {', '.join(missing)}")
        return failures
    if (
        type(report["schema_version"]) is not int
        or report["schema_version"] != 1
        or report["release"] != release
    ):
        failures.append(f"{name} schema/release mismatch")
    if not _git_sha(report["upper_commit"]):
        failures.append(f"{name} has no traceable upper commit")
    if not _git_sha(report["firmware_commit"]):
        failures.append(f"{name} has no traceable firmware commit")
    if (
        isinstance(report["hardware_revision"], bool)
        or not isinstance(report["hardware_revision"], int)
        or not 0 < report["hardware_revision"] <= 0xFFFFFFFF
    ):
        failures.append(f"{name} has no positive hardware revision")
    if not isinstance(report["config_sha256"], str) or not SHA256.fullmatch(
        report["config_sha256"]
    ):
        failures.append(f"{name} has no effective config SHA-256")
    artifacts = report["artifact_sha256"]
    if not isinstance(artifacts, dict) or not artifacts:
        failures.append(f"{name} has no generated artifact SHA-256 manifest")
    elif any(
        not isinstance(path, str)
        or not path
        or not isinstance(digest, str)
        or not SHA256.fullmatch(digest)
        for path, digest in artifacts.items()
    ):
        failures.append(f"{name} has an invalid artifact SHA-256 manifest")
    if (
        isinstance(report["parameter_crc32"], bool)
        or not isinstance(report["parameter_crc32"], int)
        or not 0 < report["parameter_crc32"] <= 0xFFFFFFFF
    ):
        failures.append(f"{name} has no measured firmware parameter CRC")
    if not isinstance(report["calibration_version"], str) or not report["calibration_version"]:
        failures.append(f"{name} has no calibration version")
    if not isinstance(report["metrics"], dict) or not report["metrics"]:
        failures.append(f"{name} has no measured metrics")
    return failures


def _validate_software_result(name, report, release):
    failures = _failed_result(name, report)
    if (
        type(report.get("schema_version")) is not int
        or report.get("schema_version") != 1
        or report.get("release") != release
        or type(report.get("platform_api_version")) is not int
        or report.get("platform_api_version") != 5
        or report.get("protocol_version") != 3
        or report.get("evidence_type") != "software"
        or report.get("gate") != name
    ):
        failures.append(f"{name} software schema/release/API/evidence mismatch")
    pair = report.get("tested_pair")
    if not isinstance(pair, dict) or set(pair) != {"upper", "firmware"}:
        failures.append(f"{name} has no tested source pair")
        return failures
    if report.get("source_pair_after") != pair:
        failures.append(f"{name} source pair changed during validation or is not recorded")
    for repository, commit_field in (("upper", "upper_commit"), ("firmware", "firmware_commit")):
        identity = pair.get(repository)
        if not isinstance(identity, dict) or set(identity) != {"commit", "dirty", "source_sha256"}:
            failures.append(f"{name} has no {repository} source identity")
            continue
        if not _git_sha(report.get(commit_field)) or identity.get("commit") != report.get(
            commit_field
        ):
            failures.append(f"{name} {repository} commit identity is invalid")
        if identity.get("dirty") is not False:
            failures.append(f"{name} {repository} tested worktree is dirty or unknown")
        if not isinstance(identity.get("source_sha256"), str) or not SHA256.fullmatch(
            identity["source_sha256"]
        ):
            failures.append(f"{name} {repository} source SHA-256 is invalid")
    checks = report.get("checks")
    if not isinstance(checks, dict) or set(checks) != SOFTWARE_CHECKS[name]:
        failures.append(f"{name} required software check set is incomplete")
    else:
        for check_name, check in checks.items():
            if (
                not isinstance(check, dict)
                or check.get("result") != "PASS"
                or type(check.get("exit_code")) is not int
                or check.get("exit_code") != 0
                or not isinstance(check.get("log_sha256"), str)
                or not SHA256.fullmatch(check["log_sha256"])
            ):
                failures.append(f"{name} check {check_name} has no successful measured execution")
    metrics = report.get("metrics")
    if (
        not isinstance(metrics, dict)
        or type(metrics.get("tests_executed")) is not int
        or metrics.get("tests_executed", 0) <= 0
    ):
        failures.append(f"{name} has no executed test count")
    elif any(
        type(metrics.get(key)) is not int or metrics.get(key) != 0 for key in ("skipped", "failed")
    ):
        failures.append(f"{name} records skipped, failed or unknown test cases")
    return failures


def _validate_manifest(manifest):
    failures = []
    if (
        type(manifest.get("schema_version")) is not int
        or manifest.get("schema_version") != 1
        or manifest.get("release") != RELEASE
        or type(manifest.get("platform_api_version")) is not int
        or manifest.get("platform_api_version") != 5
    ):
        failures.append("release manifest schema/release/API mismatch")
    if not _git_sha(manifest.get("upper_architecture_baseline_commit")) or not _git_sha(
        manifest.get("firmware_candidate_commit")
    ):
        failures.append("release manifest has an invalid candidate identity")
    for key, required in (
        ("software_gates", SOFTWARE_GATES),
        ("hardware_gates", HARDWARE_GATES),
        ("release_materials", RELEASE_MATERIALS),
    ):
        values = manifest.get(key)
        if not isinstance(values, dict) or set(values) != required:
            failures.append(f"release manifest {key} required set is incomplete or unexpected")
            continue
        if any(
            not isinstance(path, str) or not path.strip() or path == "required"
            for path in values.values()
        ):
            failures.append(f"release manifest {key} has an invalid evidence path")
    return failures


def release_failures(root=ROOT, software_reports_dir=None):
    root = Path(root)
    manifest_path = root / "verification" / "configs" / "release" / f"{RELEASE}.yaml"
    try:
        manifest = _mapping(manifest_path)
    except ValueError as exc:
        return [str(exc)]
    failures = _validate_manifest(manifest)
    if failures:
        return failures
    for name, relative in manifest["release_materials"].items():
        if not (root / relative).is_file():
            failures.append(f"missing release material {name}: {relative}")
    reports = {}
    valid_reports = set()
    for name, relative in {**manifest["hardware_gates"], **manifest["software_gates"]}.items():
        path = (
            Path(software_reports_dir) / f"{name}.yaml"
            if name in SOFTWARE_GATES and software_reports_dir is not None
            else root / relative
        )
        if not path.is_file():
            failures.append(
                f"missing {'software' if name in SOFTWARE_GATES else 'hardware'} report {name}: {path}"
            )
            continue
        try:
            reports[name] = _mapping(path)
        except ValueError as exc:
            failures.append(str(exc))
            continue
        report_failures = (
            _validate_software_result(name, reports[name], manifest["release"])
            if name in SOFTWARE_GATES
            else _validate_result(name, reports[name], manifest["release"])
        )
        failures.extend(report_failures)
        if not report_failures:
            valid_reports.add(name)
    for name in ("calibration_report", "dynamics_report"):
        if name in reports and reports[name].get("status") != "final":
            failures.append(f"{name} is not final")
    compatibility_path = root / manifest.get("release_materials", {}).get(
        "compatibility", "compatibility/firmware.yaml"
    )
    if compatibility_path.is_file():
        try:
            compatibility = _mapping(compatibility_path)
        except ValueError as exc:
            return failures + [str(exc)]
        if (
            type(compatibility.get("schema_version")) is not int
            or compatibility.get("schema_version") != 3
            or type(compatibility.get("platform_api_version")) is not int
            or compatibility.get("platform_api_version") != 5
            or not isinstance(compatibility.get("upper"), dict)
            or not isinstance(compatibility.get("firmware"), dict)
        ):
            return failures + ["compatibility schema/API/source identity mismatch"]
        upper = compatibility.get("upper", {})
        if (
            upper.get("release") != RELEASE
            or compatibility["firmware"].get("candidate_commit")
            != manifest["firmware_candidate_commit"]
        ):
            failures.append("compatibility release/candidate differs from release manifest")
        tested_upper_commit = upper.get("tested_commit")
        if not _git_sha(tested_upper_commit):
            failures.append("upper tested commit is not locked")
        if upper.get("release_candidate_commit") != tested_upper_commit:
            failures.append("upper release candidate differs from tested commit")
        for name, report in reports.items():
            if report.get("result") == "PASS" and report.get("upper_commit") != tested_upper_commit:
                failures.append(f"{name} upper commit differs from tested commit")
        if compatibility.get("release_compatible") is not True:
            failures.append("firmware compatibility is not release-compatible")
        compatible_commit = compatibility.get("firmware", {}).get("compatible_commit")
        tested_firmware_commit = compatibility.get("firmware", {}).get("tested_commit")
        if not _git_sha(compatible_commit):
            failures.append("compatible upper-v3 firmware commit is not locked")
        if tested_firmware_commit != compatible_commit:
            failures.append("tested firmware commit differs from compatibility lock")
        for report in reports.values():
            if (
                report.get("result") == "PASS"
                and report.get("firmware_commit") != compatible_commit
            ):
                failures.append("report firmware commit differs from compatibility lock")
        expected_crc = compatibility.get("firmware", {}).get("parameter_crc32")
        expected_hardware = compatibility.get("firmware", {}).get("hardware_revision")
        if type(expected_crc) is not int or not 0 < expected_crc <= 0xFFFFFFFF:
            failures.append("compatibility parameter CRC is not a measured u32")
        if type(expected_hardware) is not int or not 0 < expected_hardware <= 0xFFFFFFFF:
            failures.append("compatibility hardware revision is not a positive u32")
        passing_reports = [
            report
            for name, report in reports.items()
            if name in HARDWARE_GATES and name in valid_reports
        ]
        for report in passing_reports:
            if report.get("parameter_crc32") != expected_crc:
                failures.append("report parameter CRC differs from compatibility lock")
            if report.get("hardware_revision") != expected_hardware:
                failures.append("report hardware revision differs from compatibility lock")
        config_hashes = {report.get("config_sha256") for report in passing_reports}
        calibration_versions = {report.get("calibration_version") for report in passing_reports}
        artifact_manifests = {
            json.dumps(report.get("artifact_sha256"), sort_keys=True) for report in passing_reports
        }
        if len(config_hashes) > 1:
            failures.append("PASS reports do not share one effective config SHA-256")
        if len(calibration_versions) > 1:
            failures.append("PASS reports do not share one calibration version")
        if len(artifact_manifests) > 1:
            failures.append("PASS reports do not share one generated artifact manifest")
        source_pairs = {
            json.dumps(report.get("tested_pair"), sort_keys=True)
            for name, report in reports.items()
            if name in SOFTWARE_GATES and name in valid_reports
        }
        if len(source_pairs) > 1:
            failures.append("software PASS reports do not share one tested source pair")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--software-reports-dir", type=Path)
    args = parser.parse_args()
    failures = release_failures(software_reports_dir=args.software_reports_dir)
    if failures:
        print(f"{RELEASE} release blocked:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(f"{RELEASE} release gate PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
