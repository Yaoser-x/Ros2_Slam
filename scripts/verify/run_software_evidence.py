#!/usr/bin/env python3
"""Run Linux software checks and bind results to the actual two source trees.

Reports are separate from physical acceptance and default NOT_RUN templates.
Use an external or ignored build directory so recording evidence cannot alter
the source identity that was just tested.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
RELEASE = "v0.6.0-rc2"
GATES = ("ros_ci", "protocol_cross_test", "pty_report")


def source_identity(root: Path, companion: Path | None = None) -> dict:
    def git(*args):
        return subprocess.check_output(["git", "-C", str(root), *args])

    commit = git("rev-parse", "HEAD").decode().strip()
    pathspec = ["--", "."]
    if companion is not None and companion.is_relative_to(root):
        # CI checks out the independently identified firmware inside upper.
        # It belongs to the second identity, not upper's untracked sources.
        pathspec.append(f":(exclude){companion.relative_to(root).as_posix()}")
    dirty = bool(git("status", "--porcelain", "-z", "--untracked-files=all", *pathspec))
    files = git("ls-files", "--cached", "--others", "--exclude-standard", "-z", *pathspec).split(
        b"\0"
    )
    digest = hashlib.sha256()
    for relative in sorted(set(files)):
        if not relative:
            continue
        path = root / os.fsdecode(relative)
        digest.update(relative + b"\0")
        if path.is_symlink():
            digest.update(b"symlink\0" + os.fsencode(os.readlink(path)))
        elif path.is_file():
            digest.update(b"file\0" + path.read_bytes())
        else:
            digest.update(b"missing\0")
        digest.update(b"\0")
    return {"commit": commit, "dirty": dirty, "source_sha256": digest.hexdigest()}


def junit_counts(path: Path) -> dict:
    tree = ET.parse(path)
    cases = list(tree.iter("testcase"))
    return {
        "tests_executed": sum(case.find("skipped") is None for case in cases),
        "skipped": sum(case.find("skipped") is not None for case in cases),
        "failed": sum(
            case.find("failure") is not None or case.find("error") is not None for case in cases
        ),
    }


def build_report(gate, pair_before, pair_after, checks, counts, failures, started_at):
    failures = list(failures)
    if pair_before != pair_after:
        failures.append("source pair changed during software checks")
    if counts.get("tests_executed", 0) <= 0 or counts.get("skipped", 0) or counts.get("failed", 0):
        failures.append("test evidence is empty, skipped or failed")
    if not checks or any(check["exit_code"] != 0 for check in checks.values()):
        failures.append("one or more software checks did not succeed")
    return {
        "schema_version": 1,
        "release": RELEASE,
        "platform_api_version": 5,
        "protocol_version": 3,
        "evidence_type": "software",
        "gate": gate,
        "started_at": started_at,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "upper_commit": pair_before["upper"]["commit"],
        "firmware_commit": pair_before["firmware"]["commit"],
        "tested_pair": pair_before,
        "source_pair_after": pair_after,
        "checks": checks,
        "metrics": counts,
        "failures": failures,
        "result": "FAIL" if failures else "PASS",
    }


def commands(gate: str, output: Path, with_coverage: bool = False):
    junit = output / f"{gate}.junit.xml"
    pytest = [sys.executable, "-m", "pytest"]
    if gate == "ros_ci":
        installed = ["bash", "-c", '. install/setup.bash && exec "$@"', "software-evidence"]
        coverage = (
            [
                "--cov=src/stm32_robot_bridge",
                "--cov=src/robot_control",
                "--cov=src/robot_sensing",
                "--cov=src/robot_state_estimation",
                "--cov=src/robot_supervision",
                "--cov=src/robot_verification",
                "--cov-report=term",
                "--cov-report=xml",
            ]
            if with_coverage
            else []
        )
        smoke = (
            "import rclpy; from rclpy.node import Node; "
            "from robot_interfaces.action import ChassisOperation; "
            "from robot_interfaces.msg import HostMotionCommand; "
            "rclpy.init(); n=Node('software_evidence'); seen=[]; "
            "n.create_timer(0.001, lambda: seen.append(True)); "
            "rclpy.spin_once(n, timeout_sec=1.0); assert seen; "
            "n.destroy_node(); rclpy.shutdown()"
        )
        return {
            "colcon_build": [
                "colcon",
                "build",
                "--base-paths",
                "src",
                "--symlink-install",
                "--packages-skip",
                "ydlidar_ros2_driver",
            ],
            "colcon_test": installed
            + [
                "colcon",
                "test",
                "--base-paths",
                "src",
                "--packages-skip",
                "ydlidar_ros2_driver",
                "--return-code-on-test-failure",
            ],
            "colcon_test_result": ["colcon", "test-result", "--verbose"],
            "ros_runtime": installed + [sys.executable, "-c", smoke],
            "pytest": installed
            + pytest
            + ["tests/", "src/", f"--junitxml={junit}", "-q"]
            + coverage,
        }, junit
    name = "protocol_cross" if gate == "protocol_cross_test" else "pty"
    paths = (
        ["tests/protocol/", "tests/integration/test_real_lower_v3_contract.py"]
        if gate == "protocol_cross_test"
        else ["tests/integration/test_pty_fake_stm32.py"]
    )
    return {name: pytest + paths + [f"--junitxml={junit}", "-q"]}, junit


def run_gate(gate, upper, firmware, output, with_coverage=False):
    started_at = datetime.now(timezone.utc).isoformat()
    before = {"upper": source_identity(upper, firmware), "firmware": source_identity(firmware)}
    checks = {}
    failures = []
    command_map, junit = commands(gate, output, with_coverage)
    junit.unlink(missing_ok=True)
    environment = os.environ.copy()
    environment["FIRMWARE_CONTRACT_DIR"] = str(firmware)
    for name, argv in command_map.items():
        log = output / f"{gate}-{name}.log"
        try:
            # Each colcon process is independent; pytest uses built generated
            # ROS interfaces via the installed workspace Python environment.
            execution = subprocess.run(
                argv,
                cwd=upper,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=1800,
                check=False,
            )
            content, exit_code = execution.stdout, execution.returncode
        except (OSError, subprocess.TimeoutExpired) as exc:
            content, exit_code = str(exc).encode(), -1
        log.write_bytes(content)
        checks[name] = {
            "command": argv,
            "exit_code": exit_code,
            "result": "PASS" if exit_code == 0 else "FAIL",
            "log": str(log),
            "log_sha256": hashlib.sha256(content).hexdigest(),
        }
        if exit_code != 0:
            failures.append(f"{name} failed with exit code {exit_code}")
            break
    counts = {}
    if junit.is_file():
        try:
            counts = junit_counts(junit)
        except (OSError, ET.ParseError) as exc:
            failures.append(f"cannot read test results: {exc}")
    after = {"upper": source_identity(upper, firmware), "firmware": source_identity(firmware)}
    report = build_report(gate, before, after, checks, counts, failures, started_at)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{gate}.", dir=output)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(yaml.safe_dump(report, sort_keys=False))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output / f"{gate}.yaml")
    except Exception:
        Path(temporary).unlink(missing_ok=True)
        raise
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--firmware-root", type=Path, default=ROOT.parent / "lower")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gates", nargs="+", choices=GATES, default=list(GATES))
    parser.add_argument("--with-coverage", action="store_true")
    args = parser.parse_args()
    if platform.system() != "Linux":
        parser.error("software release evidence requires a real Linux environment")
    output = args.output_dir.resolve()
    for repository in (ROOT, args.firmware_root.resolve()):
        if output.is_relative_to(repository):
            ignored = subprocess.run(
                ["git", "-C", str(repository), "check-ignore", str(output)], capture_output=True
            )
            if ignored.returncode != 0:
                parser.error(
                    "output must be external to the source trees or inside an ignored build directory"
                )
    output.mkdir(parents=True, exist_ok=True)
    reports = [
        run_gate(gate, ROOT, args.firmware_root.resolve(), output, args.with_coverage)
        for gate in args.gates
    ]
    for report in reports:
        print(
            json.dumps(
                {
                    "gate": report["gate"],
                    "result": report["result"],
                    "tested_pair": report["tested_pair"],
                }
            )
        )
    return 0 if all(report["result"] == "PASS" for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
