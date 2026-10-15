#!/usr/bin/env python3
"""Validate timestamped physical UART HIL evidence and render a fail-closed report."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


SHA40 = re.compile(r"^[0-9a-f]{40}$")
SHA64 = re.compile(r"^[0-9a-f]{64}$")
COMPUTED_FIELDS = frozenset(
    {
        "schema_version",
        "release",
        "result",
        "metrics",
        "failures",
        "event_sha256",
        "serial_trace_sha256",
        "wheel_stop_acceptance",
    }
)


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"mapping required: {path}")
    return value


def _load_events(path: Path) -> dict[str, dict[str, int]]:
    result: dict[str, dict[str, int]] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        scenario, event = str(row["scenario"]), str(row["event"])
        timestamp = int(row["monotonic_ns"])
        if timestamp < 0 or event in result.setdefault(scenario, {}):
            raise ValueError(f"invalid or duplicate event at line {line_number}")
        result[scenario][event] = timestamp
    return result


def evaluate(
    contract: dict[str, Any], metadata: dict[str, Any], events: dict[str, dict[str, int]]
) -> tuple[list[str], dict[str, Any]]:
    failures: list[str] = []
    for key in sorted(COMPUTED_FIELDS & metadata.keys()):
        failures.append(f"metadata may not override computed field {key}")
    required_metadata = (
        "upper_commit",
        "firmware_commit",
        "hardware_revision",
        "parameter_crc32",
        "config_sha256",
        "artifact_sha256",
        "calibration_version",
    )
    for key in required_metadata:
        if metadata.get(key) in (None, ""):
            failures.append(f"metadata missing {key}")
    if (
        not isinstance(metadata.get("upper_commit"), str)
        or not SHA40.fullmatch(metadata.get("upper_commit", ""))
        or metadata.get("upper_commit") == "0" * 40
    ):
        failures.append("upper_commit is not a 40-character SHA")
    if (
        not isinstance(metadata.get("firmware_commit"), str)
        or not SHA40.fullmatch(metadata.get("firmware_commit", ""))
        or metadata.get("firmware_commit") == "0" * 40
    ):
        failures.append("firmware_commit is not a 40-character SHA")
    if metadata.get("firmware_commit") != contract["firmware_candidate_commit"]:
        failures.append("firmware_commit differs from frozen candidate")
    if not SHA64.fullmatch(str(metadata.get("config_sha256", ""))):
        failures.append("config_sha256 is not a SHA-256")
    parameter_crc = metadata.get("parameter_crc32")
    if (
        isinstance(parameter_crc, bool)
        or not isinstance(parameter_crc, int)
        or not 0 < parameter_crc <= 0xFFFFFFFF
    ):
        failures.append("parameter_crc32 must be a measured non-zero integer")
    hardware_revision = metadata.get("hardware_revision")
    if (
        isinstance(hardware_revision, bool)
        or not isinstance(hardware_revision, int)
        or not 0 < hardware_revision <= 0xFFFFFFFF
    ):
        failures.append("hardware_revision must be a positive integer")
    artifact_hashes = metadata.get("artifact_sha256")
    if not isinstance(artifact_hashes, dict) or not artifact_hashes:
        failures.append("artifact_sha256 must be a non-empty mapping")
    elif any(
        not isinstance(name, str)
        or not name
        or not isinstance(digest, str)
        or not SHA64.fullmatch(digest)
        for name, digest in artifact_hashes.items()
    ):
        failures.append("artifact_sha256 contains an invalid artifact name or digest")
    if not isinstance(metadata.get("calibration_version"), str) or not metadata.get(
        "calibration_version"
    ):
        failures.append("calibration_version must be recorded")

    metrics: dict[str, Any] = {"scenario_count": len(contract["scenarios"]), "timing_ms": {}}
    scenario_ids = [str(scenario.get("id", "")) for scenario in contract.get("scenarios", ())]
    if not scenario_ids or any(not scenario_id for scenario_id in scenario_ids):
        failures.append("contract contains an empty scenario id")
    if len(set(scenario_ids)) != len(scenario_ids):
        failures.append("contract contains duplicate scenario ids")
    for scenario in contract["scenarios"]:
        scenario_id = scenario["id"]
        observed = events.get(scenario_id, {})
        missing = [name for name in scenario["required_events"] if name not in observed]
        if missing:
            failures.append(f"{scenario_id} missing events: {','.join(missing)}")
            continue
        ordered = [observed[name] for name in scenario["required_events"]]
        if any(later <= earlier for earlier, later in zip(ordered, ordered[1:])):
            failures.append(f"{scenario_id} event timestamps are not strictly increasing")
        if "max_motor_target_zero_ms" in scenario:
            elapsed = (
                observed["motor_target_zero_at"] - observed["fault_injected_at"]
            ) / 1_000_000.0
            metrics["timing_ms"][scenario_id] = elapsed
            if elapsed > float(scenario["max_motor_target_zero_ms"]):
                failures.append(f"{scenario_id} motor target zero took {elapsed:.3f} ms")
        if "wheel_speed_below_threshold_at" in scenario["required_events"]:
            # Event names alone cannot prove a physical speed threshold.  The
            # current v3 contract has no recorded threshold/measurement rule.
            metrics.setdefault("wheel_stop_acceptance", {})[scenario_id] = "NOT_EVALUATED"
            failures.append(
                f"{scenario_id} wheel-stop threshold and measured speed acceptance are not defined"
            )
    return failures, metrics


def build_report(contract, metadata, events, event_bytes, serial_trace_bytes):
    failures, metrics = evaluate(contract, metadata, events)
    if not serial_trace_bytes:
        failures.append("serial trace is missing or empty")
    # Computed fields are written last even when hostile metadata is rejected.
    return {
        **{key: value for key, value in metadata.items() if key not in COMPUTED_FIELDS},
        "started_at": metadata.get("started_at") or datetime.now(timezone.utc).isoformat(),
        "schema_version": 1,
        "release": contract["release"],
        "event_sha256": hashlib.sha256(event_bytes).hexdigest(),
        "serial_trace_sha256": (
            hashlib.sha256(serial_trace_bytes).hexdigest() if serial_trace_bytes else None
        ),
        "metrics": metrics,
        "failures": failures,
        "result": "FAIL" if failures else "PASS",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--serial-trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    contract, metadata = _load_yaml(args.contract), _load_yaml(args.metadata)
    report = build_report(
        contract,
        metadata,
        _load_events(args.events),
        args.events.read_bytes(),
        args.serial_trace.read_bytes() if args.serial_trace.is_file() else b"",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{args.output.name}.", dir=args.output.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(yaml.safe_dump(report, sort_keys=False))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, args.output)
        directory = os.open(args.output.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return 1 if report["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
