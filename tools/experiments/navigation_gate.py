"""Aggregate 10-target navigation trials and apply the strict vehicle gate."""

import math
from collections import Counter
from pathlib import Path
from typing import Any, Dict, NamedTuple, Sequence, Tuple

import yaml


class NavigationResult(NamedTuple):
    passed: bool
    metrics: Dict[str, float]
    failed: Tuple[str, ...]
    coverage_errors: Tuple[str, ...]


def load_contract(path: Path) -> Dict[str, Any]:
    contract = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(contract, dict) or contract.get("schema_version") != 1:
        raise ValueError("unsupported navigation contract")
    if len(contract.get("targets", [])) != 10:
        raise ValueError("navigation contract must define exactly 10 targets")
    return contract


def _percentile95(values: Sequence[float]) -> float:
    try:
        converted = [float(value) for value in values]
    except (TypeError, ValueError):
        return math.inf
    finite = sorted(value for value in converted if math.isfinite(value))
    if len(finite) != len(converted) or not finite:
        return math.inf
    return finite[max(0, math.ceil(0.95 * len(finite)) - 1)]


def _passes(value: float, rule: Dict[str, Any]) -> bool:
    if not math.isfinite(float(value)):
        return False
    limit = float(rule["limit"])
    if rule["operator"] == "<=":
        return float(value) <= limit
    if rule["operator"] == ">=":
        return float(value) >= limit
    raise ValueError(f"unsupported operator: {rule['operator']}")


def evaluate_runs(contract: Dict[str, Any], runs: Sequence[Dict[str, Any]]) -> NavigationResult:
    target_ids = tuple(target["id"] for target in contract["targets"])
    expected_repetitions = int(contract["repetitions_per_target"])
    counts = Counter(str(run.get("target_id")) for run in runs)
    coverage_errors = []
    for target_id in target_ids:
        if counts[target_id] != expected_repetitions:
            coverage_errors.append(
                f"{target_id}: expected {expected_repetitions}, got {counts[target_id]}"
            )
    unknown = sorted(set(counts) - set(target_ids))
    coverage_errors.extend(f"unknown target: {target_id}" for target_id in unknown)
    for index, run in enumerate(runs):
        if not isinstance(run.get("success"), bool):
            coverage_errors.append(f"run {index}: success must be bool")
        if run.get("success") is True:
            for field in ("position_error_m", "yaw_error_rad"):
                raw_value = run.get(field)
                try:
                    finite = raw_value is not None and math.isfinite(float(raw_value))
                except (TypeError, ValueError):
                    finite = False
                if not finite:
                    coverage_errors.append(f"run {index}: invalid {field}")
        try:
            recovery_count = int(run.get("recovery_count", 0) or 0)
        except (TypeError, ValueError):
            recovery_count = -1
        if recovery_count < 0:
            coverage_errors.append(f"run {index}: invalid recovery_count")
        boolean_fields = ["collision", "manual_intervention", "unexpected_motion"]
        if "communication_degradation_count" in contract["acceptance"]:
            boolean_fields.append("communication_degradation")
        for field in boolean_fields:
            if not isinstance(run.get(field), bool):
                coverage_errors.append(f"run {index}: {field} must be bool")

    successful = [run for run in runs if run.get("success") is True]
    metrics = {
        "total_trials": len(runs),
        "successful_trials": len(successful),
        "success_rate_percent": 100.0 * len(successful) / len(runs) if runs else 0.0,
        "position_error_p95_m": _percentile95(
            [run.get("position_error_m", math.inf) for run in successful]
        ),
        "yaw_error_p95_rad": _percentile95(
            [run.get("yaw_error_rad", math.inf) for run in successful]
        ),
        "recovery_count": sum(int(run.get("recovery_count", 0) or 0) for run in runs),
        "collision_count": sum(run.get("collision") is True for run in runs),
        "manual_intervention_count": sum(run.get("manual_intervention") is True for run in runs),
        "unexpected_motion_count": sum(run.get("unexpected_motion") is True for run in runs),
        "communication_degradation_count": sum(
            run.get("communication_degradation") is True for run in runs
        ),
    }
    failed = tuple(
        name
        for name, rule in contract["acceptance"].items()
        if name not in metrics or not _passes(metrics[name], rule)
    )
    coverage_tuple = tuple(coverage_errors)
    return NavigationResult(not failed and not coverage_tuple, metrics, failed, coverage_tuple)
