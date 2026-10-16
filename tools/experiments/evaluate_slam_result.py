#!/usr/bin/env python3
"""Apply the strict SLAM acceptance gate to one measured result file."""

import argparse
import json
from pathlib import Path

import yaml

from experiment_gate import evaluate_metrics, load_experiment


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "verification" / "configs" / "experiments" / "slam-sweep-v0.6.0-rc2.yaml"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--experiment", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()

    result_data = yaml.safe_load(args.result.read_text(encoding="utf-8"))
    gate = evaluate_metrics(load_experiment(args.experiment), result_data.get("metrics", {}))
    output = {
        "passed": gate.passed,
        "missing": list(gate.missing),
        "failed": list(gate.failed),
    }
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0 if gate.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
