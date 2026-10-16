#!/usr/bin/env python3
"""Evaluate one 50-run Nav2 result file."""

import argparse
import json
import math
from pathlib import Path

import yaml

from navigation_gate import evaluate_runs, load_contract


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = (
    ROOT / "verification" / "configs" / "experiments" / "nav2-acceptance-v0.6.0-rc2.yaml"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    args = parser.parse_args()

    data = yaml.safe_load(args.result.read_text(encoding="utf-8"))
    result = evaluate_runs(load_contract(args.contract), data.get("runs", []))
    printable_metrics = {
        name: value if math.isfinite(float(value)) else None
        for name, value in result.metrics.items()
    }
    print(
        json.dumps(
            {
                "passed": result.passed,
                "metrics": printable_metrics,
                "failed": list(result.failed),
                "coverage_errors": list(result.coverage_errors),
            },
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
    )
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
