#!/usr/bin/env python3
"""Record one named benchmark bag and persist its reproducibility manifest."""

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
from pathlib import Path

import yaml

from benchmark_bags import build_record_plan, load_catalog


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG = (
    ROOT / "verification" / "configs" / "benchmarks" / "rosbag-datasets-v0.6.0-rc2.yaml"
)


def git_value(*args: str) -> str:
    result = subprocess.run(
        ("git", *args),
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset")
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    catalog = load_catalog(args.catalog)
    now = dt.datetime.now(dt.timezone.utc)
    run_id = args.run_id or now.strftime("%Y%m%dT%H%M%SZ")
    output_root = args.output_root or Path(catalog["default_output_root"])
    plan = build_record_plan(catalog, args.dataset, output_root=output_root, run_id=run_id)
    effective_config_value = os.environ.get("ROBOT_EFFECTIVE_CONFIG", "")
    effective_config = Path(effective_config_value)
    if not effective_config_value or not effective_config.is_file():
        raise SystemExit("ROBOT_EFFECTIVE_CONFIG is required; start through bin/robot")
    manifest = {
        "schema_version": 1,
        "dataset": args.dataset,
        "run_id": run_id,
        "started_utc": now.isoformat(),
        "upper_commit": git_value("rev-parse", "HEAD"),
        "upper_dirty": bool(git_value("status", "--porcelain")),
        "config_sha256": file_sha256(effective_config),
        "effective_config": str(effective_config),
        "procedure": catalog["datasets"][args.dataset]["procedure"],
        "command": list(plan.command),
    }
    if args.dry_run:
        print(
            json.dumps(
                {**manifest, "output_dir": str(plan.output_dir)}, ensure_ascii=False, indent=2
            )
        )
        return 0
    if plan.output_dir.exists():
        raise SystemExit(f"output already exists: {plan.output_dir}")
    plan.output_dir.parent.mkdir(parents=True, exist_ok=True)
    manifest_path = plan.output_dir.parent / f"{run_id}.manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return_code = 130
    try:
        return_code = subprocess.call(plan.command)
        return return_code
    finally:
        manifest["completed_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        manifest["return_code"] = return_code
        manifest["bag_path"] = str(plan.output_dir)
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
