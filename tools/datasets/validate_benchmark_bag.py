#!/usr/bin/env python3
"""Validate rosbag2 metadata against the named benchmark contract."""

import argparse
import json
from pathlib import Path

import yaml

from benchmark_bags import load_catalog, validate_metadata


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG = (
    ROOT / "verification" / "configs" / "benchmarks" / "rosbag-datasets-v0.6.0-rc2.yaml"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    args = parser.parse_args()

    metadata_path = args.bag / "metadata.yaml" if args.bag.is_dir() else args.bag
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    result = validate_metadata(load_catalog(args.catalog), args.dataset, metadata)
    print(json.dumps(result._asdict(), ensure_ascii=False, indent=2))
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
