#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from robot_config.compiler import compile_effective_config, load_effective_config
from robot_config.runtime_options import runtime_overrides


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--robot-id", default="robot_001")
    parser.add_argument(
        "--profile",
        choices=("fake", "mapping", "localization", "navigation", "hil", "diagnostics"),
        default="mapping",
    )
    parser.add_argument("--runtime-args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    overrides = None
    if args.runtime_args is not None:
        base = load_effective_config(args.config_root, robot_id=args.robot_id, profile=args.profile)
        overrides = runtime_overrides(args.runtime_args, base)
    paths = compile_effective_config(
        args.config_root,
        args.output_dir,
        robot_id=args.robot_id,
        profile=args.profile,
        overrides=overrides,
    )
    print(paths["manifest"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
