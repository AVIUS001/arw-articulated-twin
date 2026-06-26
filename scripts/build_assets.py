#!/usr/bin/env python3
"""Build USD articulation assets for ARW variants."""
from __future__ import annotations

import argparse
from pathlib import Path

from arw_articulated_twin.usd_builder import build_all_assets, build_usd


def main() -> None:
    parser = argparse.ArgumentParser(description="Build ARW articulated USD assets")
    parser.add_argument("--output", type=Path, default=Path("assets"))
    parser.add_argument("--vehicle", choices=["AER8110-1", "AER8110-2", "AER8100-1", "all"], default="all")
    args = parser.parse_args()

    if args.vehicle == "all":
        paths = build_all_assets(args.output)
    else:
        slug = args.vehicle.lower().replace("-", "_")
        paths = [build_usd(args.vehicle, args.output / f"arw_{slug}.usda")]

    for p in paths:
        print(f"Wrote {p}")


if __name__ == "__main__":
    main()
