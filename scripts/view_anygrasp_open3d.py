"""Display a saved AnyGrasp RGB-D result in a clean Open3D process."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from scale_bench.isaaclab.runtime.anygrasp_diagnostics import (
    AnyGraspOpen3DFrame,
    show_anygrasp_open3d,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()
    if not args.bundle.is_file():
        parser.error(f"bundle does not exist: {args.bundle}")
    show_anygrasp_open3d(AnyGraspOpen3DFrame.read(args.bundle))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
