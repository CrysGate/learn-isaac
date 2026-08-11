"""Subprocess-isolated headless integration coverage for ScaleBenchEnv."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).with_name("_headless_runtime_smoke.py")


def _find_asset_root() -> Path | None:
    configured = os.environ.get("SCALE_BENCH_ASSET_ROOT")
    candidates = [Path(configured)] if configured else [PROJECT_ROOT]

    common_dir = subprocess.run(
        ["git", "rev-parse", "--git-common-dir"],
        cwd=PROJECT_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if common_dir.returncode == 0:
        git_dir = Path(common_dir.stdout.strip())
        if not git_dir.is_absolute():
            git_dir = PROJECT_ROOT / git_dir
        candidates.append(git_dir.resolve().parent)

    for candidate in candidates:
        resolved = candidate.resolve()
        if (resolved / "Assets").is_dir():
            return resolved
    return None


@pytest.mark.integration
def test_headless_runtime_contract() -> None:
    asset_root = _find_asset_root()
    if asset_root is None:
        pytest.skip(
            "set SCALE_BENCH_ASSET_ROOT to run the Isaac runtime integration test"
        )

    environment = os.environ.copy()
    source_root = str(PROJECT_ROOT / "src")
    existing_pythonpath = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        source_root
        if not existing_pythonpath
        else os.pathsep.join((source_root, existing_pythonpath))
    )
    result = subprocess.run(
        [sys.executable, str(RUNNER), str(asset_root)],
        cwd=PROJECT_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=360,
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCALE_BENCH_HEADLESS_RUNTIME_OK" in output
