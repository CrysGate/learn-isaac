"""Headless end-to-end tasks for the Piper atomic skill library."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNNER = PROJECT_ROOT / "manipulation_skills" / "demo_atomic_tasks.py"


@pytest.mark.integration
def test_atomic_skills_on_existing_piper_and_doll_assets() -> None:
    asset_root = PROJECT_ROOT / "Assets"
    if not asset_root.exists():
        pytest.skip("the ScaleBench asset bundle is not available")

    environment = os.environ.copy()
    source_root = str(PROJECT_ROOT / "src")
    existing_pythonpath = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        source_root
        if not existing_pythonpath
        else os.pathsep.join((source_root, existing_pythonpath))
    )
    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--tasks",
            "move_home",
            "gripper",
            "pick",
            "pick_place",
            "reorient",
            "--robot",
            "auto",
            "--headless",
            "--viz",
            "none",
        ],
        cwd=PROJECT_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=900,
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "ATOMIC_TASKS_OK" in output
