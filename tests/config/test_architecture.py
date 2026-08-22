"""Dependency and import-safety checks for the pure Python layers."""

from __future__ import annotations

import ast
import os
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
API_PATH = PROJECT_ROOT / "src/scale_bench/api.py"
CONFIG_ROOT = PROJECT_ROOT / "src/scale_bench/config"
TASKS_ROOT = PROJECT_ROOT / "src/scale_bench/tasks"
LEGACY_PACKAGE_NAMES = ("envs", "robots", "scenes", "sensors", "sim")
FORBIDDEN_ROOTS = {
    "isaaclab",
    "isaaclab_physx",
    "isaacsim",
    "omni",
    "pxr",
    "torch",
}


def test_pure_layers_have_no_eager_simulator_or_tensor_imports() -> None:
    violations: list[str] = []
    for root in (CONFIG_ROOT, TASKS_ROOT):
        for path in root.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            # Task methods may construct native evaluator terms through delayed
            # imports. Only module-level imports affect pure-layer import safety.
            for node in tree.body:
                names: list[str] = []
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module is not None:
                    names = [node.module]
                for name in names:
                    if name.split(".", maxsplit=1)[0] in FORBIDDEN_ROOTS:
                        violations.append(
                            f"{path.relative_to(PROJECT_ROOT)}:{node.lineno}"
                        )

    assert violations == []


def test_pure_imports_do_not_load_simulator_or_tensor_modules() -> None:
    script = """
import sys
import scale_bench
import scale_bench.api
import scale_bench.config
import scale_bench.config.loader
import scale_bench.config.models.camera
import scale_bench.config.models.environment
import scale_bench.config.models.robot
import scale_bench.config.models.scene
import scale_bench.config.models.simulation
import scale_bench.tasks
import scale_bench.tasks.common.layout
import scale_bench.tasks.common.placement
import scale_bench.tasks.common.rigid_object
import scale_bench.tasks.common.task
import scale_bench.tasks.sort_dolls_by_size.config
import scale_bench.tasks.sort_dolls_by_size.task

forbidden = ('isaaclab', 'isaaclab_physx', 'isaacsim', 'omni', 'pxr', 'torch')
loaded = sorted(
    name for name in sys.modules
    if any(name == root or name.startswith(root + '.') for root in forbidden)
)
if loaded:
    raise SystemExit('forbidden modules loaded: ' + ', '.join(loaded))
"""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(PROJECT_ROOT / "src")

    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0, result.stderr or result.stdout


def test_config_package_initializers_are_lightweight() -> None:
    for relative_path in ("__init__.py", "models/__init__.py"):
        path = CONFIG_ROOT / relative_path
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        imports = [
            node
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
        ]
        assert imports == [], relative_path


def test_api_only_imports_adapter_modules_inside_functions() -> None:
    tree = ast.parse(API_PATH.read_text(encoding="utf-8"), filename=str(API_PATH))
    adapter_imports = [
        node
        for node in tree.body
        if isinstance(node, ast.ImportFrom)
        and node.module is not None
        and node.module.startswith("scale_bench.isaaclab")
    ]
    assert adapter_imports == []


def test_task_package_initializers_are_lightweight() -> None:
    for relative_path in (
        "__init__.py",
        "common/__init__.py",
        "sort_dolls_by_size/__init__.py",
    ):
        path = TASKS_ROOT / relative_path
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        imports = [
            node
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
        ]
        assert imports == [], relative_path


def test_legacy_compatibility_packages_are_removed() -> None:
    package_root = PROJECT_ROOT / "src/scale_bench"
    remaining = [
        name for name in LEGACY_PACKAGE_NAMES if (package_root / name).exists()
    ]
    assert remaining == []
