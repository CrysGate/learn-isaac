"""Pure Python task layout and rule tests."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from pydantic import ValidationError

from scale_bench.tasks.common.layout import AssetPlacement, TaskLayout
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.common.rigid_object import (
    RigidObjectAssetConfig,
    RigidObjectPhysicsConfig,
    RigidObjectTask,
    RigidObjectTaskConfig,
)
from scale_bench.tasks.sort_dolls_by_size.config import (
    DollAssetConfig,
    SortDollsBySizeConfig,
)
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize


class ExampleTask(RigidObjectTask):
    TASK_ID = "example"


@pytest.fixture
def context() -> PlacementContext:
    return PlacementContext(
        table_top_z_m=0.75,
        x_range_m=(-0.4, 0.4),
        y_range_m=(-0.5, 0.5),
    )


def _write_metadata(
    path: Path,
    *,
    size: tuple[float, float, float],
    mass: float = 0.2,
    friction: float = 0.6,
) -> None:
    path.write_text(
        json.dumps(
            {
                "physics": {
                    "size": size,
                    "mass": mass,
                    "friction": friction,
                }
            }
        ),
        encoding="utf-8",
    )


def _example_task(tmp_path: Path) -> ExampleTask:
    assets: dict[str, RigidObjectAssetConfig] = {}
    for name, size in (
        ("large", (0.12, 0.10, 0.20)),
        ("small", (0.06, 0.04, 0.10)),
    ):
        metadata_path = tmp_path / f"{name}.json"
        _write_metadata(metadata_path, size=size)
        assets[name] = RigidObjectAssetConfig(
            usd_path=str(tmp_path / f"{name}.usd"),
            metadata_path=str(metadata_path),
        )
    return ExampleTask(
        RigidObjectTaskConfig(
            instruction="Place both objects.",
            spawn_clearance_m=0.003,
            minimum_object_gap_m=0.02,
            physics=RigidObjectPhysicsConfig(restitution=0.0),
        ),
        assets,
    )


def test_rigid_object_layout_is_deterministic_and_context_driven(
    tmp_path: Path,
    context: PlacementContext,
) -> None:
    task = _example_task(tmp_path)

    first = task.generate_layout(context, 42)
    second = task.generate_layout(context, 42)
    other_seed = task.generate_layout(context, 43)

    assert first == second
    assert first != other_seed
    assert first.seed == 42
    assert tuple(first.assets) == ("large", "small")
    assert not hasattr(task, "scene_config")

    shifted = context.model_copy(update={"table_top_z_m": 1.25})
    shifted_layout = task.generate_layout(shifted, 42)
    for name, placement in first.assets.items():
        assert shifted_layout.assets[name].position_m[:2] == placement.position_m[:2]
        assert shifted_layout.assets[name].position_m[2] == pytest.approx(
            placement.position_m[2] + 0.5
        )


def test_layout_validation_and_json_round_trip(
    tmp_path: Path,
    context: PlacementContext,
) -> None:
    task = _example_task(tmp_path)
    layout = task.generate_layout(context, 7)
    layout_path = tmp_path / "layouts/7.json"

    assert layout.save(layout_path) == layout_path
    loaded = TaskLayout.load(layout_path)
    assert loaded == layout
    assert task.resolve_layout(context, layout_path=layout_path) == layout

    overlapping = layout.model_copy(
        update={
            "assets": {
                name: AssetPlacement(
                    position_m=(0.0, 0.0, placement.position_m[2]),
                    orientation_xyzw=placement.orientation_xyzw,
                )
                for name, placement in layout.assets.items()
            }
        }
    )
    with pytest.raises(ValueError, match="overlap|minimum_object_gap_m"):
        task.validate_layout(context, overlapping)


def test_generated_assets_fit_bounds_and_keep_minimum_gap(
    tmp_path: Path,
    context: PlacementContext,
) -> None:
    task = _example_task(tmp_path)
    layout = task.generate_layout(context, 99)
    radii = {
        name: math.hypot(*metadata.size[:2]) / 2.0
        for name, metadata in task.metadata.items()
    }

    for name, placement in layout.assets.items():
        radius = radii[name]
        assert context.x_range_m[0] + radius <= placement.position_m[0]
        assert placement.position_m[0] <= context.x_range_m[1] - radius
        assert context.y_range_m[0] + radius <= placement.position_m[1]
        assert placement.position_m[1] <= context.y_range_m[1] - radius

    large = layout.assets["large"].position_m
    small = layout.assets["small"].position_m
    assert math.hypot(large[0] - small[0], large[1] - small[1]) >= (
        radii["large"] + radii["small"] + task.config.minimum_object_gap_m
    )


def test_sort_dolls_target_order_comes_from_metadata(tmp_path: Path) -> None:
    dolls = []
    for asset_id, height in (("00000", 0.30), ("00001", 0.10), ("00002", 0.20)):
        metadata_path = tmp_path / f"{asset_id}.json"
        _write_metadata(metadata_path, size=(0.05, 0.05, height))
        dolls.append(
            DollAssetConfig(
                asset_id=asset_id,
                usd_path=str(tmp_path / f"{asset_id}.usd"),
                metadata_path=str(metadata_path),
            )
        )

    config = SortDollsBySizeConfig(
        instruction="Sort the dolls.",
        physics=RigidObjectPhysicsConfig(restitution=0.01),
        dolls=tuple(dolls),
    )
    task = SortDollsBySize(config)

    assert task.target_order_small_to_large == ("00001", "00002", "00000")
    assert not hasattr(SortDollsBySizeConfig, "load")


def test_task_models_reject_invalid_values() -> None:
    with pytest.raises(ValidationError, match="unit quaternion"):
        AssetPlacement(
            position_m=(0.0, 0.0, 0.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 2.0),
        )
    with pytest.raises(ValidationError, match="unique"):
        SortDollsBySizeConfig(
            instruction="Sort the dolls.",
            physics=RigidObjectPhysicsConfig(restitution=0.01),
            dolls=(
                DollAssetConfig(
                    asset_id="00000",
                    usd_path="a.usd",
                    metadata_path="a.json",
                ),
                DollAssetConfig(
                    asset_id="00000",
                    usd_path="b.usd",
                    metadata_path="b.json",
                ),
            ),
        )
