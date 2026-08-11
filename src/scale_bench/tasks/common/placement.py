"""Placement context plus deterministic tabletop layout algorithms."""

from __future__ import annotations

import math
import random
from collections.abc import Mapping
from typing import Self

from pydantic import field_validator

from scale_bench.config.base import FiniteFloat, FrozenModel
from scale_bench.config.models.scene import SceneConfig

from .layout import AssetPlacement, TaskLayout


class PlacementContext(FrozenModel):
    """Scene-derived values needed by task placement algorithms."""

    table_top_z_m: FiniteFloat
    x_range_m: tuple[FiniteFloat, FiniteFloat]
    y_range_m: tuple[FiniteFloat, FiniteFloat]

    @field_validator("x_range_m", "y_range_m")
    @classmethod
    def _validate_range(cls, value: tuple[float, float]) -> tuple[float, float]:
        if value[0] >= value[1]:
            raise ValueError("lower bound must be less than upper bound")
        return value

    @classmethod
    def from_scene_config(cls, scene_config: SceneConfig) -> Self:
        """Extract placement-only values from a scene configuration."""

        area = scene_config.task_object_placement_area
        return cls(
            table_top_z_m=scene_config.table_top_z_m,
            x_range_m=area.x_range_m,
            y_range_m=area.y_range_m,
        )


def generate_tabletop_layout(
    *,
    task_id: str,
    context: PlacementContext,
    asset_sizes_m: Mapping[str, tuple[float, float, float]],
    seed: int,
    spawn_clearance_m: float,
    minimum_object_gap_m: float,
    sampling_attempts_per_object: int,
) -> TaskLayout:
    """Deterministically sample a non-overlapping upright layout."""

    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a non-negative integer")

    radii = _footprint_radii(asset_sizes_m)
    rng = random.Random(seed)
    sampling_order = sorted(asset_sizes_m, key=radii.__getitem__, reverse=True)
    placements: dict[str, AssetPlacement] = {}

    for name in sampling_order:
        radius = radii[name]
        x_range, y_range = _center_ranges(context, name, radius)
        for _ in range(sampling_attempts_per_object):
            x_m = rng.uniform(*x_range)
            y_m = rng.uniform(*y_range)
            if any(
                math.hypot(
                    x_m - previous.position_m[0],
                    y_m - previous.position_m[1],
                )
                < radius + radii[previous_name] + minimum_object_gap_m
                for previous_name, previous in placements.items()
            ):
                continue

            yaw = rng.uniform(-math.pi, math.pi)
            placements[name] = AssetPlacement(
                position_m=(
                    x_m,
                    y_m,
                    context.table_top_z_m
                    + asset_sizes_m[name][2] / 2.0
                    + spawn_clearance_m,
                ),
                orientation_xyzw=(
                    0.0,
                    0.0,
                    math.sin(yaw / 2.0),
                    math.cos(yaw / 2.0),
                ),
            )
            break
        else:
            raise RuntimeError(
                f"Could not place {name} within task_object_placement_area "
                f"after {sampling_attempts_per_object} attempts for seed {seed}"
            )

    layout = TaskLayout(
        task_id=task_id,
        seed=seed,
        assets={name: placements[name] for name in asset_sizes_m},
    )
    validate_tabletop_layout(
        task_id=task_id,
        context=context,
        layout=layout,
        asset_sizes_m=asset_sizes_m,
        spawn_clearance_m=spawn_clearance_m,
        minimum_object_gap_m=minimum_object_gap_m,
    )
    return layout


def validate_tabletop_layout(
    *,
    task_id: str,
    context: PlacementContext,
    layout: TaskLayout,
    asset_sizes_m: Mapping[str, tuple[float, float, float]],
    spawn_clearance_m: float,
    minimum_object_gap_m: float,
) -> None:
    """Validate identity, asset set, placement bounds, height, and spacing."""

    if layout.task_id != task_id:
        raise ValueError(
            f"layout task_id {layout.task_id!r} does not match {task_id!r}"
        )
    expected_names = set(asset_sizes_m)
    actual_names = set(layout.assets)
    if actual_names != expected_names:
        missing = sorted(expected_names - actual_names)
        unexpected = sorted(actual_names - expected_names)
        raise ValueError(
            f"layout assets do not match the task; missing={missing}, "
            f"unexpected={unexpected}"
        )

    radii = _footprint_radii(asset_sizes_m)
    for name, placement in layout.assets.items():
        x_range, y_range = _center_ranges(context, name, radii[name])
        x_m, y_m, z_m = placement.position_m
        if not x_range[0] <= x_m <= x_range[1]:
            raise ValueError(
                f"{name} is outside task_object_placement_area on the X axis"
            )
        if not y_range[0] <= y_m <= y_range[1]:
            raise ValueError(
                f"{name} is outside task_object_placement_area on the Y axis"
            )

        expected_z = context.table_top_z_m + asset_sizes_m[name][2] / 2.0
        expected_z += spawn_clearance_m
        if not math.isclose(z_m, expected_z, rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError(f"{name} is not at its expected tabletop height")
        if not (
            math.isclose(placement.orientation_xyzw[0], 0.0, abs_tol=1.0e-9)
            and math.isclose(
                placement.orientation_xyzw[1], 0.0, abs_tol=1.0e-9
            )
        ):
            raise ValueError(f"{name} must have an upright yaw-only orientation")

    names = list(layout.assets)
    for index, first_name in enumerate(names):
        first = layout.assets[first_name]
        for second_name in names[index + 1 :]:
            second = layout.assets[second_name]
            distance = math.hypot(
                first.position_m[0] - second.position_m[0],
                first.position_m[1] - second.position_m[1],
            )
            required = (
                radii[first_name]
                + radii[second_name]
                + minimum_object_gap_m
            )
            if distance + 1.0e-9 < required:
                raise ValueError(
                    f"{first_name} and {second_name} overlap or violate "
                    "minimum_object_gap_m"
                )


def _footprint_radii(
    asset_sizes_m: Mapping[str, tuple[float, float, float]],
) -> dict[str, float]:
    return {
        name: math.hypot(*size[:2]) / 2.0
        for name, size in asset_sizes_m.items()
    }


def _center_ranges(
    context: PlacementContext,
    name: str,
    radius: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    x_range = (context.x_range_m[0] + radius, context.x_range_m[1] - radius)
    y_range = (context.y_range_m[0] + radius, context.y_range_m[1] - radius)
    if x_range[0] > x_range[1] or y_range[0] > y_range[1]:
        raise ValueError(f"{name} does not fit inside task_object_placement_area")
    return x_range, y_range


__all__ = [
    "PlacementContext",
    "generate_tabletop_layout",
    "validate_tabletop_layout",
]
