"""Rules and layout behavior for sorting nesting dolls by physical size."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import ClassVar

from scale_bench.tasks.common.evaluation import EvaluationResult
from scale_bench.tasks.common.layout import AssetPlacement, TaskLayout
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.common.rigid_object import RigidObjectTask
from scale_bench.tasks.common.task import EvaluatorObservation, EvaluatorTerms

from .config import DollAssetConfig, SortDollsBySizeConfig


@dataclass(frozen=True, slots=True)
class DollPlacementStatus:
    """Evaluation details for one doll and its size-ranked target slot."""

    object_name: str
    slot_index: int
    position_m: tuple[float, float, float]
    target_position_m: tuple[float, float, float]
    position_error_xyz_m: tuple[float, float, float]
    position_error_m: float
    height_error_m: float
    upright_error_rad: float
    placed: bool


@dataclass(frozen=True, slots=True)
class SortDollsProgress(EvaluationResult):
    """Task result with per-doll placement diagnostics."""

    statuses: tuple[DollPlacementStatus, ...] = ()

    @property
    def placed_count(self) -> int:
        return sum(status.placed for status in self.statuses)

    @property
    def total_count(self) -> int:
        return len(self.statuses)

    @property
    def succeeded(self) -> bool:
        return self.success


class SortDollsBySize(RigidObjectTask):
    """Place nesting dolls and expose their target size ordering."""

    TASK_ID: ClassVar[str] = "sort_dolls_by_size"

    def __init__(self, config: SortDollsBySizeConfig) -> None:
        assets = {f"doll_{doll.asset_id}": doll for doll in config.dolls}
        super().__init__(config, assets)
        self._dolls_by_name: dict[str, DollAssetConfig] = assets

        heights = [metadata.size[2] for metadata in self.metadata.values()]
        if len(heights) != len(set(heights)):
            raise ValueError("doll asset heights must be unique")

    @property
    def target_order_small_to_large(self) -> tuple[str, ...]:
        """Return asset IDs ordered by metadata height."""

        names = sorted(
            self.assets,
            key=lambda name: self.metadata[name].size[2],
        )
        return tuple(self._dolls_by_name[name].asset_id for name in names)

    @property
    def target_object_order(self) -> tuple[str, ...]:
        """Return stable scene object names from smallest to largest."""

        return tuple(
            sorted(
                self.assets,
                key=lambda name: self.metadata[name].size[2],
            )
        )

    def build_evaluator_terms(
        self,
        context: PlacementContext,
    ) -> EvaluatorTerms:
        """Build the Isaac Lab observations required by :meth:`evaluate`."""

        # Delayed imports keep task/config modules importable before Isaac Sim starts.
        from isaaclab.managers import ObservationTermCfg, SceneEntityCfg

        from scale_bench.isaaclab.mdp.observations import (
            fixed_positions,
            rigid_object_root_pos,
            rigid_object_root_quat,
        )

        object_names = self.target_object_order
        target_layout = self.target_layout(context)
        target_positions = tuple(
            target_layout.assets[name].position_m for name in object_names
        )
        asset_cfgs = tuple(SceneEntityCfg(name) for name in object_names)
        return {
            "object_positions_m": ObservationTermCfg(
                func=rigid_object_root_pos,
                params={"asset_cfgs": asset_cfgs},
            ),
            "object_orientations_xyzw": ObservationTermCfg(
                func=rigid_object_root_quat,
                params={"asset_cfgs": asset_cfgs},
            ),
            "target_positions_m": ObservationTermCfg(
                func=fixed_positions,
                params={"positions_m": target_positions},
            ),
        }

    def target_layout(self, context: PlacementContext) -> TaskLayout:
        """Build the fixed, metadata-height-aware target layout."""

        slots = self.config.target_slots
        assets = {}
        for object_name, y_m in zip(
            self.target_object_order,
            slots.y_positions_m,
            strict=True,
        ):
            object_height_m = self.metadata[object_name].size[2]
            assets[object_name] = AssetPlacement(
                position_m=(
                    slots.x_m,
                    y_m,
                    context.table_top_z_m + object_height_m / 2.0,
                ),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        return TaskLayout(
            task_id=self.task_id,
            seed=None,
            assets=assets,
        )

    def evaluate(
        self,
        observation: EvaluatorObservation,
    ) -> SortDollsProgress:
        """Evaluate one environment's final evaluator observation."""

        object_positions = _observation_matrix(
            observation,
            "object_positions_m",
        )
        target_positions = _observation_matrix(
            observation,
            "target_positions_m",
        )
        object_orientations = _observation_matrix(
            observation,
            "object_orientations_xyzw",
        )

        slots = self.config.target_slots
        statuses = []
        for slot_index, object_name in enumerate(self.target_object_order):
            position = object_positions[slot_index]
            target = target_positions[slot_index]
            error_xyz_m = tuple(
                actual - expected
                for actual, expected in zip(position, target, strict=True)
            )
            position_error_m = math.dist(position[:2], target[:2])
            height_error_m = abs(error_xyz_m[2])
            upright_error_rad = _upright_error_rad(
                object_orientations[slot_index],
                object_name,
            )
            statuses.append(
                DollPlacementStatus(
                    object_name=object_name,
                    slot_index=slot_index,
                    position_m=position,
                    target_position_m=target,
                    position_error_xyz_m=error_xyz_m,
                    position_error_m=position_error_m,
                    height_error_m=height_error_m,
                    upright_error_rad=upright_error_rad,
                    placed=(
                        position_error_m <= slots.position_tolerance_m
                        and height_error_m <= slots.height_tolerance_m
                        and upright_error_rad <= slots.upright_tolerance_rad
                    ),
                )
            )

        statuses_tuple = tuple(statuses)
        placed_count = sum(status.placed for status in statuses_tuple)
        succeeded = placed_count == len(statuses_tuple)
        return SortDollsProgress(
            success=succeeded,
            progress=placed_count / len(statuses_tuple),
            metrics={
                "placed_count": float(placed_count),
                "maximum_position_error_m": max(
                    status.position_error_m for status in statuses_tuple
                ),
                "maximum_height_error_m": max(
                    status.height_error_m for status in statuses_tuple
                ),
                "maximum_upright_error_rad": max(
                    status.upright_error_rad for status in statuses_tuple
                ),
            },
            failure_reason=(
                None if succeeded else "one or more dolls are misplaced"
            ),
            statuses=statuses_tuple,
        )


def _upright_error_rad(
    orientation_xyzw: tuple[float, ...],
    object_name: str,
) -> float:
    x, y, z, w = orientation_xyzw
    quaternion_norm = math.sqrt(x * x + y * y + z * z + w * w)
    if quaternion_norm <= 1.0e-8:
        raise ValueError(f"{object_name} orientation must be non-zero")
    x /= quaternion_norm
    y /= quaternion_norm
    up_dot = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.acos(up_dot)


def _observation_matrix(
    observation: Mapping[str, object],
    name: str,
) -> tuple[tuple[float, ...], ...]:
    """Convert one per-environment observation term to plain values."""

    value = observation[name]
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        value = tolist()
    return tuple(
        tuple(float(component) for component in row)
        for row in value  # type: ignore[union-attr]
    )


__all__ = ["DollPlacementStatus", "SortDollsBySize", "SortDollsProgress"]
