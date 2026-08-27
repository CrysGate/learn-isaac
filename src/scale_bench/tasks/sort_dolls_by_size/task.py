"""Rules and layout behavior for sorting nesting dolls by physical size."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from torch import Tensor, stack, where
from torch.linalg import vector_norm

from scale_bench.tasks.common.evaluation import EvaluationResult
from scale_bench.tasks.common.layout import AssetPlacement, TaskLayout
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.common.rigid_object import RigidObjectTask
from scale_bench.tasks.common.task import (
    BatchedEvaluatorObservation,
    EvaluatorObservation,
    EvaluatorTerms,
)

from .config import SortDollsBySizeConfig


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
class _BatchedPlacementMeasurements:
    """Shared tensor measurements with environment and doll dimensions."""

    object_positions_m: Tensor
    target_positions_m: Tensor
    position_error_xyz_m: Tensor
    position_error_m: Tensor
    height_error_m: Tensor
    upright_error_rad: Tensor
    placed: Tensor


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
        for object_name, x_m, y_m in zip(
            self.target_object_order,
            slots.x_positions_m,
            slots.y_positions_m,
            strict=True,
        ):
            object_height_m = self.metadata[object_name].size[2]
            assets[object_name] = AssetPlacement(
                position_m=(
                    x_m,
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

    def check_success(
        self,
        observation: BatchedEvaluatorObservation,
    ) -> Tensor:
        """Check all environments and dolls using batched tensor operations."""

        return self._measure_placements(observation).placed.all(dim=1)

    def evaluate(
        self,
        observation: EvaluatorObservation,
    ) -> SortDollsProgress:
        """Evaluate one environment's final evaluator observation."""

        batched_observation = {
            "object_positions_m": _unbatched_observation_tensor(
                observation,
                "object_positions_m",
                components=3,
            ).unsqueeze(0),
            "target_positions_m": _unbatched_observation_tensor(
                observation,
                "target_positions_m",
                components=3,
            ).unsqueeze(0),
            "object_orientations_xyzw": _unbatched_observation_tensor(
                observation,
                "object_orientations_xyzw",
                components=4,
            ).unsqueeze(0),
        }
        measurements = self._measure_placements(batched_observation)
        positions = measurements.object_positions_m[0].detach().cpu().tolist()
        targets = measurements.target_positions_m[0].detach().cpu().tolist()
        error_xyz = measurements.position_error_xyz_m[0].detach().cpu().tolist()
        errors = stack(
            (
                measurements.position_error_m[0],
                measurements.height_error_m[0],
                measurements.upright_error_rad[0],
            ),
            dim=-1,
        ).detach().cpu().tolist()
        placed = measurements.placed[0].detach().cpu().tolist()

        statuses = []
        for slot_index, object_name in enumerate(self.target_object_order):
            statuses.append(
                DollPlacementStatus(
                    object_name=object_name,
                    slot_index=slot_index,
                    position_m=tuple(positions[slot_index]),
                    target_position_m=tuple(targets[slot_index]),
                    position_error_xyz_m=tuple(error_xyz[slot_index]),
                    position_error_m=errors[slot_index][0],
                    height_error_m=errors[slot_index][1],
                    upright_error_rad=errors[slot_index][2],
                    placed=placed[slot_index],
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

    def _measure_placements(
        self,
        observation: BatchedEvaluatorObservation,
    ) -> _BatchedPlacementMeasurements:
        object_positions = _batched_observation_tensor(
            observation,
            "object_positions_m",
            components=3,
        )
        target_positions = _batched_observation_tensor(
            observation,
            "target_positions_m",
            components=3,
        )
        object_orientations = _batched_observation_tensor(
            observation,
            "object_orientations_xyzw",
            components=4,
        )

        position_error_xyz_m = object_positions - target_positions
        position_error_m = vector_norm(
            position_error_xyz_m[..., :2],
            dim=-1,
        )
        height_error_m = position_error_xyz_m[..., 2].abs()
        quaternion_norm = vector_norm(
            object_orientations,
            dim=-1,
        )
        safe_norm = quaternion_norm.clamp_min(1.0e-8)
        normalized_xy = object_orientations[..., :2] / safe_norm.unsqueeze(-1)
        up_dot = (1.0 - 2.0 * normalized_xy.square().sum(dim=-1)).clamp(-1.0, 1.0)
        upright_error_rad = up_dot.acos()
        upright_error_rad = where(
            quaternion_norm > 1.0e-8,
            upright_error_rad,
            upright_error_rad.new_full(
                upright_error_rad.shape,
                float("inf"),
            ),
        )

        slots = self.config.target_slots
        placed = (
            (position_error_m <= slots.position_tolerance_m)
            & (height_error_m <= slots.height_tolerance_m)
            & (upright_error_rad <= slots.upright_tolerance_rad)
        )
        return _BatchedPlacementMeasurements(
            object_positions_m=object_positions,
            target_positions_m=target_positions,
            position_error_xyz_m=position_error_xyz_m,
            position_error_m=position_error_m,
            height_error_m=height_error_m,
            upright_error_rad=upright_error_rad,
            placed=placed,
        )


def _batched_observation_tensor(
    observation: BatchedEvaluatorObservation,
    name: str,
    *,
    components: int,
) -> Tensor:
    value = observation[name]
    if value.ndim != 3 or value.shape[-1] != components:
        raise ValueError(f"{name} must have shape (num_envs, num_dolls, {components})")
    if not value.is_floating_point():
        raise TypeError(f"{name} must use a floating-point dtype")
    return value


def _unbatched_observation_tensor(
    observation: EvaluatorObservation,
    name: str,
    *,
    components: int,
) -> Tensor:
    value = observation[name]
    if value.ndim != 2 or value.shape[-1] != components:
        raise ValueError(f"{name} must have shape (num_dolls, {components})")
    return value


__all__ = ["DollPlacementStatus", "SortDollsBySize", "SortDollsProgress"]
