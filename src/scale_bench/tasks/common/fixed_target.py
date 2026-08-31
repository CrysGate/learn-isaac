"""Reusable behavior for rigid objects placed into fixed tabletop slots."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Iterator, Mapping
from dataclasses import dataclass

from torch import Tensor, where
from torch.linalg import vector_norm

from scale_bench.skills.models import PickAndPlace, Pose, SkillRequest

from .evaluation import EvaluationResult
from .layout import AssetPlacement, TaskLayout
from .placement import PlacementContext
from .rigid_object import (
    RigidObjectAssetConfig,
    RigidObjectTask,
    RigidObjectTaskConfig,
    TargetPlacementConfig,
)
from .task import (
    BatchedEvaluatorObservation,
    EvaluatorObservation,
    EvaluatorTerms,
)


@dataclass(frozen=True, slots=True)
class PlacementStatus:
    """Final object pose and geometric errors for one target slot."""

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
class PlacementResult(EvaluationResult):
    """Task result with per-object fixed-target placement diagnostics."""

    statuses: tuple[PlacementStatus, ...] = ()


@dataclass(frozen=True, slots=True)
class _BatchedPlacementMeasurements:
    """Placement measurements with environment and object dimensions."""

    object_positions_env_m: Tensor
    target_positions_env_m: Tensor
    position_errors_env_xyz_m: Tensor
    planar_position_errors_env_m: Tensor
    height_errors_env_m: Tensor
    upright_errors_env_rad: Tensor
    placed: Tensor


class FixedTargetRigidObjectTask(RigidObjectTask, ABC):
    """Common target layout, expert, and evaluation for fixed-slot tasks."""

    def __init__(
        self,
        config: RigidObjectTaskConfig,
        assets: Mapping[str, RigidObjectAssetConfig],
        *,
        target_positions_env_xy_m: tuple[tuple[float, float], ...],
        target_placement_config: TargetPlacementConfig,
    ) -> None:
        self._target_positions_env_xy_m = target_positions_env_xy_m
        self._target_placement_config = target_placement_config
        super().__init__(config, assets)

    @property
    @abstractmethod
    def target_object_order(self) -> tuple[str, ...]:
        """Return object names in fixed target-slot order."""

    def build_evaluator_terms(
        self,
        context: PlacementContext,
    ) -> EvaluatorTerms:
        """Observe all object poses and their fixed target positions."""

        from isaaclab.managers import ObservationTermCfg, SceneEntityCfg

        from scale_bench.isaaclab.mdp.observations import (
            fixed_positions,
            rigid_object_root_pos,
            rigid_object_root_quat,
        )

        object_names = self.target_object_order
        target_layout = self.target_layout(context)
        target_positions_env_m = tuple(
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
                params={"positions_m": target_positions_env_m},
            ),
        }

    def target_layout(self, context: PlacementContext) -> TaskLayout:
        """Build metadata-height-aware poses for all fixed target slots."""

        assets = {}
        for object_name, target_position_env_xy_m in zip(
            self.target_object_order,
            self._target_positions_env_xy_m,
            strict=True,
        ):
            object_height_m = self.metadata[object_name].size[2]
            assets[object_name] = AssetPlacement(
                position_m=(
                    target_position_env_xy_m[0],
                    target_position_env_xy_m[1],
                    context.table_top_z_m + object_height_m / 2.0,
                ),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        return TaskLayout(task_id=self.task_id, seed=None, assets=assets)

    def expert(
        self,
        *,
        source_layout: TaskLayout,
        target_layout: TaskLayout,
    ) -> Iterator[SkillRequest]:
        """Move every object to its fixed slot while preserving its orientation."""

        self.validate_asset_layout(source_layout)
        self.validate_asset_layout(target_layout)
        for object_name in self.target_object_order:
            source_object_placement = source_layout.assets[object_name]
            target_object_placement = target_layout.assets[object_name]
            target_object_pose_env = Pose(
                position_m=target_object_placement.position_m,
                orientation_xyzw=source_object_placement.orientation_xyzw,
            )
            yield PickAndPlace(
                object_name=object_name,
                arm="auto",
                target_object_pose_env=target_object_pose_env,
            )

    def check_success(
        self,
        observation: BatchedEvaluatorObservation,
    ) -> Tensor:
        """Require every object to satisfy its fixed-slot tolerances."""

        return self._measure_placements(observation).placed.all(dim=1)

    def _placement_statuses(
        self,
        observation: EvaluatorObservation,
    ) -> tuple[PlacementStatus, ...]:
        batched_observation = {
            "object_positions_m": _unbatched_observation_tensor(
                observation,
                "object_positions_m",
                object_count=len(self.target_object_order),
                components=3,
            ).unsqueeze(0),
            "target_positions_m": _unbatched_observation_tensor(
                observation,
                "target_positions_m",
                object_count=len(self.target_object_order),
                components=3,
            ).unsqueeze(0),
            "object_orientations_xyzw": _unbatched_observation_tensor(
                observation,
                "object_orientations_xyzw",
                object_count=len(self.target_object_order),
                components=4,
            ).unsqueeze(0),
        }
        measurements = self._measure_placements(batched_observation)
        object_positions_env_m = (
            measurements.object_positions_env_m[0].detach().cpu().tolist()
        )
        target_positions_env_m = (
            measurements.target_positions_env_m[0].detach().cpu().tolist()
        )
        position_errors_env_xyz_m = (
            measurements.position_errors_env_xyz_m[0].detach().cpu().tolist()
        )
        planar_position_errors_env_m = (
            measurements.planar_position_errors_env_m[0].detach().cpu().tolist()
        )
        height_errors_env_m = (
            measurements.height_errors_env_m[0].detach().cpu().tolist()
        )
        upright_errors_env_rad = (
            measurements.upright_errors_env_rad[0].detach().cpu().tolist()
        )
        placed = measurements.placed[0].detach().cpu().tolist()

        return tuple(
            PlacementStatus(
                object_name=object_name,
                slot_index=slot_index,
                position_m=tuple(object_positions_env_m[slot_index]),
                target_position_m=tuple(target_positions_env_m[slot_index]),
                position_error_xyz_m=tuple(
                    position_errors_env_xyz_m[slot_index]
                ),
                position_error_m=planar_position_errors_env_m[slot_index],
                height_error_m=height_errors_env_m[slot_index],
                upright_error_rad=upright_errors_env_rad[slot_index],
                placed=placed[slot_index],
            )
            for slot_index, object_name in enumerate(self.target_object_order)
        )

    def _measure_placements(
        self,
        observation: BatchedEvaluatorObservation,
    ) -> _BatchedPlacementMeasurements:
        object_count = len(self.target_object_order)
        object_positions_env_m = _batched_observation_tensor(
            observation,
            "object_positions_m",
            object_count=object_count,
            components=3,
        )
        target_positions_env_m = _batched_observation_tensor(
            observation,
            "target_positions_m",
            object_count=object_count,
            components=3,
        )
        object_orientations_env_xyzw = _batched_observation_tensor(
            observation,
            "object_orientations_xyzw",
            object_count=object_count,
            components=4,
        )

        position_errors_env_xyz_m = (
            object_positions_env_m - target_positions_env_m
        )
        planar_position_errors_env_m = vector_norm(
            position_errors_env_xyz_m[..., :2],
            dim=-1,
        )
        height_errors_env_m = position_errors_env_xyz_m[..., 2].abs()
        object_orientation_norm = vector_norm(
            object_orientations_env_xyzw,
            dim=-1,
        )
        safe_object_orientation_norm = object_orientation_norm.clamp_min(1.0e-8)
        normalized_orientation_env_xy = (
            object_orientations_env_xyzw[..., :2]
            / safe_object_orientation_norm.unsqueeze(-1)
        )
        object_up_dot_env = (
            1.0 - 2.0 * normalized_orientation_env_xy.square().sum(dim=-1)
        ).clamp(-1.0, 1.0)
        upright_errors_env_rad = object_up_dot_env.acos()
        upright_errors_env_rad = where(
            object_orientation_norm > 1.0e-8,
            upright_errors_env_rad,
            upright_errors_env_rad.new_full(
                upright_errors_env_rad.shape,
                float("inf"),
            ),
        )

        target_config = self._target_placement_config
        placed = (
            (
                planar_position_errors_env_m
                <= target_config.position_tolerance_m
            )
            & (height_errors_env_m <= target_config.height_tolerance_m)
            & (upright_errors_env_rad <= target_config.upright_tolerance_rad)
        )
        return _BatchedPlacementMeasurements(
            object_positions_env_m=object_positions_env_m,
            target_positions_env_m=target_positions_env_m,
            position_errors_env_xyz_m=position_errors_env_xyz_m,
            planar_position_errors_env_m=planar_position_errors_env_m,
            height_errors_env_m=height_errors_env_m,
            upright_errors_env_rad=upright_errors_env_rad,
            placed=placed,
        )


def _batched_observation_tensor(
    observation: BatchedEvaluatorObservation,
    name: str,
    *,
    object_count: int,
    components: int,
) -> Tensor:
    value = observation[name]
    if value.ndim != 3 or value.shape[1:] != (object_count, components):
        raise ValueError(
            f"{name} must have shape (num_envs, {object_count}, {components})"
        )
    if not value.is_floating_point():
        raise TypeError(f"{name} must use a floating-point dtype")
    return value


def _unbatched_observation_tensor(
    observation: EvaluatorObservation,
    name: str,
    *,
    object_count: int,
    components: int,
) -> Tensor:
    value = observation[name]
    expected_shape = (object_count, components)
    if value.ndim != 2 or value.shape != expected_shape:
        raise ValueError(f"{name} must have shape {expected_shape}")
    if not value.is_floating_point():
        raise TypeError(f"{name} must use a floating-point dtype")
    return value


__all__ = [
    "FixedTargetRigidObjectTask",
    "PlacementResult",
    "PlacementStatus",
    "TargetPlacementConfig",
]
