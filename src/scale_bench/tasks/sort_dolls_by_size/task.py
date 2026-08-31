"""Rules and result aggregation for sorting dolls by physical size."""

from __future__ import annotations

from typing import ClassVar

from scale_bench.tasks.common.fixed_target import (
    FixedTargetRigidObjectTask,
    PlacementResult,
)
from scale_bench.tasks.common.task import EvaluatorObservation

from .config import SortDollsBySizeConfig


class SortDollsBySize(FixedTargetRigidObjectTask):
    """Order dolls by size and summarize their fixed-slot placements."""

    TASK_ID: ClassVar[str] = "sort_dolls_by_size"

    def __init__(self, config: SortDollsBySizeConfig) -> None:
        assets = {f"doll_{doll.asset_id}": doll for doll in config.dolls}
        target_positions_env_xy_m = tuple(
            zip(
                config.target_slots.x_positions_m,
                config.target_slots.y_positions_m,
                strict=True,
            )
        )
        super().__init__(
            config,
            assets,
            target_positions_env_xy_m=target_positions_env_xy_m,
            target_placement_config=config.target_slots,
        )

    @property
    def target_object_order(self) -> tuple[str, ...]:
        """Return stable scene object names from smallest to largest."""

        return tuple(
            sorted(
                self.assets,
                key=lambda name: self.metadata[name].size[2],
            )
        )

    def evaluate(
        self,
        observation: EvaluatorObservation,
    ) -> PlacementResult:
        """Evaluate one environment's final evaluator observation."""

        statuses = self._placement_statuses(observation)
        placed_count = sum(status.placed for status in statuses)
        succeeded = placed_count == len(statuses)
        return PlacementResult(
            success=succeeded,
            progress=placed_count / len(statuses),
            metrics={
                "placed_count": float(placed_count),
                "maximum_position_error_m": max(
                    status.position_error_m for status in statuses
                ),
                "maximum_height_error_m": max(
                    status.height_error_m for status in statuses
                ),
                "maximum_upright_error_rad": max(
                    status.upright_error_rad for status in statuses
                ),
            },
            failure_reason=(None if succeeded else "one or more dolls are misplaced"),
            statuses=statuses,
        )


__all__ = ["SortDollsBySize"]
