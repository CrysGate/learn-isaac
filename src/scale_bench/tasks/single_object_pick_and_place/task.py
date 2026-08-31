"""Result aggregation for one-object pick-and-place."""

from __future__ import annotations

from typing import ClassVar

from scale_bench.tasks.common.fixed_target import (
    FixedTargetRigidObjectTask,
    PlacementResult,
)
from scale_bench.tasks.common.task import EvaluatorObservation

from .config import SingleObjectPickAndPlaceConfig


class SingleObjectPickAndPlace(FixedTargetRigidObjectTask):
    """Move one randomly initialized bottle to one fixed tabletop slot."""

    TASK_ID: ClassVar[str] = "single_object_pick_and_place"

    def __init__(self, config: SingleObjectPickAndPlaceConfig) -> None:
        self._object_name = config.object.name
        super().__init__(
            config,
            {config.object.name: config.object},
            target_positions_env_xy_m=(config.target_slot.position_xy_m,),
            target_placement_config=config.target_slot,
        )

    @property
    def object_name(self) -> str:
        return self._object_name

    @property
    def target_object_order(self) -> tuple[str, ...]:
        """Return the only object in its only target slot."""

        return (self.object_name,)

    def evaluate(
        self,
        observation: EvaluatorObservation,
    ) -> PlacementResult:
        """Build the final success result and geometric diagnostics."""

        statuses = self._placement_statuses(observation)
        status = statuses[0]
        return PlacementResult(
            success=status.placed,
            progress=float(status.placed),
            metrics={
                "position_error_m": status.position_error_m,
                "height_error_m": status.height_error_m,
                "upright_error_rad": status.upright_error_rad,
            },
            failure_reason=(
                None
                if status.placed
                else "bottle is outside the fixed target slot"
            ),
            statuses=statuses,
        )


__all__ = [
    "SingleObjectPickAndPlace",
]
