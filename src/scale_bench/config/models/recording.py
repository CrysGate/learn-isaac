"""Episode trajectory recording configuration."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal
from pydantic import Field, StrictBool, model_validator

from scale_bench.config.base import FrozenModel


DatasetName = Annotated[str, Field(min_length=1, pattern=r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")]


class RecordingConfig(FrozenModel):
    """Opt-in settings for native Isaac Lab episode recording."""

    output_dir: Path
    dataset_name: DatasetName
    export_mode: Literal[
        "all",
        "succeeded_only",
        "succeeded_failed_separate",
    ] = "all"
    compression: StrictBool = True
    overwrite_existing: StrictBool = False
    record_initial_state: StrictBool = True
    record_actions: StrictBool = True
    record_processed_actions: StrictBool = True
    record_joint_observations: StrictBool = True
    record_camera_observations: StrictBool = False
    record_scene_state: StrictBool = False

    @model_validator(mode="after")
    def validate_active_terms(self) -> "RecordingConfig":
        if self.dataset_name.endswith(".hdf5"):
            raise ValueError("dataset_name must not include the .hdf5 extension")
        term_flags = (
            self.record_initial_state,
            self.record_actions,
            self.record_processed_actions,
            self.record_joint_observations,
            self.record_camera_observations,
            self.record_scene_state,
        )
        if not any(term_flags):
            raise ValueError("recording must enable at least one recorder term")
        return self


__all__ = ["RecordingConfig"]
