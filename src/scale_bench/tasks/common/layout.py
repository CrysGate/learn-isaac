"""Serializable task layout value objects."""

from __future__ import annotations

from pathlib import Path
from typing import Self

from pydantic import Field, ValidationError, model_validator

from scale_bench.config.base import (
    FrozenModel,
    Name,
    NonNegativeInt,
    Position3,
    Quaternion,
    require_unit_quaternion,
)


class AssetPlacement(FrozenModel):
    """Environment-local initial pose of one task asset."""

    position_m: Position3
    orientation_xyzw: Quaternion

    @model_validator(mode="after")
    def _validate_orientation(self) -> Self:
        require_unit_quaternion(self.orientation_xyzw, "orientation_xyzw")
        return self


class TaskLayout(FrozenModel):
    """Serializable initial asset layout for one task and seed."""

    task_id: Name
    seed: NonNegativeInt | None
    assets: dict[Name, AssetPlacement] = Field(min_length=1)

    @classmethod
    def load(cls, layout_path: str | Path) -> Self:
        """Load and validate a JSON layout file."""

        path = Path(layout_path)
        try:
            return cls.model_validate_json(path.read_text(encoding="utf-8"))
        except (OSError, ValidationError) as error:
            raise ValueError(f"Could not load task layout {path}:\n{error}") from error

    def save(self, layout_path: str | Path) -> Path:
        """Write the layout as stable, human-readable JSON."""

        path = Path(layout_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(self.model_dump_json(indent=2) + "\n", encoding="utf-8")
        except OSError as error:
            raise ValueError(
                f"Could not export task layout {path}:\n{error}"
            ) from error
        return path


__all__ = ["AssetPlacement", "TaskLayout"]
