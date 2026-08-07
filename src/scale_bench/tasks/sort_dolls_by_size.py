"""Concrete task for sorting nesting dolls by physical size."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, ClassVar, Self

from pydantic import Field, model_validator

from scale_bench import REPOSITORY_ROOT
from scale_bench.scenes import SceneConfig

from .base import (
    RigidObjectAssetConfig,
    RigidObjectTaskConfig,
    TaskDefinition,
)

DEFAULT_SORT_DOLLS_CONFIG_PATH = (REPOSITORY_ROOT / "configs/tasks/sort_dolls_by_size.yml")
AssetId = Annotated[str, Field(pattern=r"^\d{5}$")]


class DollAssetConfig(RigidObjectAssetConfig):
    """One nesting-doll asset."""

    asset_id: AssetId


class SortDollsBySizeConfig(RigidObjectTaskConfig):
    """Typed task configuration and nesting-doll constraints."""

    DEFAULT_CONFIG_PATH: ClassVar[Path] = DEFAULT_SORT_DOLLS_CONFIG_PATH

    dolls: tuple[DollAssetConfig, ...] = Field(min_length=2)

    @model_validator(mode="after")
    def _validate_dolls(self) -> Self:
        ids = tuple(doll.asset_id for doll in self.dolls)
        if len(ids) != len(set(ids)):
            raise ValueError("doll asset_id values must be unique")
        return self


class SortDollsBySize(TaskDefinition):
    """Load and place nesting dolls without depending on a robot type."""

    TASK_ID: ClassVar[str] = "sort_dolls_by_size"

    def __init__(
        self,
        config_path: str | Path = DEFAULT_SORT_DOLLS_CONFIG_PATH,
        *,
        scene_config: SceneConfig | None = None,
    ) -> None:
        config = SortDollsBySizeConfig.load(config_path)
        assets = {f"doll_{doll.asset_id}": doll for doll in config.dolls}
        super().__init__(config, assets, scene_config=scene_config)

        heights = [metadata.size[2] for metadata in self._metadata.values()]
        if len(heights) != len(set(heights)):
            raise ValueError("doll asset heights must be unique")

    @property
    def target_order_small_to_large(self) -> tuple[str, ...]:
        """Return asset IDs ordered by metadata height."""

        names = sorted(
            self._assets,
            key=lambda name: self._metadata[name].size[2],
        )
        return tuple(self._assets[name].asset_id for name in names)


__all__ = ["SortDollsBySize", "SortDollsBySizeConfig"]
