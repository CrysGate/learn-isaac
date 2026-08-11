"""Pure data and reusable behavior for metadata-backed rigid-object tasks."""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType
from typing import ClassVar

from pydantic import BaseModel, ConfigDict, ValidationError

from scale_bench.config.base import (
    AssetReference,
    FrozenModel,
    Name,
    NonNegativeFloat,
    PositiveFloat,
    PositiveInt,
    UnitIntervalFloat,
)

from .layout import TaskLayout
from .placement import (
    PlacementContext,
    generate_tabletop_layout,
    validate_tabletop_layout,
)


class RigidObjectAssetConfig(FrozenModel):
    """Paths for one metadata-backed rigid task asset."""

    usd_path: AssetReference
    metadata_path: AssetReference


class RigidObjectPhysicsConfig(FrozenModel):
    """Contact and damping properties shared by task objects."""

    restitution: UnitIntervalFloat
    linear_damping: NonNegativeFloat = 0.1
    angular_damping: NonNegativeFloat = 0.1
    sleep_threshold: NonNegativeFloat = 0.005
    stabilization_threshold: NonNegativeFloat = 0.001


class RigidObjectTaskConfig(FrozenModel):
    """Configuration shared by metadata-backed tabletop tasks."""

    instruction: Name
    spawn_clearance_m: NonNegativeFloat = 0.003
    minimum_object_gap_m: NonNegativeFloat = 0.02
    sampling_attempts_per_object: PositiveInt = 1000
    physics: RigidObjectPhysicsConfig


class _MetadataModel(BaseModel):
    model_config = ConfigDict(extra="ignore", frozen=True)


class RigidObjectMetadata(_MetadataModel):
    """Physical properties read from an asset metadata file."""

    size: tuple[PositiveFloat, PositiveFloat, PositiveFloat]
    mass: PositiveFloat
    friction: NonNegativeFloat


def load_rigid_object_metadata(
    asset: RigidObjectAssetConfig,
) -> RigidObjectMetadata:
    """Load the size, mass, and friction used by layout and spawning."""

    path = Path(asset.metadata_path)
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        physics = document.get("physics") if isinstance(document, dict) else None
        return RigidObjectMetadata.model_validate(physics)
    except (OSError, json.JSONDecodeError, ValidationError) as error:
        raise ValueError(f"Could not load asset metadata {path}:\n{error}") from error


class RigidObjectTask:
    """Reusable deterministic layout behavior for named rigid assets."""

    TASK_ID: ClassVar[str]

    def __init__(
        self,
        config: RigidObjectTaskConfig,
        assets: Mapping[str, RigidObjectAssetConfig],
    ) -> None:
        if not assets:
            raise ValueError("a rigid-object task requires at least one asset")
        self._config = config
        self._assets = dict(assets)
        self._metadata = {
            name: load_rigid_object_metadata(asset)
            for name, asset in self._assets.items()
        }

    @property
    def task_id(self) -> str:
        return self.TASK_ID

    @property
    def instruction(self) -> str:
        return self._config.instruction

    @property
    def config(self) -> RigidObjectTaskConfig:
        """Validated task settings used by layout and adapter builders."""

        return self._config

    @property
    def assets(self) -> Mapping[str, RigidObjectAssetConfig]:
        """Immutable named asset declarations."""

        return MappingProxyType(self._assets)

    @property
    def metadata(self) -> Mapping[str, RigidObjectMetadata]:
        """Immutable physical metadata used by layout and adapter builders."""

        return MappingProxyType(self._metadata)

    def resolve_layout(
        self,
        context: PlacementContext,
        *,
        seed: int | None = None,
        layout_path: str | Path | None = None,
    ) -> TaskLayout:
        """Generate or load and validate one task layout."""

        if seed is not None and layout_path is not None:
            raise ValueError("seed and layout_path are mutually exclusive")
        layout = (
            TaskLayout.load(layout_path)
            if layout_path is not None
            else self.generate_layout(context, 0 if seed is None else seed)
        )
        self.validate_layout(context, layout)
        return layout

    def generate_layout(
        self,
        context: PlacementContext,
        seed: int,
    ) -> TaskLayout:
        return generate_tabletop_layout(
            task_id=self.task_id,
            context=context,
            asset_sizes_m=self._asset_sizes_m(),
            seed=seed,
            spawn_clearance_m=self._config.spawn_clearance_m,
            minimum_object_gap_m=self._config.minimum_object_gap_m,
            sampling_attempts_per_object=self._config.sampling_attempts_per_object,
        )

    def validate_layout(
        self,
        context: PlacementContext,
        layout: TaskLayout,
    ) -> None:
        validate_tabletop_layout(
            task_id=self.task_id,
            context=context,
            layout=layout,
            asset_sizes_m=self._asset_sizes_m(),
            spawn_clearance_m=self._config.spawn_clearance_m,
            minimum_object_gap_m=self._config.minimum_object_gap_m,
        )

    def validate_asset_layout(self, layout: TaskLayout) -> None:
        """Validate the context-free contract needed by native asset builders."""

        if layout.task_id != self.task_id:
            raise ValueError(
                f"layout task_id {layout.task_id!r} does not match {self.task_id!r}"
            )
        expected_names = set(self._assets)
        actual_names = set(layout.assets)
        if actual_names != expected_names:
            missing = sorted(expected_names - actual_names)
            unexpected = sorted(actual_names - expected_names)
            raise ValueError(
                f"layout assets do not match the task; missing={missing}, "
                f"unexpected={unexpected}"
            )

    def _asset_sizes_m(self) -> dict[str, tuple[float, float, float]]:
        return {
            name: metadata.size
            for name, metadata in self._metadata.items()
        }


__all__ = [
    "RigidObjectAssetConfig",
    "RigidObjectMetadata",
    "RigidObjectPhysicsConfig",
    "RigidObjectTask",
    "RigidObjectTaskConfig",
    "load_rigid_object_metadata",
]
