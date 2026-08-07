"""Common task interface, rigid assets, and reproducible layout support."""

from __future__ import annotations

import json
import math
import random
from collections.abc import Mapping
from pathlib import Path
from typing import Annotated, ClassVar, Literal, Self

import isaaclab.sim as sim_utils
import yaml
from isaaclab.assets import RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictInt,
    ValidationError,
    field_validator,
)

from scale_bench import REPOSITORY_ROOT
from scale_bench.scenes import SceneConfig


FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
UnitIntervalFloat = Annotated[float, Field(ge=0.0, le=1.0, allow_inf_nan=False)]
PositiveInt = Annotated[StrictInt, Field(gt=0)]
NonNegativeInt = Annotated[StrictInt, Field(ge=0)]
Name = Annotated[str, Field(min_length=1)]


class StrictModel(BaseModel):
    """Immutable Pydantic model that rejects unknown keys."""

    model_config = ConfigDict(extra="forbid", frozen=True)


class AssetPlacement(StrictModel):
    """Environment-local initial pose of one task asset."""

    position_m: tuple[FiniteFloat, FiniteFloat, FiniteFloat]
    orientation_xyzw: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]

    @field_validator("orientation_xyzw")
    @classmethod
    def _validate_orientation(
        cls,
        value: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        norm_squared = sum(component * component for component in value)
        if not 0.999998 <= norm_squared <= 1.000002:
            raise ValueError("orientation_xyzw must be a unit quaternion")
        return value


class TaskLayout(StrictModel):
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
            raise ValueError(f"Could not export task layout {path}:\n{error}") from error
        return path


class RigidObjectAssetConfig(StrictModel):
    """Paths for one metadata-backed rigid task asset."""

    usd_path: Name
    metadata_path: Name


class RigidObjectPhysicsConfig(StrictModel):
    """Contact and damping properties shared by task objects."""

    restitution: UnitIntervalFloat
    linear_damping: NonNegativeFloat = 0.1
    angular_damping: NonNegativeFloat = 0.1
    sleep_threshold: NonNegativeFloat = 0.005
    stabilization_threshold: NonNegativeFloat = 0.001


class RigidObjectTaskConfig(StrictModel):
    """Configuration shared by metadata-backed tabletop tasks."""

    DEFAULT_CONFIG_PATH: ClassVar[Path | None] = None

    instruction: Name
    spawn_clearance_m: NonNegativeFloat = 0.003
    minimum_object_gap_m: NonNegativeFloat = 0.02
    sampling_attempts_per_object: PositiveInt = 1000
    physics: RigidObjectPhysicsConfig

    @classmethod
    def load(cls, config_path: str | Path | None = None) -> Self:
        path = config_path if config_path is not None else cls.DEFAULT_CONFIG_PATH
        if path is None:
            raise ValueError(f"{cls.__name__} does not define DEFAULT_CONFIG_PATH")
        path = repository_path(path)
        try:
            return cls.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))
        except (OSError, yaml.YAMLError, ValidationError) as error:
            raise ValueError(f"Could not load task config {path}:\n{error}") from error


class _MetadataModel(BaseModel):
    model_config = ConfigDict(extra="ignore", frozen=True)


class RigidObjectMetadata(_MetadataModel):
    """Physical properties read from an asset metadata file."""

    size: tuple[PositiveFloat, PositiveFloat, PositiveFloat]
    mass: PositiveFloat
    friction: NonNegativeFloat


def repository_path(path: str | Path) -> Path:
    """Resolve a repository-relative path."""

    path = Path(path)
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def resolve_asset_path(path: str) -> str:
    """Resolve a local asset path while preserving URI paths."""

    return path if "://" in path else str(repository_path(path))


def load_rigid_object_metadata(asset: RigidObjectAssetConfig) -> RigidObjectMetadata:
    """Load the size, mass, and friction used by layout and spawning."""

    path = Path(resolve_asset_path(asset.metadata_path))
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        physics = document.get("physics") if isinstance(document, dict) else None
        return RigidObjectMetadata.model_validate(physics)
    except (OSError, json.JSONDecodeError, ValidationError) as error:
        raise ValueError(f"Could not load asset metadata {path}:\n{error}") from error


class TaskDefinition:
    """Common public task interface and tabletop rigid-asset implementation.

    Concrete tasks only supply their typed configuration and named assets.
    This class handles deterministic layouts, JSON import/export, validation,
    metadata, and direct scene configuration.
    """

    TASK_ID: ClassVar[str]

    def __init__(
        self,
        config: RigidObjectTaskConfig,
        assets: Mapping[str, RigidObjectAssetConfig],
        *,
        scene_config: SceneConfig | None = None,
    ) -> None:
        self._config = config
        self._assets = dict(assets)
        self._scene_config = scene_config or SceneConfig.load()
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

    def add_assets_to_scene(
        self,
        scene_cfg: InteractiveSceneCfg,
        *,
        seed: int | None = None,
        layout_path: str | Path | None = None,
        export_layout_path: str | Path | None = None,
    ) -> TaskLayout:
        """Install task assets directly from a seed or a JSON layout file."""

        if seed is not None and layout_path is not None:
            raise ValueError("seed and layout_path are mutually exclusive")

        layout = (
            TaskLayout.load(layout_path)
            if layout_path is not None
            else self.generate_layout(0 if seed is None else seed)
        )
        self.validate_layout(layout)

        conflicts = [name for name in self._assets if hasattr(scene_cfg, name)]
        if conflicts:
            raise ValueError("scene_cfg already contains task asset fields: " + ", ".join(conflicts))

        asset_cfgs = {
            name: self._build_asset_cfg(name, layout.assets[name])
            for name in self._assets
        }
        for name, asset_cfg in asset_cfgs.items():
            setattr(scene_cfg, name, asset_cfg)

        if export_layout_path is not None:
            layout.save(export_layout_path)
        return layout

    def generate_layout(self, seed: int) -> TaskLayout:
        """Deterministically sample a non-overlapping upright layout."""
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError("seed must be a non-negative integer")

        rng = random.Random(seed)
        radii = self._footprint_radii()
        sampling_order = sorted(self._assets, key=radii.__getitem__, reverse=True)
        placements: dict[str, AssetPlacement] = {}

        for name in sampling_order:
            radius = radii[name]
            x_range, y_range = self._center_ranges(name, radius)
            for _ in range(self._config.sampling_attempts_per_object):
                x_m = rng.uniform(*x_range)
                y_m = rng.uniform(*y_range)
                if any(
                    math.hypot(
                        x_m - previous.position_m[0],
                        y_m - previous.position_m[1],
                    )
                    < radius + radii[previous_name] + self._config.minimum_object_gap_m
                    for previous_name, previous in placements.items()
                ):
                    continue

                yaw = rng.uniform(-math.pi, math.pi)
                height = self._metadata[name].size[2]
                placements[name] = AssetPlacement(
                    position_m=(
                        x_m,
                        y_m,
                        self._scene_config.table_top_z_m
                        + height / 2.0
                        + self._config.spawn_clearance_m,
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
                    f"after {self._config.sampling_attempts_per_object} attempts "
                    f"for seed {seed}"
                )

        layout = TaskLayout(
            task_id=self.task_id,
            seed=seed,
            assets={name: placements[name] for name in self._assets},
        )
        self.validate_layout(layout)
        return layout

    def validate_layout(self, layout: TaskLayout) -> None:
        """Validate identity, asset set, placement area, height, and spacing."""

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

        radii = self._footprint_radii()
        for name, placement in layout.assets.items():
            metadata = self._metadata[name]
            x_range, y_range = self._center_ranges(name, radii[name])
            x_m, y_m, z_m = placement.position_m
            if not (x_range[0] <= x_m <= x_range[1]):
                raise ValueError(
                    f"{name} is outside task_object_placement_area on the X axis"
                )
            if not (y_range[0] <= y_m <= y_range[1]):
                raise ValueError(
                    f"{name} is outside task_object_placement_area on the Y axis"
                )

            expected_z = (
                self._scene_config.table_top_z_m
                + metadata.size[2] / 2.0
                + self._config.spawn_clearance_m
            )
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
                    + self._config.minimum_object_gap_m
                )
                if distance + 1.0e-9 < required:
                    raise ValueError(
                        f"{first_name} and {second_name} overlap or violate "
                        "minimum_object_gap_m"
                    )

    def _footprint_radii(self) -> dict[str, float]:
        return {
            name: math.hypot(*metadata.size[:2]) / 2.0
            for name, metadata in self._metadata.items()
        }

    def _center_ranges(
        self,
        name: str,
        radius: float,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        area = self._scene_config.task_object_placement_area
        x_range = (area.x_range_m[0] + radius, area.x_range_m[1] - radius)
        y_range = (area.y_range_m[0] + radius, area.y_range_m[1] - radius)
        if x_range[0] > x_range[1] or y_range[0] > y_range[1]:
            raise ValueError(f"{name} does not fit inside task_object_placement_area")
        return x_range, y_range

    def _build_asset_cfg(
        self,
        name: str,
        placement: AssetPlacement,
    ) -> RigidObjectCfg:
        asset = self._assets[name]
        metadata = self._metadata[name]
        physics = self._config.physics
        return RigidObjectCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Task/Objects/{name}",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=placement.position_m,
                rot=placement.orientation_xyzw,
            ),
            spawn=sim_utils.UsdFileCfg(
                usd_path=resolve_asset_path(asset.usd_path),
                mass_props=sim_utils.MassPropertiesCfg(mass=metadata.mass),
                rigid_props=sim_utils.PhysxRigidBodyPropertiesCfg(
                    linear_damping=physics.linear_damping,
                    angular_damping=physics.angular_damping,
                    sleep_threshold=physics.sleep_threshold,
                    stabilization_threshold=physics.stabilization_threshold,
                ),
                physics_material=RigidBodyMaterialBaseCfg(
                    static_friction=metadata.friction,
                    dynamic_friction=metadata.friction,
                    restitution=physics.restitution,
                ),
            ),
        )


__all__ = [
    "AssetPlacement",
    "RigidObjectAssetConfig",
    "RigidObjectPhysicsConfig",
    "RigidObjectTaskConfig",
    "TaskDefinition",
    "TaskLayout",
]
