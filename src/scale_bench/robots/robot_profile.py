"""Load one robot YAML and build the corresponding Isaac Lab configuration."""

from __future__ import annotations

import math
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Self, TypeAlias

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

from scale_bench.sensors import CameraConvention, CameraProfile

if TYPE_CHECKING:
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sensors import CameraCfg


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
Name = Annotated[str, Field(min_length=1)]
RelativePrimPath = Annotated[
    str,
    Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$"),
]
PrimName = Annotated[str, Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")]
JointNames = Annotated[tuple[Name, ...], Field(min_length=1)]
ActuatorValue: TypeAlias = (
    NonNegativeFloat | dict[str, NonNegativeFloat] | None
)


class _ProfileModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


def _require_unique(names: tuple[str, ...], label: str) -> None:
    if len(names) != len(set(names)):
        raise ValueError(f"{label} contains duplicate names")


def _require_unit_quaternion(
    orientation_xyzw: tuple[float, float, float, float],
) -> None:
    norm = math.sqrt(sum(value * value for value in orientation_xyzw))
    if not math.isclose(norm, 1.0, abs_tol=1.0e-6):
        raise ValueError("orientation_xyzw must be a unit quaternion")


class TcpProfile(_ProfileModel):
    """Fixed benchmark TCP relative to an authored robot frame."""

    parent_frame: Name
    position_m: tuple[FiniteFloat, FiniteFloat, FiniteFloat] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
    ] = (0.0, 0.0, 0.0, 1.0)

    @model_validator(mode="after")
    def _validate_quaternion(self) -> Self:
        _require_unit_quaternion(self.orientation_xyzw)
        return self


class MountedCameraProfile(_ProfileModel):
    """Camera model and pose relative to an authored robot prim."""

    profile_path: Name
    parent_prim_path: RelativePrimPath
    sensor_prim_name: PrimName
    position_m: tuple[FiniteFloat, FiniteFloat, FiniteFloat] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
    ] = (0.0, 0.0, 0.0, 1.0)
    convention: CameraConvention = "opengl"

    @model_validator(mode="after")
    def _validate_quaternion(self) -> Self:
        _require_unit_quaternion(self.orientation_xyzw)
        return self


class KinematicsProfile(_ProfileModel):
    base_body: Name
    arm_joint_names: JointNames
    ee_body: Name
    tcp: TcpProfile # Tool Center Point

    @model_validator(mode="after")
    def _validate_joint_names(self) -> Self:
        _require_unique(self.arm_joint_names, "arm_joint_names")
        return self


class ImplicitActuatorProfile(_ProfileModel):
    joint_names: JointNames
    stiffness: ActuatorValue = None
    damping: ActuatorValue = None
    effort_limit_sim: ActuatorValue = None
    velocity_limit_sim: ActuatorValue = None

    @model_validator(mode="after")
    def _validate_joint_names(self) -> Self:
        _require_unique(self.joint_names, "actuator joint_names")
        return self


class ParallelJawGripperProfile(_ProfileModel):
    joint_names: JointNames
    command_joint_names: JointNames
    finger_body_names: tuple[Name, Name]
    min_aperture_m: NonNegativeFloat
    max_aperture_m: PositiveFloat
    closed_positions: dict[str, FiniteFloat]
    open_positions: dict[str, FiniteFloat]

    @model_validator(mode="after")
    def _validate_gripper(self) -> Self:
        _require_unique(self.joint_names, "gripper joint_names")
        _require_unique(self.command_joint_names, "gripper command_joint_names")
        if self.finger_body_names[0] == self.finger_body_names[1]:
            raise ValueError("finger_body_names must contain two different bodies")
        if self.max_aperture_m <= self.min_aperture_m:
            raise ValueError("max_aperture_m must be greater than min_aperture_m")

        state_joints = set(self.joint_names)
        command_joints = set(self.command_joint_names)
        if not command_joints <= state_joints:
            raise ValueError("command_joint_names must be a subset of joint_names")
        for field_name in ("closed_positions", "open_positions"):
            if set(getattr(self, field_name)) != command_joints:
                raise ValueError(
                    f"{field_name} keys must exactly match command_joint_names"
                )
        return self


class RobotProfile(_ProfileModel):
    """The small, typed boundary between a robot YAML and Isaac Lab."""

    name: Name
    usd_path: Name
    urdf_path: Name | None = None
    scale: tuple[PositiveFloat, PositiveFloat, PositiveFloat] | None = None
    fixed_base: bool = True
    disable_gravity: bool = False
    self_collisions: bool = False
    initial_joint_positions: dict[str, FiniteFloat]
    kinematics: KinematicsProfile
    actuators: dict[str, ImplicitActuatorProfile]
    gripper: ParallelJawGripperProfile
    camera: MountedCameraProfile | None = None

    @model_validator(mode="after")
    def _validate_joint_contract(self) -> Self:
        arm_joints = set(self.kinematics.arm_joint_names)
        gripper_joints = set(self.gripper.joint_names)
        if overlap := arm_joints & gripper_joints:
            raise ValueError(f"arm and gripper joints overlap: {sorted(overlap)}")

        declared_joints = arm_joints | gripper_joints
        initial_joints = set(self.initial_joint_positions)
        if initial_joints != declared_joints:
            raise ValueError(
                "initial_joint_positions must exactly cover arm and gripper joints; "
                f"missing={sorted(declared_joints - initial_joints)}, "
                f"unexpected={sorted(initial_joints - declared_joints)}"
            )

        actuated_joints: set[str] = set()
        for actuator_name, actuator in self.actuators.items():
            overlap = actuated_joints & set(actuator.joint_names)
            if overlap:
                raise ValueError(
                    f"actuator {actuator_name!r} overlaps another actuator: "
                    f"{sorted(overlap)}"
                )
            actuated_joints.update(actuator.joint_names)

        if unknown := actuated_joints - declared_joints:
            raise ValueError(f"actuators reference unknown joints: {sorted(unknown)}")
        required = arm_joints | set(self.gripper.command_joint_names)
        if missing := required - actuated_joints:
            raise ValueError(f"joints have no actuator: {sorted(missing)}")
        return self

    @classmethod
    def load(cls, config_path: str | Path) -> Self:
        """Load and validate a YAML file relative to the repository root."""

        path = Path(config_path)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            profile = cls.model_validate(document)
        except (OSError, yaml.YAMLError, ValidationError) as error:
            raise ValueError(
                f"Could not load robot profile {path}:\n{error}"
            ) from error

        for asset_path in (profile.usd_path, profile.urdf_path):
            if asset_path is None or "://" in asset_path:
                continue
            resolved = profile._resolve_asset_path(asset_path)
            if not Path(resolved).is_file():
                raise ValueError(f"Robot asset does not exist: {resolved}")
        if profile.camera is not None:
            CameraProfile.load(profile.camera.profile_path)
        return profile

    @staticmethod
    def _resolve_asset_path(asset_path: str) -> str:
        if "://" in asset_path:
            return asset_path
        path = Path(asset_path)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        return str(path.resolve())

    def build_articulation_cfg(
        self,
        *,
        prim_path: str | None = None,
    ) -> ArticulationCfg:
        """Build a fresh ``ArticulationCfg`` from this profile."""

        import isaaclab.sim as sim_utils
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import ArticulationCfg
        from isaaclab_physx.sim.schemas import (
            PhysxArticulationRootPropertiesCfg,
            PhysxRigidBodyPropertiesCfg,
        )

        def copy_value(value: ActuatorValue) -> float | dict[str, float] | None:
            return dict(value) if isinstance(value, Mapping) else value

        actuators = {
            name: ImplicitActuatorCfg(
                joint_names_expr=list(spec.joint_names),
                stiffness=copy_value(spec.stiffness),
                damping=copy_value(spec.damping),
                effort_limit_sim=copy_value(spec.effort_limit_sim),
                velocity_limit_sim=copy_value(spec.velocity_limit_sim),
            )
            for name, spec in self.actuators.items()
        }
        joint_order = (
            *self.kinematics.arm_joint_names,
            *self.gripper.joint_names,
        )
        cfg = ArticulationCfg(
            spawn=sim_utils.UsdFileCfg(
                usd_path=self._resolve_asset_path(self.usd_path),
                scale=self.scale,
                rigid_props=PhysxRigidBodyPropertiesCfg(
                    disable_gravity=self.disable_gravity
                ),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    fix_root_link=self.fixed_base,
                    enabled_self_collisions=self.self_collisions,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos={
                    joint_name: self.initial_joint_positions[joint_name]
                    for joint_name in joint_order
                }
            ),
            actuators=actuators,
        )
        if prim_path is not None:
            cfg.prim_path = prim_path
        return cfg

    def build_camera_cfg(self, *, robot_prim_path: str) -> CameraCfg | None:
        """Build the camera mounted below ``robot_prim_path``, when configured."""

        if self.camera is None:
            return None
        root_prim_path = robot_prim_path.rstrip("/")
        if not root_prim_path:
            raise ValueError("robot_prim_path must not be empty")

        mount = self.camera
        profile = CameraProfile.load(mount.profile_path)
        return profile.build_camera_cfg(
            prim_path=(
                f"{root_prim_path}/{mount.parent_prim_path}/"
                f"{mount.sensor_prim_name}"
            ),
            position_m=mount.position_m,
            orientation_xyzw=mount.orientation_xyzw,
            convention=mount.convention,
        )


__all__ = ["RobotProfile"]
