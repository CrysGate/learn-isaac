"""Robot joints, actuators, gripper, TCP, and mounted-camera configuration."""

from __future__ import annotations

from typing import Annotated, Self, TypeAlias
from pydantic import Field, StrictBool, model_validator

from scale_bench.config.base import (
    AssetReference,
    CameraConvention,
    ConfigReference,
    FiniteFloat,
    FrozenModel,
    Name,
    NonNegativeFloat,
    OptionalAssetReference,
    PositiveFloat,
    Position3,
    Quaternion,
    require_unique,
    require_unit_quaternion,
)


RelativePrimPath = Annotated[str, Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$")]
PrimName = Annotated[str, Field(pattern=r"^[A-Za-z_][A-Za-z0-9_]*$")]
JointNames = Annotated[tuple[Name, ...], Field(min_length=1)]
ActuatorValue: TypeAlias = NonNegativeFloat | dict[str, NonNegativeFloat] | None


class TcpConfig(FrozenModel):
    parent_frame: Name
    position_m: Position3 = (0.0, 0.0, 0.0)
    orientation_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)

    @model_validator(mode="after")
    def _validate_quaternion(self) -> Self:
        require_unit_quaternion(self.orientation_xyzw, "orientation_xyzw")
        return self


class MountedCameraConfig(FrozenModel):
    profile_path: ConfigReference
    parent_prim_path: RelativePrimPath
    sensor_prim_name: PrimName
    position_m: Position3 = (0.0, 0.0, 0.0)
    orientation_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)
    convention: CameraConvention = "opengl"

    @model_validator(mode="after")
    def _validate_quaternion(self) -> Self:
        require_unit_quaternion(self.orientation_xyzw, "orientation_xyzw")
        return self


class KinematicsConfig(FrozenModel):
    base_body: Name
    arm_joint_names: JointNames
    ee_body: Name
    tcp: TcpConfig

    @model_validator(mode="after")
    def _validate_joint_names(self) -> Self:
        require_unique(self.arm_joint_names, "arm_joint_names")
        return self


class ImplicitActuatorConfig(FrozenModel):
    joint_names: JointNames
    stiffness: ActuatorValue = None
    damping: ActuatorValue = None
    effort_limit_sim: ActuatorValue = None
    velocity_limit_sim: ActuatorValue = None

    @model_validator(mode="after")
    def _validate_joint_names(self) -> Self:
        require_unique(self.joint_names, "actuator joint_names")
        return self


class ParallelJawGripperConfig(FrozenModel):
    joint_names: JointNames
    command_joint_names: JointNames
    finger_body_names: tuple[Name, Name]
    min_aperture_m: NonNegativeFloat
    max_aperture_m: PositiveFloat
    closed_positions: dict[str, FiniteFloat]
    open_positions: dict[str, FiniteFloat]

    @model_validator(mode="after")
    def _validate_gripper(self) -> Self:
        require_unique(self.joint_names, "gripper joint_names")
        require_unique(self.command_joint_names, "gripper command_joint_names")
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
        unchanged = [
            joint_name
            for joint_name in self.command_joint_names
            if self.closed_positions[joint_name] == self.open_positions[joint_name]
        ]
        if unchanged:
            raise ValueError(
                "open and closed positions must differ for command joints: "
                f"{unchanged}"
            )
        return self


class RobotConfig(FrozenModel):
    """Complete simulator-independent robot description."""

    name: Name
    usd_path: AssetReference
    urdf_path: OptionalAssetReference = None
    fixed_base: StrictBool = True
    disable_gravity: StrictBool = False
    self_collisions: StrictBool = False
    initial_joint_positions: dict[str, FiniteFloat]
    kinematics: KinematicsConfig
    actuators: dict[str, ImplicitActuatorConfig]
    gripper: ParallelJawGripperConfig
    camera: MountedCameraConfig | None = None

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
