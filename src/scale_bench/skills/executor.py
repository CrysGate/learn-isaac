"""Play planned commands and assemble one batched action per frame."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Protocol, TypeAlias

import torch
from torch import Tensor

from .commands import Hold, MoveToJoints, MoveToPose, SetGripper, SkillCommand
from .models import Arm


@dataclass(frozen=True, slots=True)
class CommandActionLayout:
    """Slices and gripper targets for one dual-arm action row."""

    action_dim: int
    left_arm: tuple[int, int]
    left_gripper: tuple[int, int]
    right_arm: tuple[int, int]
    right_gripper: tuple[int, int]
    left_gripper_open: tuple[float, ...]
    left_gripper_closed: tuple[float, ...]
    right_gripper_open: tuple[float, ...]
    right_gripper_closed: tuple[float, ...]

    def __post_init__(self) -> None:
        ranges = (
            self.left_arm,
            self.left_gripper,
            self.right_arm,
            self.right_gripper,
        )
        covered = [index for start, stop in ranges for index in range(start, stop)]
        if self.action_dim <= 0 or sorted(covered) != list(range(self.action_dim)):
            raise ValueError("action ranges must cover action_dim exactly once")
        targets = (
            (self.left_gripper, self.left_gripper_open),
            (self.left_gripper, self.left_gripper_closed),
            (self.right_gripper, self.right_gripper_open),
            (self.right_gripper, self.right_gripper_closed),
        )
        for (start, stop), values in targets:
            if len(values) != stop - start or any(
                not math.isfinite(value) for value in values
            ):
                raise ValueError("gripper targets must match their action range")

    def arm_range(self, arm: Arm) -> tuple[int, int]:
        return self.left_arm if arm == "left" else self.right_arm

    def gripper_range(self, arm: Arm) -> tuple[int, int]:
        return self.left_gripper if arm == "left" else self.right_gripper

    def gripper_target(self, arm: Arm, *, closed: bool) -> tuple[float, ...]:
        if arm == "left":
            return self.left_gripper_closed if closed else self.left_gripper_open
        return self.right_gripper_closed if closed else self.right_gripper_open


class CommandEnvironment(Protocol):
    num_envs: int
    device: str

    def hold_action(self) -> Tensor: ...


@dataclass(frozen=True, slots=True)
class CommandBatch:
    action: Tensor
    completed_after_step: Tensor
    labels: Mapping[int, str]

    def __post_init__(self) -> None:
        object.__setattr__(self, "labels", MappingProxyType(dict(self.labels)))


@dataclass(slots=True)
class _HoldExecution:
    command: Hold
    step_index: int = 0


@dataclass(slots=True)
class _GripperExecution:
    command: SetGripper
    target: Tensor
    step_index: int = 0


@dataclass(slots=True)
class _MotionExecution:
    command: MoveToPose | MoveToJoints
    targets: Tensor
    step_index: int = 0


_Execution: TypeAlias = _HoldExecution | _GripperExecution | _MotionExecution


class CommandExecutor:
    """Play trajectories and preserve latched gripper targets."""

    def __init__(self, env: CommandEnvironment, layout: CommandActionLayout) -> None:
        self._env = env
        self._layout = layout
        self._executions: dict[int, _Execution] = {}
        self._arm_targets: dict[tuple[int, Arm], Tensor] = {}
        self._gripper_targets: dict[tuple[int, Arm], Tensor] = {}

    @property
    def layout(self) -> CommandActionLayout:
        return self._layout

    def begin(self, env_id: int, command: SkillCommand) -> None:
        hold = self._env.hold_action()
        if isinstance(command, Hold):
            execution: _Execution = _HoldExecution(command)
        elif isinstance(command, SetGripper):
            target = hold.new_tensor(
                self._layout.gripper_target(command.arm, closed=command.closed)
            )
            self._gripper_targets[(env_id, command.arm)] = target
            execution = _GripperExecution(command, target)
        else:
            targets = command.trajectory.positions
            execution = _MotionExecution(command, targets)
        self._executions[env_id] = execution

    def next_actions(self, active_mask: Tensor) -> CommandBatch:
        action = self._env.hold_action().clone()

        completed = torch.zeros_like(active_mask)
        labels: dict[int, str] = {}
        env_ids = torch.nonzero(active_mask, as_tuple=False).flatten().cpu().tolist()
        for env_id in env_ids:
            execution = self._executions.get(env_id)
            self._apply_latched_arms(action, env_id)
            self._apply_latched_grippers(action, env_id)
            labels[env_id] = execution.command.label

            if isinstance(execution, _MotionExecution):
                start, stop = self._layout.arm_range(execution.command.arm)
                action[env_id, start:stop] = execution.targets[execution.step_index]
                step_count = execution.targets.shape[0]
            elif isinstance(execution, _GripperExecution):
                start, stop = self._layout.gripper_range(execution.command.arm)
                action[env_id, start:stop] = execution.target
                step_count = execution.command.steps
            else:
                step_count = execution.command.steps

            execution.step_index += 1
            if execution.step_index == step_count:
                if isinstance(execution, _MotionExecution):
                    self._arm_targets[(env_id, execution.command.arm)] = (
                        execution.targets[-1].clone()
                    )
                completed[env_id] = True
                del self._executions[env_id]

        return CommandBatch(action, completed, labels)

    def abort(self, env_ids: Sequence[int]) -> None:
        self._clear(env_ids)

    def reset(self, env_ids: Sequence[int]) -> None:
        self._clear(env_ids)

    def _clear(self, env_ids: Sequence[int]) -> None:
        for env_id in env_ids:
            self._executions.pop(env_id, None)
            self._arm_targets.pop((env_id, "left"), None)
            self._arm_targets.pop((env_id, "right"), None)
            self._gripper_targets.pop((env_id, "left"), None)
            self._gripper_targets.pop((env_id, "right"), None)

    def _apply_latched_arms(self, action: Tensor, env_id: int) -> None:
        for arm in ("left", "right"):
            target = self._arm_targets.get((env_id, arm))
            if target is not None:
                start, stop = self._layout.arm_range(arm)
                action[env_id, start:stop] = target

    def _apply_latched_grippers(self, action: Tensor, env_id: int) -> None:
        for arm in ("left", "right"):
            target = self._gripper_targets.get((env_id, arm))
            if target is not None:
                start, stop = self._layout.gripper_range(arm)
                action[env_id, start:stop] = target


__all__ = [
    "CommandActionLayout",
    "CommandBatch",
    "CommandEnvironment",
    "CommandExecutor",
]
