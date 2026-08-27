"""Stateful event terms used by ScaleBench environments."""

from __future__ import annotations

import operator
from collections.abc import Sequence
from typing import Any

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import EventTermCfg, ManagerTermBase

from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.common.task import Task


class ResetTaskLayout(ManagerTermBase):
    """Restore each environment's immutable initial task layout."""

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        self._task: Task = cfg.params["task"]
        self._layouts: tuple[TaskLayout, ...] = cfg.params["layouts"]
        self._context: PlacementContext = cfg.params["context"]
        if len(self._layouts) != env.num_envs:
            raise ValueError(
                f"expected {env.num_envs} task layouts, got {len(self._layouts)}"
            )
        self._asset_names = tuple(self._layouts[0].assets)

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: Sequence[int] | torch.Tensor | slice | None,
        task: Task,
        layouts: Sequence[TaskLayout],
        context: PlacementContext,
    ) -> None:
        del task, layouts, context
        resolved_env_ids = resolve_env_ids(env_ids, env.num_envs)
        if not resolved_env_ids:
            return
        selected_layouts = [self._layouts[env_id] for env_id in resolved_env_ids]
        self._write_layouts(env, resolved_env_ids, selected_layouts)

    def assign_layouts(
        self,
        env_ids: Sequence[int] | torch.Tensor | slice | None,
        layouts: Sequence[TaskLayout],
    ) -> None:
        """Assign validated layouts to the next reset of selected environments."""

        resolved_env_ids = resolve_env_ids(env_ids, self.num_envs)
        resolved_layouts = tuple(layouts)
        if len(resolved_layouts) == 1 and len(resolved_env_ids) > 1:
            resolved_layouts *= len(resolved_env_ids)
        if len(resolved_layouts) != len(resolved_env_ids):
            raise ValueError(
                "task_layouts must contain one layout or one per selected "
                f"environment ({len(resolved_env_ids)}); got {len(resolved_layouts)}"
            )
        for layout in resolved_layouts:
            self._task.validate_layout(self._context, layout)
        updated = list(self._layouts)
        for env_id, layout in zip(
            resolved_env_ids,
            resolved_layouts,
            strict=True,
        ):
            updated[env_id] = layout
        self._layouts = tuple(updated)

    def episode_info(
        self,
        env_ids: Sequence[int] | torch.Tensor | slice | None,
    ) -> dict[str, Any]:
        """Return metadata for the environments affected by the latest reset."""

        resolved_env_ids = resolve_env_ids(env_ids, self.num_envs)
        return {
            "env_ids": tuple(resolved_env_ids),
            "task_id": self._task.task_id,
            "instruction": self._task.instruction,
            "layout_seeds": tuple(
                self._layouts[env_id].seed for env_id in resolved_env_ids
            ),
        }

    def _write_layouts(
        self,
        env: ManagerBasedEnv,
        env_ids: Sequence[int],
        layouts: list[TaskLayout],
    ) -> None:
        env_id_tensor = torch.tensor(env_ids, device=env.device, dtype=torch.long)
        origins = env.scene.env_origins[env_id_tensor]

        for asset_name in self._asset_names:
            root_poses = torch.tensor(
                [
                    (
                        *layout.assets[asset_name].position_m,
                        *layout.assets[asset_name].orientation_xyzw,
                    )
                    for layout in layouts
                ],
                device=env.device,
                dtype=origins.dtype,
            )
            root_poses[:, :3] += origins
            env.scene[asset_name].write_root_pose_to_sim_index(
                root_pose=root_poses,
                env_ids=env_id_tensor,
            )


def resolve_env_ids(
    env_ids: Sequence[int] | torch.Tensor | slice | None,
    num_envs: int,
) -> tuple[int, ...]:
    """Resolve and strictly validate environment indices."""

    if env_ids is None:
        return tuple(range(num_envs))
    if isinstance(env_ids, slice):
        return tuple(range(num_envs)[env_ids])
    if isinstance(env_ids, torch.Tensor):
        if env_ids.ndim != 1:
            raise ValueError("env_ids must be a one-dimensional sequence")
        values = env_ids.tolist()
    else:
        try:
            values = list(env_ids)
        except TypeError as error:
            raise TypeError("env_ids must be a one-dimensional sequence") from error

    resolved_values: list[int] = []
    for env_id in values:
        if isinstance(env_id, bool):
            raise TypeError("env_ids must contain only integers")
        try:
            resolved_values.append(operator.index(env_id))
        except TypeError as error:
            raise TypeError("env_ids must contain only integers") from error
    resolved = tuple(resolved_values)

    if any(env_id < 0 or env_id >= num_envs for env_id in resolved):
        raise IndexError(f"environment ids must be in [0, {num_envs})")
    if len(resolved) != len(set(resolved)):
        raise ValueError("env_ids must not contain duplicates")
    return resolved


__all__ = ["ResetTaskLayout", "resolve_env_ids"]
