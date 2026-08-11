"""Stateful event terms used by ScaleBench environments."""

from __future__ import annotations

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
        self._context: PlacementContext = cfg.params["context"]
        self._layouts: tuple[TaskLayout, ...] = cfg.params["layouts"]
        if len(self._layouts) != env.num_envs:
            raise ValueError(
                f"expected {env.num_envs} task layouts, got {len(self._layouts)}"
            )
        for layout in self._layouts:
            self._task.validate_layout(self._context, layout)
        self._asset_names = tuple(self._layouts[0].assets)

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: Sequence[int] | torch.Tensor | slice | None,
        task: Task,
        context: PlacementContext,
        layouts: tuple[TaskLayout, ...],
    ) -> None:
        del task, context, layouts
        resolved_env_ids = _resolve_env_ids(env_ids, env.num_envs)
        if not resolved_env_ids:
            return
        selected_layouts = [self._layouts[env_id] for env_id in resolved_env_ids]
        self._write_layouts(env, resolved_env_ids, selected_layouts)

    def episode_info(
        self,
        env_ids: Sequence[int] | torch.Tensor | slice | None,
    ) -> dict[str, Any]:
        """Return metadata for the environments affected by the latest reset."""

        resolved_env_ids = _resolve_env_ids(env_ids, self.num_envs)
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
        env_ids: list[int],
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


def _resolve_env_ids(
    env_ids: Sequence[int] | torch.Tensor | slice | None,
    num_envs: int,
) -> list[int]:
    if env_ids is None:
        resolved = list(range(num_envs))
    elif isinstance(env_ids, slice):
        resolved = list(range(num_envs))[env_ids]
    elif isinstance(env_ids, torch.Tensor):
        resolved = [int(env_id) for env_id in env_ids.tolist()]
    else:
        resolved = [int(env_id) for env_id in env_ids]

    if any(env_id < 0 or env_id >= num_envs for env_id in resolved):
        raise IndexError(f"environment ids must be in [0, {num_envs})")
    return resolved


__all__ = ["ResetTaskLayout"]
