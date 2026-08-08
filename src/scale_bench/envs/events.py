"""Environment event terms owned by the ScaleBench runtime."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import EventTermCfg, ManagerTermBase

from scale_bench.tasks import TaskDefinition, TaskLayout


class ResetTaskLayout(ManagerTermBase):
    """Reset task objects and retain episode state for each environment."""

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        self._task: TaskDefinition = cfg.params["task"]
        self._initial_layout: TaskLayout = cfg.params["initial_layout"]
        self._resample_on_reset: bool = cfg.params["resample_on_reset"]
        self._task.validate_layout(self._initial_layout)
        if self._resample_on_reset and self._initial_layout.seed is None:
            raise ValueError("resampled task layouts require an initial layout seed")

        self._episode_counts = [0] * env.num_envs
        self._layout_seeds: list[int | None] = [None] * env.num_envs

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: Sequence[int] | torch.Tensor | slice | None,
        task: TaskDefinition,
        initial_layout: TaskLayout,
        resample_on_reset: bool,
    ) -> None:
        del task, initial_layout, resample_on_reset
        resolved_env_ids = _resolve_env_ids(env_ids, env.num_envs)
        if not resolved_env_ids:
            return
        layouts = [self._next_layout(env_id) for env_id in resolved_env_ids]
        self._write_layouts(env, resolved_env_ids, layouts)

        for env_id, layout in zip(resolved_env_ids, layouts, strict=True):
            self._layout_seeds[env_id] = layout.seed
            self._episode_counts[env_id] += 1

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
                self._layout_seeds[env_id] for env_id in resolved_env_ids
            ),
        }

    def _next_layout(self, env_id: int) -> TaskLayout:
        if not self._resample_on_reset:
            return self._initial_layout

        base_seed = self._initial_layout.seed
        if base_seed is None:
            raise RuntimeError("resampled task layout has no base seed")
        seed = base_seed + env_id + self._episode_counts[env_id] * self.num_envs
        return self._task.generate_layout(seed)

    def _write_layouts(
        self,
        env: ManagerBasedEnv,
        env_ids: list[int],
        layouts: list[TaskLayout],
    ) -> None:
        env_id_tensor = torch.tensor(env_ids, device=env.device, dtype=torch.long)
        origins = env.scene.env_origins[env_id_tensor]

        for asset_name in self._initial_layout.assets:
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
