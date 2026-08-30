"""Isaac Lab Event Manager configuration declarations."""

from __future__ import annotations

from dataclasses import MISSING

import isaaclab.envs.mdp as mdp
from isaaclab.envs.manager_based_env_cfg import DefaultEventManagerCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.utils.configclass import configclass

from scale_bench.isaaclab.mdp.events import synchronize_tensor_pose_resets_for_rtx

@configclass
class EventsCfg(DefaultEventManagerCfg):
    """Reset the complete scene, including stale articulation targets."""

    reset_scene_to_default = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset",
        params={"reset_joint_targets": True},
    )
    task_layout: EventTerm = MISSING
    synchronize_tensor_pose_resets_for_rtx = EventTerm(
        func=synchronize_tensor_pose_resets_for_rtx,
        mode="reset",
    )


__all__ = ["EventsCfg"]
