"""Isaac Lab Event Manager configuration declarations."""

from __future__ import annotations

import isaaclab.envs.mdp as mdp
from isaaclab.envs.manager_based_env_cfg import DefaultEventManagerCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.utils.configclass import configclass


@configclass
class EventsCfg(DefaultEventManagerCfg):
    """Reset the complete scene, including stale articulation targets."""

    reset_scene_to_default = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset",
        params={"reset_joint_targets": True},
    )
    task_layout: EventTerm | None = None


__all__ = ["EventsCfg"]
