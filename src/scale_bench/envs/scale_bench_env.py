"""ScaleBench manager-based runtime entry."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.common import VecEnvObs
from isaaclab.envs.mdp.actions import JointPositionAction
from isaaclab.sensors import CameraCfg

from .env_cfg import ScaleBenchEnvCfg
from .events import ResetTaskLayout


class ScaleBenchEnv(ManagerBasedEnv):
    """Own the simulation lifecycle and expose runtime-derived metadata."""

    def __init__(self, cfg: ScaleBenchEnvCfg) -> None:
        self._task_layout_reset: ResetTaskLayout | None = None
        super().__init__(cfg)

        if cfg.events.task_layout is not None:
            term = self.event_manager.get_term_cfg("task_layout").func
            if not isinstance(term, ResetTaskLayout):
                raise RuntimeError("task_layout event did not initialize correctly")
            self._task_layout_reset = term
        self._validate_manager_contract()

    def load_managers(self) -> None:
        """Load native managers and run startup events for this subclass."""

        super().load_managers()
        if "startup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="startup")

    @property
    def get_IO_descriptors(self) -> dict[str, Any]:
        """Extend native descriptors using the initialized runtime objects."""

        descriptors = super().get_IO_descriptors
        self._annotate_action_descriptors(descriptors["actions"])
        self._annotate_observation_descriptors(descriptors["observations"])
        camera_update_periods = {
            name: sensor.cfg.update_period
            for name, sensor in self.scene.sensors.items()
            if isinstance(sensor.cfg, CameraCfg)
        }
        render_dt = self.physics_dt * self.cfg.sim.render_interval
        descriptors["runtime"] = {
            "physics_dt": self.physics_dt,
            "step_dt": self.step_dt,
            "render_dt": render_dt,
            "physics_frequency_hz": 1.0 / self.physics_dt,
            "step_frequency_hz": 1.0 / self.step_dt,
            "render_frequency_hz": 1.0 / render_dt,
            "control_decimation": self.cfg.decimation,
            "arm_action_mode": self.cfg.arm_action_mode,
            "camera_update_periods": camera_update_periods,
        }
        return descriptors

    def step(self, action: torch.Tensor) -> tuple[VecEnvObs, dict]:
        """Validate and execute one action under the public environment contract."""

        expected_shape = (self.num_envs, self.action_manager.total_action_dim)
        if not isinstance(action, torch.Tensor):
            raise TypeError("action must be a torch.Tensor")
        if action.shape != expected_shape:
            raise ValueError(
                f"action shape must be {expected_shape}, got {tuple(action.shape)}"
            )
        if action.dtype != torch.float32:
            raise TypeError(f"action dtype must be torch.float32, got {action.dtype}")
        if action.device != torch.device(self.device):
            raise ValueError(
                f"action must be on environment device {self.device}, "
                f"got {action.device}"
            )
        if not torch.isfinite(action).all():
            raise ValueError("action must contain only finite values")
        return super().step(action)

    def reset(
        self,
        seed: int | None = None,
        env_ids: Sequence[int] | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[VecEnvObs, dict]:
        observation, info = super().reset(
            seed=seed,
            env_ids=env_ids,
            options=options,
        )
        if self._task_layout_reset is not None:
            info["episode"] = self._task_layout_reset.episode_info(env_ids)
        return observation, info

    def _annotate_action_descriptors(
        self,
        descriptors: list[dict[str, Any]],
    ) -> None:
        term_names = self.action_manager.active_terms
        term_dims = self.action_manager.action_term_dim
        if len(descriptors) != len(term_names):
            raise RuntimeError("action descriptor count does not match active terms")

        start = 0
        for name, dim, descriptor in zip(term_names, term_dims, descriptors):
            descriptor["name"] = name
            descriptor["shape"] = [dim]
            descriptor["slice"] = [start, start + dim]
            term = self.action_manager.get_term(name)
            if isinstance(term, JointPositionAction):
                self._annotate_joint_position_descriptor(descriptor, term)
            start += dim

    def _annotate_joint_position_descriptor(
        self,
        descriptor: dict[str, Any],
        term: JointPositionAction,
    ) -> None:
        asset = self.scene[term.cfg.asset_name]
        joint_ids, joint_names = asset.find_joints(
            term.cfg.joint_names,
            preserve_order=term.cfg.preserve_order,
        )
        if joint_names != term.cfg.joint_names:
            raise RuntimeError(
                f"{term.cfg.asset_name} joints resolved as {joint_names}, "
                f"expected profile order {term.cfg.joint_names}"
            )
        descriptor["action_type"] = "AbsoluteJointPosition"
        descriptor["joint_names"] = list(joint_names)
        descriptor["extras"].update(
            units="rad or m, joint-dependent",
            limits=asset.data.joint_pos_limits.torch[0, joint_ids]
            .detach()
            .cpu()
            .tolist(),
        )

    def _annotate_observation_descriptors(
        self,
        descriptors: dict[str, list[dict[str, Any]]],
    ) -> None:
        for group_name, group_descriptors in descriptors.items():
            term_names = self.observation_manager.active_terms[group_name]
            if len(group_descriptors) != len(term_names):
                raise RuntimeError(
                    f"{group_name} observation descriptor count does not match "
                    "active terms"
                )
            for name, descriptor in zip(term_names, group_descriptors):
                descriptor["name"] = name

    def _validate_manager_contract(self) -> None:
        expected_action_terms = [
            "left_arm",
            "left_gripper",
            "right_arm",
            "right_gripper",
        ]
        if self.action_manager.active_terms != expected_action_terms:
            raise RuntimeError(
                "action term order does not match the public contract: "
                f"{self.action_manager.active_terms}"
            )
        if self.observation_manager.group_obs_concatenate["policy"]:
            raise RuntimeError("policy observation terms must not be concatenated")

        descriptors = self.get_IO_descriptors
        action_dims = [item["shape"][0] for item in descriptors["actions"]]
        if action_dims != self.action_manager.action_term_dim:
            raise RuntimeError("action descriptors do not match manager dimensions")

        observation_descriptors = descriptors["observations"]["policy"]
        observation_dims = self.observation_manager.group_obs_term_dim["policy"]
        for descriptor, actual_shape in zip(
            observation_descriptors,
            observation_dims,
        ):
            if tuple(descriptor["shape"]) != tuple(actual_shape):
                raise RuntimeError(
                    f"observation descriptor {descriptor['name']!r} has shape "
                    f"{descriptor['shape']}, manager resolved {actual_shape}"
                )
            name = descriptor["name"]
            if name.endswith("_camera_rgb"):
                self._validate_camera_descriptor(
                    descriptor,
                    dtype="torch.uint8",
                    channels=3,
                )
            elif name.endswith("_camera_depth"):
                self._validate_camera_descriptor(
                    descriptor,
                    dtype="torch.float32",
                    channels=1,
                )

    @staticmethod
    def _validate_camera_descriptor(
        descriptor: dict[str, Any],
        *,
        dtype: str,
        channels: int,
    ) -> None:
        extras = descriptor["extras"]
        if descriptor["dtype"] != dtype:
            raise RuntimeError(
                f"camera observation {descriptor['name']!r} must use {dtype}, "
                f"got {descriptor['dtype']}"
            )
        if descriptor["shape"][-1] != channels or extras["layout"] != "HWC":
            raise RuntimeError(
                f"camera observation {descriptor['name']!r} has an invalid "
                "layout or channel count"
            )


__all__ = ["ScaleBenchEnv"]
