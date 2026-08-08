"""ScaleBench manager-based runtime entry."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.common import VecEnvObs
from isaaclab.sensors import CameraCfg

from .env_config import ScaleBenchEnvCfg
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

    def load_managers(self) -> None:
        """Load native managers and run startup events for this subclass."""

        super().load_managers()
        if "startup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="startup")

    @property
    def get_IO_descriptors(self) -> dict[str, Any]:
        """Extend native descriptors using the initialized runtime objects."""

        descriptors = super().get_IO_descriptors
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
            "camera_update_periods": camera_update_periods,
        }
        return descriptors

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


__all__ = ["ScaleBenchEnv"]
