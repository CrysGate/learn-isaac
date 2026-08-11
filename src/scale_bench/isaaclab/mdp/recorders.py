"""Recorder terms for the ScaleBench dataset contract."""

from __future__ import annotations

from isaaclab.managers import RecorderTerm


class PolicyObservationsRecorder(RecorderTerm):
    """Record the configured named policy observations before each action."""

    def record_pre_step(self):
        policy_observations = self._env.obs_buf["policy"]
        missing = [
            name
            for name in self.cfg.observation_names
            if name not in policy_observations
        ]
        if missing:
            raise RuntimeError(
                "configured recorder observations are missing at runtime: "
                + ", ".join(missing)
            )
        return "obs", {
            name: policy_observations[name]
            for name in self.cfg.observation_names
        }


__all__ = ["PolicyObservationsRecorder"]
