"""Recorder terms for the ScaleBench dataset contract."""

from __future__ import annotations

import torch
from isaaclab.managers import RecorderTerm

from scale_bench.runtime.recording import SEMANTIC_TEXT_BYTES


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


class SemanticEventsRecorder(RecorderTerm):
    """Record UTF-8 skill, command, and subgoal text for every action frame."""

    def record_pre_step(self):
        events = self._env.step_semantics
        return "semantic", {
            field_name: _encode_semantic_text(
                tuple(
                    None if event is None else getattr(event, field_name)
                    for event in events
                ),
                device=self._env.device,
            )
            for field_name in ("skill", "command_label", "subgoal")
        }


def _encode_semantic_text(
    values: tuple[str | None, ...],
    *,
    device: str,
) -> torch.Tensor:
    """
    Encode semantic text, using a zero row when that field has no value.
    """

    encoded = torch.zeros(
        (len(values), SEMANTIC_TEXT_BYTES),
        dtype=torch.uint8,
        device=device,
    )
    for index, value in enumerate(values):
        if value is None:
            continue
        raw = value.encode("utf-8")
        encoded[index, : len(raw)] = torch.tensor(
            tuple(raw),
            dtype=torch.uint8,
            device=device,
        )
    return encoded


__all__ = ["PolicyObservationsRecorder", "SemanticEventsRecorder"]
