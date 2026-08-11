"""Recorder term contract tests that do not launch a simulation."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
import torch

from scale_bench.isaaclab.managers.recorders import (
    PolicyObservationsRecorderCfg,
)
from scale_bench.isaaclab.mdp.recorders import PolicyObservationsRecorder


def test_policy_observation_recorder_preserves_named_dictionary_terms() -> None:
    joint_pos = torch.tensor([[1.0, 2.0]])
    camera_rgb = torch.ones((1, 2, 2, 3), dtype=torch.uint8)
    env = SimpleNamespace(
        obs_buf={
            "policy": {
                "joint_pos": joint_pos,
                "camera_rgb": camera_rgb,
            }
        }
    )
    term = PolicyObservationsRecorder(
        PolicyObservationsRecorderCfg(observation_names=("joint_pos",)),
        env,
    )

    key, value = term.record_pre_step()

    assert key == "obs"
    assert value == {"joint_pos": joint_pos}


def test_policy_observation_recorder_rejects_runtime_contract_drift() -> None:
    env = SimpleNamespace(obs_buf={"policy": {}})
    term = PolicyObservationsRecorder(
        PolicyObservationsRecorderCfg(observation_names=("joint_pos",)),
        env,
    )

    with pytest.raises(RuntimeError, match="joint_pos"):
        term.record_pre_step()
