"""Task evaluator observation term tests without a running simulation."""

from __future__ import annotations

from types import SimpleNamespace

import torch
from isaaclab.managers import SceneEntityCfg

from scale_bench.isaaclab.mdp.observations import (
    fixed_positions,
    rigid_object_root_pos,
    rigid_object_root_quat,
)


class _Scene(dict):
    def __init__(self, *args, env_origins: torch.Tensor, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.env_origins = env_origins


def test_rigid_object_evaluator_terms_preserve_order_and_local_frames() -> None:
    origins = torch.tensor([[10.0, 0.0, 0.0], [20.0, 1.0, 0.0]])
    first_positions = torch.tensor([[11.0, 2.0, 3.0], [21.0, 3.0, 4.0]])
    second_positions = torch.tensor([[14.0, 5.0, 6.0], [24.0, 6.0, 7.0]])
    first_quaternions = torch.tensor(
        [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0]]
    )
    second_quaternions = torch.tensor(
        [[0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]
    )
    scene = _Scene(
        {
            "first": SimpleNamespace(
                data=SimpleNamespace(
                    root_pos_w=SimpleNamespace(torch=first_positions),
                    root_quat_w=SimpleNamespace(torch=first_quaternions),
                )
            ),
            "second": SimpleNamespace(
                data=SimpleNamespace(
                    root_pos_w=SimpleNamespace(torch=second_positions),
                    root_quat_w=SimpleNamespace(torch=second_quaternions),
                )
            ),
        },
        env_origins=origins,
    )
    env = SimpleNamespace(scene=scene)
    asset_cfgs = (SceneEntityCfg("second"), SceneEntityCfg("first"))

    positions = rigid_object_root_pos(env, asset_cfgs)
    orientations = rigid_object_root_quat(env, asset_cfgs)

    torch.testing.assert_close(positions[:, 0], second_positions - origins)
    torch.testing.assert_close(positions[:, 1], first_positions - origins)
    torch.testing.assert_close(orientations[:, 0], second_quaternions)
    torch.testing.assert_close(orientations[:, 1], first_quaternions)


def test_fixed_positions_broadcasts_independent_batches() -> None:
    env = SimpleNamespace(device="cpu", num_envs=2)

    positions = fixed_positions(env, ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)))
    positions[0, 0, 0] = 99.0

    assert positions.shape == (2, 2, 3)
    assert positions.dtype == torch.float32
    assert positions[1, 0, 0].item() == 1.0
