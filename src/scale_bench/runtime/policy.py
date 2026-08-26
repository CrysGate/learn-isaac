"""Policy controller contract and fixed-batch rollout runner."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol, TypeAlias

import torch
from torch import Tensor

from scale_bench.tasks.common.task import Task

from .driver import DriverEnvironment, EpisodeDriver
from .episodes import EpisodeResult, EpisodeSpec, EpisodeState, EpisodeTermination
from .episodes import TerminationReason


PolicyObservation: TypeAlias = Mapping[str, Tensor]


@dataclass(frozen=True, slots=True)
class EpisodeContext:
    """Stable episode information supplied when policy state is reset."""

    env_id: int
    spec: EpisodeSpec
    instruction: str


@dataclass(frozen=True, slots=True)
class PolicyOutput:
    """One policy action batch and an optional active termination mask."""

    action: Tensor
    done_mask: Tensor | None = None


class PolicyController(Protocol):
    """Minimal stateful policy interface used by policy rollout."""

    def reset(
        self,
        env_ids: Tensor,
        contexts: Sequence[EpisodeContext],
    ) -> None: ...

    def act(
        self,
        observation: PolicyObservation,
        active_mask: Tensor,
    ) -> Tensor | PolicyOutput: ...


class PolicyEnvironment(DriverEnvironment, Protocol):
    """Environment additions needed to build safe inactive actions."""

    @property
    def task(self) -> Task: ...

    def hold_action(self) -> Tensor: ...


class PolicyRolloutRunner:
    """Run policy inference over one fixed batch through EpisodeDriver."""

    def __init__(
        self,
        env: PolicyEnvironment,
        policy: PolicyController,
    ) -> None:
        self._env = env
        self._policy = policy
        self._driver = EpisodeDriver(env)

    @property
    def num_envs(self) -> int:
        return self._env.num_envs

    def run_batch(
        self,
        states: Sequence[EpisodeState],
    ) -> Mapping[str, EpisodeResult]:
        """Run at most one assigned episode in each environment slot."""

        snapshot = self._driver.start(states)
        state_by_env_id = {state.env_id: state for state in states}
        active_env_ids = torch.nonzero(
            snapshot.active_mask,
            as_tuple=False,
        ).flatten()
        if active_env_ids.numel():
            contexts = tuple(
                EpisodeContext(
                    env_id=env_id,
                    spec=state_by_env_id[env_id].spec,
                    instruction=self._env.task.instruction,
                )
                for env_id in active_env_ids.detach().cpu().tolist()
            )
            self._policy.reset(active_env_ids, contexts)

        while self._driver.is_active:
            active_mask = snapshot.active_mask
            policy_output = self._policy.act(
                _policy_observation(snapshot.observation),
                active_mask,
            )
            output = (
                policy_output
                if isinstance(policy_output, PolicyOutput)
                else PolicyOutput(action=policy_output)
            )
            hold_action = self._env.hold_action()
            action, terminations = _prepare_policy_step(
                output,
                active_mask=active_mask,
                hold_action=hold_action,
            )
            snapshot = self._driver.step(
                action,
                success_verification_mask=active_mask,
                terminations=terminations,
            )
        return self._driver.results


def _prepare_policy_step(
    output: PolicyOutput,
    *,
    active_mask: Tensor,
    hold_action: Tensor,
) -> tuple[Tensor, dict[int, EpisodeTermination]]:
    action = output.action
    if not isinstance(action, Tensor):
        raise TypeError("policy action must be a torch.Tensor")
    if action.shape != hold_action.shape:
        raise ValueError(
            f"policy action shape must be {tuple(hold_action.shape)}, "
            f"got {tuple(action.shape)}"
        )
    if action.dtype != hold_action.dtype:
        raise TypeError(
            f"policy action dtype must be {hold_action.dtype}, got {action.dtype}"
        )
    if action.device != hold_action.device:
        raise ValueError(
            f"policy action device must be {hold_action.device}, got {action.device}"
        )
    if active_mask.shape != (action.shape[0],):
        raise ValueError("active_mask must contain one value per environment")
    if active_mask.dtype != torch.bool or active_mask.device != action.device:
        raise TypeError("active_mask must be a bool tensor on the action device")

    invalid_mask = active_mask & ~torch.isfinite(action).all(dim=1)
    done_mask = _resolve_done_mask(
        output.done_mask,
        active_mask=active_mask,
    )
    invalid_env_ids = torch.nonzero(
        invalid_mask,
        as_tuple=False,
    ).flatten().detach().cpu().tolist()
    done_env_ids = torch.nonzero(
        done_mask & ~invalid_mask,
        as_tuple=False,
    ).flatten().detach().cpu().tolist()
    terminations = {
        **{
            env_id: EpisodeTermination(
                TerminationReason.INVALID_ACTION,
                retryable=False,
                message="policy produced a non-finite action row",
            )
            for env_id in invalid_env_ids
        },
        **{
            env_id: EpisodeTermination(TerminationReason.CONTROLLER_FINISHED)
            for env_id in done_env_ids
        },
    }
    submitted_mask = active_mask & ~invalid_mask & ~done_mask
    submitted_action = torch.where(
        submitted_mask.unsqueeze(-1),
        action,
        hold_action,
    )
    return submitted_action, terminations


def _resolve_done_mask(
    done_mask: Tensor | None,
    *,
    active_mask: Tensor,
) -> Tensor:
    if done_mask is None:
        return torch.zeros_like(active_mask)
    if not isinstance(done_mask, Tensor):
        raise TypeError("policy done_mask must be a torch.Tensor")
    if done_mask.shape != active_mask.shape:
        raise ValueError("policy done_mask must contain one value per environment")
    if done_mask.dtype != torch.bool:
        raise TypeError("policy done_mask must use torch.bool")
    if done_mask.device != active_mask.device:
        raise ValueError("policy done_mask must be on the active-mask device")
    return done_mask & active_mask


def _policy_observation(observation: object) -> PolicyObservation:
    if not isinstance(observation, Mapping):
        raise TypeError("environment observation must be a mapping")
    policy_observation = observation.get("policy")
    if not isinstance(policy_observation, Mapping):
        raise RuntimeError("environment observation has no policy group")
    return policy_observation


__all__ = [
    "EpisodeContext",
    "PolicyController",
    "PolicyObservation",
    "PolicyOutput",
    "PolicyRolloutRunner",
]
