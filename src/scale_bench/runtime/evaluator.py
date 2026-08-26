"""Stateful, batched episode evaluation independent of Isaac Lab."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Protocol

import torch
from torch import Tensor

from scale_bench.tasks.common.evaluation import EvaluationResult
from scale_bench.tasks.common.task import BatchedEvaluatorObservation, Task


class EpisodeEvaluator(Protocol):
    """Stateful evaluation lifecycle consumed by the episode driver."""

    def reset(self, env_ids: Tensor) -> None: ...

    def update(
        self,
        observation: BatchedEvaluatorObservation,
        success_verification_mask: Tensor,
    ) -> Tensor: ...

    def finalize(
        self,
        env_ids: Tensor,
        observation: BatchedEvaluatorObservation,
    ) -> Mapping[int, EvaluationResult]: ...


class TaskEpisodeEvaluator:
    """Add stability and lifecycle state around a task's tensor predicate."""

    def __init__(
        self,
        task: Task,
        *,
        num_envs: int,
        device: str,
    ) -> None:
        self._task = task
        self._num_envs = num_envs
        self._device = torch.device(device)
        self._tracked = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self._consecutive_successes = torch.zeros(
            num_envs,
            dtype=torch.long,
            device=device,
        )
        self._success = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def reset(self, env_ids: Tensor) -> None:
        resolved = self._resolve_env_ids(env_ids)
        self._tracked[resolved] = True
        self._consecutive_successes[resolved] = 0
        self._success[resolved] = False

    def update(
        self,
        observation: BatchedEvaluatorObservation,
        success_verification_mask: Tensor,
    ) -> Tensor:
        """Advance stability only for slots in success verification."""

        tensor_observation = self._validate_observation(observation)
        raw_success = self._task.check_success(tensor_observation)

        self._consecutive_successes = torch.where(
            self._tracked & success_verification_mask & raw_success,
            self._consecutive_successes + 1,
            torch.where(
                self._tracked,
                torch.zeros_like(self._consecutive_successes),
                self._consecutive_successes,
            ),
        )
        newly_stable = (
            self._consecutive_successes
            >= self._task.evaluator_spec.success_stability_steps
        )
        self._success |= self._tracked & newly_stable
        return self._success.clone()

    def finalize(
        self,
        env_ids: Tensor,
        observation: BatchedEvaluatorObservation,
    ) -> Mapping[int, EvaluationResult]:
        resolved = self._resolve_env_ids(env_ids)
        tensor_observation = self._validate_observation(observation)
        env_id_values = resolved.detach().cpu().tolist()
        results = {}
        for env_id in env_id_values:
            result = self._task.evaluate(
                {
                    name: value[env_id]
                    for name, value in tensor_observation.items()
                }
            )
            stable_success = bool(self._success[env_id].item())
            confirmed_success = result.success and stable_success
            if result.success != confirmed_success:
                result = replace(
                    result,
                    success=confirmed_success,
                    failure_reason=(
                        "success condition was not stable for "
                        f"{self._task.evaluator_spec.success_stability_steps} "
                        "verification steps"
                    ),
                )
            results[env_id] = result
        self._tracked[resolved] = False
        return results

    def _resolve_env_ids(self, env_ids: Tensor) -> Tensor:
        if not isinstance(env_ids, Tensor):
            raise TypeError("env_ids must be a torch.Tensor")
        if env_ids.device != self._device:
            raise ValueError("env_ids must be on the evaluator device")
        if env_ids.dtype not in {torch.int32, torch.int64}:
            raise TypeError("env_ids must use an integer dtype")
        if env_ids.ndim != 1:
            raise ValueError("env_ids must be one-dimensional")
        return env_ids.to(dtype=torch.long)

    def _validate_observation(
        self,
        observation: BatchedEvaluatorObservation,
    ) -> dict[str, Tensor]:
        resolved = {}
        for name, value in observation.items():
            if not isinstance(value, Tensor):
                raise TypeError(f"evaluator observation {name!r} must be a tensor")
            if value.ndim == 0 or value.shape[0] != self._num_envs:
                raise ValueError(
                    f"evaluator observation {name!r} must have batch dimension "
                    f"{self._num_envs}"
                )
            if value.device != self._device:
                raise ValueError(
                    "evaluator observations must share the evaluator device"
                )
            resolved[name] = value
        return resolved


__all__ = [
    "EpisodeEvaluator",
    "TaskEpisodeEvaluator",
]
