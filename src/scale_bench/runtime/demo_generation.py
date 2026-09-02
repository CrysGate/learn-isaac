"""Task-expert and skill-program execution over the common episode driver."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass

import torch
from torch import Tensor

from scale_bench.skills import (
    CommandExecutor,
    Hold,
    Pick,
    PickAndPlace,
    SkillCommand,
    SkillContext,
    SkillError,
    SkillPlanner,
    SkillRequest,
    pick,
    pick_and_place,
)

from .driver import DriverEnvironment, EpisodeDriver
from .episodes import EpisodeResult, EpisodeState, EpisodeTermination
from .episodes import TerminationReason
from .logging import episode_log_context
from .recording import StepSemantics


ExpertFactory = Callable[[EpisodeState], Iterator[SkillRequest]]
SkillContextFactory = Callable[[EpisodeState], SkillContext]
SkillPlannerFactory = Callable[[EpisodeState], SkillPlanner]


@dataclass(slots=True)
class _ProgramState:
    episode_id: str
    expert: Iterator[SkillRequest]
    context: SkillContext
    planner: SkillPlanner
    commands: Iterator[SkillCommand] | None = None
    skill_name: str | None = None
    subgoal: str | None = None
    command_running: bool = False
    verification_started: bool = False


class DemoGenerationRunner:
    """Run one independent task expert and command cursor per env slot."""

    def __init__(
        self,
        env: DriverEnvironment,
        executor: CommandExecutor,
        *,
        expert_factory: ExpertFactory,
        context_factory: SkillContextFactory,
        planner_factory: SkillPlannerFactory,
    ) -> None:
        self._env = env
        self._executor = executor
        self._expert_factory = expert_factory
        self._context_factory = context_factory
        self._planner_factory = planner_factory
        self._driver = EpisodeDriver(env)
        self._programs: dict[int, _ProgramState] = {}

    @property
    def num_envs(self) -> int:
        return self._env.num_envs

    def run_batch(
        self,
        states: Sequence[EpisodeState],
    ) -> Mapping[str, EpisodeResult]:
        self._executor.reset(tuple(state.env_id for state in states))
        snapshot = self._driver.start(states)
        state_by_env_id = {state.env_id: state for state in states}
        active_env_ids = torch.nonzero(
            snapshot.active_mask,
            as_tuple=False,
        ).flatten().detach().cpu().tolist()
        self._programs = {
            env_id: _ProgramState(
                episode_id=state_by_env_id[env_id].spec.episode_id,
                expert=self._expert_factory(state_by_env_id[env_id]),
                context=self._context_factory(state_by_env_id[env_id]),
                planner=self._planner_factory(state_by_env_id[env_id]),
            )
            for env_id in active_env_ids
        }

        try:
            while self._driver.is_active:
                active_mask = snapshot.active_mask
                terminations = self._prepare_commands(active_mask)
                success_verification_mask = torch.zeros_like(active_mask)
                for env_id, program in self._programs.items():
                    if program.verification_started and env_id not in terminations:
                        success_verification_mask[env_id] = True
                command_mask = active_mask.clone()
                if terminations:
                    command_mask[
                        torch.tensor(
                            tuple(terminations),
                            dtype=torch.long,
                            device=command_mask.device,
                        )
                    ] = False

                if command_mask.any().item():
                    command_batch = self._executor.next_actions(command_mask)
                    action = command_batch.action
                    semantic_events = {
                        env_id: StepSemantics(
                            skill=self._programs[env_id].skill_name,
                            command_label=label,
                            subgoal=self._programs[env_id].subgoal,
                        )
                        for env_id, label in command_batch.labels.items()
                    }
                else:
                    command_batch = None
                    action = self._env.hold_action()
                    semantic_events = {}

                snapshot = self._driver.step(
                    action,
                    success_verification_mask=success_verification_mask,
                    terminations=terminations,
                    semantic_events=semantic_events,
                )
                if command_batch is not None:
                    completed_commands = torch.nonzero(
                        command_batch.completed_after_step,
                        as_tuple=False,
                    ).flatten().detach().cpu().tolist()
                    for env_id in completed_commands:
                        program = self._programs.get(env_id)
                        if program is not None:
                            program.command_running = False
                self._remove_completed_programs(snapshot.completed)
        except Exception:
            self._executor.abort(tuple(self._programs))
            self._driver.abort()
            raise
        return self._driver.results

    def _prepare_commands(
        self,
        active_mask: Tensor,
    ) -> dict[int, EpisodeTermination]:
        terminations = {}
        active_env_ids = torch.nonzero(
            active_mask,
            as_tuple=False,
        ).flatten().detach().cpu().tolist()
        for env_id in active_env_ids:
            program = self._programs[env_id]
            if program.command_running:
                continue
            try:
                with episode_log_context(
                    episode_id=program.episode_id,
                    env_id=env_id,
                ):
                    command = _next_command(program)
                if command is None:
                    if not program.verification_started:
                        command = Hold(
                            steps=(
                                self._env.task.evaluator_spec.success_stability_steps + 1
                            ),
                            label="verify_success",
                        )
                        program.verification_started = True
                        program.skill_name = "verification"
                        program.subgoal = None
                    else:
                        terminations[env_id] = EpisodeTermination(
                            TerminationReason.CONTROLLER_FINISHED
                        )
                        continue
                self._executor.begin(env_id, command)
                program.command_running = True
            except SkillError as error:
                terminations[env_id] = EpisodeTermination(
                    TerminationReason.SKILL_FAILED,
                    retryable=False,
                    message=str(error),
                )
        return terminations

    def _remove_completed_programs(
        self,
        results: Mapping[str, EpisodeResult],
    ) -> None:
        completed_episode_ids = set(results)
        env_ids = tuple(
            env_id
            for env_id, program in self._programs.items()
            if program.episode_id in completed_episode_ids
        )
        self._executor.abort(env_ids)
        for env_id in env_ids:
            del self._programs[env_id]


def _next_command(program: _ProgramState) -> SkillCommand | None:
    while True:
        if program.commands is not None:
            try:
                return next(program.commands)
            except StopIteration:
                program.commands = None
                program.skill_name = None
                program.subgoal = None
        try:
            request = next(program.expert)
        except StopIteration:
            return None
        if isinstance(request, Pick):
            program.commands = pick(program.context, program.planner, request)
            program.skill_name = "pick"
            program.subgoal = request.object_name
        elif isinstance(request, PickAndPlace):
            program.commands = pick_and_place(
                program.context,
                program.planner,
                request,
            )
            program.skill_name = "pick_and_place"
            program.subgoal = request.object_name
        else:
            raise SkillError(
                f"no skill program is mapped for request {type(request).__name__}"
            )


__all__ = [
    "DemoGenerationRunner",
    "ExpertFactory",
    "SkillContextFactory",
    "SkillPlannerFactory",
]
