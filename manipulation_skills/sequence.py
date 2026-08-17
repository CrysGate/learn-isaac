"""Sequential composition for lazily constructed atomic skills."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from .core import AtomicSkill, SkillStep


SkillFactory = Callable[[], AtomicSkill]


class SkillSequence:
    """Run skill factories in order and stop on the first failure."""

    def __init__(self, factories: Sequence[SkillFactory]) -> None:
        if not factories:
            raise ValueError("SkillSequence requires at least one skill factory")
        self._factories = tuple(factories)
        self._index = 0
        self._active: AtomicSkill | None = None
        self._done = False
        self._succeeded = False
        self.message = ""

    @property
    def done(self) -> bool:
        return self._done

    @property
    def succeeded(self) -> bool:
        return self._succeeded

    def tick(self) -> SkillStep:
        if self._done:
            if self._active is None:
                raise RuntimeError("completed sequence has no terminal skill")
            terminal = self._active.tick()
            return self._wrap(terminal)

        if self._active is None:
            self._active = self._factories[self._index]()

        active_index = self._index
        result = self._active.tick()
        if result.done:
            if not result.succeeded:
                self._done = True
                self.message = result.message
            elif self._index == len(self._factories) - 1:
                self._done = True
                self._succeeded = True
                self.message = result.message
            else:
                self._index += 1
                self._active = None
        return self._wrap(result, active_index)

    def _wrap(self, result: SkillStep, index: int | None = None) -> SkillStep:
        return SkillStep(
            action=result.action,
            phase=f"{self._index if index is None else index}:{result.phase}",
            done=self.done,
            succeeded=self.succeeded,
            message=self.message,
        )


__all__ = ["SkillFactory", "SkillSequence"]
