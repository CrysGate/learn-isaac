"""Simulator-independent semantic events sent to episode recorders."""

from dataclasses import dataclass


SEMANTIC_TEXT_BYTES = 64


@dataclass(frozen=True, slots=True)
class StepSemantics:
    """
    Semantic identity of a frame produced by an active skill command.
    """

    skill: str
    command_label: str
    subgoal: str | None

    def __post_init__(self) -> None:
        _validate_semantic_text("skill", self.skill)
        _validate_semantic_text("command_label", self.command_label)
        if self.subgoal is not None:
            _validate_semantic_text("subgoal", self.subgoal)


def _validate_semantic_text(field_name: str, value: str) -> None:
    if not isinstance(value, str):
        raise TypeError(f"{field_name} must be a string")
    if not value:
        raise ValueError(f"{field_name} must be non-empty")
    if len(value.encode("utf-8")) > SEMANTIC_TEXT_BYTES:
        raise ValueError(f"{field_name} must fit in {SEMANTIC_TEXT_BYTES} UTF-8 bytes")


__all__ = ["SEMANTIC_TEXT_BYTES", "StepSemantics"]
