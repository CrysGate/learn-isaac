"""Human-readable console and structured JSONL logging for ScaleBench runs."""

from __future__ import annotations

import json
import logging
import sys
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

from rich.console import Console
from rich.text import Text
from rich.traceback import Traceback


LogFormat = Literal["pretty", "json"]
LogValue = (
    str
    | int
    | float
    | bool
    | list[object]
    | tuple[object, ...]
    | Mapping[str, object]
)
EventFields = Mapping[str, LogValue]

_LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
}
_LEVEL_STYLES = {
    logging.DEBUG: "dim cyan",
    logging.INFO: "blue",
    logging.WARNING: "bold yellow",
    logging.ERROR: "bold red",
    logging.CRITICAL: "bold white on red",
}
_LEVEL_SYMBOLS = {
    logging.DEBUG: "·",
    logging.INFO: "●",
    logging.WARNING: "▲",
    logging.ERROR: "✖",
    logging.CRITICAL: "✖",
}
_SUCCESS_EVENTS = {"PLAN", "READY", "SUMMARY"}


@dataclass(frozen=True, slots=True)
class EpisodeLogContext:
    """Identifiers shared by every event emitted while advancing one episode."""

    episode_id: str
    env_id: int


_EPISODE_LOG_CONTEXT: ContextVar[EpisodeLogContext | None] = ContextVar(
    "scale_bench_episode_log_context",
    default=None,
)


@contextmanager
def episode_log_context(*, episode_id: str, env_id: int) -> Iterator[None]:
    """Attach episode identity to logs emitted while preparing one env command."""

    token = _EPISODE_LOG_CONTEXT.set(EpisodeLogContext(episode_id, env_id))
    try:
        yield
    finally:
        _EPISODE_LOG_CONTEXT.reset(token)


class _EpisodeContextFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        fields = dict(getattr(record, "event_fields", {}))
        context = _EPISODE_LOG_CONTEXT.get()
        if context is not None:
            fields.setdefault("episode_id", context.episode_id)
            fields.setdefault("env_id", context.env_id)
        record.event_fields = fields
        return True


class _RichEventHandler(logging.Handler):
    """Render a short, consistently ordered event line on the terminal."""

    def __init__(self) -> None:
        super().__init__()
        self._console = Console(stderr=True)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            event = str(getattr(record, "event", "LOG"))
            fields = dict(getattr(record, "event_fields", {}))
            line = Text()
            line.append(
                datetime.fromtimestamp(record.created).strftime("%H:%M:%S"),
                style="dim",
            )
            line.append(" ")
            event_style = (
                "bold green"
                if event in _SUCCESS_EVENTS and record.levelno == logging.INFO
                else _LEVEL_STYLES.get(record.levelno, "bold")
            )
            line.append(_LEVEL_SYMBOLS.get(record.levelno, "●"), style=event_style)
            line.append(" ")
            line.append(f"{event:<8}", style=event_style)
            context = _console_context(event, fields)
            if context:
                line.append(f" {context}", style="dim")
            message = record.getMessage()
            if message:
                line.append(f" {message}")
            self._console.print(line)
            if record.exc_info is not None:
                self._console.print(
                    Traceback.from_exception(*record.exc_info, show_locals=False)
                )
        except Exception:
            self.handleError(record)


class _JsonEventFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, object] = {
            "timestamp": datetime.fromtimestamp(
                record.created,
                tz=timezone.utc,
            ).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "level": record.levelname,
            "logger": record.name,
            "event": str(getattr(record, "event", "log")),
            "message": record.getMessage(),
        }
        payload.update(dict(getattr(record, "event_fields", {})))
        if record.exc_info is not None:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            default=str,
        )


def configure_logging(
    *,
    console_level: str,
    console_format: LogFormat,
    jsonl_path: Path | None,
) -> None:
    """Configure ScaleBench logs for one CLI process.

    ``jsonl_path`` is omitted for interactive runs that need only terminal
    output. Batch and debugging runs pass a path to retain every DEBUG event.
    """

    logger = logging.getLogger("scale_bench")
    logger.handlers.clear()
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    context_filter = _EpisodeContextFilter()
    if console_format == "pretty":
        console_handler: logging.Handler = _RichEventHandler()
    else:
        console_handler = logging.StreamHandler(sys.stderr)
        console_handler.setFormatter(_JsonEventFormatter())
    console_handler.setLevel(_LEVELS[console_level])
    console_handler.addFilter(context_filter)
    logger.addHandler(console_handler)

    if jsonl_path is not None:
        resolved_jsonl_path = jsonl_path.resolve()
        resolved_jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(
            resolved_jsonl_path,
            mode="a",
            encoding="utf-8",
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(_JsonEventFormatter())
        file_handler.addFilter(context_filter)
        logger.addHandler(file_handler)


def _console_context(event: str, fields: Mapping[str, object]) -> str:
    context = []
    if "env_id" in fields:
        context.append(f"env={fields['env_id']}")
    if "object" in fields:
        context.append(str(fields["object"]))
    if "arm" in fields:
        arm = str(fields["arm"])
        context.append({"left": "L", "right": "R"}.get(arm, arm))
    return " ".join(context)


__all__ = [
    "EventFields",
    "LogFormat",
    "configure_logging",
    "episode_log_context",
]
