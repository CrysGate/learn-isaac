"""Single YAML/JSON loading, path resolution, and error-wrapping boundary."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, TypeVar

import yaml
from pydantic import BaseModel, ValidationError

from scale_bench.config.paths import resolve_asset_reference, resolve_config_reference


ConfigT = TypeVar("ConfigT", bound=BaseModel)


class ConfigLoadError(ValueError):
    """Configuration failure annotated with the source file path."""

    def __init__(self, source_path: Path, detail: str) -> None:
        self.source_path = source_path
        super().__init__(f"Could not load config {source_path}:\n{detail}")


def load_config(
    config_path: str | Path,
    model_type: type[ConfigT],
    *,
    asset_root: str | Path | None = None,
) -> ConfigT:
    """Read, validate, and resolve one YAML or JSON configuration file."""

    path = Path(config_path)
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()

    try:
        document = _read_document(path)
        model = model_type.model_validate(document)
        return _resolve_model_paths(
            model,
            source_path=path,
            asset_root=asset_root,
        )
    except ConfigLoadError:
        raise
    except (OSError, json.JSONDecodeError, yaml.YAMLError, ValidationError) as error:
        raise ConfigLoadError(path, str(error)) from error


def _read_document(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    suffix = path.suffix.lower()
    if suffix == ".json":
        return json.loads(text)
    if suffix in {".yaml", ".yml"}:
        return yaml.safe_load(text)
    raise ConfigLoadError(path, "file extension must be .json, .yaml, or .yml")


def _resolve_model_paths(
    model: ConfigT,
    *,
    source_path: Path,
    asset_root: str | Path | None,
    location: tuple[str, ...] = (),
) -> ConfigT:
    updates: dict[str, Any] = {}
    for field_name, field in type(model).model_fields.items():
        value = getattr(model, field_name)
        field_location = (*location, field_name)
        metadata = field.json_schema_extra or {}
        path_kind = metadata.get("path_kind")
        if value is not None and path_kind is not None:
            updates[field_name] = _resolve_path_value(
                value,
                path_kind=path_kind,
                source_path=source_path,
                asset_root=asset_root,
                location=field_location,
            )
            continue
        resolved = _resolve_nested_value(
            value,
            source_path=source_path,
            asset_root=asset_root,
            location=field_location,
        )
        if resolved is not value:
            updates[field_name] = resolved
    if not updates:
        return model
    return model.model_copy(update=updates)


def _resolve_nested_value(
    value: Any,
    *,
    source_path: Path,
    asset_root: str | Path | None,
    location: tuple[str, ...],
) -> Any:
    if isinstance(value, BaseModel):
        return _resolve_model_paths(
            value,
            source_path=source_path,
            asset_root=asset_root,
            location=location,
        )
    if isinstance(value, tuple):
        resolved = tuple(
            _resolve_nested_value(
                item,
                source_path=source_path,
                asset_root=asset_root,
                location=(*location, str(index)),
            )
            for index, item in enumerate(value)
        )
        return value if all(a is b for a, b in zip(value, resolved)) else resolved
    if isinstance(value, list):
        return [
            _resolve_nested_value(
                item,
                source_path=source_path,
                asset_root=asset_root,
                location=(*location, str(index)),
            )
            for index, item in enumerate(value)
        ]
    if isinstance(value, dict):
        return {
            key: _resolve_nested_value(
                item,
                source_path=source_path,
                asset_root=asset_root,
                location=(*location, str(key)),
            )
            for key, item in value.items()
        }
    return value


def _resolve_path_value(
    value: str,
    *,
    path_kind: str,
    source_path: Path,
    asset_root: str | Path | None,
    location: tuple[str, ...],
) -> str:
    if path_kind == "config":
        return resolve_config_reference(value, source_path=source_path)
    if path_kind != "asset":
        raise ConfigLoadError(
            source_path,
            f"{'.'.join(location)}: unknown path kind {path_kind!r}",
        )

    resolved = resolve_asset_reference(
        value,
        source_path=source_path,
        asset_root=asset_root,
    )
    if not Path(resolved).is_file():
        raise ConfigLoadError(
            source_path,
            f"{'.'.join(location)}: asset does not exist: {resolved}",
        )
    return resolved
