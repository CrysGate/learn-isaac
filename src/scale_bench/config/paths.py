"""Explicit path semantics for configuration and asset references."""

from __future__ import annotations

from pathlib import Path


def resolve_config_reference(value: str, *, source_path: str | Path) -> str:
    """Resolve a config reference relative to the file containing it."""

    return _resolve_reference(value, base_dir=Path(source_path).parent)


def resolve_asset_reference(
    value: str,
    *,
    source_path: str | Path,
    asset_root: str | Path | None = None,
) -> str:
    """Resolve an asset against an explicit root or the containing config."""

    base_dir = Path(source_path).parent if asset_root is None else Path(asset_root)
    if not base_dir.is_absolute():
        base_dir = Path.cwd() / base_dir
    return _resolve_reference(value, base_dir=base_dir)


def _resolve_reference(value: str, *, base_dir: Path) -> str:
    if Path(value).is_absolute():
        return value
    return str((base_dir / value).resolve())
