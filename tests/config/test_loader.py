"""Tests for the unified structured config loader and path semantics."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scale_bench.config.base import (
    AssetReference,
    ConfigReference,
    FrozenModel,
    OptionalAssetReference,
    PositiveInt,
)
from scale_bench.config.loader import ConfigLoadError, load_config


class ReferencesConfig(FrozenModel):
    count: PositiveInt
    config_path: ConfigReference
    asset_path: AssetReference
    optional_asset_path: OptionalAssetReference = None


def test_yaml_paths_resolve_by_declared_reference_kind(tmp_path: Path) -> None:
    config_dir = tmp_path / "configs" / "example"
    config_dir.mkdir(parents=True)
    nested_config = config_dir / "nested.yml"
    nested_config.write_text("value: 1\n", encoding="utf-8")
    asset = config_dir / "mesh.usd"
    asset.write_text("asset", encoding="utf-8")
    source = config_dir / "preset.yml"
    source.write_text(
        "count: 1\n"
        "config_path: nested.yml\n"
        "asset_path: mesh.usd\n",
        encoding="utf-8",
    )

    config = load_config(source, ReferencesConfig)

    assert config.config_path == str(nested_config)
    assert config.asset_path == str(asset)
    assert config.optional_asset_path is None


def test_asset_root_applies_only_to_asset_references(tmp_path: Path) -> None:
    config_dir = tmp_path / "configs"
    asset_root = tmp_path / "assets"
    config_dir.mkdir()
    asset_root.mkdir()
    (config_dir / "nested.yml").write_text("value: 1\n", encoding="utf-8")
    asset = asset_root / "mesh.usd"
    asset.write_text("asset", encoding="utf-8")
    source = config_dir / "preset.json"
    source.write_text(
        json.dumps(
            {
                "count": 2,
                "config_path": "nested.yml",
                "asset_path": "mesh.usd",
            }
        ),
        encoding="utf-8",
    )

    config = load_config(source, ReferencesConfig, asset_root=asset_root)

    assert config.config_path == str(config_dir / "nested.yml")
    assert config.asset_path == str(asset)


def test_absolute_references_are_preserved(tmp_path: Path) -> None:
    absolute_asset = tmp_path / "absolute.usd"
    absolute_asset.write_text("asset", encoding="utf-8")
    optional_asset = tmp_path / "optional.usd"
    optional_asset.write_text("asset", encoding="utf-8")
    absolute_config = tmp_path / "nested.yml"
    absolute_config.write_text("value: 1\n", encoding="utf-8")
    source = tmp_path / "preset.yml"
    source.write_text(
        "count: 1\n"
        f"config_path: {absolute_config}\n"
        f"asset_path: {absolute_asset}\n"
        f"optional_asset_path: {optional_asset}\n",
        encoding="utf-8",
    )

    config = load_config(source, ReferencesConfig)

    assert config.config_path == str(absolute_config)
    assert config.asset_path == str(absolute_asset)
    assert config.optional_asset_path == str(optional_asset)


def test_missing_asset_error_contains_source_and_field_location(tmp_path: Path) -> None:
    source = tmp_path / "missing.yml"
    source.write_text(
        "count: 1\nconfig_path: nested.yml\nasset_path: missing.usd\n",
        encoding="utf-8",
    )

    with pytest.raises(ConfigLoadError) as error:
        load_config(source, ReferencesConfig)

    message = str(error.value)
    assert str(source) in message
    assert "asset_path" in message
    assert "missing.usd" in message


def test_validation_error_contains_source_and_field_location(tmp_path: Path) -> None:
    source = tmp_path / "invalid.yml"
    source.write_text(
        "count: 0\nconfig_path: nested.yml\nasset_path: mesh.usd\n",
        encoding="utf-8",
    )

    with pytest.raises(ConfigLoadError) as error:
        load_config(source, ReferencesConfig)

    message = str(error.value)
    assert str(source) in message
    assert "count" in message


def test_malformed_yaml_and_unsupported_extension_are_wrapped(tmp_path: Path) -> None:
    malformed = tmp_path / "malformed.yml"
    malformed.write_text("value: [\n", encoding="utf-8")
    with pytest.raises(ConfigLoadError, match=str(malformed)):
        load_config(malformed, ReferencesConfig)

    unsupported = tmp_path / "preset.toml"
    unsupported.write_text("count = 1\n", encoding="utf-8")
    with pytest.raises(ConfigLoadError, match="file extension"):
        load_config(unsupported, ReferencesConfig)
