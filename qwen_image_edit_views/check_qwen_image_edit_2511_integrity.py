#!/usr/bin/env python3
"""Check local Qwen-Image-Edit-2511 diffusers layout (shards + key configs).

Run from repo root:
  python3 qwen_image_edit_views/check_qwen_image_edit_2511_integrity.py

One-liner (same as running the file):
  python3 -c "import runpy; runpy.run_path('qwen_image_edit_views/check_qwen_image_edit_2511_integrity.py', run_name='__main__')"

Or with env:
  MODEL_DIR=/path/to/Qwen-Image-Edit-2511 python3 qwen_image_edit_views/check_qwen_image_edit_2511_integrity.py
"""

from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path


def check_sharded_dir(component_dir: Path, index_file: str) -> list[str]:
    errs: list[str] = []
    idx_path = component_dir / index_file
    if not idx_path.is_file():
        errs.append(f"missing index: {idx_path}")
        return errs
    try:
        idx = json.loads(idx_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        errs.append(f"invalid JSON {idx_path}: {e}")
        return errs
    wm = idx.get("weight_map") or {}
    shards = sorted(set(wm.values()))
    if not shards:
        errs.append(f"empty weight_map: {idx_path}")
        return errs
    m = re.match(r"^(.+)-(\d+)-of-(\d+)\.safetensors$", shards[0])
    if not m:
        errs.append(f"unrecognized shard name: {shards[0]!r}")
        return errs
    prefix, n = m.group(1), int(m.group(3))
    expected = {f"{prefix}-{i:05d}-of-{n:05d}.safetensors" for i in range(1, n + 1)}
    got = set(shards)
    if got != expected:
        missing = sorted(expected - got)
        extra = sorted(got - expected)
        if missing:
            errs.append(
                f"{component_dir.name}: index missing shard refs: {missing[:6]}{'...' if len(missing) > 6 else ''}"
            )
        if extra:
            errs.append(
                f"{component_dir.name}: unexpected shard names in index: {extra[:6]}{'...' if len(extra) > 6 else ''}"
            )
    total_meta = (idx.get("metadata") or {}).get("total_size")
    sum_sz = 0
    files_ok = True
    for name in sorted(expected):
        fp = component_dir / name
        if not fp.is_file():
            errs.append(f"missing weight file: {fp}")
            files_ok = False
            continue
        sz = fp.stat().st_size
        if sz < 1024 * 1024:
            errs.append(f"suspiciously small: {fp} ({sz} bytes)")
            files_ok = False
        sum_sz += sz
    if total_meta is not None and sum_sz and files_ok:
        tm = int(total_meta)
        if tm > 0 and abs(sum_sz - tm) / tm > 0.01:
            errs.append(
                f"{component_dir.name}: metadata total_size={tm} vs sum(shards)={sum_sz} (>1% diff)"
            )
    return errs


def check_vae(vae_dir: Path) -> list[str]:
    errs: list[str] = []
    single = vae_dir / "diffusion_pytorch_model.safetensors"
    idx = vae_dir / "diffusion_pytorch_model.safetensors.index.json"
    if idx.is_file():
        errs.extend(check_sharded_dir(vae_dir, idx.name))
    elif single.is_file():
        if single.stat().st_size < 1024 * 1024:
            errs.append(f"vae single file too small: {single}")
    else:
        errs.append(f"vae: missing both {single.name} and {idx.name}")
    return errs


def main() -> int:
    root = Path(os.environ.get("MODEL_DIR", "qwen_image_edit_views/models/Qwen-Image-Edit-2511"))
    if not root.is_absolute():
        root = (Path.cwd() / root).resolve()
    errs: list[str] = []

    if not root.is_dir():
        errs.append(f"not a directory: {root}")
        print("\n".join(errs))
        return 1

    mi = root / "model_index.json"
    if not mi.is_file():
        errs.append(f"missing {mi}")
    else:
        try:
            spec = json.loads(mi.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            errs.append(f"model_index.json: {e}")
            spec = {}
        for key in ("processor", "scheduler", "text_encoder", "tokenizer", "transformer", "vae"):
            if key not in spec:
                errs.append(f"model_index.json missing key: {key}")
            elif not (root / key).is_dir():
                errs.append(f"missing component directory: {root / key}")

    for sub, cfg in (
        ("transformer", "config.json"),
        ("text_encoder", "config.json"),
        ("vae", "config.json"),
        ("scheduler", "scheduler_config.json"),
    ):
        p = root / sub / cfg
        if not p.is_file():
            errs.append(f"missing {p}")

    errs.extend(check_sharded_dir(root / "transformer", "diffusion_pytorch_model.safetensors.index.json"))
    errs.extend(check_sharded_dir(root / "text_encoder", "model.safetensors.index.json"))
    errs.extend(check_vae(root / "vae"))

    tok = root / "tokenizer"
    for name in ("tokenizer_config.json", "vocab.json"):
        if not (tok / name).is_file():
            errs.append(f"missing {tok / name}")

    if errs:
        print("FAIL:")
        print("\n".join(errs))
        return 1
    print("OK:", root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
