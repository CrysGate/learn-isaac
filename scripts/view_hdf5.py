"""Inspect ScaleBench HDF5 recordings with a NiceGUI application."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np
from nicegui import app, events, run, ui
from nicegui.element import Element
from nicegui.elements.button import Button
from nicegui.elements.echart import EChart
from nicegui.elements.label import Label

BLOCK_SIZE = 16
TILE_WIDTH = 320
CAMERA_ORDER = ("left_robot", "overhead", "right_robot")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="HDF5 recording to open")
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="Local HTTP port; change it when 8765 is already occupied.",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return args


def _natural_key(value: str) -> list[object]:
    return [
        int(part) if part.isdigit() else part.lower()
        for part in re.split(r"(\d+)", value)
    ]


def _recording_fps(data_group: h5py.Group) -> float:
    raw_env_args = data_group.attrs.get("env_args", "")
    if isinstance(raw_env_args, bytes):
        raw_env_args = raw_env_args.decode("utf-8", errors="replace")
    if isinstance(raw_env_args, str):
        try:
            sim_args = json.loads(raw_env_args).get("sim_args", {})
            dt = float(sim_args["dt"])
            decimation = int(sim_args["decimation"])
            if dt > 0.0 and decimation > 0:
                return 1.0 / (dt * decimation)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass
    return 30.0


def _json_value(value: Any) -> Any:
    """Convert the scalar, array, and string types permitted by HDF5."""
    if isinstance(value, np.ndarray):
        if value.dtype == np.uint8 and value.ndim >= 1:
            payload = value.reshape(-1).tobytes().rstrip(b"\x00")
            try:
                decoded = payload.decode("utf-8")
                if not decoded or all(
                    char.isprintable() or char in "\r\n\t" for char in decoded
                ):
                    return decoded
            except UnicodeDecodeError:
                pass
        if value.dtype.kind == "S":
            decoded = np.char.decode(value, "utf-8", errors="replace")
            return decoded.item() if decoded.ndim == 0 else decoded.tolist()
        if value.ndim == 0:
            return _json_value(value.item())
        return [_json_value(item) for item in value]
    if isinstance(value, np.generic):
        return _json_value(value.item())
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").rstrip("\x00")
    if isinstance(value, str):
        stripped = value.strip()
        if stripped.startswith(("{", "[")):
            try:
                return json.loads(stripped)
            except json.JSONDecodeError:
                pass
        return value
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    if isinstance(value, (bool, int)):
        return value
    return str(value)


def _attrs(group: h5py.Group) -> dict[str, Any]:
    return {name: _json_value(group.attrs[name]) for name in sorted(group.attrs)}


def _depth_range(dataset: h5py.Dataset, frame_count: int) -> tuple[float, float]:
    sample_count = min(BLOCK_SIZE, frame_count)
    start = max(0, (frame_count - sample_count) // 2)
    depth = np.asarray(dataset[start : start + sample_count])
    valid = depth[np.isfinite(depth) & (depth > 0.0)]
    if not valid.size:
        return 0.0, 1.0
    minimum, maximum = np.percentile(valid, (1.0, 99.0))
    if minimum >= maximum:
        maximum = minimum + 1.0
    return float(minimum), float(maximum)


def _episode_frame_count(episode: h5py.Group, observations: h5py.Group) -> int:
    if "num_samples" not in episode.attrs:
        raise ValueError(f"{episode.name} has no num_samples attribute")
    frame_count = int(episode.attrs["num_samples"])
    if frame_count > 0:
        return frame_count
    observation_lengths: set[int] = set()

    def collect_length(_relative_path: str, node: h5py.Dataset | h5py.Group) -> None:
        if isinstance(node, h5py.Dataset) and node.ndim > 0 and node.shape[0] > 0:
            observation_lengths.add(int(node.shape[0]))

    observations.visititems(collect_length)
    if len(observation_lengths) == 1:
        return observation_lengths.pop()
    if not observation_lengths:
        raise ValueError(f"episode {episode.name!r} contains no frames")
    raise ValueError(
        f"{episode.name} has inconsistent observation lengths: "
        + ", ".join(str(length) for length in sorted(observation_lengths))
    )


class HDF5Viewer:
    """Read recording metadata and lazily prepare browser-friendly media."""

    def __init__(self, recording_path: Path):
        self.recording_path = recording_path
        self._temporary_directory = tempfile.TemporaryDirectory(prefix="hdf5-viewer-")
        self.video_directory = Path(self._temporary_directory.name)
        self.video_cache_token = os.urandom(8).hex()
        self._episodes: dict[str, dict[str, Any]] = {}
        self._states_cache: dict[str, list[dict[str, Any]]] = {}
        self._states_lock = threading.Lock()
        self.metadata = self._inspect_recording()
        if any(episode["cameras"] for episode in self._episodes.values()):
            ffmpeg_path = shutil.which("ffmpeg")
            if not ffmpeg_path:
                raise RuntimeError(
                    "ffmpeg is required to prepare recordings that contain cameras"
                )
            self._ffmpeg_path = ffmpeg_path
        self._video_locks = {
            episode_name: threading.Lock() for episode_name in self._episodes
        }
        for index, episode_info in enumerate(self._episodes.values()):
            if episode_info["cameras"]:
                episode_info["video_path"] = (
                    self.video_directory / f"episode-{index}.mp4"
                )

    def close(self) -> None:
        self._temporary_directory.cleanup()

    def _inspect_recording(self) -> dict[str, Any]:
        with h5py.File(self.recording_path, "r") as recording:
            if "data" not in recording or not isinstance(recording["data"], h5py.Group):
                raise ValueError("the recording has no HDF5 group at /data")
            data_group = recording["data"]
            fps = _recording_fps(data_group)
            public_episodes: list[dict[str, Any]] = []
            for episode_name in sorted(data_group.keys(), key=_natural_key):
                episode = data_group[episode_name]
                if not isinstance(episode, h5py.Group):
                    continue
                episode_info = self._inspect_episode(episode_name, episode, fps)
                self._episodes[episode_name] = episode_info
                public_episodes.append(episode_info["public"])
            if not public_episodes:
                raise ValueError("the /data group contains no viewable episodes")
            return {
                "file": {
                    "name": self.recording_path.name,
                    "path": str(self.recording_path),
                    "size_bytes": self.recording_path.stat().st_size,
                },
                "fps": fps,
                "data_attrs": _attrs(data_group),
                "root_attrs": _attrs(recording),
                "episodes": public_episodes,
            }

    def _inspect_episode(
        self,
        episode_name: str,
        episode: h5py.Group,
        fps: float,
    ) -> dict[str, Any]:
        observations = episode.get("obs")
        if observations is None:
            raise ValueError(f"/data/{episode_name} has no observation group")
        if not isinstance(observations, h5py.Group):
            raise TypeError(f"/data/{episode_name}/obs is not an HDF5 group")
        frame_count = _episode_frame_count(episode, observations)
        rgb_camera_names = {
            key.removesuffix("_camera_rgb")
            for key in observations
            if key.endswith("_camera_rgb")
        }
        depth_camera_names = {
            key.removesuffix("_camera_depth")
            for key in observations
            if key.endswith("_camera_depth")
        }
        if rgb_camera_names != depth_camera_names:
            missing_depth = sorted(
                rgb_camera_names - depth_camera_names, key=_natural_key
            )
            missing_rgb = sorted(
                depth_camera_names - rgb_camera_names, key=_natural_key
            )
            details = []
            if missing_depth:
                details.append("missing depth for " + ", ".join(missing_depth))
            if missing_rgb:
                details.append("missing RGB for " + ", ".join(missing_rgb))
            raise ValueError(
                f"{observations.name} has unpaired cameras: {'; '.join(details)}"
            )
        camera_names = sorted(
            rgb_camera_names,
            key=lambda name: (
                CAMERA_ORDER.index(name) if name in CAMERA_ORDER else len(CAMERA_ORDER),
                _natural_key(name),
            ),
        )

        cameras: list[dict[str, Any]] = []
        image_paths: set[str] = set()
        for camera_name in camera_names:
            rgb_path = f"obs/{camera_name}_camera_rgb"
            depth_path = f"obs/{camera_name}_camera_depth"
            rgb = episode[rgb_path]
            depth = episode[depth_path]
            if rgb.ndim != 4 or rgb.shape[0] != frame_count or rgb.shape[-1] != 3:
                raise ValueError(f"unexpected RGB shape at {rgb.name}: {rgb.shape}")
            if depth.ndim not in (3, 4) or depth.shape[0] != frame_count:
                raise ValueError(
                    f"unexpected depth shape at {depth.name}: {depth.shape}"
                )
            if depth.ndim == 4 and depth.shape[-1] != 1:
                raise ValueError(
                    f"unexpected depth channels at {depth.name}: {depth.shape}"
                )
            if rgb.shape[1:3] != depth.shape[1:3]:
                raise ValueError(f"RGB and depth resolution differ for {camera_name}")
            minimum, maximum = _depth_range(depth, frame_count)
            cameras.append(
                {
                    "name": camera_name,
                    "rgb_path": rgb_path,
                    "depth_path": depth_path,
                    "width": int(rgb.shape[2]),
                    "height": int(rgb.shape[1]),
                    "depth_min": minimum,
                    "depth_max": maximum,
                }
            )
            image_paths.update((rgb_path, depth_path))

        frame_fields: list[dict[str, Any]] = []
        static_fields: dict[str, dict[str, Any]] = {}

        def inspect_dataset(
            relative_path: str, node: h5py.Dataset | h5py.Group
        ) -> None:
            if not isinstance(node, h5py.Dataset) or relative_path in image_paths:
                return
            info = {"dtype": str(node.dtype), "shape": list(node.shape)}
            is_episode_level = relative_path.startswith(
                ("initial_state/", "termination/")
            )
            if node.ndim > 0 and node.shape[0] == frame_count and not is_episode_level:
                frame_fields.append({"path": relative_path, **info})
            else:
                static_fields[relative_path] = {
                    **info,
                    "value": _json_value(np.asarray(node[()])),
                }

        episode.visititems(inspect_dataset)
        frame_fields.sort(key=lambda item: _natural_key(item["path"]))
        static_fields = dict(
            sorted(static_fields.items(), key=lambda item: _natural_key(item[0]))
        )
        joint_fields = [
            {"path": field["path"], "joint_count": field["shape"][1]}
            for field in frame_fields
            if field["path"].startswith("obs/")
            and field["path"].endswith("_joint_pos")
            and len(field["shape"]) == 2
            and field["shape"][1] > 0
            and np.issubdtype(episode[field["path"]].dtype, np.number)
        ]
        attrs = _attrs(episode)
        public = {
            "name": episode_name,
            "frame_count": frame_count,
            "duration_seconds": frame_count / fps,
            "success": bool(attrs.get("success", False)),
            "attrs": attrs,
            "cameras": cameras,
            "joint_fields": joint_fields,
            "frame_fields": frame_fields,
            "static_fields": static_fields,
        }
        episode_info = {
            "public": public,
            "frame_count": frame_count,
            "cameras": cameras,
            "frame_paths": [field["path"] for field in frame_fields],
        }
        if cameras:
            first_rgb = observations[f"{camera_names[0]}_camera_rgb"]
            tile_height = max(
                2,
                round(TILE_WIDTH * first_rgb.shape[1] / first_rgb.shape[2]) // 2 * 2,
            )
            episode_info.update(
                tile_height=tile_height,
                video_width=TILE_WIDTH * len(cameras),
                video_height=tile_height * 2,
            )
        return episode_info

    def states(self, episode_name: str) -> list[dict[str, Any]]:
        if episode_name not in self._episodes:
            raise KeyError(f"unknown episode: {episode_name}")
        with self._states_lock:
            cached = self._states_cache.get(episode_name)
            if cached is not None:
                return cached
        episode_info = self._episodes[episode_name]
        with h5py.File(self.recording_path, "r") as recording:
            episode = recording[f"data/{episode_name}"]
            state_arrays = {
                path: np.asarray(episode[path][()])
                for path in episode_info["frame_paths"]
            }
        frames = [
            {path: _json_value(values[index]) for path, values in state_arrays.items()}
            for index in range(int(episode_info["frame_count"]))
        ]
        with self._states_lock:
            cached = self._states_cache.get(episode_name)
            if cached is not None:
                return cached
            self._states_cache[episode_name] = frames
            return frames

    def prepare_video(self, episode_name: str) -> Path:
        if episode_name not in self._episodes:
            raise KeyError(f"unknown episode: {episode_name}")
        episode_info = self._episodes[episode_name]
        if not episode_info["cameras"]:
            raise ValueError(f"episode {episode_name!r} has no camera observations")
        output_path = episode_info["video_path"]
        with self._video_locks[episode_name]:
            if output_path.is_file():
                return output_path
            self._encode_video(episode_name, episode_info, output_path)
        return output_path

    def _encode_video(
        self,
        episode_name: str,
        episode_info: dict[str, Any],
        output_path: Path,
    ) -> None:
        temporary_path = output_path.with_suffix(".part.mp4")
        fps = float(self.metadata["fps"])
        keyframe_interval = max(1, round(fps))
        command = [
            self._ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "bgr24",
            "-video_size",
            f"{episode_info['video_width']}x{episode_info['video_height']}",
            "-framerate",
            f"{fps:.12g}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "21",
            "-g",
            str(keyframe_interval),
            "-keyint_min",
            str(keyframe_interval),
            "-sc_threshold",
            "0",
            "-bf",
            "0",
            "-pix_fmt",
            "yuv420p",
            "-fps_mode",
            "cfr",
            "-movflags",
            "+faststart",
            str(temporary_path),
        ]
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        if process.stdin is None or process.stderr is None:
            process.kill()
            raise RuntimeError("failed to open ffmpeg pipes")
        try:
            with h5py.File(self.recording_path, "r") as recording:
                episode = recording[f"data/{episode_name}"]
                for start in range(0, int(episode_info["frame_count"]), BLOCK_SIZE):
                    stop = min(start + BLOCK_SIZE, int(episode_info["frame_count"]))
                    image_blocks = [
                        (
                            np.asarray(episode[camera["rgb_path"]][start:stop]),
                            np.asarray(episode[camera["depth_path"]][start:stop]),
                        )
                        for camera in episode_info["cameras"]
                    ]
                    for offset in range(stop - start):
                        sheet = self._render_sheet(
                            image_blocks,
                            episode_info["cameras"],
                            int(episode_info["tile_height"]),
                            offset,
                        )
                        process.stdin.write(sheet.tobytes())
            process.stdin.close()
            error_output = process.stderr.read().decode("utf-8", errors="replace")
            return_code = process.wait()
            if return_code != 0:
                raise RuntimeError(
                    error_output.strip() or f"ffmpeg exited with {return_code}"
                )
            os.replace(temporary_path, output_path)
        except Exception:
            if process.poll() is None:
                process.kill()
                process.wait()
            temporary_path.unlink(missing_ok=True)
            raise

    @staticmethod
    def _render_sheet(
        image_blocks: list[tuple[np.ndarray, np.ndarray]],
        cameras: list[dict[str, Any]],
        tile_height: int,
        offset: int,
    ) -> np.ndarray:
        sheet = np.zeros(
            (tile_height * 2, TILE_WIDTH * len(image_blocks), 3), dtype=np.uint8
        )
        for camera_index, ((rgb_frames, depth_frames), camera) in enumerate(
            zip(image_blocks, cameras, strict=True)
        ):
            rgb = rgb_frames[offset]
            if rgb.dtype != np.uint8:
                rgb = np.clip(rgb, 0, 255).astype(np.uint8)
            rgb_bgr = cv2.resize(
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                (TILE_WIDTH, tile_height),
                interpolation=cv2.INTER_AREA,
            )
            left = camera_index * TILE_WIDTH
            sheet[:tile_height, left : left + TILE_WIDTH] = rgb_bgr
            depth = depth_frames[offset]
            if depth.ndim == 3:
                depth = depth[..., 0]
            valid = np.isfinite(depth) & (depth > 0.0)
            minimum = float(camera["depth_min"])
            maximum = float(camera["depth_max"])
            normalized = np.clip(
                (np.where(valid, depth, maximum) - minimum) / (maximum - minimum),
                0.0,
                1.0,
            )
            depth_u8 = np.round((1.0 - normalized) * 255.0).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
            depth_color[~valid] = 0
            depth_color = cv2.resize(
                depth_color,
                (TILE_WIDTH, tile_height),
                interpolation=cv2.INTER_AREA,
            )
            sheet[tile_height:, left : left + TILE_WIDTH] = depth_color
        return sheet


STYLES = """
:root {
  --viewer-header-height: 60px;
  --viewer-toolbar-height: 76px;
  --viewer-bg: #f5f6f7;
  --viewer-surface: #ffffff;
  --viewer-surface-alt: #f0f3f4;
  --viewer-ink: #172126;
  --viewer-muted: #66737a;
  --viewer-faint: #8b969c;
  --viewer-line: #d9dfe2;
  --viewer-line-strong: #c6ced2;
  --viewer-accent: #087c74;
  --viewer-accent-soft: #e5f3f1;
  --viewer-focus: #e49b18;
  --viewer-dark: #1d2529;
}
* { letter-spacing: 0; }
html { background: var(--viewer-bg); }
body {
  margin: 0; background: var(--viewer-bg); color: var(--viewer-ink);
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont,
    "Segoe UI", sans-serif;
}
.nicegui-content { padding: 0; gap: 0; }
.viewer-header {
  min-height: var(--viewer-header-height); padding: 0 22px;
  background: var(--viewer-dark); color: #f7f9fa;
  display: flex; align-items: center; gap: 16px;
  border-bottom: 1px solid #303b40;
}
.header-identity { min-width: 0; flex-wrap: nowrap; align-items: center; gap: 10px; }
.brand-mark {
  width: 32px; height: 32px; flex: 0 0 32px; display: grid; place-items: center;
  border-radius: 6px; background: var(--viewer-accent); color: #ffffff;
}
.brand-mark .q-icon { font-size: 19px; }
.header-copy { min-width: 0; gap: 0; }
.viewer-brand {
  font-size: 12px; line-height: 1.2; font-weight: 800; text-transform: uppercase;
  white-space: nowrap;
}
.viewer-file {
  max-width: min(42vw, 520px); color: #b7c1c5; font-size: 11px;
  overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
}
.header-facts { margin-left: auto; flex-wrap: nowrap; align-items: center; gap: 0; }
.header-fact {
  padding: 0 12px; border-left: 1px solid #3a454a; color: #aeb9bd;
  font-size: 10px; white-space: nowrap;
}
.viewer-status {
  display: inline-flex; align-items: center; gap: 7px; min-width: 54px;
  font-size: 11px; font-weight: 750; white-space: nowrap;
}
.viewer-status::before {
  content: ""; width: 7px; height: 7px; border-radius: 50%; background: currentColor;
  box-shadow: 0 0 0 3px color-mix(in srgb, currentColor 18%, transparent);
}
.viewer-status.success { color: #62c99b; }
.viewer-status.failure { color: #ee8c7d; }
.header-action { color: #c8d0d3; }
.control-strip {
  width: 100%; min-height: var(--viewer-toolbar-height); padding: 12px 22px;
  background: rgba(255, 255, 255, 0.98);
  border-bottom: 1px solid var(--viewer-line); display: grid;
  grid-template-columns: minmax(280px, 350px) 124px minmax(260px, 1fr) 218px 96px;
  align-items: center; gap: 14px; position: sticky;
  top: var(--viewer-header-height); z-index: 1000;
  box-shadow: 0 4px 14px rgba(24, 35, 40, 0.05);
}
.episode-select .q-field__control,
.frame-input .q-field__control { border-radius: 6px; }
.transport { display: flex; flex-wrap: nowrap; align-items: center; justify-content: center; gap: 4px; }
.transport .q-btn { width: 36px; height: 36px; }
.transport .play-button { width: 40px; height: 40px; }
.timeline-group {
  min-width: 0; display: grid; grid-template-columns: minmax(120px, 1fr) 122px;
  align-items: center; gap: 12px;
}
.timecode {
  color: var(--viewer-muted); font: 11px/1.3 "SFMono-Regular", Consolas, monospace;
  text-align: right; white-space: nowrap;
}
.speed-toggle { min-width: 0; border: 1px solid var(--viewer-line); border-radius: 6px; overflow: hidden; }
.speed-toggle .q-btn { min-width: 40px; height: 36px; padding: 0 7px; font-size: 11px; }
.frame-input { width: 96px; }
.workspace {
  width: min(100%, 1920px); min-height: calc(100vh - 136px); margin: 0 auto;
  display: grid; grid-template-columns: minmax(0, 1fr) 392px;
  grid-template-areas: "media inspector" "tracks inspector"; align-items: start;
}
.media-pane { grid-area: media; min-width: 0; padding: 20px 22px 0; }
.tracks-pane { grid-area: tracks; min-width: 0; padding: 0 22px 48px; }
.episode-context {
  min-height: 54px; margin-bottom: 12px; display: flex; align-items: center;
  justify-content: space-between; gap: 18px;
}
.episode-copy { min-width: 0; gap: 1px; }
.context-kicker { color: var(--viewer-muted); font-size: 10px; font-weight: 750; text-transform: uppercase; }
.episode-title {
  max-width: 45vw; font: 750 15px/1.35 "SFMono-Regular", Consolas, monospace;
  overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
}
.overview-metrics { flex: 0 0 auto; display: grid; grid-template-columns: repeat(4, auto); }
.metric-item { min-width: 72px; padding: 0 14px; border-left: 1px solid var(--viewer-line); }
.metric-label { color: var(--viewer-muted); font-size: 9px; font-weight: 700; text-transform: uppercase; }
.metric-value { margin-top: 1px; color: var(--viewer-ink); font: 750 12px/1.3 "SFMono-Regular", Consolas, monospace; }
.media-stage {
  width: 100%; position: relative; overflow: hidden; border-radius: 6px;
  transition: min-height 160ms ease, background-color 160ms ease, border-color 160ms ease;
}
.media-stage.has-media { min-height: 240px; background: #111719; border: 1px solid #344147; }
.media-stage.no-media { min-height: 132px; background: var(--viewer-surface); border: 1px dashed var(--viewer-line-strong); }
.media-stage video {
  width: 100%; height: auto; max-height: 68vh; object-fit: contain;
  display: block; background: #0c1113;
}
.loading-line {
  position: absolute; inset: 0; z-index: 2; justify-content: center;
  background: rgba(248, 250, 250, 0.94); color: var(--viewer-muted);
}
.has-media .loading-line { background: rgba(17, 23, 25, 0.94); color: #e5edef; }
.no-camera { min-height: 130px; justify-content: center; gap: 4px; color: var(--viewer-muted); }
.no-camera .q-icon { color: var(--viewer-faint); }
.no-camera-title { color: #445159; font-size: 13px; font-weight: 750; }
.no-camera-caption { color: var(--viewer-faint); font-size: 10px; }
.camera-legend { display: grid; gap: 1px; background: #344147; border-top: 1px solid #344147; }
.camera-label { min-width: 0; padding: 9px 12px; background: #20292d; color: #edf3f4; }
.camera-name { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; font-size: 12px; font-weight: 700; }
.camera-meta { color: #a5b3b8; font-size: 10px; }
.state-pane {
  grid-area: inspector; min-width: 0; height: calc(100vh - var(--viewer-header-height) - var(--viewer-toolbar-height));
  position: sticky; top: calc(var(--viewer-header-height) + var(--viewer-toolbar-height));
  display: flex; flex-direction: column; overflow: hidden;
  border-left: 1px solid var(--viewer-line); background: var(--viewer-surface);
}
.inspector-head {
  min-height: 54px; padding: 0 16px; display: flex; align-items: center; gap: 10px;
  border-bottom: 1px solid var(--viewer-line);
}
.inspector-icon { color: var(--viewer-accent); }
.inspector-title { font-size: 13px; font-weight: 800; }
.inspector-frame {
  margin-left: auto; color: var(--viewer-muted);
  font: 700 10px "SFMono-Regular", Consolas, monospace;
}
.state-tabs { flex: 0 0 auto; border-bottom: 1px solid var(--viewer-line); }
.state-tabs .q-tab { min-height: 44px; font-size: 11px; }
.state-panels {
  flex: 1 1 auto; min-height: 0; overflow-x: hidden; overflow-y: auto;
  background: transparent;
}
.state-panels .q-panel, .state-panels .q-tab-panel { min-height: 100%; }
.state-panels .q-tab-panel { padding: 14px 16px 22px; }
.inspector-filter { margin-bottom: 8px; }
.field-count { margin: 8px 2px 3px; color: var(--viewer-faint); font-size: 9px; text-transform: uppercase; }
.section-head {
  margin-top: 26px; min-height: 48px; padding-bottom: 10px;
  border-bottom: 1px solid var(--viewer-line-strong); display: flex;
  align-items: end; gap: 12px;
}
.section-title { font-size: 15px; font-weight: 800; }
.section-summary { margin-left: auto; color: var(--viewer-muted); font-size: 11px; }
.joint-group { padding: 18px 0 22px; border-bottom: 1px solid var(--viewer-line); }
.joint-group-head { display: flex; align-items: baseline; gap: 12px; }
.joint-group-title { font-size: 13px; font-weight: 750; }
.joint-group-path { color: var(--viewer-muted); font: 10px "SFMono-Regular", Consolas, monospace; overflow-wrap: anywhere; }
.joint-grid { margin-top: 12px; display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 280px), 1fr)); gap: 10px; }
.joint-plot {
  overflow: hidden; border: 1px solid var(--viewer-line); border-radius: 6px;
  background: var(--viewer-surface); box-shadow: 0 1px 2px rgba(25, 37, 42, 0.04);
}
.joint-plot-head { min-height: 38px; padding: 8px 12px 0; display: flex; align-items: center; gap: 8px; }
.joint-label { font-size: 10px; font-weight: 750; text-transform: uppercase; }
.joint-value { margin-left: auto; color: var(--viewer-accent); font: 750 11px "SFMono-Regular", Consolas, monospace; }
.joint-chart { width: 100%; height: 164px; }
.data-section-title { margin: 4px 0 6px; color: #3b484f; font-size: 11px; font-weight: 800; text-transform: uppercase; }
.data-row { padding: 11px 0 12px; border-bottom: 1px solid #e8ecee; }
.data-path { color: #334148; font: 700 10px "SFMono-Regular", Consolas, monospace; overflow-wrap: anywhere; }
.data-meta { margin-top: 2px; color: var(--viewer-faint); font: 9px "SFMono-Regular", Consolas, monospace; }
.data-value {
  max-height: 144px; margin-top: 7px; padding: 7px 8px; overflow: auto;
  border-left: 2px solid #c8d4d6; background: #f5f7f8; color: #263137;
  font: 10px/1.5 "SFMono-Regular", Consolas, monospace;
  white-space: pre-wrap; overflow-wrap: anywhere;
}
.empty-copy { padding: 28px 8px; color: var(--viewer-muted); font-size: 12px; text-align: center; }
.meta-dialog { width: min(760px, 92vw); max-height: 84vh; border-radius: 6px; }
.meta-dialog-head { padding-bottom: 10px; border-bottom: 1px solid var(--viewer-line); }
.meta-dialog-title { font-size: 14px; font-weight: 800; }
.meta-scroll { width: 100%; max-height: 64vh; overflow: auto; }
@media (max-width: 1280px) {
  :root { --viewer-toolbar-height: 112px; }
  .control-strip { grid-template-columns: minmax(260px, 1fr) 124px 218px 96px; }
  .timeline-group { grid-column: 1 / -1; grid-row: 2; }
}
@media (max-width: 1040px) {
  .workspace {
    grid-template-columns: minmax(0, 1fr);
    grid-template-areas: "media" "inspector" "tracks";
  }
  .state-pane {
    height: auto; max-height: 560px; position: static; border-left: 0;
    border-top: 1px solid var(--viewer-line); border-bottom: 1px solid var(--viewer-line);
  }
  .state-panels { max-height: 450px; }
  .episode-title { max-width: 52vw; }
}
@media (max-width: 760px) {
  :root { --viewer-header-height: 58px; --viewer-toolbar-height: 166px; }
  .viewer-header { padding: 0 12px; gap: 10px; }
  .brand-mark { width: 30px; height: 30px; flex-basis: 30px; }
  .viewer-brand { font-size: 11px; }
  .viewer-file { max-width: 42vw; font-size: 10px; }
  .header-facts { display: none; }
  .viewer-status { margin-left: auto; min-width: 48px; font-size: 10px; }
  .control-strip {
    padding: 10px 12px 12px; grid-template-columns: minmax(0, 1fr) auto;
    grid-template-rows: 42px 40px 42px; gap: 7px 10px;
  }
  .episode-select { grid-column: 1 / -1; grid-row: 1; }
  .transport { grid-column: 1; grid-row: 2; justify-content: flex-start; }
  .speed-toggle { grid-column: 2; grid-row: 2; }
  .speed-toggle .q-btn { min-width: 34px; padding: 0 5px; }
  .timeline-group { grid-column: 1 / -1; grid-row: 3; grid-template-columns: minmax(100px, 1fr) 126px; gap: 8px; }
  .timecode { font-size: 10px; }
  .frame-input { display: none; }
  .media-pane { padding: 14px 12px 0; }
  .tracks-pane { padding: 0 12px 36px; }
  .episode-context { min-height: 48px; margin-bottom: 10px; }
  .episode-title { max-width: 58vw; font-size: 12px; }
  .overview-metrics { grid-template-columns: repeat(2, auto); }
  .metric-item { min-width: 58px; padding: 0 9px; }
  .metric-item:nth-child(-n+2) { display: none; }
  .joint-chart { height: 154px; }
  .inspector-head { padding: 0 12px; }
  .state-panels .q-tab-panel { padding: 12px 12px 20px; }
}
@media (max-width: 420px) {
  .viewer-file { max-width: 36vw; }
  .viewer-status { min-width: 42px; }
  .speed-toggle .q-btn { min-width: 32px; padding: 0 4px; }
  .timeline-group { grid-template-columns: minmax(92px, 1fr) 126px; }
}
"""


def _format_bytes(size_bytes: int) -> str:
    value = float(size_bytes)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if value < 1024.0 or unit == "TB":
            return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
        value /= 1024.0
    raise AssertionError("unreachable")


def _display_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.8g}"
    if isinstance(value, str):
        return value
    return json.dumps(value, ensure_ascii=False, separators=(", ", ": "))


def _timecode(frame: int, fps: float) -> str:
    seconds = frame / fps
    minutes, seconds = divmod(seconds, 60.0)
    return f"{int(minutes):02d}:{seconds:05.2f}"


class ViewerPage:
    """Own all NiceGUI elements and playback state for one browser client."""

    def __init__(self, viewer: HDF5Viewer):
        self.viewer = viewer
        self.fps = float(viewer.metadata["fps"])
        self.episode = viewer.metadata["episodes"][0]
        self.states: list[dict[str, Any]] = []
        self.frame = 0
        self.speed = 1.0
        self.playing = False
        self.ready = False
        self.generation = 0
        self.clock_frame = 0
        self.clock_started_at = time.monotonic()
        self.state_value_labels: dict[str, Label] = {}
        self.state_rows: list[tuple[str, Element]] = []
        self.joint_charts: list[tuple[EChart, Label, list[float]]] = []
        self._build()
        ui.timer(0.05, self._tick_data_playback)
        ui.timer(0.0, self._load_initial_episode, once=True)

    def _build(self) -> None:
        ui.colors(
            primary="#087c74",
            secondary="#4f6570",
            accent="#e49b18",
            positive="#18855f",
            negative="#ba4a43",
            warning="#d58b12",
        )
        ui.add_css(STYLES)
        file_info = self.viewer.metadata["file"]
        with ui.header().classes("viewer-header"):
            with ui.row().classes("header-identity"):
                with ui.element("div").classes("brand-mark"):
                    ui.icon("dataset")
                with ui.column().classes("header-copy"):
                    ui.label("HDF5 Inspector").classes("viewer-brand")
                    ui.label(file_info["name"]).classes("viewer-file").tooltip(
                        file_info["path"]
                    )
            with ui.row().classes("header-facts"):
                ui.label(_format_bytes(int(file_info["size_bytes"]))).classes(
                    "header-fact"
                )
                ui.label(
                    f"{len(self.viewer.metadata['episodes'])} Episodes"
                ).classes("header-fact")
            self.status_badge = ui.label("").classes("viewer-status")
            self._build_metadata_dialog()

        episode_options = {
            episode["name"]: (
                f"{episode['name']} · {episode['frame_count']} 帧 · "
                f"{'成功' if episode['success'] else '未成功'}"
            )
            for episode in self.viewer.metadata["episodes"]
        }
        with ui.element("section").classes("control-strip"):
            self.episode_select = (
                ui.select(
                    episode_options,
                    value=self.episode["name"],
                    label="Episode",
                    on_change=self._on_episode_change,
                )
                .props("dense outlined options-dense")
                .classes("episode-select w-full")
            )
            with ui.row().classes("transport"):
                self.previous_button = self._icon_button(
                    "skip_previous", self._previous, "上一帧 · ←"
                )
                self.play_button = ui.button(
                    icon="play_arrow", on_click=self._toggle_playback
                ).props("unelevated round dense color=primary").classes("play-button")
                with self.play_button:
                    self.play_tooltip = ui.tooltip("播放 · 空格")
                self.next_button = self._icon_button(
                    "skip_next", self._next, "下一帧 · →"
                )
            with ui.element("div").classes("timeline-group"):
                self.timeline = (
                    ui.slider(
                        min=0,
                        max=self.episode["frame_count"] - 1,
                        step=1,
                        value=0,
                        on_change=self._on_timeline_change,
                    )
                    .props("color=primary track-size=3px thumb-size=14px")
                    .classes("w-full")
                )
                self.time_label = ui.label("").classes("timecode")
            self.speed_toggle = (
                ui.toggle(
                    {
                        0.25: ".25x",
                        0.5: ".5x",
                        1.0: "1x",
                        1.5: "1.5x",
                        2.0: "2x",
                    },
                    value=1.0,
                    on_change=self._on_speed_change,
                )
                .props("unelevated no-caps spread color=primary")
                .classes("speed-toggle")
            )
            self.frame_input = (
                ui.number(
                    label="帧",
                    value=0,
                    min=0,
                    max=self.episode["frame_count"] - 1,
                    precision=0,
                    on_change=self._on_frame_input,
                )
                .props("dense outlined")
                .classes("frame-input")
            )

        with ui.element("main").classes("workspace"):
            with ui.element("section").classes("media-pane"):
                with ui.element("div").classes("episode-context"):
                    with ui.column().classes("episode-copy"):
                        ui.label("当前 Episode").classes("context-kicker")
                        self.episode_title = ui.label("").classes("episode-title")
                    with ui.element("div").classes("overview-metrics"):
                        self.metric_values: dict[str, Label] = {}
                        for metric_name, metric_label in (
                            ("frames", "帧数"),
                            ("duration", "时长"),
                            ("fps", "FPS"),
                            ("cameras", "相机"),
                        ):
                            with ui.element("div").classes("metric-item"):
                                ui.label(metric_label).classes("metric-label")
                                self.metric_values[metric_name] = ui.label("").classes(
                                    "metric-value"
                                )
                with ui.element("div").classes("media-stage") as self.media_stage:
                    self.video = ui.video("", controls=False).classes("w-full")
                    self.video.on(
                        "timeupdate",
                        js_handler="(event) => emitEvent('hdf5_video_time', event.target.currentTime)",
                    )
                    self.video.on(
                        "play", js_handler="() => emitEvent('hdf5_video_play')"
                    )
                    self.video.on(
                        "pause", js_handler="() => emitEvent('hdf5_video_pause')"
                    )
                    self.video.on(
                        "ended", js_handler="() => emitEvent('hdf5_video_ended')"
                    )
                    with ui.column().classes(
                        "no-camera items-center w-full"
                    ) as self.no_camera:
                        ui.icon("videocam_off", size="28px")
                        ui.label("无相机观测").classes("no-camera-title")
                        ui.label("此 Episode 仅包含状态数据").classes(
                            "no-camera-caption"
                        )
                    with ui.row().classes(
                        "loading-line items-center w-full"
                    ) as self.loading_line:
                        ui.spinner("dots", size="30px", color="primary")
                        self.loading_label = ui.label("")
                    self.camera_legend = ui.element("div").classes("camera-legend")

            with ui.element("aside").classes("state-pane"):
                with ui.element("div").classes("inspector-head"):
                    ui.icon("manage_search", size="20px").classes("inspector-icon")
                    ui.label("数据检查器").classes("inspector-title")
                    self.inspector_frame_label = ui.label("").classes(
                        "inspector-frame"
                    )
                with (
                    ui.tabs()
                    .props(
                        "dense align=left active-color=primary "
                        "indicator-color=primary"
                    )
                    .classes("state-tabs w-full")
                ) as state_tabs:
                    frame_tab = ui.tab("当前帧", icon="timeline")
                    episode_tab = ui.tab("Episode", icon="dataset")
                with ui.tab_panels(state_tabs, value=frame_tab).classes(
                    "state-panels w-full"
                ):
                    with ui.tab_panel(frame_tab):
                        self.search = (
                            ui.input(
                                placeholder="筛选字段路径",
                                on_change=self._on_search,
                            )
                            .props("dense outlined clearable")
                            .classes("inspector-filter w-full")
                        )
                        self.frame_field_count = ui.label("").classes("field-count")
                        self.frame_fields_container = ui.element("div")
                    with ui.tab_panel(episode_tab):
                        self.episode_fields_container = ui.element("div")

            with ui.element("section").classes("tracks-pane"):
                with ui.element("div").classes("section-head"):
                    ui.label("关节轨迹").classes("section-title")
                    self.joint_summary = ui.label("").classes("section-summary")
                self.joint_container = ui.element("div")

        ui.on("hdf5_video_time", self._on_video_time, throttle=0.05)
        ui.on("hdf5_video_play", self._on_video_play)
        ui.on("hdf5_video_pause", self._on_video_pause)
        ui.on("hdf5_video_ended", self._on_video_ended)
        ui.keyboard(on_key=self._on_key, repeating=False)
        self._update_episode_header()
        self._set_transport_enabled(False)

    def _build_metadata_dialog(self) -> None:
        with ui.dialog() as metadata_dialog, ui.card().classes("meta-dialog"):
            with ui.row().classes("meta-dialog-head w-full items-center"):
                ui.icon("data_object", size="20px").classes("text-primary")
                ui.label("录制元数据").classes("meta-dialog-title")
                ui.space()
                ui.button(icon="close", on_click=metadata_dialog.close).props(
                    "flat round dense"
                ).tooltip("关闭")
            with ui.element("div").classes("meta-scroll"):
                ui.label("Root attributes").classes("data-section-title mt-2")
                self._render_mapping(self.viewer.metadata["root_attrs"])
                ui.label("/data attributes").classes("data-section-title mt-5")
                self._render_mapping(self.viewer.metadata["data_attrs"])
        ui.button(icon="info", on_click=metadata_dialog.open).props(
            "flat round dense"
        ).classes("header-action").tooltip("录制元数据")

    @staticmethod
    def _render_mapping(values: dict[str, Any]) -> None:
        if not values:
            ui.label("无属性").classes("empty-copy")
            return
        for path, value in values.items():
            with ui.element("div").classes("data-row"):
                ui.label(path).classes("data-path")
                ui.label(_display_value(value)).classes("data-value")

    @staticmethod
    def _icon_button(icon: str, handler: Callable[[], None], tooltip: str) -> Button:
        return (
            ui.button(icon=icon, on_click=handler)
            .props("flat round dense")
            .tooltip(tooltip)
        )

    async def _load_initial_episode(self) -> None:
        await self._select_episode(self.episode["name"])

    async def _on_episode_change(self, event: events.ValueChangeEventArguments) -> None:
        await self._select_episode(str(event.value))

    async def _select_episode(self, episode_name: str) -> None:
        self.generation += 1
        generation = self.generation
        self._stop_playback()
        self.ready = False
        self._set_transport_enabled(False)
        self.episode = next(
            episode
            for episode in self.viewer.metadata["episodes"]
            if episode["name"] == episode_name
        )
        self.frame = 0
        self.states = []
        self._update_episode_header()
        if self.episode["cameras"]:
            self.media_stage.classes(add="has-media", remove="no-media")
        else:
            self.media_stage.classes(add="no-media", remove="has-media")
        self.video.set_visibility(False)
        self.no_camera.set_visibility(False)
        self.camera_legend.set_visibility(False)
        cameras = self.episode["cameras"]
        self.loading_label.set_text(
            "正在准备 RGB-D 视频" if cameras else "正在载入状态数据"
        )
        self.loading_line.set_visibility(True)
        try:
            if cameras:
                states, video_path = await asyncio.gather(
                    run.io_bound(self.viewer.states, episode_name),
                    run.io_bound(self.viewer.prepare_video, episode_name),
                )
            else:
                states = await run.io_bound(self.viewer.states, episode_name)
        except (
            KeyError,
            OSError,
            RuntimeError,
            TypeError,
            ValueError,
            cv2.error,
        ) as error:
            if generation == self.generation:
                self.loading_label.set_text(f"载入失败：{error}")
                ui.notify(str(error), type="negative")
            return
        if generation != self.generation:
            return

        self.states = states
        self._build_state_fields()
        self._build_episode_fields()
        self._build_joint_charts()
        self._build_camera_legend()
        if cameras:
            video_url = (
                f"/viewer-media/{video_path.name}?v={self.viewer.video_cache_token}"
            )
            self.video.set_source(video_url)
            self.video.set_visibility(True)
            self.video.client.run_javascript(
                f"getElement({self.video.id}).$el.playbackRate = {self.speed}"
            )
        else:
            self.no_camera.set_visibility(True)
        self.loading_line.set_visibility(False)
        self.ready = True
        self._set_transport_enabled(True)
        self._set_frame(0)

    def _update_episode_header(self) -> None:
        success = bool(self.episode["success"])
        self.status_badge.set_text("成功" if success else "未成功")
        self.status_badge.classes(
            add="success" if success else "failure",
            remove="failure" if success else "success",
        )
        frame_count = int(self.episode["frame_count"])
        self.timeline._props["max"] = frame_count - 1
        self.timeline.update()
        self.frame_input._props["max"] = frame_count - 1
        self.frame_input.update()
        self.episode_title.set_text(self.episode["name"])
        self.metric_values["frames"].set_text(str(frame_count))
        self.metric_values["duration"].set_text(
            f"{float(self.episode['duration_seconds']):.2f} s"
        )
        self.metric_values["fps"].set_text(f"{self.fps:.3g}")
        self.metric_values["cameras"].set_text(str(len(self.episode["cameras"])))
        self.frame_field_count.set_text(
            f"{len(self.episode['frame_fields'])} 个逐帧字段"
        )
        self.time_label.set_text(f"00:00.00 / {_timecode(frame_count - 1, self.fps)}")
        self.inspector_frame_label.set_text(f"帧 0 / {frame_count - 1}")

    def _set_transport_enabled(self, enabled: bool) -> None:
        controls = (
            self.previous_button,
            self.play_button,
            self.next_button,
            self.timeline,
            self.speed_toggle,
            self.frame_input,
        )
        for control in controls:
            control.enable() if enabled else control.disable()

    def _build_camera_legend(self) -> None:
        self.camera_legend.clear()
        cameras = self.episode["cameras"]
        if not cameras:
            self.camera_legend.set_visibility(False)
            return
        self.camera_legend.style(
            f"grid-template-columns: repeat({len(cameras)}, minmax(0, 1fr))"
        )
        with self.camera_legend:
            for camera in cameras:
                with ui.element("div").classes("camera-label"):
                    ui.label(camera["name"]).classes("camera-name")
                    ui.label(
                        f"RGB + DEPTH · {camera['width']}x{camera['height']} · "
                        f"{camera['depth_min']:.3g}-{camera['depth_max']:.3g} m"
                    ).classes("camera-meta")
        self.camera_legend.set_visibility(True)

    def _build_state_fields(self) -> None:
        self.frame_fields_container.clear()
        self.state_value_labels = {}
        self.state_rows = []
        fields = self.episode["frame_fields"]
        with self.frame_fields_container:
            if not fields:
                ui.label("没有逐帧字段").classes("empty-copy")
                return
            first_frame = self.states[0]
            query = str(self.search.value or "").strip().lower()
            for field in fields:
                path = field["path"]
                with ui.element("div").classes("data-row") as row:
                    ui.label(path).classes("data-path")
                    shape = "x".join(str(size) for size in field["shape"])
                    ui.label(f"{field['dtype']} · [{shape}]").classes("data-meta")
                    value_label = ui.label(_display_value(first_frame[path])).classes(
                        "data-value"
                    )
                row.set_visibility(not query or query in path.lower())
                self.state_value_labels[path] = value_label
                self.state_rows.append((path.lower(), row))

    def _build_episode_fields(self) -> None:
        self.episode_fields_container.clear()
        with self.episode_fields_container:
            ui.label("Attributes").classes("data-section-title")
            self._render_mapping(self.episode["attrs"])
            ui.label("Static datasets").classes("data-section-title mt-5")
            static_fields = self.episode["static_fields"]
            if not static_fields:
                ui.label("没有静态数据集").classes("empty-copy")
                return
            for path, field in static_fields.items():
                with ui.element("div").classes("data-row"):
                    ui.label(path).classes("data-path")
                    shape = "x".join(str(size) for size in field["shape"])
                    ui.label(f"{field['dtype']} · [{shape}]").classes("data-meta")
                    ui.label(_display_value(field["value"])).classes("data-value")

    def _build_joint_charts(self) -> None:
        self.joint_container.clear()
        self.joint_charts = []
        joint_fields = self.episode["joint_fields"]
        total_joints = sum(int(field["joint_count"]) for field in joint_fields)
        self.joint_summary.set_text(
            f"{len(joint_fields)} 组 · {total_joints} 关节 · {self.episode['frame_count']} 帧"
            if joint_fields
            else "没有关节位置字段"
        )
        with self.joint_container:
            for field in joint_fields:
                path = field["path"]
                group_name = path.removeprefix("obs/").removesuffix("_joint_pos")
                with ui.element("section").classes("joint-group"):
                    with ui.element("div").classes("joint-group-head"):
                        ui.label(group_name).classes("joint-group-title")
                        ui.label(path).classes("joint-group-path")
                    with ui.element("div").classes("joint-grid"):
                        for joint_index in range(int(field["joint_count"])):
                            values = [
                                float(frame[path][joint_index]) for frame in self.states
                            ]
                            with ui.element("div").classes("joint-plot"):
                                with ui.element("div").classes("joint-plot-head"):
                                    ui.label(f"joint {joint_index}").classes(
                                        "joint-label"
                                    )
                                    current_label = ui.label(
                                        f"{values[0]:.5f} rad"
                                    ).classes("joint-value")
                                chart = ui.echart(self._chart_options(values)).classes(
                                    "joint-chart"
                                )
                            self.joint_charts.append((chart, current_label, values))

    @staticmethod
    def _chart_options(values: list[float]) -> dict[str, Any]:
        return {
            "animation": False,
            "grid": {"left": 46, "right": 14, "top": 16, "bottom": 28},
            "tooltip": {
                "trigger": "axis",
                "confine": True,
                "backgroundColor": "rgba(29, 37, 41, 0.94)",
                "borderWidth": 0,
                "textStyle": {"color": "#f6f8f9", "fontSize": 10},
                "axisPointer": {
                    "type": "line",
                    "lineStyle": {"color": "#e49b18", "width": 1},
                },
            },
            "xAxis": {
                "type": "category",
                "data": list(range(len(values))),
                "boundaryGap": False,
                "axisLabel": {"fontSize": 8, "color": "#849097"},
                "axisTick": {"show": False},
                "axisLine": {"lineStyle": {"color": "#ccd3d6"}},
            },
            "yAxis": {
                "type": "value",
                "scale": True,
                "splitNumber": 3,
                "axisLabel": {"fontSize": 8, "color": "#849097"},
                "axisLine": {"show": False},
                "axisTick": {"show": False},
                "splitLine": {"lineStyle": {"color": "#edf0f1"}},
            },
            "series": [
                {
                    "id": "trajectory",
                    "type": "line",
                    "data": values,
                    "showSymbol": False,
                    "lineStyle": {"width": 1.7, "color": "#087c74"},
                    "areaStyle": {"color": "rgba(8, 124, 116, 0.07)"},
                    "markLine": {
                        "silent": True,
                        "symbol": "none",
                        "label": {"show": False},
                        "lineStyle": {"color": "#e49b18", "width": 1},
                        "data": [{"xAxis": 0}],
                    },
                }
            ],
        }

    def _set_frame(self, frame: int) -> None:
        last_frame = int(self.episode["frame_count"]) - 1
        frame = min(max(frame, 0), last_frame)
        self.frame = frame
        self.timeline.set_value(frame)
        self.frame_input.set_value(frame)
        self.time_label.set_text(
            f"{_timecode(frame, self.fps)} / {_timecode(last_frame, self.fps)}"
        )
        self.inspector_frame_label.set_text(f"帧 {frame} / {last_frame}")
        if self.states:
            frame_values = self.states[frame]
            for path, label in self.state_value_labels.items():
                label.set_text(_display_value(frame_values[path]))
            for chart, label, values in self.joint_charts:
                label.set_text(f"{values[frame]:.5f} rad")
                chart.run_chart_method(
                    "setOption",
                    {
                        "series": [
                            {
                                "id": "trajectory",
                                "markLine": {"data": [{"xAxis": frame}]},
                            }
                        ]
                    },
                )

    def _seek(self, frame: int) -> None:
        if not self.ready:
            return
        last_frame = int(self.episode["frame_count"]) - 1
        frame = min(max(frame, 0), last_frame)
        self._set_frame(frame)
        if self.episode["cameras"]:
            self.video.seek(frame / self.fps)
        elif self.playing:
            self.clock_frame = frame
            self.clock_started_at = time.monotonic()

    def _previous(self) -> None:
        self._seek(self.frame - 1)

    def _next(self) -> None:
        self._seek(self.frame + 1)

    def _on_timeline_change(self, event: events.ValueChangeEventArguments) -> None:
        self._seek(round(float(event.value)))

    def _on_frame_input(self, event: events.ValueChangeEventArguments) -> None:
        # A number input briefly reports None while the user replaces its text.
        if event.value is not None:
            self._seek(round(float(event.value)))

    def _on_speed_change(self, event: events.ValueChangeEventArguments) -> None:
        self.speed = float(event.value)
        if self.episode["cameras"]:
            self.video.client.run_javascript(
                f"getElement({self.video.id}).$el.playbackRate = {self.speed}"
            )
        elif self.playing:
            self.clock_frame = self.frame
            self.clock_started_at = time.monotonic()

    def _toggle_playback(self) -> None:
        if not self.ready:
            return
        if self.playing:
            self._stop_playback()
            return
        if self.frame >= int(self.episode["frame_count"]) - 1:
            self._seek(0)
        self.playing = True
        self.play_button.set_icon("pause")
        self.play_tooltip.set_text("暂停")
        if self.episode["cameras"]:
            self.video.play()
        else:
            self.clock_frame = self.frame
            self.clock_started_at = time.monotonic()

    def _stop_playback(self) -> None:
        if self.playing and self.episode["cameras"]:
            self.video.pause()
        self.playing = False
        self.play_button.set_icon("play_arrow")
        self.play_tooltip.set_text("播放")

    def _tick_data_playback(self) -> None:
        if not self.ready or not self.playing or self.episode["cameras"]:
            return
        elapsed_frames = (
            (time.monotonic() - self.clock_started_at) * self.fps * self.speed
        )
        target_frame = self.clock_frame + int(elapsed_frames)
        last_frame = int(self.episode["frame_count"]) - 1
        if target_frame >= last_frame:
            self._set_frame(last_frame)
            self._stop_playback()
        elif target_frame != self.frame:
            self._set_frame(target_frame)

    def _on_video_time(self, event: events.GenericEventArguments) -> None:
        if self.ready and self.episode["cameras"]:
            self._set_frame(round(float(event.args) * self.fps))

    def _on_video_play(self) -> None:
        self.playing = True
        self.play_button.set_icon("pause")
        self.play_tooltip.set_text("暂停")

    def _on_video_pause(self) -> None:
        self.playing = False
        self.play_button.set_icon("play_arrow")
        self.play_tooltip.set_text("播放")

    def _on_video_ended(self) -> None:
        self._set_frame(int(self.episode["frame_count"]) - 1)
        self._on_video_pause()

    def _on_key(self, event: events.KeyEventArguments) -> None:
        if not event.action.keydown:
            return
        if event.key.space:
            self._toggle_playback()
        elif event.key.arrow_left:
            self._previous()
        elif event.key.arrow_right:
            self._next()
        elif event.key.home:
            self._seek(0)
        elif event.key.end:
            self._seek(int(self.episode["frame_count"]) - 1)

    def _on_search(self, event: events.ValueChangeEventArguments) -> None:
        # The clearable Quasar input emits None when its clear button is used.
        query = str(event.value or "").strip().lower()
        for path, row in self.state_rows:
            row.set_visibility(not query or query in path)


def create_gui(viewer: HDF5Viewer) -> None:
    app.add_static_files("/viewer-media", viewer.video_directory, max_cache_age=0)
    app.on_shutdown(viewer.close)

    @ui.page("/", title="HDF5 回放面板", language="zh-CN")
    def index_page() -> None:
        ViewerPage(viewer)


def main() -> None:
    args = parse_args()
    recording_path = args.recording.expanduser().resolve()
    if not recording_path.is_file():
        raise SystemExit(f"error: HDF5 recording does not exist: {recording_path}")
    try:
        viewer = HDF5Viewer(recording_path)
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    create_gui(viewer)
    print(
        f"Serving {recording_path}\n"
        f"Episodes: {len(viewer.metadata['episodes'])}\n"
        f"Open http://127.0.0.1:{args.port}"
    )
    try:
        ui.run(
            host="127.0.0.1",
            port=args.port,
            title="HDF5 回放面板",
            language="zh-CN",
            show=False,
            reload=False,
            dark=False,
        )
    except KeyboardInterrupt:
        print("\nStopping viewer")


if __name__ in {"__main__", "__mp_main__"}:
    main()
