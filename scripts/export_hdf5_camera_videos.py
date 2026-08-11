"""Export RGB and false-color depth videos from a ScaleBench HDF5 episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import h5py
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="Input HDF5 recording")
    parser.add_argument("--demo", default="demo_0", help="Episode group name")
    parser.add_argument(
        "--camera",
        choices=("left_robot", "right_robot", "overhead"),
        default="right_robot",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory (default: <recording_stem>_videos)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        help="Output frame rate (default: derive from recording metadata)",
    )
    parser.add_argument("--depth-min-m", type=float)
    parser.add_argument("--depth-max-m", type=float)
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="FourCC video codec (default: mp4v)",
    )
    args = parser.parse_args()

    if args.fps is not None and args.fps <= 0.0:
        parser.error("--fps must be positive")
    if args.depth_min_m is not None and args.depth_min_m < 0.0:
        parser.error("--depth-min-m must be non-negative")
    if args.depth_max_m is not None and args.depth_max_m <= 0.0:
        parser.error("--depth-max-m must be positive")
    if (
        args.depth_min_m is not None
        and args.depth_max_m is not None
        and args.depth_min_m >= args.depth_max_m
    ):
        parser.error("--depth-min-m must be smaller than --depth-max-m")
    if len(args.codec) != 4:
        parser.error("--codec must contain exactly four characters")
    return args


def _recording_fps(data_group: h5py.Group) -> float:
    raw_env_args = data_group.attrs.get("env_args")
    if isinstance(raw_env_args, bytes):
        raw_env_args = raw_env_args.decode("utf-8")
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


def _sample_depth_values(depth_frames: h5py.Dataset) -> np.ndarray:
    samples: list[np.ndarray] = []
    for start in range(0, depth_frames.shape[0], 32):
        chunk = np.asarray(depth_frames[start : start + 32]).reshape(-1)
        valid = chunk[np.isfinite(chunk) & (chunk > 0.0)]
        if valid.size:
            stride = max(1, valid.size // 100_000)
            samples.append(valid[::stride])
    if not samples:
        raise ValueError("the depth dataset contains no positive finite values")
    return np.concatenate(samples)


def _depth_range(
    depth_frames: h5py.Dataset,
    minimum_m: float | None,
    maximum_m: float | None,
) -> tuple[float, float]:
    samples = _sample_depth_values(depth_frames)
    detected_min, detected_max = np.percentile(samples, (1.0, 99.0))
    minimum = float(detected_min) if minimum_m is None else minimum_m
    maximum = float(detected_max) if maximum_m is None else maximum_m
    if minimum >= maximum:
        raise ValueError(
            f"invalid depth visualization range: {minimum:g} m to {maximum:g} m"
        )
    return minimum, maximum


def _video_writer(
    path: Path,
    *,
    codec: str,
    fps: float,
    width: int,
    height: int,
) -> cv2.VideoWriter:
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*codec),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(f"could not open video writer for {path}")
    return writer


def main() -> None:
    args = parse_args()
    recording_path = args.recording.expanduser().resolve()
    if not recording_path.is_file():
        raise FileNotFoundError(f"recording does not exist: {recording_path}")

    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else recording_path.parent / f"{recording_path.stem}_videos"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = output_dir / f"{args.demo}_{args.camera}_rgb.mp4"
    depth_path = output_dir / f"{args.demo}_{args.camera}_depth.mp4"

    with h5py.File(recording_path, "r") as recording:
        episode_path = f"data/{args.demo}"
        if episode_path not in recording:
            available = ", ".join(recording.get("data", {}).keys()) or "none"
            raise KeyError(
                f"episode {args.demo!r} not found; available episodes: {available}"
            )
        observations = recording[episode_path].get("obs")
        if observations is None:
            raise KeyError(f"{episode_path} contains no observation data")

        rgb_key = f"{args.camera}_camera_rgb"
        depth_key = f"{args.camera}_camera_depth"
        if rgb_key not in observations or depth_key not in observations:
            raise KeyError(
                f"{episode_path}/obs does not contain {rgb_key!r} and {depth_key!r}"
            )
        rgb_frames = observations[rgb_key]
        depth_frames = observations[depth_key]
        if rgb_frames.ndim != 4 or rgb_frames.shape[-1] != 3:
            raise ValueError(f"unexpected RGB shape: {rgb_frames.shape}")
        if depth_frames.ndim not in (3, 4) or (
            depth_frames.ndim == 4 and depth_frames.shape[-1] != 1
        ):
            raise ValueError(f"unexpected depth shape: {depth_frames.shape}")
        if rgb_frames.shape[:3] != depth_frames.shape[:3]:
            raise ValueError(
                "RGB and depth frame counts or resolutions do not match: "
                f"{rgb_frames.shape} versus {depth_frames.shape}"
            )

        frame_count, height, width = rgb_frames.shape[:3]
        fps = args.fps or _recording_fps(recording["data"])
        depth_min_m, depth_max_m = _depth_range(
            depth_frames,
            args.depth_min_m,
            args.depth_max_m,
        )
        rgb_writer = _video_writer(
            rgb_path,
            codec=args.codec,
            fps=fps,
            width=width,
            height=height,
        )
        depth_writer = _video_writer(
            depth_path,
            codec=args.codec,
            fps=fps,
            width=width,
            height=height,
        )
        try:
            for index in range(frame_count):
                rgb = np.asarray(rgb_frames[index])
                if rgb.dtype != np.uint8:
                    rgb = np.clip(rgb, 0, 255).astype(np.uint8)
                rgb_writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

                depth = np.asarray(depth_frames[index])
                if depth.ndim == 3:
                    depth = depth[..., 0]
                valid = np.isfinite(depth) & (depth > 0.0)
                depth_for_vis = np.where(valid, depth, depth_max_m)
                normalized = np.clip(
                    (depth_for_vis - depth_min_m)
                    / (depth_max_m - depth_min_m),
                    0.0,
                    1.0,
                )
                depth_u8 = np.round((1.0 - normalized) * 255.0).astype(
                    np.uint8
                )
                depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
                depth_color[~valid] = 0
                depth_writer.write(depth_color)
        finally:
            rgb_writer.release()
            depth_writer.release()

    print(
        f"exported {frame_count} frames at {fps:g} FPS\n"
        f"RGB: {rgb_path}\n"
        f"Depth: {depth_path}\n"
        f"Depth visualization range: {depth_min_m:g} m to {depth_max_m:g} m"
    )


if __name__ == "__main__":
    main()
