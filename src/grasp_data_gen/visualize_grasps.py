"""Command-line entry point for the interactive grasp viewer."""

from __future__ import annotations

import argparse
from pathlib import Path
import traceback
from typing import Any, Sequence

from grasp_data_gen.models import GraspFileData
from grasp_data_gen.results import load_grasp_file, resolve_asset_path


HEADLESS_FRAME_LIMIT = 4
VIEWPORT_WARMUP_FRAMES = 8


def _non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return parsed


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Visualize successful configured-gripper poses in Isaac Sim."
    )
    parser.add_argument(
        "--grasp-file",
        type=Path,
        required=True,
        help="Generated successful_grasps.yaml to visualize.",
    )
    parser.add_argument(
        "--start-index",
        type=_non_negative_int,
        default=0,
        help="Zero-based rank of the initially visible successful grasp.",
    )
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="Initially overlay all successful grippers.",
    )
    parser.add_argument("--headless", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--max-frames",
        type=_positive_int,
        help=argparse.SUPPRESS,
    )
    return parser


def _parse_args(
    parser: argparse.ArgumentParser,
    argv: Sequence[str] | None = None,
) -> argparse.Namespace:
    args = parser.parse_args(argv)
    args.grasp_file = args.grasp_file.expanduser().resolve()
    if not args.grasp_file.is_file():
        parser.error(f"--grasp-file does not exist: {args.grasp_file}")
    if args.headless and args.max_frames is None:
        args.max_frames = HEADLESS_FRAME_LIMIT
    return args


def _load_inputs(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
) -> tuple[GraspFileData, Path, Path]:
    try:
        data = load_grasp_file(args.grasp_file)
        robot_usd = resolve_asset_path(
            data.robot_usd,
            args.grasp_file,
            "robot USD",
        )
        object_usd = resolve_asset_path(
            data.object_usd,
            args.grasp_file,
            "object USD",
        )
        if args.start_index >= len(data.grasps):
            raise ValueError(
                f"--start-index {args.start_index} is outside "
                f"0..{len(data.grasps) - 1}"
            )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return data, robot_usd, object_usd


def _enable_viewer_extensions() -> None:
    import omni.kit.app

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if not extension_manager.is_extension_enabled("isaacsim.util.debug_draw"):
        extension_manager.set_extension_enabled_immediate(
            "isaacsim.util.debug_draw",
            True,
        )


def _run_viewer(
    args: argparse.Namespace,
    data: GraspFileData,
    robot_usd: Path,
    object_usd: Path,
    simulation_app: Any,
) -> int:
    _enable_viewer_extensions()

    from grasp_data_gen.isaac.viewer import GraspViewer, build_viewer_scene

    scene = build_viewer_scene(data, robot_usd, object_usd)
    viewer = GraspViewer(
        scene,
        data.grasps,
        start_index=args.start_index,
        show_all=args.show_all,
        headless=args.headless,
    )
    try:
        for _ in range(VIEWPORT_WARMUP_FRAMES):
            simulation_app.update()
        viewer.fit_camera()
        print(
            "GRASP_VIEWER_READY "
            f"grasps={len(data.grasps)} file={args.grasp_file}",
            flush=True,
        )
        frame_count = 0
        while simulation_app.is_running():
            simulation_app.update()
            frame_count += 1
            if args.max_frames is not None and frame_count >= args.max_frames:
                break
    finally:
        viewer.close()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = _parse_args(parser, argv)
    data, robot_usd, object_usd = _load_inputs(parser, args)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(launch_config={"headless": args.headless})
    try:
        result = _run_viewer(
            args,
            data,
            robot_usd,
            object_usd,
            simulation_app,
        )
    except BaseException:
        traceback.print_exc()
        simulation_app.close(exit_code=1)
        raise
    else:
        simulation_app.close()
        return result


if __name__ == "__main__":
    raise SystemExit(main())
