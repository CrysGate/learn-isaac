"""Command-line entry point for configured grasp generation."""

from __future__ import annotations

import argparse
from pathlib import Path
import traceback
from typing import Any, Sequence

from grasp_data_gen.config import GraspGenerationConfig, load_grasp_config
from grasp_data_gen.results import build_metadata, write_results
from scale_bench.config.models.robot import RobotConfig


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = MODULE_ROOT / "piper.yml"


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _parse_args(
    generation: GraspGenerationConfig,
    argv: Sequence[str] | None = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate and physically validate configured grasp poses.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--object-usd", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=(PROJECT_ROOT / "outputs/grasp_data" / generation.name / "00000"))
    parser.add_argument("--num-candidates", type=_positive_int, default=generation.sampler.num_candidates)
    parser.add_argument("--num-orientations", type=_positive_int, default=generation.sampler.num_orientations)
    parser.add_argument("--seed", type=int, default=generation.sampler.random_seed)
    parser.add_argument("--gui", action="store_true", help="Open a lit viewport that follows grasp physics evaluation.")

    args = parser.parse_args(argv)
    args.object_usd = args.object_usd.expanduser().resolve()
    args.output_dir = args.output_dir.expanduser().resolve()
    if not args.object_usd.is_file():
        parser.error(f"--object-usd does not exist: {args.object_usd}")
    return args


def _run(
    args: argparse.Namespace,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
    simulation_app: Any,
) -> int:
    from grasp_data_gen.generation import generate_candidates
    from grasp_data_gen.isaac.geometry import to_pose_data
    from grasp_data_gen.isaac.scene import (
        build_evaluation_scene,
        create_grasping_manager,
        export_stage,
    )
    from grasp_data_gen.validation import evaluate_candidates

    robot_usd = Path(robot.usd_path)
    scene = build_evaluation_scene(
        robot_usd,
        args.object_usd,
        generation,
        robot,
    )
    if args.gui:
        from grasp_data_gen.isaac.viewer import configure_evaluation_preview
        configure_evaluation_preview(scene, simulation_app)
    scene.gripper.open()
    export_stage(scene, args.output_dir / "evaluation_stage.usda")
    manager = create_grasping_manager(scene)
    candidates = generate_candidates(
        scene,
        manager,
        generation,
        simulation_app,
        num_candidates=args.num_candidates,
        num_orientations=args.num_orientations,
        random_seed=args.seed,
    )
    evaluated = evaluate_candidates(
        manager,
        scene,
        candidates,
        generation.validation,
        simulation_app,
    )
    metadata = build_metadata(
        generation=generation,
        robot=robot,
        generation_config_path=DEFAULT_CONFIG,
        robot_usd=robot_usd,
        object_usd=args.object_usd,
        base_to_tcp=to_pose_data(scene.gripper.base_to_tcp),
        sampler_config=manager.sampler_config,
        records=evaluated.records,
        feasible_count=len(candidates.feasible),
        successful=evaluated.successful,
        support_height=candidates.support_height_m,
    )
    write_results(
        args.output_dir,
        metadata,
        evaluated.records,
        evaluated.successful,
    )
    print(
        "GRASP_GENERATION_RESULT "
        f"generated={len(evaluated.records)} "
        f"task_feasible={len(candidates.feasible)} "
        f"evaluated={len(candidates.feasible)} "
        f"accepted={len(evaluated.successful)} output={args.output_dir}",
        flush=True,
    )
    return 0


def _enable_extension(extension_manager: Any, name: str) -> None:
    if not extension_manager.is_extension_enabled(name):
        extension_manager.set_extension_enabled_immediate(name, True)


def _enable_grasping_extension() -> None:
    import omni.kit.app
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    _enable_extension(extension_manager, "isaacsim.replicator.grasping")


def main(argv: Sequence[str] | None = None) -> int:
    generation, robot = load_grasp_config(DEFAULT_CONFIG, asset_root=PROJECT_ROOT)
    args = _parse_args(generation, argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    launch_config: dict[str, Any] = {"headless": not args.gui}
    if args.gui:
        launch_config["extra_args"] = ["--enable", "omni.physx.ui"]
    simulation_app = SimulationApp(launch_config=launch_config)
    try:
        _enable_grasping_extension()
        result = _run(args, generation, robot, simulation_app)
    except BaseException:
        traceback.print_exc()
        simulation_app.close(exit_code=1)
        raise
    else:
        simulation_app.close()
        return result


if __name__ == "__main__":
    raise SystemExit(main())
