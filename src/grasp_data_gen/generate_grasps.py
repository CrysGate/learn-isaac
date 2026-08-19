"""Command-line entry point for configured grasp generation."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import traceback
from typing import Any, Sequence

from grasp_data_gen.config import GraspGenerationConfig, load_grasp_config
from grasp_data_gen.results import build_metadata, write_results
from scale_bench.config.models.robot import RobotConfig


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = MODULE_ROOT / "piper.yml"
USD_EXTENSIONS = frozenset({".usd", ".usda", ".usdc", ".usdz"})


@dataclass(frozen=True)
class GenerationJob:
    object_usd: Path
    output_dir: Path


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _find_object_usds(object_dir: Path) -> tuple[Path, ...]:
    """Recursively find supported USD assets in a stable order."""

    return tuple(
        path
        for path in sorted(object_dir.rglob("*"))
        if path.is_file() and path.suffix.lower() in USD_EXTENSIONS
    )


def _build_batch_jobs(
    object_dir: Path,
    output_dir: Path,
    object_usds: Sequence[Path],
) -> tuple[GenerationJob, ...]:
    """Map input assets to non-overlapping output directories."""

    relative_paths = tuple(path.relative_to(object_dir) for path in object_usds)
    files_per_parent = Counter(path.parent for path in relative_paths)
    jobs = []
    for object_usd, relative_path in zip(object_usds, relative_paths, strict=True):
        is_only_asset = files_per_parent[relative_path.parent] == 1
        if relative_path.parent != Path(".") and is_only_asset:
            relative_output = relative_path.parent
        else:
            relative_output = relative_path.with_suffix("")
        jobs.append(
            GenerationJob(
                object_usd=object_usd,
                output_dir=output_dir / relative_output,
            )
        )

    output_dirs = [job.output_dir for job in jobs]
    if len(output_dirs) != len(set(output_dirs)):
        raise ValueError(
            "input assets map to duplicate output directories; "
            "place same-named assets in separate directories"
        )
    return tuple(jobs)


def _parse_args(
    generation: GraspGenerationConfig,
    argv: Sequence[str] | None = None,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate and physically validate grasp poses for one USD asset or "
            "all USD assets in a directory."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--object-usd", type=Path, help="One object USD asset.")
    inputs.add_argument("--object-dir", type=Path, help="Directory searched recursively for object USD assets.")
    parser.add_argument("--output-dir", type=Path, help="Output directory. Batch outputs are placed in subdirectories.")
    parser.add_argument("--num-candidates", type=_positive_int, default=generation.sampler.num_candidates)
    parser.add_argument("--num-orientations", type=_positive_int, default=generation.sampler.num_orientations)
    parser.add_argument("--seed", type=int, default=generation.sampler.random_seed)
    parser.add_argument("--gui", action="store_true", help="Open a lit viewport that follows grasp physics evaluation.")

    args = parser.parse_args(argv)
    default_output = PROJECT_ROOT / "outputs/grasp_data" / generation.name
    if args.object_usd is not None:
        args.object_usd = args.object_usd.expanduser().resolve()
        if not args.object_usd.is_file():
            parser.error(f"--object-usd does not exist: {args.object_usd}")
        output_dir = args.output_dir or default_output / "00000"
        args.jobs = (
            GenerationJob(
                object_usd=args.object_usd,
                output_dir=output_dir.expanduser().resolve(),
            ),
        )
    else:
        args.object_dir = args.object_dir.expanduser().resolve()
        if not args.object_dir.is_dir():
            parser.error(f"--object-dir does not exist: {args.object_dir}")
        output_dir = (args.output_dir or default_output).expanduser().resolve()
        if output_dir == args.object_dir or output_dir.is_relative_to(args.object_dir):
            parser.error("--output-dir must be outside --object-dir")
        object_usds = _find_object_usds(args.object_dir)
        if not object_usds:
            supported = ", ".join(sorted(USD_EXTENSIONS))
            parser.error(
                f"--object-dir contains no supported USD assets ({supported}): "
                f"{args.object_dir}"
            )
        try:
            args.jobs = _build_batch_jobs(
                args.object_dir,
                output_dir,
                object_usds,
            )
        except ValueError as error:
            parser.error(str(error))
    args.output_dir = output_dir.expanduser().resolve()
    return args


def _run_job(
    args: argparse.Namespace,
    job: GenerationJob,
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
        job.object_usd,
        generation,
        robot,
    )
    if args.gui:
        from grasp_data_gen.isaac.viewer import configure_evaluation_preview
        configure_evaluation_preview(scene, simulation_app)
    scene.gripper.open()
    export_stage(scene, job.output_dir / "evaluation_stage.usda")
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
        object_usd=job.object_usd,
        base_to_tcp=to_pose_data(scene.gripper.base_to_tcp),
        sampler_config=manager.sampler_config,
        records=evaluated.records,
        feasible_count=len(candidates.feasible),
        successful=evaluated.successful,
        support_height=candidates.support_height_m,
    )
    write_results(
        job.output_dir,
        metadata,
        evaluated.records,
        evaluated.successful,
    )
    print(
        "GRASP_GENERATION_RESULT "
        f"generated={len(evaluated.records)} "
        f"task_feasible={len(candidates.feasible)} "
        f"evaluated={len(candidates.feasible)} "
        f"accepted={len(evaluated.successful)} output={job.output_dir}",
        flush=True,
    )
    return 0


def _run_jobs(
    args: argparse.Namespace,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
    simulation_app: Any,
) -> int:
    if args.object_dir is None:
        job = args.jobs[0]
        job.output_dir.mkdir(parents=True, exist_ok=True)
        return _run_job(args, job, generation, robot, simulation_app)

    failed_count = 0
    total = len(args.jobs)
    for index, job in enumerate(args.jobs, start=1):
        print(f"[{index}/{total}] Generating grasps for {job.object_usd}", flush=True)
        job.output_dir.mkdir(parents=True, exist_ok=True)
        try:
            _run_job(args, job, generation, robot, simulation_app)
        except Exception:
            failed_count += 1
            traceback.print_exc()

    print(
        "GRASP_BATCH_RESULT "
        f"total={total} succeeded={total - failed_count} "
        f"failed={failed_count} output={args.output_dir}",
        flush=True,
    )
    return 1 if failed_count else 0


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

    from isaacsim import SimulationApp

    launch_config: dict[str, Any] = {"headless": not args.gui}
    if args.gui:
        launch_config["extra_args"] = ["--enable", "omni.physx.ui"]
    simulation_app = SimulationApp(launch_config=launch_config)
    try:
        _enable_grasping_extension()
        result = _run_jobs(args, generation, robot, simulation_app)
    except BaseException:
        traceback.print_exc()
        simulation_app.close(exit_code=1)
        raise
    else:
        simulation_app.close()
        return result


if __name__ == "__main__":
    raise SystemExit(main())
