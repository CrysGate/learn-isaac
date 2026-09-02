"""Generate a validated CuRobo collision model from a RobotConfig and URDF.

The benchmark RobotConfig and its referenced URDF remain authoritative.  This
script only derives collision spheres and a self-collision ignore matrix; it
does not import defaults from an existing CuRobo sample profile.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path
from typing import Any

import numpy as np
import torch
import trimesh
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from curobo._src.geom.sphere_fit.types import SphereFitMetrics
from curobo._src.robot.builder.builder_robot import RobotBuilder
from curobo._src.types.robot import RobotCfg
from curobo._src.util.xrdf_util import convert_curobo_to_xrdf
from curobo._src.util_file import write_yaml
from curobo.types import DeviceCfg

from scale_bench.config.models.robot import RobotConfig


DEFAULT_PROFILE = PROJECT_ROOT / "configs/robots/piper.yml"
DEFAULT_OUTPUT = PROJECT_ROOT / "configs/robots/curobo/piper.yml"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument(
        "--reuse-generated",
        type=Path,
        default=None,
        help="Post-process and revalidate a previously generated RobotBuilder YAML.",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--sphere-density", type=float, default=1.0)
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument(
        "--refit-link",
        action="append",
        default=["link8:2.0"],
        metavar="LINK:DENSITY",
        help="Refit a difficult link at a different density; repeatable.",
    )
    parser.add_argument(
        "--prune-collisions",
        action="store_true",
        help="Sample and ignore link pairs that appear unable to collide.",
    )
    parser.add_argument("--collision-samples", type=int, default=100)
    parser.add_argument("--collision-batch-size", type=int, default=4096)
    parser.add_argument("--min-link-coverage", type=float, default=0.80)
    parser.add_argument("--max-mean-protrusion-m", type=float, default=0.008)
    parser.add_argument("--max-mean-surface-gap-m", type=float, default=0.005)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.seed < 0:
        raise ValueError("--seed must be non-negative")
    if args.sphere_density <= 0.0 or args.iterations <= 0:
        raise ValueError("sphere density and iterations must be positive")
    if args.collision_samples <= 0 or args.collision_batch_size <= 0:
        raise ValueError("collision sampling values must be positive")

    robot_config = _load_source_robot_config(args.robot_config)
    if robot_config.urdf_path is None:
        raise ValueError("RobotConfig must reference an authoritative URDF")

    device_cfg = DeviceCfg(device=args.device, dtype=torch.float32)
    builder = RobotBuilder(
        urdf_path=robot_config.urdf_path,
        asset_path=str(_robot_asset_root(Path(robot_config.urdf_path))),
        tool_frames=[robot_config.kinematics.tcp.parent_frame],
        device_cfg=device_cfg,
    )
    source_generated = args.reuse_generated.resolve() if args.reuse_generated else None
    if source_generated is None:
        _seed(args.seed)
        builder.fit_collision_spheres(
            sphere_density=args.sphere_density,
            iterations=args.iterations,
            compute_metrics=True,
            # The fixed base intentionally contacts the mounting surface.  Remove
            # spheres below that surface instead of teaching the planner to ignore
            # the entire table.
            clip_links={robot_config.kinematics.base_body: ("z", 0.0)},
        )
        for index, refit in enumerate(args.refit_link):
            link_name, density = _parse_refit(refit)
            _seed(args.seed + index + 1)
            builder.refit_link_spheres(
                link_name,
                sphere_density=density,
                iterations=args.iterations,
                compute_metrics=True,
            )

        builder.compute_collision_matrix(
            prune_collisions=args.prune_collisions,
            num_samples=args.collision_samples,
            batch_size=args.collision_batch_size,
            seed=args.seed,
        )
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        builder.save(builder.build(), str(output))
        document = yaml.safe_load(output.read_text(encoding="utf-8"))
        link_metrics = builder.link_metrics
    else:
        document = yaml.safe_load(source_generated.read_text(encoding="utf-8"))
        _validate_generated_source(
            document,
            robot_config,
            source_path=source_generated,
        )
        _remove_stale_refit_ignores(document, builder, args.refit_link)
        _clip_link_spheres(
            document["kinematics"]["collision_spheres"][
                robot_config.kinematics.base_body
            ],
            axis=2,
            offset=0.0,
            buffer_m=0.02,
        )
        link_metrics = _compute_existing_metrics(
            builder,
            document["kinematics"]["collision_spheres"],
            device_cfg=device_cfg,
            seed=args.seed,
        )

    _validate_metrics(
        link_metrics,
        base_link=robot_config.kinematics.base_body,
        min_coverage=args.min_link_coverage,
        max_mean_protrusion_m=args.max_mean_protrusion_m,
        max_mean_surface_gap_m=args.max_mean_surface_gap_m,
    )

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    _apply_authoritative_robot_contract(
        document,
        robot_config=robot_config,
        urdf_path=Path(robot_config.urdf_path),
        config_path=output,
    )
    expected_joints = tuple(robot_config.kinematics.arm_joint_names)
    actual_joints = tuple(document["kinematics"]["cspace"]["joint_names"])
    if actual_joints != expected_joints:
        raise ValueError(
            "generated CuRobo joint order mismatch: "
            f"{actual_joints} != {expected_joints}"
        )
    cuda_load_validated = device_cfg.device.type == "cuda" and torch.cuda.is_available()
    if cuda_load_validated:
        # Extra-link parsing currently allocates a CUDA Pose internally even
        # when DeviceCfg is CPU, so the full CuRobo load check is CUDA-only.
        validation_document = deepcopy(document)
        validation_kinematics = validation_document["kinematics"]
        validation_kinematics["urdf_path"] = str(
            Path(robot_config.urdf_path).resolve()
        )
        validation_kinematics["asset_root_path"] = str(
            _robot_asset_root(Path(robot_config.urdf_path))
        )
        parsed = RobotCfg.create(validation_document, device_cfg=device_cfg)
        if tuple(parsed.cspace.joint_names) != expected_joints:
            raise ValueError("CuRobo changed the validated planner joint order")

    output.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )

    xrdf_path = output.with_suffix(".xrdf")
    write_yaml(convert_curobo_to_xrdf(document), str(xrdf_path))
    metrics_path = output.with_suffix(".metrics.json")
    metrics_path.write_text(
        json.dumps(
            _metrics_document(
                link_metrics,
                document,
                robot_config,
                args,
                source_generated=source_generated,
                cuda_load_validated=cuda_load_validated,
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        f"generated {output.relative_to(PROJECT_ROOT)} with "
        f"{sum(len(value) for value in document['kinematics']['collision_spheres'].values())} "
        "fitted spheres; planner joint contract validated"
    )
    print(f"wrote {xrdf_path.relative_to(PROJECT_ROOT)}")
    print(f"wrote {metrics_path.relative_to(PROJECT_ROOT)}")
    return 0


def _seed(seed: int) -> None:
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _load_source_robot_config(profile_path: Path) -> RobotConfig:
    """Load the source profile without requiring its generated output yet."""

    profile = profile_path.resolve()
    document = yaml.safe_load(profile.read_text(encoding="utf-8"))
    robot_config = RobotConfig.model_validate(document)
    if robot_config.urdf_path is None:
        return robot_config
    urdf_path = Path(robot_config.urdf_path)
    if not urdf_path.is_absolute():
        urdf_path = (PROJECT_ROOT / urdf_path).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"authoritative URDF does not exist: {urdf_path}")
    return robot_config.model_copy(update={"urdf_path": str(urdf_path)})


def _robot_asset_root(urdf_path: Path) -> Path:
    resolved = urdf_path.resolve()
    for parent in resolved.parents:
        if parent.name == "piper":
            return parent
    return resolved.parent


def _validate_generated_source(
    document: dict[str, Any],
    robot_config: RobotConfig,
    *,
    source_path: Path,
) -> None:
    kinematics = document.get("kinematics", {})
    if kinematics.get("base_link") != robot_config.kinematics.base_body:
        raise ValueError("reused generated YAML has the wrong base link")
    if tuple(kinematics.get("tool_frames", ())) != (
        robot_config.kinematics.tcp.parent_frame,
    ):
        raise ValueError("reused generated YAML has the wrong tool frame")
    source_urdf = Path(kinematics.get("urdf_path", ""))
    if not source_urdf.is_absolute():
        source_urdf = source_path.parent / source_urdf
    source_urdf = source_urdf.resolve()
    if source_urdf != Path(robot_config.urdf_path).resolve():
        raise ValueError("reused generated YAML was built from a different URDF")


def _clip_link_spheres(
    spheres: list[dict[str, Any]],
    *,
    axis: int,
    offset: float,
    buffer_m: float,
) -> None:
    kept = []
    for sphere in spheres:
        distance = float(sphere["center"][axis]) - offset
        if distance <= buffer_m:
            continue
        radius = min(float(sphere["radius"]), distance - buffer_m)
        if radius > 0.0:
            kept.append({"center": list(sphere["center"]), "radius": radius})
    if not kept:
        raise ValueError("mount-plane clipping removed every base-link sphere")
    spheres[:] = kept


def _remove_stale_refit_ignores(
    document: dict[str, Any],
    builder: RobotBuilder,
    refit_specs: list[str],
) -> None:
    """Do not preserve sampled non-neighbor ignores after sphere geometry changed."""

    ignore = document["kinematics"]["self_collision_ignore"]
    for spec in refit_specs:
        link_name, _ = _parse_refit(spec)
        parent = builder._parser._parent_map[link_name]["parent"]  # noqa: SLF001
        for source_link, ignored_links in ignore.items():
            if source_link == link_name:
                ignored_links[:] = [value for value in ignored_links if value == parent]
            elif source_link != parent:
                ignored_links[:] = [
                    value for value in ignored_links if value != link_name
                ]


def _compute_existing_metrics(
    builder: RobotBuilder,
    collision_spheres: dict[str, list[dict[str, Any]]],
    *,
    device_cfg: DeviceCfg,
    seed: int,
) -> dict[str, Any]:
    metrics = {}
    for index, (link_name, spheres) in enumerate(collision_spheres.items()):
        geometry = builder._parser.get_link_geometry(  # noqa: SLF001
            link_name,
            use_collision_mesh=False,
        )
        meshes = [
            value.get_trimesh_mesh(transform_with_pose=True)
            for value in geometry
        ]
        meshes = [value for value in meshes if value is not None]
        if not meshes:
            raise ValueError(f"generated collision link has no mesh: {link_name}")
        mesh = meshes[0] if len(meshes) == 1 else trimesh.util.concatenate(meshes)
        _seed(seed + index)
        metrics[link_name] = _compute_sphere_fit_metrics_cpu(
            mesh,
            np.asarray([value["center"] for value in spheres]),
            np.asarray([value["radius"] for value in spheres]),
        )
    return metrics


def _compute_sphere_fit_metrics_cpu(
    mesh: trimesh.Trimesh,
    centers: np.ndarray,
    radii: np.ndarray,
) -> SphereFitMetrics:
    """Mirror CuRobo's quality metrics when its Warp query has no CPU path."""

    centers_t = torch.as_tensor(centers, dtype=torch.float32)
    radii_t = torch.as_tensor(radii, dtype=torch.float32)
    interior = trimesh.sample.volume_mesh(mesh, count=20_000)[:10_000]
    interior_t = torch.as_tensor(interior, dtype=torch.float32)
    interior_distance = torch.cdist(interior_t, centers_t)
    coverage = (
        (interior_distance < radii_t.unsqueeze(0)).any(dim=1).float().mean().item()
    )

    sphere_surface = []
    for center, radius in zip(centers_t, radii_t, strict=True):
        phi = torch.rand(200) * 2.0 * np.pi
        cos_theta = torch.rand(200) * 2.0 - 1.0
        sin_theta = torch.sqrt(1.0 - cos_theta**2)
        sphere_surface.append(
            center
            + radius
            * torch.stack(
                (
                    sin_theta * torch.cos(phi),
                    sin_theta * torch.sin(phi),
                    cos_theta,
                ),
                dim=1,
            )
        )
    sphere_surface_np = torch.cat(sphere_surface).numpy()
    signed_distance = trimesh.proximity.signed_distance(mesh, sphere_surface_np)
    outside = signed_distance < 0.0
    outside_distances = np.abs(signed_distance[outside])

    surface, _ = trimesh.sample.sample_surface(mesh, 5_000)
    surface_distance = torch.cdist(
        torch.as_tensor(surface, dtype=torch.float32),
        centers_t,
    ) - radii_t.unsqueeze(0)
    gaps = torch.relu(surface_distance.amin(dim=1))
    mesh_volume = (
        float(mesh.volume)
        if mesh.is_watertight and mesh.volume > 0.0
        else float(np.prod(mesh.bounds[1] - mesh.bounds[0]))
    )
    sphere_volume = float(4.0 / 3.0 * np.pi * torch.sum(radii_t**3).item())
    return SphereFitMetrics(
        num_spheres=len(centers),
        coverage=coverage,
        protrusion=float(np.mean(outside)),
        protrusion_dist_mean=(
            float(np.mean(outside_distances)) if len(outside_distances) else 0.0
        ),
        protrusion_dist_p95=(
            float(np.quantile(outside_distances, 0.95))
            if len(outside_distances)
            else 0.0
        ),
        surface_gap_mean=float(gaps.mean().item()),
        surface_gap_p95=float(torch.quantile(gaps, 0.95).item()),
        max_uncovered_gap=float(gaps.max().item()),
        volume_ratio=sphere_volume / max(mesh_volume, 1.0e-12),
    )


def _parse_refit(value: str) -> tuple[str, float]:
    try:
        link_name, raw_density = value.rsplit(":", 1)
        density = float(raw_density)
    except ValueError as error:
        raise ValueError(
            f"invalid --refit-link {value!r}; expected LINK:DENSITY"
        ) from error
    if not link_name or density <= 0.0:
        raise ValueError("refit link name and density must be positive")
    return link_name, density


def _validate_metrics(
    metrics: dict[str, Any],
    *,
    base_link: str,
    min_coverage: float,
    max_mean_protrusion_m: float,
    max_mean_surface_gap_m: float,
) -> None:
    if not metrics:
        raise ValueError("RobotBuilder did not produce sphere-fit metrics")
    failures = []
    for link_name, value in metrics.items():
        # Base-link coverage is intentionally reduced by the mounting-plane clip.
        if link_name == base_link:
            continue
        if value.coverage < min_coverage:
            failures.append(
                f"{link_name} coverage {value.coverage:.3f} < {min_coverage:.3f}"
            )
        if value.protrusion_dist_mean > max_mean_protrusion_m:
            failures.append(
                f"{link_name} mean protrusion {value.protrusion_dist_mean:.4f}m"
            )
        if value.surface_gap_mean > max_mean_surface_gap_m:
            failures.append(
                f"{link_name} mean surface gap {value.surface_gap_mean:.4f}m"
            )
    if failures:
        raise ValueError("collision-sphere quality validation failed:\n" + "\n".join(failures))


def _apply_authoritative_robot_contract(
    document: dict[str, Any],
    *,
    robot_config: RobotConfig,
    urdf_path: Path,
    config_path: Path,
) -> None:
    kinematics = document["kinematics"]
    expected_base = robot_config.kinematics.base_body
    expected_tool = robot_config.kinematics.tcp.parent_frame
    if kinematics.get("base_link") != expected_base:
        raise ValueError("RobotBuilder base link does not match RobotConfig")
    if tuple(kinematics.get("tool_frames", ())) != (expected_tool,):
        raise ValueError("RobotBuilder tool frame does not match RobotConfig")

    config_dir = config_path.resolve().parent
    kinematics["urdf_path"] = os.path.relpath(
        urdf_path.resolve(),
        start=config_dir,
    )
    kinematics["asset_root_path"] = os.path.relpath(
        _robot_asset_root(urdf_path),
        start=config_dir,
    )
    cspace = kinematics["cspace"]
    generated_joint_names = tuple(cspace["joint_names"])
    arm_joint_names = tuple(robot_config.kinematics.arm_joint_names)
    missing = [name for name in arm_joint_names if name not in generated_joint_names]
    if missing:
        raise ValueError(f"RobotBuilder cspace is missing arm joints: {missing}")
    indices = [generated_joint_names.index(name) for name in arm_joint_names]
    for field_name, value in tuple(cspace.items()):
        if isinstance(value, list) and len(value) == len(generated_joint_names):
            cspace[field_name] = [value[index] for index in indices]
    cspace["joint_names"] = list(arm_joint_names)
    cspace["default_joint_position"] = list(
        _urdf_reference_positions(urdf_path, arm_joint_names)
    )
    locked_joints = {
        name: robot_config.initial_joint_positions[name]
        for name in generated_joint_names
        if name not in arm_joint_names
    }
    kinematics["lock_joints"] = locked_joints or None

    attached_link = "attached_object"
    collision_links = list(kinematics["collision_link_names"])
    if attached_link not in collision_links:
        collision_links.append(attached_link)
    kinematics["collision_link_names"] = collision_links
    kinematics["extra_collision_spheres"] = {attached_link: 16}
    kinematics["extra_links"] = {
        attached_link: {
            "fixed_transform": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "joint_name": "attached_object_joint",
            "joint_type": "FIXED",
            "link_name": attached_link,
            "parent_link_name": expected_tool,
        }
    }
    kinematics["grasp_contact_link_names"] = [
        *robot_config.gripper.finger_body_names,
        attached_link,
    ]
    kinematics["self_collision_buffer"][attached_link] = 0.0
    ignore = kinematics["self_collision_ignore"]
    for link_name in (
        robot_config.kinematics.ee_body,
        *robot_config.gripper.finger_body_names,
    ):
        ignored = ignore.setdefault(link_name, [])
        if attached_link not in ignored:
            ignored.append(attached_link)


def _urdf_reference_positions(
    urdf_path: Path,
    joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    """Use URDF limit midpoints as CuRobo's interior reference posture."""

    root = ET.parse(urdf_path).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    references = []
    for joint_name in joint_names:
        joint = joints.get(joint_name)
        limit = None if joint is None else joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF joint {joint_name!r} has no finite limits")
        lower = float(limit.get("lower", "nan"))
        upper = float(limit.get("upper", "nan"))
        if not np.isfinite(lower) or not np.isfinite(upper) or lower >= upper:
            raise ValueError(f"URDF joint {joint_name!r} has invalid limits")
        references.append((lower + upper) / 2.0)
    return tuple(references)


def _metrics_document(
    link_metrics: dict[str, Any],
    document: dict[str, Any],
    robot_config: RobotConfig,
    args: argparse.Namespace,
    *,
    source_generated: Path | None,
    cuda_load_validated: bool,
) -> dict[str, Any]:
    return {
        "generator": "scripts/generate_curobo_robot_config.py",
        "robot_config": str(Path(args.robot_config).resolve().relative_to(PROJECT_ROOT)),
        "authoritative_urdf": str(
            Path(robot_config.urdf_path).resolve().relative_to(PROJECT_ROOT)
        ),
        "seed": args.seed,
        "sphere_density": args.sphere_density,
        "refit_links": list(args.refit_link),
        "base_mount_clip": {robot_config.kinematics.base_body: ["z", 0.0]},
        "collision_pruning_enabled": (
            args.prune_collisions
        ),
        "reused_generated_source": (
            source_generated.name if source_generated is not None else None
        ),
        "stale_refit_non_neighbor_ignores_removed": source_generated is not None,
        "full_curobo_cuda_load_validated": cuda_load_validated,
        "total_fitted_spheres": sum(
            len(value)
            for value in document["kinematics"]["collision_spheres"].values()
        ),
        "links": {
            link_name: vars(metrics)
            for link_name, metrics in sorted(link_metrics.items())
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())
