"""Build and interact with an Isaac Sim grasp visualization scene."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, TYPE_CHECKING

import omni.ui
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.util.debug_draw import _debug_draw
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

from grasp_data_gen.isaac.geometry import Pose, base_pose_from_tcp_pose, from_pose_data
from grasp_data_gen.isaac.usd import new_stage, reference_asset, reference_materials, set_local_pose
from grasp_data_gen.models import GraspFileData, SuccessfulGrasp

if TYPE_CHECKING:
    from grasp_data_gen.isaac.scene import EvaluationScene


WORLD_PATH = "/World"
OBJECT_PATH = f"{WORLD_PATH}/Object"
CAMERA_PATH = "/OmniverseKit_Persp"
CAMERA_DISTANCE_IN_RADII = 3.0
CAMERA_DIRECTION = Gf.Vec3d(1.25, 1.4, 0.9).GetNormalized()
EVALUATION_MINIMUM_CAMERA_RADIUS_M = 0.1
VIEWPORT_WARMUP_FRAMES = 8


@dataclass(frozen=True)
class ViewerScene:
    stage: Usd.Stage
    grasp_roots: tuple[Usd.Prim, ...]
    tcp_poses: tuple[Pose, ...]


def _add_lighting(stage: Usd.Stage) -> None:
    dome = UsdLux.DomeLight.Define(stage, f"{WORLD_PATH}/DomeLight")
    dome.CreateIntensityAttr(800.0)
    key = UsdLux.DistantLight.Define(stage, f"{WORLD_PATH}/KeyLight")
    key.CreateIntensityAttr(1800.0)
    key.CreateAngleAttr(2.0)
    set_local_pose(
        key.GetPrim(),
        orientation=Gf.Quatd(
            Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -35.0).GetQuat()
        ),
    )


def _validate_gripper_assets(
    data: GraspFileData,
    robot_usd: Path,
) -> Sdf.Path:
    source_stage = Usd.Stage.Open(str(robot_usd))
    if source_stage is None:
        raise RuntimeError(f"could not open robot USD: {robot_usd}")
    source_root = Sdf.Path(data.gripper_definition.source_root_prim)
    if not source_root.IsAbsolutePath() or not source_root.IsPrimPath():
        raise ValueError(
            "gripper_definition.source_root_prim is not a valid prim path"
        )
    for link_name in data.gripper_definition.link_names:
        source_path = source_root.AppendPath(link_name)
        if not source_stage.GetPrimAtPath(source_path).IsValid():
            raise ValueError(f"robot USD is missing configured link: {source_path}")
    return source_root


def build_viewer_scene(
    data: GraspFileData,
    robot_usd: Path,
    object_usd: Path,
) -> ViewerScene:
    """Compose the object and stored closed-gripper poses for visualization."""

    stage = new_stage(WORLD_PATH)
    object_prim = reference_asset(stage, OBJECT_PATH, object_usd)
    set_local_pose(object_prim)

    _add_lighting(stage)

    source_root = _validate_gripper_assets(data, robot_usd)
    stage.DefinePrim(data.gripper_definition.source_root_prim, "Scope")
    reference_materials(
        stage,
        robot_usd,
        source_root,
        data.gripper_definition.material_prim_paths,
    )

    base_to_tcp = from_pose_data(data.tcp_definition.base_to_tcp)
    roots = []
    tcp_poses = []
    for rank, grasp in enumerate(data.grasps):
        tcp_pose = from_pose_data(grasp.tcp_pose)
        base_pose = base_pose_from_tcp_pose(tcp_pose, base_to_tcp)
        root_path = f"{WORLD_PATH}/Grasps/grasp_{rank}"
        root = UsdGeom.Xform.Define(stage, root_path).GetPrim()
        set_local_pose(root, *base_pose)
        for link_name, pose in grasp.evaluation.link_poses_base.items():
            link = reference_asset(
                stage,
                f"{root_path}/{link_name}",
                robot_usd,
                source_root.AppendPath(link_name),
            )
            set_local_pose(link, *from_pose_data(pose))
        roots.append(root)
        tcp_poses.append(tcp_pose)

    return ViewerScene(
        stage=stage,
        grasp_roots=tuple(roots),
        tcp_poses=tuple(tcp_poses),
    )


def _combined_world_range(prims: Iterable[Usd.Prim]) -> Gf.Range3d | None:
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    )
    combined = Gf.Range3d()
    for prim in prims:
        combined.UnionWith(cache.ComputeWorldBound(prim).ComputeAlignedRange())
    return None if combined.IsEmpty() else combined


def _fit_camera(prims: Iterable[Usd.Prim], minimum_radius: float) -> None:
    bounds = _combined_world_range(prims)
    if bounds is None:
        return
    center = 0.5 * (bounds.GetMin() + bounds.GetMax())
    radius = max(0.5 * bounds.GetSize().GetLength(), minimum_radius)
    eye = center + CAMERA_DIRECTION * (radius * CAMERA_DISTANCE_IN_RADII)
    ViewportManager.set_camera_view(
        CAMERA_PATH,
        eye=list(eye),
        target=list(center),
    )


def configure_evaluation_preview(
    scene: EvaluationScene,
    simulation_app: Any,
) -> None:
    """Light and frame an interactive grasp evaluation scene."""

    _add_lighting(scene.stage)
    for _ in range(VIEWPORT_WARMUP_FRAMES):
        simulation_app.update()
    _fit_camera(
        (scene.object_prim, scene.gripper.root_prim),
        EVALUATION_MINIMUM_CAMERA_RADIUS_M,
    )
    simulation_app.update()


class GraspViewer:
    """Manage grasp visibility, TCP axes, and the viewport toolbar."""

    AXIS_COLORS = (
        (1.0, 0.15, 0.15, 1.0),
        (0.2, 1.0, 0.25, 1.0),
        (0.2, 0.55, 1.0, 1.0),
    )
    AXIS_TO_OBJECT_RATIO = 0.2
    FALLBACK_AXIS_LENGTH_M = 0.04

    def __init__(
        self,
        scene: ViewerScene,
        grasps: tuple[SuccessfulGrasp, ...],
        *,
        start_index: int,
        show_all: bool,
        headless: bool,
    ) -> None:
        self._scene = scene
        self._grasps = grasps
        self._current = start_index
        self._show_all = show_all
        object_range = _combined_world_range(
            [scene.stage.GetPrimAtPath(OBJECT_PATH)]
        )
        object_diagonal = (
            0.0 if object_range is None else object_range.GetSize().GetLength()
        )
        self._axis_length_m = (
            self.AXIS_TO_OBJECT_RATIO * object_diagonal
            if object_diagonal > 0.0
            else self.FALLBACK_AXIS_LENGTH_M
        )
        self._draw = _debug_draw.acquire_debug_draw_interface()
        self._window = None
        self._status_label = None

        if not headless:
            self._build_ui()
        self.refresh()

    def _build_ui(self) -> None:
        self._window = omni.ui.Window(
            "Grasp poses",
            width=320,
            height=104,
        )
        with self._window.frame:
            with omni.ui.VStack(spacing=6):
                self._status_label = omni.ui.Label("", height=24)
                with omni.ui.HStack(height=30, spacing=6):
                    omni.ui.Button(
                        "<",
                        width=44,
                        tooltip="Previous grasp",
                        clicked_fn=self.previous,
                    )
                    omni.ui.Button(
                        ">",
                        width=44,
                        tooltip="Next grasp",
                        clicked_fn=self.next,
                    )
                    omni.ui.Button(
                        "All",
                        width=64,
                        tooltip="Toggle all grasps",
                        clicked_fn=self.toggle_all,
                    )
                    omni.ui.Button(
                        "Reset camera",
                        tooltip="Fit the object and gripper in view",
                        clicked_fn=self.fit_camera,
                    )

    def _visible_indices(self) -> range | tuple[int]:
        return (
            range(len(self._scene.grasp_roots))
            if self._show_all
            else (self._current,)
        )

    def _update_status(self) -> None:
        if self._status_label is None:
            return
        if self._show_all:
            self._status_label.text = (
                f"All accepted grasps: {len(self._scene.grasp_roots)}"
            )
            return
        grasp = self._grasps[self._current]
        self._status_label.text = (
            f"Rank {self._current + 1}/{len(self._scene.grasp_roots)}  "
            f"candidate {grasp.candidate_id}  score {grasp.score:.3f}"
        )

    def _draw_tcp_axes(self) -> None:
        self._draw.clear_lines()
        starts = []
        ends = []
        colors = []
        widths = []
        local_axes = (
            Gf.Vec3d(self._axis_length_m, 0.0, 0.0),
            Gf.Vec3d(0.0, self._axis_length_m, 0.0),
            Gf.Vec3d(0.0, 0.0, self._axis_length_m),
        )
        for index in self._visible_indices():
            position, orientation = self._scene.tcp_poses[index]
            for local_axis, color in zip(
                local_axes, self.AXIS_COLORS, strict=True
            ):
                starts.append(tuple(position))
                ends.append(tuple(position + orientation.Transform(local_axis)))
                colors.append(color)
                widths.append(4.0)
        self._draw.draw_lines(starts, ends, colors, widths)

    def refresh(self) -> None:
        visible = set(self._visible_indices())
        for index, root in enumerate(self._scene.grasp_roots):
            imageable = UsdGeom.Imageable(root)
            if index in visible:
                imageable.MakeVisible()
            else:
                imageable.MakeInvisible()
        self._draw_tcp_axes()
        self._update_status()

    def previous(self) -> None:
        self._show_all = False
        self._current = (self._current - 1) % len(self._scene.grasp_roots)
        self.refresh()

    def next(self) -> None:
        self._show_all = False
        self._current = (self._current + 1) % len(self._scene.grasp_roots)
        self.refresh()

    def toggle_all(self) -> None:
        self._show_all = not self._show_all
        self.refresh()

    def fit_camera(self) -> None:
        prims = [self._scene.stage.GetPrimAtPath(OBJECT_PATH)]
        prims.extend(
            self._scene.grasp_roots[index] for index in self._visible_indices()
        )
        _fit_camera(prims, 4.0 * self._axis_length_m)

    def close(self) -> None:
        self._draw.clear_lines()
        _debug_draw.release_debug_draw_interface(self._draw)
        if self._window is not None:
            self._window.visible = False
