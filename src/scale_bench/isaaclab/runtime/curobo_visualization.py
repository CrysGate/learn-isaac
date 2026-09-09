"""Browse exact CuRobo collision snapshots after an episode finishes."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from curobo.scene import Cuboid, Sphere

from scale_bench.skills.geometry import compose_pose
from scale_bench.skills.models import Arm, Pose
from scale_bench.skills.planner import PlanningStage

if TYPE_CHECKING:
    from isaaclab.markers import VisualizationMarkers

LOGGER = logging.getLogger(__name__)
STAGE_ORDER: tuple[PlanningStage, ...] = (
    "pre_grasp",
    "grasp",
    "lift",
    "pre_place",
    "adjust",
    "place",
    "retreat",
    "clear",
)


@dataclass(frozen=True, slots=True)
class _CollisionSnapshot:
    arm: Arm
    sphere_positions_world_m: tuple[tuple[float, float, float], ...]
    sphere_scales_m: tuple[tuple[float, float, float], ...]
    sphere_marker_indices: tuple[int, ...]
    cuboid_positions_world_m: tuple[tuple[float, float, float], ...]
    cuboid_orientations_world_xyzw: tuple[tuple[float, float, float, float], ...]
    cuboid_scales_m: tuple[tuple[float, float, float], ...]
    cuboid_marker_indices: tuple[int, ...]


class CuroboPlanningVisualizer:
    """Collect selected planning stages and browse them in a paused Kit viewer."""

    def __init__(
        self,
        env_origin_world_m: tuple[float, float, float],
    ) -> None:
        import isaaclab.sim as sim_utils
        import omni.ui
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        self._env_origin_world_m = env_origin_world_m
        self._pending_snapshots: dict[PlanningStage, _CollisionSnapshot] = {}
        self._captured_snapshots: dict[PlanningStage, _CollisionSnapshot] = {}
        self._stage_order: tuple[PlanningStage, ...] = ()
        self._current_stage_index = 0

        self._sphere_markers: VisualizationMarkers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/CuRobo/RobotCollisionSpheres",
                markers={
                    "robot": sim_utils.SphereCfg(
                        radius=1.0,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(0.1, 0.65, 1.0),
                            emissive_color=(0.02, 0.13, 0.2),
                            opacity=0.45,
                        ),
                    ),
                    "attached_object": sim_utils.SphereCfg(
                        radius=1.0,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(1.0, 0.45, 0.05),
                            emissive_color=(0.2, 0.09, 0.01),
                            opacity=0.65,
                        ),
                    ),
                },
            )
        )
        self._cuboid_markers: VisualizationMarkers = VisualizationMarkers(
            VisualizationMarkersCfg(
                prim_path="/Visuals/CuRobo/WorldCuboids",
                markers={
                    "table": sim_utils.CuboidCfg(
                        size=(1.0, 1.0, 1.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(1.0, 0.8, 0.1),
                            opacity=0.18,
                        ),
                    ),
                    "object": sim_utils.CuboidCfg(
                        size=(1.0, 1.0, 1.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(1.0, 0.2, 0.12),
                            opacity=0.28,
                        ),
                    ),
                    "camera_stand": sim_utils.CuboidCfg(
                        size=(1.0, 1.0, 1.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(0.65, 0.65, 0.7),
                            opacity=0.3,
                        ),
                    ),
                    "other_arm": sim_utils.CuboidCfg(
                        size=(1.0, 1.0, 1.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(0.2, 1.0, 0.45),
                            opacity=0.22,
                        ),
                    ),
                },
            )
        )
        self._sphere_markers.set_visibility(False)
        self._cuboid_markers.set_visibility(False)

        self._window = omni.ui.Window(
            "CuRobo collision stages",
            width=360,
            height=96,
            visible=False,
        )
        with self._window.frame:
            with omni.ui.VStack(spacing=6):
                self._status_label = omni.ui.Label("", height=28)
                with omni.ui.HStack(height=30, spacing=6):
                    omni.ui.Button(
                        "<",
                        width=48,
                        tooltip="Previous stage",
                        clicked_fn=self._previous_stage,
                    )
                    omni.ui.Button(
                        ">",
                        width=48,
                        tooltip="Next stage",
                        clicked_fn=self._next_stage,
                    )
                    omni.ui.Button(
                        "Close",
                        tooltip="Close collision stage browser",
                        clicked_fn=self._close_browser,
                    )

    def capture(
        self,
        stage: PlanningStage,
        arm: Arm,
        active_robot_spheres_base: list[Sphere],
        attached_sphere_indices: set[int],
        arm_base_pose_env: Pose,
        table_cuboid_base: Cuboid,
        camera_stand_cuboids_base: list[Cuboid],
        object_cuboids_base: list[Cuboid],
        other_arm_cuboids_base: list[Cuboid],
    ) -> None:
        """Cache one successful solver input until its operation selects it."""

        sphere_positions_world_m = []
        sphere_scales_m = []
        sphere_marker_indices = []
        for sphere_index, sphere in enumerate(active_robot_spheres_base):
            sphere_radius_m = float(sphere.radius)
            if sphere_radius_m <= 0.0:
                continue
            sphere_pose_base = Pose(
                tuple(float(value) for value in sphere.pose[:3]),
                (0.0, 0.0, 0.0, 1.0),
            )
            sphere_pose_world = self._pose_world_from_base(
                arm_base_pose_env,
                sphere_pose_base,
            )
            sphere_positions_world_m.append(sphere_pose_world.position_m)
            sphere_scales_m.append((sphere_radius_m,) * 3)
            sphere_marker_indices.append(
                1 if sphere_index in attached_sphere_indices else 0
            )

        typed_cuboids_base = [
            (table_cuboid_base, 0),
            *((cuboid, 1) for cuboid in object_cuboids_base),
            *((cuboid, 2) for cuboid in camera_stand_cuboids_base),
            *((cuboid, 3) for cuboid in other_arm_cuboids_base),
        ]
        cuboid_positions_world_m = []
        cuboid_orientations_world_xyzw = []
        cuboid_scales_m = []
        cuboid_marker_indices = []
        for cuboid_base, marker_index in typed_cuboids_base:
            cuboid_pose_base = Pose(
                tuple(float(value) for value in cuboid_base.pose[:3]),
                (
                    float(cuboid_base.pose[4]),
                    float(cuboid_base.pose[5]),
                    float(cuboid_base.pose[6]),
                    float(cuboid_base.pose[3]),
                ),
            )
            cuboid_pose_world = self._pose_world_from_base(
                arm_base_pose_env,
                cuboid_pose_base,
            )
            cuboid_positions_world_m.append(cuboid_pose_world.position_m)
            cuboid_orientations_world_xyzw.append(
                cuboid_pose_world.orientation_xyzw
            )
            cuboid_scales_m.append(tuple(float(value) for value in cuboid_base.dims))
            cuboid_marker_indices.append(marker_index)

        self._pending_snapshots[stage] = _CollisionSnapshot(
            arm=arm,
            sphere_positions_world_m=tuple(sphere_positions_world_m),
            sphere_scales_m=tuple(sphere_scales_m),
            sphere_marker_indices=tuple(sphere_marker_indices),
            cuboid_positions_world_m=tuple(cuboid_positions_world_m),
            cuboid_orientations_world_xyzw=tuple(
                cuboid_orientations_world_xyzw
            ),
            cuboid_scales_m=tuple(cuboid_scales_m),
            cuboid_marker_indices=tuple(cuboid_marker_indices),
        )

    def commit(self, stages: tuple[PlanningStage, ...]) -> None:
        """Keep snapshots only after the enclosing operation selected them."""

        for stage in stages:
            snapshot = self._pending_snapshots.get(stage)
            if snapshot is None:
                raise RuntimeError(f"CuRobo stage {stage!r} has no successful snapshot")
            self._captured_snapshots[stage] = snapshot

    def browse(self) -> None:
        """Block on a paused post-run browser for all selected stage snapshots."""

        if not self._pending_snapshots and not self._captured_snapshots:
            LOGGER.warning("CuRobo stage browser has no attempted stages to display")
            return

        import omni.kit.app
        import omni.timeline

        self._stage_order = tuple(
            stage
            for stage in STAGE_ORDER
            if stage in self._captured_snapshots or stage in self._pending_snapshots
        )
        self._current_stage_index = 0
        self._sphere_markers.set_visibility(True)
        self._cuboid_markers.set_visibility(True)

        timeline = omni.timeline.get_timeline_interface()
        simulation_was_playing = timeline.is_playing()
        if simulation_was_playing:
            timeline.pause()
        try:
            self._show_current_stage()
            self._window.visible = True
            app = omni.kit.app.get_app()
            while app.is_running() and self._window.visible:
                app.update()
        finally:
            self._window.visible = False
            self._sphere_markers.set_visibility(False)
            self._cuboid_markers.set_visibility(False)
            if simulation_was_playing:
                timeline.play()

    def _show_current_stage(self) -> None:
        stage = self._stage_order[self._current_stage_index]
        if stage in self._captured_snapshots:
            snapshot = self._captured_snapshots[stage]
            selection_status = "selected"
        else:
            snapshot = self._pending_snapshots[stage]
            selection_status = "attempted"
        self._sphere_markers.visualize(
            translations=torch.tensor(
                snapshot.sphere_positions_world_m,
                dtype=torch.float32,
            ),
            scales=torch.tensor(snapshot.sphere_scales_m, dtype=torch.float32),
            marker_indices=torch.tensor(
                snapshot.sphere_marker_indices,
                dtype=torch.int32,
            ),
        )
        self._cuboid_markers.visualize(
            translations=torch.tensor(
                snapshot.cuboid_positions_world_m,
                dtype=torch.float32,
            ),
            orientations=torch.tensor(
                snapshot.cuboid_orientations_world_xyzw,
                dtype=torch.float32,
            ),
            scales=torch.tensor(snapshot.cuboid_scales_m, dtype=torch.float32),
            marker_indices=torch.tensor(
                snapshot.cuboid_marker_indices,
                dtype=torch.int32,
            ),
        )
        self._status_label.text = (
            f"{self._current_stage_index + 1}/{len(self._stage_order)}  "
            f"{stage}  {selection_status}  arm={snapshot.arm}"
        )
        attached_sphere_count = sum(snapshot.sphere_marker_indices)
        LOGGER.info(
            "CuRobo stage=%s status=%s arm=%s robot_spheres=%d "
            "attached_spheres=%d world_cuboids=%d",
            stage,
            selection_status,
            snapshot.arm,
            len(snapshot.sphere_marker_indices) - attached_sphere_count,
            attached_sphere_count,
            len(snapshot.cuboid_marker_indices),
        )

    def _previous_stage(self) -> None:
        self._current_stage_index = (
            self._current_stage_index - 1
        ) % len(self._stage_order)
        self._show_current_stage()

    def _next_stage(self) -> None:
        self._current_stage_index = (
            self._current_stage_index + 1
        ) % len(self._stage_order)
        self._show_current_stage()

    def _close_browser(self) -> None:
        self._window.visible = False

    def _pose_world_from_base(
        self,
        arm_base_pose_env: Pose,
        subject_pose_base: Pose,
    ) -> Pose:
        subject_pose_env = compose_pose(arm_base_pose_env, subject_pose_base)
        return Pose(
            tuple(
                coordinate_env_m + env_origin_coordinate_world_m
                for coordinate_env_m, env_origin_coordinate_world_m in zip(
                    subject_pose_env.position_m,
                    self._env_origin_world_m,
                    strict=True,
                )
            ),
            subject_pose_env.orientation_xyzw,
        )


__all__ = ["CuroboPlanningVisualizer"]
