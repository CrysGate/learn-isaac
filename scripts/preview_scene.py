"""Preview a common or task-extended YAML scene in Isaac Sim."""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

# Kept lightweight so argparse can reject unknown tasks before Isaac Sim starts.
SUPPORTED_TASK_IDS = ("sort_dolls_by_size",)

from scale_bench.sim import SimConfig

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--config", type=Path, default=Path("configs/scene/default.yml"))
parser.add_argument(
    "--sim-config",
    type=Path,
    default=Path("configs/sim/default.yml"),
    help="Simulation, PhysX, and rendering YAML preset.",
)
parser.add_argument(
    "--task",
    choices=SUPPORTED_TASK_IDS,
    default=None,
    help="Preview a task scene by its task ID; omit to preview the common scene.",
)
layout_source = parser.add_mutually_exclusive_group()
layout_source.add_argument(
    "--seed",
    type=int,
    default=None,
    help="Seed used to generate the task layout (default: 0).",
)
layout_source.add_argument(
    "--layout",
    type=Path,
    default=None,
    help="Initialize task assets from an exported layout JSON file.",
)
parser.add_argument(
    "--export-layout",
    type=Path,
    default=None,
    help="Export the generated or loaded task layout as JSON.",
)
parser.add_argument(
    "--left-robot-config",
    type=Path,
    default=Path("configs/robots/piper.yml"),
)
parser.add_argument(
    "--right-robot-config",
    type=Path,
    default=Path("configs/robots/piper.yml"),
)
parser.add_argument(
    "--max-steps",
    type=int,
    default=None,
    help="Exit after this many simulation steps; useful for headless smoke tests.",
)
parser.add_argument(
    "--camera-frustum-length-m",
    type=float,
    default=0.75,
    help="Visual length of camera frustums in metres.",
)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"], enable_cameras=True, device=None)
args = parser.parse_args()
if args.max_steps is not None and args.max_steps <= 0:
    parser.error("--max-steps must be positive")
if args.camera_frustum_length_m <= 0.0:
    parser.error("--camera-frustum-length-m must be positive")
if args.seed is not None and args.seed < 0:
    parser.error("--seed must be non-negative")
if args.task is None and (
    args.seed is not None or args.layout is not None or args.export_layout is not None
):
    parser.error("--seed, --layout, and --export-layout require --task")

try:
    sim_config = SimConfig.load(args.sim_config)
except ValueError as error:
    parser.error(str(error))
if args.device is None:
    args.device = sim_config.device
if args.rendering_mode is None:
    args.rendering_mode = sim_config.render.rendering_mode

preview_overlays_enabled = not args.headless and "kit" in (args.visualizer or ())
camera_frustum_length_m = args.camera_frustum_length_m

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene

if preview_overlays_enabled:
    from isaacsim.core.experimental.utils.app import enable_extension

    enable_extension("isaacsim.util.debug_draw")

    import omni.ui
    from isaacsim.util.debug_draw import _debug_draw
    from pxr import Gf

from scale_bench.robots import RobotProfile
from scale_bench.scenes import SceneConfig, create_dual_arm_tabletop_scene_cfg
from scale_bench.tasks import SortDollsBySize


Point = tuple[float, float, float]
Line = tuple[Point, Point]
Color = tuple[float, float, float, float]


def _camera_frustum_lines(camera, length_m: float) -> list[Line]:
    """Build world-space frustum lines from an Isaac Lab camera sensor."""

    height, width = camera.data.image_shape
    lines: list[Line] = []
    for position, quaternion, intrinsic in zip(
        camera.data.pos_w.torch.tolist(),
        camera.data.quat_w_world.torch.tolist(),
        camera.data.intrinsic_matrices.torch.tolist(),
        strict=True,
    ):
        fx, fy = intrinsic[0][0], intrinsic[1][1]
        cx, cy = intrinsic[0][2], intrinsic[1][2]
        rotation = Gf.Quatd(quaternion[3], Gf.Vec3d(*quaternion[:3]))

        corners: list[Point] = []
        for u, v in ((0.0, 0.0), (width, 0.0), (width, height), (0.0, height)):
            local_corner = Gf.Vec3d(
                length_m,
                -(u - cx) * length_m / fx,
                -(v - cy) * length_m / fy,
            )
            offset = rotation.Transform(local_corner)
            corners.append(tuple(position[index] + offset[index] for index in range(3)))

        origin = tuple(position)
        lines.extend((origin, corner) for corner in corners)
        lines.extend((corners[index], corners[(index + 1) % 4]) for index in range(4))
    return lines


class ScenePreviewOverlay:
    """Draw optional placement-area and camera-frustum overlays."""

    CAMERA_COLORS: dict[str, Color] = {
        "left_robot_camera": (0.0, 0.75, 1.0, 1.0),
        "right_robot_camera": (1.0, 0.35, 0.75, 1.0),
        "overhead_camera": (1.0, 0.75, 0.0, 1.0),
    }

    def __init__(
        self,
        scene: InteractiveScene,
        scene_config: SceneConfig,
        frustum_length_m: float,
    ) -> None:
        self._scene = scene
        self._scene_config = scene_config
        self._frustum_length_m = frustum_length_m
        self._draw = _debug_draw.acquire_debug_draw_interface()
        self._area_model = omni.ui.SimpleBoolModel(True)
        self._frustum_model = omni.ui.SimpleBoolModel(True)

        self._window = omni.ui.Window("Scene overlays", width=280, height=90)
        with self._window.frame:
            with omni.ui.VStack(spacing=4):
                with omni.ui.HStack(height=24):
                    omni.ui.Label("Placement area")
                    omni.ui.CheckBox(model=self._area_model, width=24)
                with omni.ui.HStack(height=24):
                    omni.ui.Label("Camera frustums")
                    omni.ui.CheckBox(model=self._frustum_model, width=24)

    def draw(self) -> None:
        """Redraw enabled overlays using the latest scene state."""

        self._draw.clear_lines()
        groups: list[tuple[list[Line], Color, float]] = []

        if self._area_model.as_bool:
            area = self._scene_config.task_object_placement_area
            z_m = self._scene_config.table_top_z_m + 0.003
            area_lines: list[Line] = []
            for origin in self._scene.env_origins.tolist():
                corners = [
                    (origin[0] + x_m, origin[1] + y_m, origin[2] + z_m)
                    for x_m, y_m in (
                        (area.x_range_m[0], area.y_range_m[0]),
                        (area.x_range_m[1], area.y_range_m[0]),
                        (area.x_range_m[1], area.y_range_m[1]),
                        (area.x_range_m[0], area.y_range_m[1]),
                    )
                ]
                area_lines.extend(
                    (corners[index], corners[(index + 1) % 4])
                    for index in range(4)
                )
            groups.append((area_lines, (0.2, 1.0, 0.2, 1.0), 4.0))

        if self._frustum_model.as_bool:
            for camera_name, color in self.CAMERA_COLORS.items():
                camera = self._scene.sensors.get(camera_name)
                if camera is not None:
                    groups.append(
                        (
                            _camera_frustum_lines(camera, self._frustum_length_m),
                            color,
                            2.0,
                        )
                    )

        styled_lines = [
            (line, color, width)
            for lines, color, width in groups
            for line in lines
        ]
        if styled_lines:
            self._draw.draw_lines(
                [line[0] for line, _, _ in styled_lines],
                [line[1] for line, _, _ in styled_lines],
                [color for _, color, _ in styled_lines],
                [width for _, _, width in styled_lines],
            )

    def close(self) -> None:
        self._draw.clear_lines()
        _debug_draw.release_debug_draw_interface(self._draw)
        self._window.visible = False


def main() -> None:
    sim = sim_utils.SimulationContext(
        sim_config.build_simulation_cfg(device=args.device)
    )
    sim.set_camera_view((2.6, 2.2, 2.2), (0.0, 0.0, 0.8))

    scene_config = SceneConfig.load(args.config)
    left_profile = RobotProfile.load(args.left_robot_config)
    right_profile = RobotProfile.load(args.right_robot_config)
    scene_cfg = create_dual_arm_tabletop_scene_cfg(
        left_robot_profile=left_profile,
        right_robot_profile=right_profile,
        config_path=args.config,
    )
    task = SortDollsBySize(scene_config=scene_config) if args.task is not None else None
    layout = None
    if task is not None:
        layout = task.add_assets_to_scene(
            scene_cfg,
            seed=args.seed,
            layout_path=args.layout,
            export_layout_path=args.export_layout,
        )
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    for robot_name in ("left_robot", "right_robot"):
        robot = scene[robot_name]
        joint_pos = robot.data.default_joint_pos.torch.clone()
        joint_vel = robot.data.default_joint_vel.torch.clone()
        robot.write_joint_state_to_sim_index(
            position=joint_pos,
            velocity=joint_vel,
            full_data=True,
        )
        robot.set_joint_position_target_index(
            target=joint_pos,
            full_data=True,
        )

    overlay = (
        ScenePreviewOverlay(
            scene,
            scene_config,
            camera_frustum_length_m,
        )
        if preview_overlays_enabled
        else None
    )

    preview_name = f"task '{task.task_id}'" if task is not None else "common scene"
    layout_message = ""
    if layout is not None:
        source = f"layout {args.layout}" if args.layout is not None else f"seed {layout.seed}"
        layout_message = f"Task assets use {source}. "
    print(
        f"Loaded {preview_name} from {args.config} with "
        f"{left_profile.name} (left) and {right_profile.name} (right). "
        f"Simulation runs at {sim_config.physics_frequency_hz:g} Hz and renders "
        f"at {sim_config.render_frequency_hz:g} Hz from {args.sim_config}. "
        f"{layout_message}Close the window to exit."
    )

    step_count = 0
    try:
        while simulation_app.is_running():
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())
            if overlay is not None:
                overlay.draw()
            step_count += 1
            if args.max_steps is not None and step_count >= args.max_steps:
                break
    finally:
        if overlay is not None:
            overlay.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
