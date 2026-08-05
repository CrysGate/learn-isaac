"""Open a YAML scene configuration in Isaac Sim."""

import argparse
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--config", type=Path, default=Path("configs/scene/default.yml"))
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
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"], enable_cameras=True)
args = parser.parse_args()
if args.max_steps is not None and args.max_steps <= 0:
    parser.error("--max-steps must be positive")

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene

from scale_bench.robots import RobotProfile
from scale_bench.scenes import create_dual_arm_tabletop_scene_cfg


def main() -> None:
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            device=args.device,
            render=sim_utils.RenderCfg(
                enable_translucency=True,
                enable_reflections=True,
                enable_global_illumination=True,
                antialiasing_mode="DLAA",
                dlss_mode=2,
                rendering_mode="quality",
            ),
        )
    )
    sim.set_camera_view((2.6, 2.2, 2.2), (0.0, 0.0, 0.8))

    left_profile = RobotProfile.load(args.left_robot_config)
    right_profile = RobotProfile.load(args.right_robot_config)
    scene = InteractiveScene(
        create_dual_arm_tabletop_scene_cfg(
            left_robot_cfg=left_profile.build_articulation_cfg(),
            right_robot_cfg=right_profile.build_articulation_cfg(),
            config_path=args.config,
        )
    )
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

    print(
        f"Scene loaded from {args.config} with "
        f"{left_profile.name} (left) and {right_profile.name} (right). "
        "Close the window to exit."
    )

    step_count = 0
    while simulation_app.is_running():
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
        step_count += 1
        if args.max_steps is not None and step_count >= args.max_steps:
            break


if __name__ == "__main__":
    main()
    simulation_app.close()
