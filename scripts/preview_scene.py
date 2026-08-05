"""Open a YAML scene configuration in Isaac Sim."""

import argparse
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--config", type=Path, default=Path("configs/scene/default.yml"))
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"], enable_cameras=True)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.scene import InteractiveScene

from scale_bench.scenes import create_dual_arm_tabletop_scene_cfg


def piper_cfg() -> ArticulationCfg:
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(REPOSITORY_ROOT / "Assets/Robots/piper/Piper.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True,
                enabled_self_collisions=False,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={"joint[1-6]": 0.0, "gripper_joint": 0.04, "joint8": 0.04},
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"], stiffness=400.0, damping=40.0
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["gripper_joint", "joint8"],
                stiffness=1000.0,
                damping=100.0,
            ),
        },
    )


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

    robot = piper_cfg()
    scene = InteractiveScene(
        create_dual_arm_tabletop_scene_cfg(
            left_robot_cfg=robot,
            right_robot_cfg=robot,
            config_path=args.config,
        )
    )
    sim.reset()
    print(f"Scene loaded from {args.config}. Close the window to exit.")

    while simulation_app.is_running():
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())


if __name__ == "__main__":
    main()
    simulation_app.close()
