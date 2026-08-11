"""Native cfg builder tests that do not require a running simulation."""

from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest
import yaml

from scale_bench.config.models.camera import CameraConfig
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.config.models.simulation import SimulationConfig
from scale_bench.isaaclab.builders.camera import build_camera_cfg
from scale_bench.isaaclab.builders.environment import build_environment_cfg
from scale_bench.isaaclab.builders.rigid_object_task import RigidObjectTaskBuilder
from scale_bench.isaaclab.builders.robot import build_robot_cfg
from scale_bench.isaaclab.builders.scene import build_scene_cfg
from scale_bench.isaaclab.builders.simulation import build_simulation_cfg
from scale_bench.isaaclab.managers.actions import build_actions_cfg
from scale_bench.isaaclab.managers.observations import build_observations_cfg
from scale_bench.tasks.common.layout import AssetPlacement, TaskLayout
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.common.rigid_object import (
    RigidObjectAssetConfig,
    RigidObjectPhysicsConfig,
    RigidObjectTask,
    RigidObjectTaskConfig,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ADAPTER_ROOT = PROJECT_ROOT / "src/scale_bench/isaaclab"
CAMERA_CONFIG_PATH = PROJECT_ROOT / "configs/cameras/d435.yml"


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture
def camera_config() -> CameraConfig:
    return CameraConfig.model_validate(_yaml(CAMERA_CONFIG_PATH))


@pytest.fixture
def robot_config() -> RobotConfig:
    data = _yaml(PROJECT_ROOT / "configs/robots/piper.yml")
    data["usd_path"] = "/fixtures/piper.usd"
    data["urdf_path"] = "/fixtures/piper.urdf"
    data["camera"]["profile_path"] = str(CAMERA_CONFIG_PATH)
    return RobotConfig.model_validate(data)


@pytest.fixture
def scene_config() -> SceneConfig:
    data = _yaml(PROJECT_ROOT / "configs/scene/default.yml")
    data["room"]["usd_path"] = "/fixtures/room.usd"
    data["ground"]["material_path"] = None
    data["table"]["material_path"] = None
    data["camera"]["profile_path"] = str(CAMERA_CONFIG_PATH)
    data["camera"]["stand_usd_path"] = "/fixtures/camera_stand.usd"
    data["lighting"]["texture_path"] = "/fixtures/studio.hdr"
    return SceneConfig.model_validate(data)


@pytest.fixture
def simulation_config() -> SimulationConfig:
    return SimulationConfig.model_validate(
        _yaml(PROJECT_ROOT / "configs/sim/default.yml")
    )


@pytest.fixture
def environment_config() -> EnvironmentConfig:
    return EnvironmentConfig.model_validate(
        _yaml(PROJECT_ROOT / "configs/envs/default.yml")
    )


def test_adapter_package_initializers_are_lightweight() -> None:
    for path in [ADAPTER_ROOT / "__init__.py", *ADAPTER_ROOT.glob("*/__init__.py")]:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        imports = [
            node
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
        ]
        assert imports == [], path.relative_to(PROJECT_ROOT)


def test_adapter_does_not_depend_on_compatibility_packages() -> None:
    forbidden = {
        "scale_bench.envs",
        "scale_bench.robots",
        "scale_bench.scenes",
        "scale_bench.sensors",
        "scale_bench.sim",
    }
    violations: list[str] = []
    for path in ADAPTER_ROOT.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or node.module is None:
                continue
            if any(
                node.module == root or node.module.startswith(root + ".")
                for root in forbidden
            ):
                violations.append(f"{path.relative_to(PROJECT_ROOT)}:{node.lineno}")
    assert violations == []


def test_camera_builder_returns_independent_cfgs(
    camera_config: CameraConfig,
) -> None:
    kwargs = {
        "prim_path": "{ENV_REGEX_NS}/Camera",
        "position_m": (0.0, 0.0, 0.0),
        "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "convention": "opengl",
    }
    first = build_camera_cfg(camera_config, **kwargs)
    second = build_camera_cfg(camera_config, **kwargs)

    assert first is not second
    assert first.spawn is not second.spawn
    assert (first.width, first.height) == (640, 480)
    assert first.data_types == ["rgb", "distance_to_image_plane"]


def test_robot_builder_uses_authored_usd_dimensions(
    robot_config: RobotConfig,
) -> None:
    first = build_robot_cfg(robot_config)
    second = build_robot_cfg(robot_config)

    assert first is not second
    assert first.spawn is not second.spawn
    assert first.spawn.usd_path == robot_config.usd_path
    assert first.spawn.scale is None
    assert list(first.init_state.joint_pos) == [
        *robot_config.kinematics.arm_joint_names,
        *robot_config.gripper.joint_names,
    ]


def test_scene_and_manager_builders_preserve_public_order(
    robot_config: RobotConfig,
    scene_config: SceneConfig,
    environment_config: EnvironmentConfig,
) -> None:
    scene_cfg = build_scene_cfg(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        scene_config=scene_config,
        environment_config=environment_config,
        num_envs=2,
    )
    actions = build_actions_cfg(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        arm_action_mode="joint_position",
    )
    observations = build_observations_cfg(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        scene_cfg=scene_cfg,
    )

    assert scene_cfg.num_envs == 2
    assert scene_cfg.left_robot is not scene_cfg.right_robot
    assert scene_cfg.left_robot_camera is not scene_cfg.right_robot_camera
    assert list(vars(actions)) == [
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    ]
    policy_terms = [
        name
        for name in vars(observations.policy)
        if name.startswith(("left_", "right_", "overhead_"))
    ]
    assert policy_terms == [
        "left_arm_joint_pos",
        "left_gripper_joint_pos",
        "right_arm_joint_pos",
        "right_gripper_joint_pos",
        "left_robot_camera_rgb",
        "left_robot_camera_depth",
        "right_robot_camera_rgb",
        "right_robot_camera_depth",
        "overhead_camera_rgb",
        "overhead_camera_depth",
    ]


@pytest.mark.parametrize(
    ("overrides", "field_name"),
    [
        ({"num_envs": 0}, "num_envs"),
        ({"env_spacing_m": -1.0}, "env_spacing_m"),
    ],
)
def test_scene_builder_validates_environment_overrides(
    robot_config: RobotConfig,
    scene_config: SceneConfig,
    environment_config: EnvironmentConfig,
    overrides: dict,
    field_name: str,
) -> None:
    with pytest.raises(ValueError, match=field_name):
        build_scene_cfg(
            left_robot_config=robot_config,
            right_robot_config=robot_config,
            scene_config=scene_config,
            environment_config=environment_config,
            **overrides,
        )


def test_simulation_and_environment_builders_validate_timing(
    robot_config: RobotConfig,
    scene_config: SceneConfig,
    simulation_config: SimulationConfig,
    environment_config: EnvironmentConfig,
) -> None:
    sim_cfg = build_simulation_cfg(simulation_config, device="cpu")
    env_cfg = build_environment_cfg(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        scene_config=scene_config,
        simulation_config=simulation_config,
        environment_config=environment_config,
        device="cpu",
        num_envs=2,
    )

    assert sim_cfg.device == "cpu"
    assert env_cfg.scene.num_envs == 2
    assert env_cfg.sim.dt * env_cfg.decimation == pytest.approx(1.0 / 30.0)

    mismatched = simulation_config.model_copy(update={"render_interval": 2})
    with pytest.raises(ValueError, match="render_interval"):
        build_environment_cfg(
            left_robot_config=robot_config,
            right_robot_config=robot_config,
            scene_config=scene_config,
            simulation_config=mismatched,
            environment_config=environment_config,
            device="cpu",
        )


def test_rigid_object_native_cfg_is_owned_by_task_builder(
    tmp_path: Path,
    robot_config: RobotConfig,
    scene_config: SceneConfig,
    simulation_config: SimulationConfig,
    environment_config: EnvironmentConfig,
) -> None:
    class ExampleTask(RigidObjectTask):
        TASK_ID = "example"

    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps(
            {
                "physics": {
                    "size": [0.05, 0.05, 0.1],
                    "mass": 0.2,
                    "friction": 0.6,
                }
            }
        ),
        encoding="utf-8",
    )
    task = ExampleTask(
        RigidObjectTaskConfig(
            instruction="Place the object.",
            physics=RigidObjectPhysicsConfig(restitution=0.0),
        ),
        {
            "object": RigidObjectAssetConfig(
                usd_path=str(tmp_path / "object.usd"),
                metadata_path=str(metadata_path),
            )
        },
    )
    context = PlacementContext.from_scene_config(scene_config)
    layout = task.generate_layout(context, 7)

    first = RigidObjectTaskBuilder().build_assets(task, layout)
    second = RigidObjectTaskBuilder().build_assets(task, layout)

    assert "add_assets_to_scene" not in vars(RigidObjectTask)
    assert first["object"] is not second["object"]
    assert first["object"].spawn.usd_path == str(tmp_path / "object.usd")

    env_cfg = build_environment_cfg(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        scene_config=scene_config,
        simulation_config=simulation_config,
        environment_config=environment_config,
        task=task,
        task_layout_seed=7,
        device="cpu",
        num_envs=2,
    )
    assert env_cfg.scene.object.spawn.usd_path == str(tmp_path / "object.usd")
    assert [
        layout.seed for layout in env_cfg.events.task_layout.params["layouts"]
    ] == [7, 8]

    class MissingAssetBuilder:
        def build_assets(self, task, layout):
            del task, layout
            return {}

    with pytest.raises(ValueError, match="TaskBuilder assets do not match"):
        build_environment_cfg(
            left_robot_config=robot_config,
            right_robot_config=robot_config,
            scene_config=scene_config,
            simulation_config=simulation_config,
            environment_config=environment_config,
            task=task,
            task_layout_seed=7,
            task_builder=MissingAssetBuilder(),
            device="cpu",
        )


def test_seed_generated_layouts_are_validated_before_native_building(
    robot_config: RobotConfig,
    scene_config: SceneConfig,
    simulation_config: SimulationConfig,
    environment_config: EnvironmentConfig,
) -> None:
    class RejectSecondLayoutTask:
        task_id = "reject_second_layout"
        instruction = "Generate two layouts."

        def generate_layout(self, context, seed):
            del context
            return TaskLayout(
                task_id=self.task_id,
                seed=seed,
                assets={
                    "object": AssetPlacement(
                        position_m=(0.0, 0.0, 0.0),
                        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                    )
                },
            )

        def validate_layout(self, context, layout):
            del context
            if layout.seed == 8:
                raise ValueError("generated layout 8 was rejected")

    class UnusedBuilder:
        def build_assets(self, task, layout):
            raise AssertionError("builder must not run before layout validation")

    with pytest.raises(ValueError, match="generated layout 8 was rejected"):
        build_environment_cfg(
            left_robot_config=robot_config,
            right_robot_config=robot_config,
            scene_config=scene_config,
            simulation_config=simulation_config,
            environment_config=environment_config,
            task=RejectSecondLayoutTask(),
            task_layout_seed=7,
            task_builder=UnusedBuilder(),
            device="cpu",
            num_envs=2,
        )


def test_task_assets_are_resolved_before_native_building(
    tmp_path: Path,
) -> None:
    asset_root = tmp_path / "assets"
    asset_root.mkdir()
    (asset_root / "object.usd").write_text("usd", encoding="utf-8")
    (asset_root / "metadata.json").write_text(
        json.dumps(
            {
                "physics": {
                    "size": [0.05, 0.05, 0.1],
                    "mass": 0.2,
                    "friction": 0.6,
                }
            }
        ),
        encoding="utf-8",
    )
    config_path = tmp_path / "task.yml"
    config_path.write_text(
        "instruction: Place the object.\n"
        "physics:\n"
        "  restitution: 0.0\n"
        "asset:\n"
        "  usd_path: object.usd\n"
        "  metadata_path: metadata.json\n",
        encoding="utf-8",
    )

    class ConfigWithAsset(RigidObjectTaskConfig):
        asset: RigidObjectAssetConfig

    from scale_bench.config.loader import load_config

    config = load_config(config_path, ConfigWithAsset, asset_root=asset_root)

    assert config.asset.usd_path == str(asset_root / "object.usd")
    assert config.asset.metadata_path == str(asset_root / "metadata.json")
