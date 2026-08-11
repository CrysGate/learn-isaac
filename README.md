# ScaleBench

[English](README.md) | [简体中文](README.zh-CN.md)

Configuration-first Isaac Lab building blocks for scale-aware, dual-arm manipulation scenes.

ScaleBench keeps robot semantics, camera, scene, task, simulation, and environment parameters in YAML, validates them at the boundary, and compiles them into native Isaac Lab configuration objects. The current implementation provides reusable scene construction, one seed/layout-driven task, and a formal manager-based runtime entry, but not a complete policy or evaluation pipeline.

## What is implemented

- **Typed robot profiles** — validate joints, TCP, actuators, gripper, and an optional robot-mounted camera, then build fresh Isaac Lab configs.
- **Reusable camera profiles** — keep camera optics and output parameters separate from scene- and robot-local sensor poses.
- **Typed scene metadata** — validate every scene-preset section through dedicated nested models and expose task-object placement bounds in environment-local coordinates.
- **Typed simulation presets** — validate the few timing, gravity, rendering, and manipulation-stability settings that define benchmark behavior, while inheriting other Isaac Lab defaults.
- **A reusable dual-arm scene** — compose a room, textured ground and table, two independently configured robots with wrist cameras, an overhead RGB-D camera, and environment lighting.
- **A small task contract and first task** — add task-owned assets directly to a common scene from a deterministic seed or a reusable layout file; `SortDollsBySize` is the first robot-independent example.
- **A manager-based environment entry** — compose Scene, Task, Sim, runtime, and manager configs into `ScaleBenchEnvCfg`, with `ScaleBenchEnv` as the sole owner of reset, stepping, and simulation lifetime.
- **Profile-driven actions** — control both arms and grippers with dynamically sized absolute command-joint targets in physical units.
- **Named policy observations** — expose ordered robot state and raw RGB-D from configured cameras without leaking task or evaluation ground truth.
- **Texture-correct procedural surfaces** — `UvCuboidCfg` authors face-varying UV coordinates so MDL materials tile correctly on cuboids.
- **A runnable scene preview** — launch the default scene with placement-area and camera-frustum overlays, or run a short headless smoke test.

> [!NOTE]
> Joint-space Action and policy Observation Manager terms are connected. End-effector control, evaluators, episode orchestration, recording, and benchmark reporting remain future work.

## Architecture

```text
robot / scene / camera YAML ──► config.loader / pure models ───┐
task YAML + seed/layout ──────► TaskDefinition ─────────────────┤
sim YAML ─────────────────────► SimulationConfig ───────────────┤
env YAML ─────────────────────► EnvironmentConfig ──────────────┤
                                                                ▼
                                            create_env_cfg() / ScaleBenchEnvCfg
                                                                │
                                                                ▼
                                                      ScaleBenchEnv
                                           reset() / step() / IO descriptors
```

The boundary is intentionally small: robot-specific details stay in robot profiles, scene-specific details stay in scene presets, and downstream code receives standard Isaac Lab configuration objects.

## Requirements

The dependency configuration in `pyproject.toml` and `uv.lock` targets:

- Python 3.12
- Isaac Sim 6.0.1
- PyTorch 2.10 with CUDA 12.8 wheels
- [`uv`](https://docs.astral.sh/uv/) for dependency management

Isaac Lab is consumed as a local editable dependency from `third_parties/IsaacLab`; the known-compatible checkout used by this workspace is `release/3.0.0-beta2`. Your machine must also satisfy Isaac Sim's GPU, driver, and operating-system requirements. The project does not install system-level NVIDIA drivers.

## Installation

Clone the repository and place the expected Isaac Lab checkout under `third_parties/IsaacLab`:

```bash
git clone https://github.com/CrysGate/learn-isaac.git
cd learn-isaac

mkdir -p third_parties
git clone --branch release/3.0.0-beta2 --depth 1 \
  https://github.com/isaac-sim/IsaacLab.git \
  third_parties/IsaacLab

uv sync --frozen
```

The required project asset bundle is not tracked by this Git repository. Before using the default preset, obtain the canonical bundle separately from the project maintainer or your organization's asset storage, or provide compatible assets at these exact paths. Include every texture and transitive USD dependency referenced by these files:

```text
Assets/
├── Background/brown_photostudio_02_4k.hdr
├── Material/material_0122/Mahogany_Planks.mdl
├── Material/material_0564/Wood_Tiles_Fineline.mdl
├── Object/Geometry/camera_stand_aloha/aloha_front_camera_stand_realsense_d435.usd
├── Object/Rigid/matryoshka_dolls/{00000..00004}/{object.usdz,metadata.json}
├── Robots/piper/Piper.usd
├── Robots/piper/piper_description/urdf/piper.urdf
└── Room/Simple_Room_nolight/simple_room_nolight.usd
```

Configuration references resolve from their containing file. The preview passes `--asset-root .` by default, so run it from the asset-root directory or provide another explicit root.

## Quick start

Open the default dual-Piper scene in Isaac Sim:

```bash
uv run python scripts/preview_scene.py
```

Preview a task scene by its stable task ID:

```bash
uv run python scripts/preview_scene.py --task sort_dolls_by_size
```

Generate and export a reproducible layout, or load it again later:

```bash
uv run python scripts/preview_scene.py --task sort_dolls_by_size \
  --seed 42 --export-layout layouts/sort_dolls_by_size/42.json
uv run python scripts/preview_scene.py --task sort_dolls_by_size \
  --layout layouts/sort_dolls_by_size/42.json
```

Run a two-step headless smoke test:

```bash
uv run python scripts/preview_scene.py --viz none --max-steps 2
```

Use different robot or scene profiles without changing Python code:

```bash
uv run python scripts/preview_scene.py \
  --config configs/scene/default.yml \
  --sim-config configs/sim/default.yml \
  --env-config configs/envs/default.yml \
  --left-robot-config configs/robots/piper.yml \
  --right-robot-config configs/robots/piper.yml \
  --device cuda:0
```

Useful preview options:

| Option | Purpose |
|---|---|
| `--config PATH` | Select the scene YAML. |
| `--sim-config PATH` | Select simulation, rendering, and PhysX parameters. |
| `--env-config PATH` | Select control decimation and reset lifecycle parameters. |
| `--task TASK_ID` | Add a task's assets to the common scene (currently `sort_dolls_by_size`). |
| `--seed N` | Generate a deterministic task layout; defaults to zero. |
| `--layout PATH` | Load task asset poses from an exported layout JSON file. |
| `--export-layout PATH` | Save the generated or loaded task layout. |
| `--asset-root PATH` | Resolve asset references against an explicit root. |
| `--left-robot-config PATH` | Select the left robot profile. |
| `--right-robot-config PATH` | Select the right robot profile. |
| `--device VALUE` | Choose `cpu`, `cuda`, or a device such as `cuda:0`. |
| `--viz none` | Disable visualizers for headless execution. |
| `--max-steps N` | Exit after a bounded number of environment steps. |
| `--camera-frustum-length-m M` | Set the displayed camera-frustum length in metres. |

Run `uv run python scripts/preview_scene.py --help` for all Isaac Lab launcher options.

## Core API

### Configuration

[`load_config()`](src/scale_bench/config/loader.py) is the only YAML/JSON loading boundary. The five immutable pure-Python models contain data and local validation only:

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.config.models.simulation import SimulationConfig

robot = load_config("configs/robots/piper.yml", RobotConfig, asset_root=".")
scene = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
sim = load_config("configs/sim/default.yml", SimulationConfig)
environment = load_config("configs/envs/default.yml", EnvironmentConfig)
```

Models forbid unknown fields, are frozen, reject non-finite numeric values, and do not read files or build Isaac Lab objects. Configuration references resolve relative to the file containing them. Asset references resolve relative to `asset_root` when supplied, otherwise relative to the containing file. Absolute paths are preserved, and missing local assets are reported with the source path and field location. Only local configuration and asset paths are supported. `num_envs`, spacing, cloning, control, and reset settings belong to `EnvironmentConfig`, not `SceneConfig`.

The default simulation preset runs physics at 120 Hz and renders every four steps at 30 Hz. Materials, Fabric, logging, solver iterations, and GPU buffers inherit the installed Isaac Lab defaults.

### Environment runtime

[`create_env_cfg()`](src/scale_bench/envs/env_cfg.py) compiles loaded `RobotConfig`, `SceneConfig`, an optional task layout source, `SimulationConfig`, and `EnvironmentConfig` into a native `ScaleBenchEnvCfg`:

```python
from scale_bench.envs import ScaleBenchEnv, create_env_cfg

env_cfg = create_env_cfg(
    left_robot_profile=robot,
    right_robot_profile=robot,
    scene_config=scene,
    sim_config=sim,
    runtime_config=environment,
    task=task,
    task_layout_seed=42,
)
env = ScaleBenchEnv(env_cfg)
try:
    observation, info = env.reset()
finally:
    env.close()
```

`ScaleBenchEnv` subclasses Isaac Lab's `ManagerBasedEnv` and is the only owner of `SimulationContext`, `InteractiveScene`, reset, step, and cleanup. Runtime IO metadata is derived from the initialized environment instead of injected from build-time inputs. With `task_layout_seed`, environment `i` receives the layout generated from `task_layout_seed + i` once during configuration; every full or partial reset restores that same layout. Alternatively, `task_layouts` accepts either one layout to broadcast to every environment or exactly `num_envs` layouts mapped by `env_id`. `info["episode"]` reports the affected environment IDs and their stable layout seeds. The builder rejects presets where render or camera updates are not synchronized with `step_dt`. Action terms follow `left_arm | left_gripper | right_arm | right_gripper`; `observation["policy"]` is a named, non-concatenated robot-state and RGB-D dictionary. Runtime IO descriptors expose the resolved dimensions, slices, joint order, camera metadata, and timing.

### Robot configuration

[`RobotConfig`](src/scale_bench/config/models/robot.py) validates robot semantics independently of Isaac Sim. Loading it through `load_config()`:

- resolves configuration references relative to the robot YAML and assets against the explicit asset root;
- rejects unknown fields and non-finite numeric values;
- requires unique arm, gripper, and actuator joint names;
- verifies that initial positions exactly cover all declared joints;
- verifies actuator coverage and prevents overlapping actuator groups;
- validates the TCP, parallel-jaw gripper, and optional camera-mount contract;
- checks that local USD and optional URDF assets exist.

`scale_bench.robots.RobotProfile` remains as a temporary compatibility facade during migration. New code should use `RobotConfig` and `load_config()`.

### Camera configuration

[`CameraConfig`](src/scale_bench/config/models/camera.py) owns image dimensions, update period, data types, pinhole intrinsics, distortion metadata, focal length, clipping range, and coordinate convention:

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.camera import CameraConfig

camera = load_config("configs/cameras/d435.yml", CameraConfig)
```

Scene and robot configs reference it while keeping installation poses local to their owning asset. The D435 config is reused by both Piper wrist cameras and the overhead camera.

### Scene template

[`create_dual_arm_tabletop_scene_cfg()`](src/scale_bench/scenes/scene_template.py) currently adapts loaded pure configs into the native scene after `AppLauncher` has initialized Isaac Sim:

```python
from scale_bench.scenes import create_dual_arm_tabletop_scene_cfg

scene_cfg = create_dual_arm_tabletop_scene_cfg(
    left_robot_profile=robot,
    right_robot_profile=robot,
    scene_config=scene,
    environment_config=environment,
)
```

`SceneConfig` validates static scene sections, including asset references, finite poses, positive dimensions, material parameters, unit quaternions, camera conventions, and ordered XY placement bounds. Its `table_top_z_m` property and placement-area metadata can be reused without duplicating scene geometry calculations.

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.scene import SceneConfig

scene_metadata = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
placement_area = scene_metadata.task_object_placement_area
table_top_z_m = scene_metadata.table_top_z_m
```

The scene contains:

- a USD room and dome light;
- collision-enabled, textured ground and table surfaces;
- independent left and right robot mounts;
- left and right robot-mounted D435-style RGB-D sensors;
- a camera stand and overhead D435-style RGB-D sensor;
- environment count, spacing, physics replication, and Fabric cloning supplied by `EnvironmentConfig`.

Robot bases and the camera stand are positioned relative to the computed table-top height. Changing the table height therefore keeps mounted assets aligned automatically.

### Tasks

`TaskDefinition` keeps the shared task behavior in one place: metadata loading, deterministic sampling, placement validation, rigid-object construction, and layout JSON import/export. `SortDollsBySize` only declares its dolls, instruction, and size-order target. No task-specific scene subclass is needed; named `RigidObjectCfg` fields are added directly to the common `InteractiveSceneCfg` instance.

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.scene import SceneConfig
from scale_bench.tasks import SortDollsBySize

scene_metadata = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
task = SortDollsBySize(scene_config=scene_metadata)
layout = task.resolve_layout(seed=42)
layout.save("layouts/sort_dolls_by_size/42.json")
task.add_assets_to_scene(scene_cfg, layout)
instruction = task.instruction
target_order = task.target_order_small_to_large
```

Calling `task.resolve_layout(layout_path=...)` restores and validates the exact saved poses before they are registered with `add_assets_to_scene()`. Generated and loaded layouts are both checked against `task_object_placement_area`; each complete XY footprint stays inside the area and task objects maintain the configured minimum gap. Pass `config_path="configs/tasks/my_sort_dolls.yml"` to `SortDollsBySize` to use an alternative task YAML. Piper/curobo planning, robot assignment, recording, success evaluation, and the application lifecycle remain outside this task layer.

### UV cuboids

[`UvCuboidCfg`](src/scale_bench/scenes/uv_cuboid.py) extends Isaac Lab's `CuboidCfg` with an `uv_scale` setting. Its spawner delegates geometry and physics creation to Isaac Lab, then authors 24 face-varying `st` values—four per cube face—for predictable material tiling.

## Configuration

### Adding a robot

1. Copy [`configs/robots/piper.yml`](configs/robots/piper.yml).
2. Update the asset paths, initial joint state, kinematic frames, TCP, actuator groups, gripper semantics, and optional camera mount.
3. Keep quaternions in `xyzw` order and distances in metres.
4. Validate the profile without launching Isaac Sim:

   ```bash
   PYTHONPATH=src uv run python -c \
     'from scale_bench.config.loader import load_config; from scale_bench.config.models.robot import RobotConfig; print(load_config("configs/robots/my_robot.yml", RobotConfig, asset_root=".").name)'
   ```

5. Preview the robot in the scene with `--left-robot-config` or `--right-robot-config`.

Do not add robot-name branches to the scene code. If a new robot needs an unsupported actuator or end-effector type, extend the profile model explicitly.

### Customizing the scene

Copy [`configs/scene/default.yml`](configs/scene/default.yml) and edit the relevant section:

| Section | Controls |
|---|---|
| `room` | Room USD and uniform scale. |
| `ground`, `table` | Pose, dimensions, material, friction, restitution, and UV tiling. |
| `task_object_placement_area` | Task-object XY placement bounds in the environment-local frame. |
| `robot_mounts` | Left and right base poses relative to the table top. |
| `camera` | Camera profile reference, stand pose, and sensor transform. |
| `lighting` | HDR environment texture and intensity. |

Environment count, spacing, physics replication, and Fabric cloning are configured in `configs/envs/*.yml`.

### Customizing the simulation

Copy [`configs/sim/default.yml`](configs/sim/default.yml) to create a simulation preset. Timing and gravity are top-level settings; `render` selects observation quality, and `physx` contains the one manipulation-specific override currently justified by runtime behavior. Everything else follows Isaac Lab defaults and should only become public configuration after a benchmark requirement demonstrates that it must vary. Use `--sim-config` to select the preset and `--device` for a temporary machine-specific override.

Copy [`configs/envs/default.yml`](configs/envs/default.yml) to change environment cloning, arm action mode, control decimation, reset rerenders, texture waiting, or the environment seed. The current arm mode is `joint_position`; this explicit dispatch point is reserved for later end-effector modes. The builder requires render interval, control decimation, and camera update periods to describe one synchronous environment rate.

### Validating changes

Run `uv run pytest` for the pure configuration model, loader/path, and dependency-boundary tests. These tests do not exercise initialized managers or rendered observations. For changes to environment composition, reset events, actions, observations, runtime descriptors, simulation startup, or rendering, also run the bounded headless preview smoke test shown above.

## Repository layout

```text
src/scale_bench/
├── config/
│   ├── base.py             # immutable model base and shared constraints
│   ├── loader.py           # YAML/JSON loading and error wrapping
│   ├── paths.py            # config-reference and asset-reference semantics
│   └── models/             # pure camera, robot, scene, sim, and env models
├── envs/
│   ├── action_cfg.py       # Action Manager cfg and profile compiler
│   ├── env_cfg.py          # native EnvCfg composition and timing validation
│   ├── events.py           # task layout reset and per-env episode state
│   ├── mdp/                # runtime observation terms
│   ├── observation_cfg.py  # Observation Manager groups and cfg compiler
│   ├── runtime_config.py   # transitional EnvironmentConfig facade
│   └── scale_bench_env.py  # formal ManagerBasedEnv runtime entry
├── sim/                    # transitional simulation compatibility/builder code
├── robots/                 # transitional robot compatibility/builder code
├── scenes/
│   ├── scene_config.py     # transitional SceneConfig facade
│   ├── scene_template.py   # dual-arm tabletop scene compiler
│   └── uv_cuboid.py        # cuboid spawner with face-varying UVs
├── sensors/                # transitional camera compatibility/builder code
└── tasks/
    ├── base.py             # common task, rigid-asset, and layout behavior
    └── sort_dolls_by_size.py # one concrete task

configs/
├── cameras/d435.yml        # reusable camera profile
├── envs/default.yml        # cloning, control, and reset lifecycle settings
├── robots/piper.yml        # reference robot profile
├── scene/default.yml       # static scene assets and local poses
├── sim/default.yml         # simulation, rendering, and PhysX settings
└── tasks/sort_dolls_by_size.yml

scripts/preview_scene.py    # interactive preview and headless smoke entry point
```

The project uses a `src` layout but is not installed as a package (`tool.uv.package = false`). The included preview script adds `src` to `sys.path`; for your own standalone scripts, run with `PYTHONPATH=src` or add the directory explicitly.

## Troubleshooting

- **`Robot asset does not exist`** — check the path in the robot YAML and ensure the `Assets/` bundle is present.
- **A local Isaac Lab dependency cannot be found** — verify that `third_parties/IsaacLab/source/...` exists before running `uv sync`.
- **`No module named scale_bench` in a custom script** — launch it with `PYTHONPATH=src` from the repository root.
- **Different environments overlap** — increase `env_spacing_m` in the environment config, especially after changing the room scale.

## Further reading

- [Robot profile contract](docs/robot_profiles.md)
- [Scene template guide](docs/scene_template.md)
- [Current benchmark architecture and boundaries](docs/benchmark_architecture.md)

## License

This project is released under the [MIT License](LICENSE).
