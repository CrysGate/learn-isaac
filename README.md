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
- **A small task contract and first task** — describe task-owned assets and layouts independently of the simulator; `SortDollsBySize` is the first robot-independent example.
- **A manager-based environment entry** — create `ScaleBenchEnv` through a safe public API while keeping application startup under caller control.
- **Profile-driven actions** — control both arms and grippers with dynamically sized absolute command-joint targets in physical units.
- **Named policy observations** — expose ordered robot state and raw RGB-D from configured cameras without leaking task or evaluation ground truth.
- **Texture-correct procedural surfaces** — `UvCuboidCfg` authors face-varying UV coordinates so MDL materials tile correctly on cuboids.
- **A runnable scene preview** — launch the default scene with placement-area and camera-frustum overlays, or run a short headless smoke test.

> [!NOTE]
> Joint-space Action, policy Observation, and opt-in Recorder Manager terms are connected. End-effector control, evaluators, episode orchestration, and benchmark reporting remain future work.

## Architecture

```text
robot / scene / camera YAML ──► config.loader / pure models ───┐
task YAML + scene context ────► Task / TaskLayout ──────────────┤
sim YAML ─────────────────────► SimulationConfig ───────────────┤
env YAML ─────────────────────► EnvironmentConfig ──────────────┤
                                                                ▼
                                                  api.create_env()
                                                                │
                                                                ▼
                                           internal cfg ─► ScaleBenchEnv
                                           reset() / step() / IO descriptors
```

The boundary is intentionally small: robot-specific details stay in robot configs, scene-specific details stay in scene presets, and applications receive one initialized environment through the public API.

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

[`load_config()`](src/scale_bench/config/loader.py) is the only YAML/JSON loading boundary. The six immutable pure-Python models contain data and local validation only:

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.recording import RecordingConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.config.models.simulation import SimulationConfig

robot = load_config("configs/robots/piper.yml", RobotConfig, asset_root=".")
scene = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
sim = load_config("configs/sim/default.yml", SimulationConfig)
environment = load_config("configs/envs/default.yml", EnvironmentConfig)
recording = RecordingConfig(
    output_dir="outputs/datasets",
    dataset_name="sort_dolls_seed_42",
)
```

Models forbid unknown fields, are frozen, reject non-finite numeric values, and do not read files or build Isaac Lab objects. Configuration references resolve relative to the file containing them. Asset references resolve relative to `asset_root` when supplied, otherwise relative to the containing file. Absolute paths are preserved, and missing local assets are reported with the source path and field location. Only local configuration and asset paths are supported. `num_envs`, spacing, cloning, control, and reset settings belong to `EnvironmentConfig`, not `SceneConfig`.

The default simulation preset runs physics at 120 Hz and renders every four steps at 30 Hz. Materials, Fabric, logging, solver iterations, and GPU buffers inherit the installed Isaac Lab defaults.

### Environment runtime

[`create_env()`](src/scale_bench/api.py) accepts loaded `RobotConfig`, `SceneConfig`, `SimulationConfig`, `EnvironmentConfig`, and an optional Task layout source. It delays all adapter imports until the function is called after Isaac Sim startup:

```python
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True, enable_cameras=True)
simulation_app = app_launcher.app

from scale_bench.api import create_env

env = create_env(
    left_robot_config=robot,
    right_robot_config=robot,
    scene_config=scene,
    simulation_config=sim,
    environment_config=environment,
    recording_config=recording,
    task=task,
    base_seed=42,
)
try:
    observation, info = env.reset()
    # Run the policy/evaluator loop, then export before resetting these envs.
    env.complete_episodes(success=[True] * env.num_envs)
finally:
    env.close()
    simulation_app.close()
```

`ScaleBenchEnv` subclasses Isaac Lab's `ManagerBasedEnv` and is the only owner of `SimulationContext`, `InteractiveScene`, reset, step, and cleanup. `AppLauncher` and its application remain caller-owned. Runtime IO metadata is derived from initialized managers and sensors in `isaaclab/runtime/io_descriptors.py`. With `base_seed`, environment `i` receives the layout generated from `base_seed + i` once during configuration; every full or partial reset restores that same layout. Alternatively, `layouts` accepts either one layout to broadcast or exactly `num_envs` layouts. `info["episode"]` reports affected environment IDs and stable layout seeds.

Recording is opt-in through `RecordingConfig`. The default recording terms store the initial relative scene state, raw and processed actions, and joint observations in HDF5. Camera RGB-D is independently enabled with `record_camera_observations=True`; per-step scene truth also requires explicit opt-in. `complete_episodes()` writes success and exports selected environment buffers; call it before their next reset. With `overwrite_existing=False`, occupied names are advanced automatically (`rollout.hdf5`, `rollout_1.hdf5`, ...); `overwrite_existing=True` explicitly reuses the requested name.

Native camera, robot, scene, simulation, task, manager, and environment cfg implementations live under [`scale_bench.isaaclab`](src/scale_bench/isaaclab). The pre-refactor `envs`, `scenes`, `robots`, `sensors`, and `sim` import paths have been removed; application code uses the pure configuration models and `scale_bench.api`.

### Robot configuration

[`RobotConfig`](src/scale_bench/config/models/robot.py) validates robot semantics independently of Isaac Sim. Loading it through `load_config()`:

- resolves configuration references relative to the robot YAML and assets against the explicit asset root;
- rejects unknown fields and non-finite numeric values;
- requires unique arm, gripper, and actuator joint names;
- verifies that initial positions exactly cover all declared joints;
- verifies actuator coverage and prevents overlapping actuator groups;
- validates the TCP, parallel-jaw gripper, and optional camera-mount contract;
- checks that local USD and optional URDF assets exist.

### Camera configuration

[`CameraConfig`](src/scale_bench/config/models/camera.py) owns image dimensions, update period, data types, pinhole intrinsics, distortion metadata, focal length, clipping range, and coordinate convention:

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.camera import CameraConfig

camera = load_config("configs/cameras/d435.yml", CameraConfig)
```

Scene and robot configs reference it while keeping installation poses local to their owning asset. The D435 config is reused by both Piper wrist cameras and the overhead camera.

### Scene configuration

`SceneConfig` validates static scene sections, including asset references, finite poses, positive dimensions, material parameters, unit quaternions, camera conventions, and ordered XY placement bounds. Its `table_top_z_m` property and placement-area metadata can be reused without duplicating scene geometry calculations.

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.scene import SceneConfig

scene_metadata = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
placement_area = scene_metadata.task_object_placement_area
table_top_z_m = scene_metadata.table_top_z_m
```

`create_env()` combines the loaded scene, robot, camera, and environment configurations into the native dual-arm scene after Isaac Sim has started.

The scene contains:

- a USD room and dome light;
- collision-enabled, textured ground and table surfaces;
- independent left and right robot mounts;
- left and right robot-mounted D435-style RGB-D sensors;
- a camera stand and overhead D435-style RGB-D sensor;
- environment count, spacing, physics replication, and Fabric cloning supplied by `EnvironmentConfig`.

Robot bases and the camera stand are positioned relative to the computed table-top height. Changing the table height therefore keeps mounted assets aligned automatically.

### Tasks

The `Task` protocol exposes only task identity, instruction, and context-driven layout generation and validation. `RigidObjectTask` provides reusable metadata, deterministic tabletop placement, and layout JSON behavior without retaining `SceneConfig`. `SortDollsBySize` only declares its dolls and size-order target. Native `RigidObjectCfg` construction belongs to the adapter's TaskBuilder.

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.scene import SceneConfig
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.sort_dolls_by_size.config import SortDollsBySizeConfig
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize

scene_metadata = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
task_config = load_config(
    "configs/tasks/sort_dolls_by_size.yml",
    SortDollsBySizeConfig,
    asset_root=".",
)
context = PlacementContext.from_scene_config(scene_metadata)
task = SortDollsBySize(task_config)
layout = task.resolve_layout(context, seed=42)
layout.save("layouts/sort_dolls_by_size/42.json")
instruction = task.instruction
target_order = task.target_order_small_to_large
```

Calling `task.resolve_layout(context, layout_path=...)` restores and validates the exact saved poses. Pass it through `create_env(..., task=task, layouts=(layout,))`; the environment builder derives the same context from `SceneConfig`, selects the built-in TaskBuilder, and registers fresh native asset cfgs. To use another task YAML, load it into `SortDollsBySizeConfig` first. Piper/curobo planning, robot assignment, recording, success evaluation, and the application lifecycle remain outside this task layer.

### UV cuboids

[`UvCuboidCfg`](src/scale_bench/isaaclab/spawners/uv_cuboid.py) extends Isaac Lab's `CuboidCfg` with an `uv_scale` setting. Its spawner delegates geometry and physics creation to Isaac Lab, then authors 24 face-varying `st` values—four per cube face—for predictable material tiling.

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

Run `uv run pytest` for the fast configuration, task, builder, runtime-contract, and dependency-boundary tests. The initialized two-environment runtime suite is isolated behind the `integration` marker because it requires Isaac Sim, a supported GPU, and the external asset bundle:

```bash
uv run pytest -m integration -q
```

The integration test launches the public API in a child process and covers create/reset/step/close, rendered RGB-D observations, initialized IO descriptors, per-environment layout seeds, and partial reset. `SCALE_BENCH_ASSET_ROOT` can point it at an asset bundle outside the checkout; Git worktrees automatically use the main worktree's `Assets/` when available.

## Repository layout

```text
src/scale_bench/
├── api.py                  # safe public create_env entry with delayed imports
├── config/
│   ├── base.py             # immutable model base and shared constraints
│   ├── loader.py           # YAML/JSON loading and error wrapping
│   ├── paths.py            # config-reference and asset-reference semantics
│   └── models/             # pure camera, robot, scene, sim, and env models
├── isaaclab/
│   ├── builders/           # pure-data to native cfg conversion
│   ├── managers/           # Action, Observation, and Event cfg declarations
│   ├── mdp/                # manager runtime terms
│   ├── runtime/            # ScaleBenchEnv and initialized IO descriptors
│   └── spawners/           # project-specific native spawners
└── tasks/
    ├── common/             # Task contract, layouts, placement, rigid assets
    └── sort_dolls_by_size/ # task-specific config and rules

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
