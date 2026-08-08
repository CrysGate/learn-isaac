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
- **Texture-correct procedural surfaces** — `UvCuboidCfg` authors face-varying UV coordinates so MDL materials tile correctly on cuboids.
- **A runnable scene preview** — launch the default scene with placement-area and camera-frustum overlays, or run a short headless smoke test.

> [!NOTE]
> Action and Observation Manager terms are not connected yet. The current environment entry therefore has zero action dimensions and no policy observations; evaluators, episode orchestration, recording, and benchmark reporting also remain future work.

## Architecture

```text
robot / scene / camera YAML ──► typed profiles ──► scene cfg ──┐
task YAML + seed/layout ──────► TaskDefinition ─────────────────┤
sim YAML ─────────────────────► SimConfig ──────────────────────┤
env YAML ─────────────────────► EnvRuntimeConfig ───────────────┤
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

All relative configuration paths are resolved from the repository root, so keep the asset layout intact.

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
| `--left-robot-config PATH` | Select the left robot profile. |
| `--right-robot-config PATH` | Select the right robot profile. |
| `--device VALUE` | Choose `cpu`, `cuda`, or a device such as `cuda:0`. |
| `--viz none` | Disable visualizers for headless execution. |
| `--max-steps N` | Exit after a bounded number of environment steps. |
| `--camera-frustum-length-m M` | Set the displayed camera-frustum length in metres. |

Run `uv run python scripts/preview_scene.py --help` for all Isaac Lab launcher options.

## Core API

### Simulation presets

[`SimConfig`](src/scale_bench/sim/simulation_config.py) validates `configs/sim/*.yml` without launching Isaac Sim. After `AppLauncher` starts, it builds a fresh native config for the environment or a standalone preview:

```python
from scale_bench.sim import SimConfig

sim_profile = SimConfig.load("configs/sim/default.yml")
simulation_cfg = sim_profile.build_simulation_cfg(device="cuda:0")
```

The default preset runs physics at 120 Hz and renders every four steps at 30 Hz. It deliberately exposes only parameters that affect benchmark timing, gravity, observation quality, or manipulation stability. Materials, Fabric, logging, solver iterations, and GPU buffers inherit the installed Isaac Lab defaults. Unknown fields and invalid timing or device values are rejected at load time. Scene cloning parameters such as `num_envs` remain in the scene preset.

### Environment runtime

[`create_env_cfg()`](src/scale_bench/envs/env_config.py) compiles loaded robot profiles, `SceneConfig`, an optional task layout, `SimConfig`, and [`EnvRuntimeConfig`](src/scale_bench/envs/runtime_config.py) directly into a native `ScaleBenchEnvCfg`:

```python
from scale_bench.envs import EnvRuntimeConfig, ScaleBenchEnv, create_env_cfg

runtime = EnvRuntimeConfig.load("configs/envs/default.yml")
layout = task.resolve_layout(seed=42)
env_cfg = create_env_cfg(
    left_robot_profile=left,
    right_robot_profile=right,
    scene_config=scene,
    sim_config=sim,
    runtime_config=runtime,
    task=task,
    layout=layout,
    resample_task_layouts=True,
)
env = ScaleBenchEnv(env_cfg)
try:
    observation, info = env.reset()
finally:
    env.close()
```

`ScaleBenchEnv` subclasses Isaac Lab's `ManagerBasedEnv` and is the only owner of `SimulationContext`, `InteractiveScene`, reset, step, and cleanup. Runtime IO metadata is derived from the initialized environment instead of injected from build-time inputs. The reset event assigns deterministic layouts per `env_id`; `info["episode"]` reports the affected environment IDs and layout seeds. The builder rejects presets where render or camera updates are not synchronized with `step_dt`. Action and Observation manager configs are currently explicit empty extension points for the next stages.

### Robot profiles

[`RobotProfile`](src/scale_bench/robots/robot_profile.py) is the typed boundary between `configs/robots/*.yml` and Isaac Lab:

```python
from scale_bench.robots import RobotProfile

profile = RobotProfile.load("configs/robots/piper.yml")
robot_cfg = profile.build_articulation_cfg(
    prim_path="{ENV_REGEX_NS}/Robot",
)
camera_cfg = profile.build_camera_cfg(
    robot_prim_path="{ENV_REGEX_NS}/Robot",
)
```

Loading and validating a profile does not require a running simulator. Start Isaac Lab's `AppLauncher` before calling `build_articulation_cfg()`; the included [`preview_scene.py`](scripts/preview_scene.py) shows the required startup and import order.

`RobotProfile.load()`:

- resolves relative paths from the repository root;
- rejects unknown fields and non-finite numeric values;
- requires unique arm, gripper, and actuator joint names;
- verifies that initial positions exactly cover all declared joints;
- verifies actuator coverage and prevents overlapping actuator groups;
- validates the TCP, parallel-jaw gripper, and optional camera-mount contract;
- loads and validates the referenced camera profile;
- checks that local USD and optional URDF assets exist.

`build_articulation_cfg()` returns a new Isaac Lab `ArticulationCfg` on every call. `build_camera_cfg()` returns a camera below the supplied robot root, or `None` when the robot has no camera. The current robot implementation supports implicit actuators, parallel-jaw grippers, and one mounted camera.

### Camera profiles

[`CameraProfile`](src/scale_bench/sensors/camera_profile.py) validates reusable camera optics and output settings independently of Isaac Sim:

```python
from scale_bench.sensors import CameraProfile

profile = CameraProfile.load("configs/cameras/d435.yml")
```

The profile owns image dimensions, update period, data types, pinhole intrinsics, distortion metadata, focal length, and clipping range. Scene and robot profiles reference it while keeping installation poses local to their owning asset. The D435 profile is reused by both Piper wrist cameras and the overhead camera.

### Scene template

[`create_dual_arm_tabletop_scene_cfg()`](src/scale_bench/scenes/scene_template.py) combines two robot profiles with the scene preset:

```python
from scale_bench.robots import RobotProfile
from scale_bench.scenes import create_dual_arm_tabletop_scene_cfg

left = RobotProfile.load("configs/robots/piper.yml")
right = RobotProfile.load("configs/robots/piper.yml")

scene_cfg = create_dual_arm_tabletop_scene_cfg(
    left_robot_profile=left,
    right_robot_profile=right,
    config_path="configs/scene/default.yml",
    num_envs=1,
)
```

This snippet is intended to run after `AppLauncher` has initialized Isaac Sim.

`SceneConfig` validates every scene section with dedicated nested models, including non-empty asset-path fields, finite poses, positive dimensions, material parameters, unit quaternions, camera conventions, runtime types, and the ordered XY bounds in `task_object_placement_area`. Its `table_top_z_m` property and placement-area metadata can be reused by task builders and visualization tools without duplicating scene geometry calculations.

```python
from scale_bench.scenes import SceneConfig

scene_metadata = SceneConfig.load("configs/scene/default.yml")
placement_area = scene_metadata.task_object_placement_area
table_top_z_m = scene_metadata.table_top_z_m
```

The nested schema types are also public from `scale_bench.scenes`: `RoomConfig`, `SurfaceConfig`, `TaskObjectPlacementArea`, `RobotMountConfig`, `RobotMountsConfig`, `OverheadCameraConfig`, `LightingConfig`, and `SceneRuntimeConfig`. Prefer `SceneConfig.load()` when loading a complete preset so path resolution, validation, and process-local caching remain consistent.

The scene contains:

- a USD room and dome light;
- collision-enabled, textured ground and table surfaces;
- independent left and right robot mounts;
- left and right robot-mounted D435-style RGB-D sensors;
- a camera stand and overhead D435-style RGB-D sensor;
- configurable environment count, spacing, physics replication, and Fabric cloning.

Robot bases and the camera stand are positioned relative to the computed table-top height. Changing the table height therefore keeps mounted assets aligned automatically.

### Tasks

`TaskDefinition` keeps the shared task behavior in one place: metadata loading, deterministic sampling, placement validation, rigid-object construction, and layout JSON import/export. `SortDollsBySize` only declares its dolls, instruction, and size-order target. No task-specific scene subclass is needed; named `RigidObjectCfg` fields are added directly to the common `InteractiveSceneCfg` instance.

```python
from scale_bench.scenes import SceneConfig
from scale_bench.tasks import SortDollsBySize

scene_metadata = SceneConfig.load("configs/scene/default.yml")
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
     'from scale_bench.robots import RobotProfile; p = RobotProfile.load("configs/robots/my_robot.yml"); print(p.name)'
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
| `runtime` | Environment count, spacing, physics replication, and Fabric cloning. |

Scene YAML files are cached per process. Restart the preview process after editing a scene preset.

### Customizing the simulation

Copy [`configs/sim/default.yml`](configs/sim/default.yml) to create a simulation preset. Timing and gravity are top-level settings; `render` selects observation quality, and `physx` contains the one manipulation-specific override currently justified by runtime behavior. Everything else follows Isaac Lab defaults and should only become public configuration after a benchmark requirement demonstrates that it must vary. Use `--sim-config` to select the preset and `--device` for a temporary machine-specific override.

Copy [`configs/envs/default.yml`](configs/envs/default.yml) to change control decimation, reset rerenders, texture waiting, or the environment seed. The builder requires render interval, control decimation, and camera update periods to describe one synchronous environment rate.

### Validating changes

Run the automated contract and layout tests without launching an interactive simulator:

```bash
uv run pytest -q
```

The tests cover environment-preset validation, manager composition, synchronized timing, runtime descriptors, and episode metadata. Keep the bounded headless preview smoke test for changes that affect actual simulation startup or rendering.

## Repository layout

```text
src/scale_bench/
├── envs/
│   ├── env_config.py       # native EnvCfg composition and timing validation
│   ├── events.py           # task layout reset and per-env episode state
│   ├── runtime_config.py   # environment lifecycle YAML schema
│   └── scale_bench_env.py  # formal ManagerBasedEnv runtime entry
├── sim/
│   └── simulation_config.py # simulation YAML and SimulationCfg builder
├── robots/
│   └── robot_profile.py    # robot schema and articulation/camera builders
├── scenes/
│   ├── scene_config.py     # typed scene YAML and placement bounds
│   ├── scene_template.py   # dual-arm tabletop scene compiler
│   └── uv_cuboid.py        # cuboid spawner with face-varying UVs
├── sensors/
│   └── camera_profile.py   # YAML schema, validation, CameraCfg builder
└── tasks/
    ├── base.py             # common task, rigid-asset, and layout behavior
    └── sort_dolls_by_size.py # one concrete task

configs/
├── cameras/d435.yml        # reusable camera profile
├── env/default.yml         # control and reset lifecycle settings
├── robots/piper.yml        # reference robot profile
├── scene/default.yml       # scene-local poses and environment settings
├── sim/default.yml         # simulation, rendering, and PhysX settings
└── tasks/sort_dolls_by_size.yml

scripts/preview_scene.py    # interactive preview and headless smoke entry point
```

The project uses a `src` layout but is not installed as a package (`tool.uv.package = false`). The included preview script adds `src` to `sys.path`; for your own standalone scripts, run with `PYTHONPATH=src` or add the directory explicitly.

## Troubleshooting

- **`Robot asset does not exist`** — check the path in the robot YAML and ensure the `Assets/` bundle is present.
- **A local Isaac Lab dependency cannot be found** — verify that `third_parties/IsaacLab/source/...` exists before running `uv sync`.
- **`No module named scale_bench` in a custom script** — launch it with `PYTHONPATH=src` from the repository root.
- **Different environments overlap** — increase `runtime.env_spacing_m`, especially after changing the room scale.

## Further reading

- [Robot profile contract](docs/robot_profiles.md)
- [Scene template guide](docs/scene_template.md)
- [Current benchmark architecture and boundaries](docs/benchmark_architecture.md)

## License

This project is released under the [MIT License](LICENSE).
