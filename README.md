# ScaleBench

[English](README.md) | [简体中文](README.zh-CN.md)

Configuration-first Isaac Lab building blocks for scale-aware, dual-arm manipulation scenes.

ScaleBench keeps robot semantics, camera parameters, and scene parameters in YAML, validates profiles at the boundary, and compiles them into native Isaac Lab configuration objects. The current implementation focuses on reusable robot and scene construction rather than a complete task, policy, or evaluation pipeline.

## What is implemented

- **Typed robot profiles** — validate joints, TCP, actuators, gripper, and an optional robot-mounted camera, then build fresh Isaac Lab configs.
- **Reusable camera profiles** — keep camera optics and output parameters separate from scene- and robot-local sensor poses.
- **A reusable dual-arm scene** — compose a room, textured ground and table, two independently configured robots with wrist cameras, an overhead RGB-D camera, and environment lighting.
- **Texture-correct procedural surfaces** — `UvCuboidCfg` authors face-varying UV coordinates so MDL materials tile correctly on cuboids.
- **A runnable scene preview** — launch the default scene in Isaac Sim or run a short headless smoke test.

> [!NOTE]
> The code under `src/scale_bench` currently provides the configuration and scene foundation. Tasks, episode orchestration, recording, and benchmark reporting are not implemented in this package yet.

## Architecture

```text
configs/robots/*.yml
        │
        ▼
  RobotProfile ── validation ──► ArticulationCfg ─────┤
                └──────────────► robot CameraCfg ─────┤
                                                      │
configs/scene/*.yml ──────────────────────────────────┼─► DualArmTabletopSceneCfg
                                                      │
configs/cameras/*.yml ──► CameraProfile ──► CameraCfg ┤
                                                      │
  UvCuboidCfg ── textured ground and table ───────────┘
                                                              │
                                                              ▼
                                                    Isaac Lab InteractiveScene
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

The default preset also expects the project asset bundle at `Assets/`. If it is not included in your checkout, provide these files, together with any textures and transitive USD dependencies they reference:

```text
Assets/
├── Background/brown_photostudio_02_4k.hdr
├── Material/material_0122/Mahogany_Planks.mdl
├── Material/material_0564/Wood_Tiles_Fineline.mdl
├── Object/Geometry/camera_stand_aloha/aloha_front_camera_stand_realsense_d435.usd
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

Run a two-step headless smoke test:

```bash
uv run python scripts/preview_scene.py --viz none --max-steps 2
```

Use different robot or scene profiles without changing Python code:

```bash
uv run python scripts/preview_scene.py \
  --config configs/scene/default.yml \
  --left-robot-config configs/robots/piper.yml \
  --right-robot-config configs/robots/piper.yml \
  --device cuda:0
```

Useful preview options:

| Option | Purpose |
|---|---|
| `--config PATH` | Select the scene YAML. |
| `--left-robot-config PATH` | Select the left robot profile. |
| `--right-robot-config PATH` | Select the right robot profile. |
| `--device VALUE` | Choose `cpu`, `cuda`, or a device such as `cuda:0`. |
| `--viz none` | Disable visualizers for headless execution. |
| `--max-steps N` | Exit after a bounded number of simulation steps. |

Run `uv run python scripts/preview_scene.py --help` for all Isaac Lab launcher options.

## Core API

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

The scene contains:

- a USD room and dome light;
- collision-enabled, textured ground and table surfaces;
- independent left and right robot mounts;
- left and right robot-mounted D435-style RGB-D sensors;
- a camera stand and overhead D435-style RGB-D sensor;
- configurable environment count, spacing, physics replication, and Fabric cloning.

Robot bases and the camera stand are positioned relative to the computed table-top height. Changing the table height therefore keeps mounted assets aligned automatically.

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
| `robot_mounts` | Left and right base poses relative to the table top. |
| `camera` | Camera profile reference, stand pose, and sensor transform. |
| `lighting` | HDR environment texture and intensity. |
| `runtime` | Environment count, spacing, physics replication, and Fabric cloning. |

Scene YAML files are cached per process. Restart the preview process after editing a scene preset.

## Repository layout

```text
src/scale_bench/
├── robots/
│   └── robot_profile.py    # robot schema and articulation/camera builders
├── scenes/
│   ├── scene_template.py   # dual-arm tabletop scene compiler
│   └── uv_cuboid.py        # cuboid spawner with face-varying UVs
└── sensors/
    └── camera_profile.py   # YAML schema, validation, CameraCfg builder

configs/
├── cameras/d435.yml        # reusable camera profile
├── robots/piper.yml        # reference robot profile
└── scene/default.yml       # scene-local poses and environment settings

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
- [Benchmark architecture direction](docs/benchmark_architecture.md) — design goals; not all components are implemented yet

## License

This project is released under the [MIT License](LICENSE).
