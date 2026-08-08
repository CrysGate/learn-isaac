# 双臂桌面场景模板

`DualArmTabletopSceneCfg` 是当前 `scale_bench` 已实现的场景拓扑：在每个环境中组合房间、地面、桌面、两台带挂载相机的机器人、相机支架和顶视 RGB-D 相机，并使用一盏全局环境光。默认 Piper 场景一共创建三台相机。

场景编译实现位于 [`src/scale_bench/scenes/scene_template.py`](../src/scale_bench/scenes/scene_template.py)，场景 YAML schema 位于 [`src/scale_bench/scenes/scene_config.py`](../src/scale_bench/scenes/scene_config.py)。默认场景配置位于 [`configs/scene/default.yml`](../configs/scene/default.yml)，默认相机参数位于 [`configs/cameras/d435.yml`](../configs/cameras/d435.yml)。

```text
RobotProfile YAML ──► left/right ArticulationCfg ─────┐
                  └─► left/right robot CameraCfg ─────┤
                                                      │
Scene YAML ──► SceneConfig ───────────────────────────┼─► DualArmTabletopSceneCfg
                                                      │
CameraProfile YAML ──► CameraProfile ──► CameraCfg ───┤
                                                      │
UvCuboidCfg ──► textured ground/table ────────────────┘
                                                              │
                                                              ▼
                                                    InteractiveScene
```

## 场景组成

| 配置成员 | USD prim | 内容 |
|---|---|---|
| `room` | `{ENV_REGEX_NS}/Room` | 可缩放的房间 USD。 |
| `ground` | `{ENV_REGEX_NS}/Ground` | 带碰撞、物理材质、视觉材质和 UV 的长方体地面。 |
| `table` | `{ENV_REGEX_NS}/Table` | 带碰撞、物理材质、视觉材质和 UV 的长方体桌面。 |
| `camera_stand` | `{ENV_REGEX_NS}/CameraStand` | 放置在桌面高度上的相机支架 USD。 |
| `left_robot` | `{ENV_REGEX_NS}/LeftRobot` | 左侧 `ArticulationCfg` 的挂载副本。 |
| `right_robot` | `{ENV_REGEX_NS}/RightRobot` | 右侧 `ArticulationCfg` 的挂载副本。 |
| `left_robot_camera` | `{ENV_REGEX_NS}/LeftRobot/link6/camera/D435Sensor` | 随左机器人腕部运动的 RGB-D 相机。 |
| `right_robot_camera` | `{ENV_REGEX_NS}/RightRobot/link6/camera/D435Sensor` | 随右机器人腕部运动的 RGB-D 相机。 |
| `overhead_camera` | `{ENV_REGEX_NS}/CameraStand/OverheadCamera` | Pinhole RGB-D 相机。 |
| `environment_light` | `/World/EnvironmentLight` | 使用 HDR 纹理的全局 dome light。 |

左右机器人配置会先复制再设置 prim path 和安装位姿，传入的原始 `ArticulationCfg` 不会被修改。因此左右两侧可以使用不同的 robot profile。

## 快速预览

从仓库根目录打开默认场景：

```bash
uv run python scripts/preview_scene.py
```

预览 `sort_dolls_by_size` 任务，并按 seed 生成或从文件恢复任务布局：

```bash
uv run python scripts/preview_scene.py --task sort_dolls_by_size \
  --seed 45 --export-layout layouts/sort_dolls_by_size/45.json
uv run python scripts/preview_scene.py --task sort_dolls_by_size \
  --layout layouts/sort_dolls_by_size/42.json
```

使用 `--export-layout PATH` 可以保存本次生成或加载的布局。`--seed` 与 `--layout` 互斥；`--seed`、`--layout` 和 `--export-layout` 都要求同时传入 `--task`。选择任务但不指定 seed 或 layout 时默认使用 seed `0`。

执行两步无界面冒烟验证：

```bash
uv run python scripts/preview_scene.py --viz none --max-steps 2
```

选择自定义场景和左右机器人：

```bash
uv run python scripts/preview_scene.py \
  --config configs/scene/default.yml \
  --left-robot-config configs/robots/piper.yml \
  --right-robot-config configs/robots/piper.yml \
  --device cuda:0
```

`preview_scene.py` 默认启用相机和 Kit visualizer。使用 `--viz none` 可以关闭 visualizer；`--max-steps` 必须是正整数。

Kit 预览默认绘制绿色的任务物体放置区域，以及三台相机的彩色视锥。浮动的 `Scene overlays` 面板可分别开关 `Placement area` 和 `Camera frustums`；腕部相机运动时视锥会同步更新。视锥显示长度可通过正数参数 `--camera-frustum-length-m` 调整。使用 `--headless` 或 `--viz none` 时不会创建这些预览元素。

运行以下命令可以查看完整 launcher 参数：

```bash
uv run python scripts/preview_scene.py --help
```

## Python API

Isaac Lab 要求先启动 `AppLauncher`，再导入依赖 Isaac Sim runtime 的场景模块。以下片段应放在 `AppLauncher` 初始化之后；完整顺序以 [`scripts/preview_scene.py`](../scripts/preview_scene.py) 为准。

```python
from isaaclab.scene import InteractiveScene

from scale_bench.robots import RobotProfile
from scale_bench.scenes import create_dual_arm_tabletop_scene_cfg

left = RobotProfile.load("configs/robots/piper.yml")
right = RobotProfile.load("configs/robots/piper.yml")

scene_cfg = create_dual_arm_tabletop_scene_cfg(
    left_robot_profile=left,
    right_robot_profile=right,
    config_path="configs/scene/default.yml",
)
scene = InteractiveScene(scene_cfg)
```

工厂函数参数：

| 参数 | 必需 | 作用 |
|---|---:|---|
| `left_robot_profile` | 是 | 左侧 `RobotProfile`；用于生成 articulation 和机器人相机。 |
| `right_robot_profile` | 是 | 右侧 `RobotProfile`；用于生成 articulation 和机器人相机。 |
| `config_path` | 否 | 场景 YAML；默认是 `configs/scene/default.yml`。 |
| `num_envs` | 否 | 覆盖 YAML 中的 `runtime.num_envs`。 |
| `env_spacing_m` | 否 | 覆盖 YAML 中的 `runtime.env_spacing_m`。 |

左右两侧都必须提供 robot profile。场景会分别从 profile 构建 articulation 和挂载相机。`replicate_physics` 和 `clone_in_fabric` 当前始终读取 YAML 的 `runtime` 配置。

## 场景 YAML

### `room`

```yaml
room:
  usd_path: Assets/Room/Simple_Room_nolight/simple_room_nolight.usd
  scale: 0.5
```

`scale` 是三轴统一缩放，缺省值为 `0.5`。

### `ground` 与 `table`

两者使用相同字段：

```yaml
table:
  position_m: [0.0, -0.05, 0.74]
  size_m: [1.4, 1.1, 0.05]
  material_path: Assets/Material/material_0122/Mahogany_Planks.mdl
  uv_scale: [1.0, 1.0]
  static_friction: 0.8
  dynamic_friction: 0.8
  restitution: 0.0
```

- `position_m` 是长方体中心位置，`size_m` 是完整三轴尺寸。
- `material_path` 可以为 `null`，此时不创建视觉材质。
- `uv_scale` 可省略，默认 `[1.0, 1.0]`。
- 碰撞始终启用，摩擦和恢复系数写入 rigid-body physics material。

### `task_object_placement_area`

```yaml
task_object_placement_area:
  x_range_m: [-0.65, 0.65]
  y_range_m: [-0.20, 0.45]
```

该字段定义环境局部坐标系 XY 平面上的任务物体放置范围。默认区域位于机械臂前方至桌面远端，并在桌边保留 5 cm 边距；物体的 Z 坐标由任务根据桌面顶面高度确定。

```python
from scale_bench.scenes import SceneConfig

scene_config = SceneConfig.load("configs/scene/default.yml")
area = scene_config.task_object_placement_area
```

`SceneConfig` 会校验全部嵌套区块，拒绝未知字段、无效尺寸和材质参数、非单位四元数、非法相机坐标约定，以及非有限或上下界颠倒的放置区域。后续增加场景级配置时，应在对应的具名模型中添加字段。

`scale_bench.scenes` 公开导出 `RoomConfig`、`SurfaceConfig`、`TaskObjectPlacementArea`、`RobotMountConfig`、`RobotMountsConfig`、`OverheadCameraConfig`、`LightingConfig` 和 `SceneRuntimeConfig`。加载完整场景 preset 时应优先调用 `SceneConfig.load()`，以统一路径解析、严格校验和进程内缓存行为。

### `robot_mounts`

```yaml
robot_mounts:
  left:
    position_xy_m: [-0.3, -0.45]
    orientation_xyzw: [0.0, 0.0, 0.70710678, 0.70710678]
  right:
    position_xy_m: [0.3, -0.45]
    orientation_xyzw: [0.0, 0.0, 0.70710678, 0.70710678]
```

桌面顶面高度按以下公式计算：

```text
table_top_z = table.position_m[2] + table.size_m[2] / 2
```

机器人底座使用 YAML 中的 `position_xy_m` 和 `orientation_xyzw`，Z 坐标自动设置为 `table_top_z`。修改桌面高度后不需要手动同步机器人高度。

### 机器人相机

机器人相机不写在 scene YAML 中。相机型号、输出与内参来自 `configs/cameras/*.yml`，相对于机器人 frame 的安装关系来自 `configs/robots/*.yml`。场景工厂只为左右机器人补上各自的根路径：

```text
{ENV_REGEX_NS}/LeftRobot  + link6/camera/D435Sensor
{ENV_REGEX_NS}/RightRobot + link6/camera/D435Sensor
```

传感器作为各自机器人腕部 prim 的子节点生成，因此会跟随机械臂运动。Isaac Lab 场景会先创建 articulation，再创建传感器。

### `camera`

场景 YAML 只负责引用相机 profile，并描述支架和传感器在当前场景中的安装关系：

```yaml
camera:
  profile_path: configs/cameras/d435.yml
  stand_usd_path: Assets/Object/Geometry/camera_stand_aloha/aloha_front_camera_stand_realsense_d435.usd
  stand_position_xy_m: [0.0, -0.47]
  stand_orientation_xyzw: [0.0, 0.0, 0.70710678, 0.70710678]
  sensor_local_position_m: [0.06376095, 0.0003435, 0.55816412]
  sensor_local_orientation_xyzw: [0.26866805, -0.26866791, -0.65407767, 0.65407754]
  convention: opengl
```

- `profile_path` 引用一份可由多个相机复用的参数配置。
- 支架的 XY 和四元数由场景指定，Z 坐标自动使用 `table_top_z`。
- `sensor_local_*` 是传感器相对于支架的局部位姿，四元数顺序为 `xyzw`。
- `convention` 定义四元数采用的相机坐标轴约定：`opengl` 为 `-Z` 向前、`+Y` 向上，`ros` 为 `+Z` 向前、`-Y` 向上，`world` 为 `+X` 向前、`+Z` 向上。

### 相机 profile YAML

相机型号自身的光学和输出参数保存在独立 YAML 中：

```yaml
model: Intel RealSense D435
width: 640
height: 480
update_period_s: 0.03333333333333333
data_types: [rgb, distance_to_image_plane]
focal_length_mm: 1.93
intrinsic_source: aligned_to_color
intrinsic_matrix_px: [604.5, 0.0, 320.0, 0.0, 604.5, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
clipping_range_m: [0.105, 10.0]
```

[`CameraProfile`](../src/scale_bench/sensors/camera_profile.py) 会拒绝未知字段、非有限数值、重复的 `data_types`、无效针孔内参和顺序错误的裁剪平面。加载后的 profile 不可修改，每次调用 `build_camera_cfg()` 都会生成新的 Isaac Lab `CameraCfg`。

`model`、`intrinsic_source`、`distortion_model` 和 `distortion_coefficients` 当前作为标定元数据保存；Isaac Lab 场景使用针孔模型，不会根据 distortion 字段模拟镜头畸变。

### `lighting`

`texture_path` 指向 HDR 环境纹理，`intensity` 设置 dome light 强度。环境光使用全局 prim `/World/EnvironmentLight`，多个克隆环境共享该光源。

### `runtime`

```yaml
runtime:
  num_envs: 1
  env_spacing_m: 5.0
  replicate_physics: true
  clone_in_fabric: false
```

增加环境数量或修改房间缩放时，应同步检查 `env_spacing_m`，确保相邻房间几何不会重叠。函数参数 `num_envs` 和 `env_spacing_m` 可以临时覆盖对应 YAML 值，而不修改 preset。

## 路径、缓存与错误行为

- 场景配置、机器人 profile、相机 profile 和本地资产的相对路径都从仓库根目录解析。
- 包含 `://` 的资产路径会作为 URI 原样传给 Isaac Lab。
- 场景 YAML 会按路径在进程内缓存；编辑场景 preset 后应重启预览进程。
- 场景 YAML 的所有区块和放置区域都由 `SceneConfig` 的具名嵌套模型校验；构建 Isaac Lab 配置时只读取已经验证的属性。
- 相机 YAML 使用严格的 Pydantic schema；加载失败时会以 `ValueError` 报告 profile 路径和具体校验错误。
- 默认 preset 依赖 `Assets/` 中的房间、材质、相机支架和 HDR 文件，以及这些资产的传递依赖；该资产包不由 Git 仓库分发，运行预览前必须单独准备。
- `sort_dolls_by_size` 任务还依赖 `Assets/Object/Rigid/matryoshka_dolls/{00000..00004}/` 下的 `object.usdz`、`metadata.json` 及其传递依赖。

## `UvCuboidCfg`

[`UvCuboidCfg`](../src/scale_bench/scenes/uv_cuboid.py) 是场景内部用于地面和桌面的 `CuboidCfg` 扩展。

它先调用 Isaac Lab 原生 `spawn_cuboid()` 完成几何、变换、克隆、碰撞和材质创建，再为每个生成的 `UsdGeom.Cube` 写入：

- 名为 `st` 的 `TexCoord2fArray` primvar；
- `faceVarying` interpolation；
- 每个表面四个、总计 24 个 UV 值。

`uv_scale=(u, v)` 控制每个表面的纹理重复次数。两个值都必须大于零，否则生成时会抛出 `ValueError`。

## 当前边界

该模板只定义稳定的场景拓扑和默认视觉、物理参数，不负责：

- 生成任务物体；
- 采样 episode 布局或随机参数；
- 定义观测、动作、奖励或成功条件；
- 创建 policy、evaluator 或 recorder。

任务物体不属于这个公共模板。Task 层直接在构建完成的 `InteractiveSceneCfg` 实例上增加具名资产配置；`SortDollsBySize` 可按 seed 生成布局，也可从导出的 layout JSON 恢复五个套娃的初始位姿。Env 配置组装、Policy/Evaluator 数据边界、Recorder 和 episode runner 尚未实现。机器人配置约定见 [`robot_profiles.md`](robot_profiles.md)，整体设计状态见 [`benchmark_architecture.md`](benchmark_architecture.md)。
