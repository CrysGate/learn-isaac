# 双臂桌面场景模板

`DualArmTabletopSceneCfg` 是当前 `scale_bench` 已实现的场景拓扑：在每个环境中组合房间、地面、桌面、两台机器人、相机支架和顶视 RGB-D 相机，并使用一盏全局环境光。

场景实现位于 [`src/scale_bench/scenes/scene_template.py`](../src/scale_bench/scenes/scene_template.py)，默认配置位于 [`configs/scene/default.yml`](../configs/scene/default.yml)。

```text
RobotProfile YAML ──► left/right ArticulationCfg ─────┐
                                                      │
Scene YAML ───────────────────────────────────────────┼─► DualArmTabletopSceneCfg
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
| `overhead_camera` | `{ENV_REGEX_NS}/CameraStand/D435Sensor` | Pinhole RGB-D 相机。 |
| `environment_light` | `/World/EnvironmentLight` | 使用 HDR 纹理的全局 dome light。 |

左右机器人配置会先复制再设置 prim path 和安装位姿，传入的原始 `ArticulationCfg` 不会被修改。因此左右两侧可以使用不同的 robot profile。

## 快速预览

从仓库根目录打开默认场景：

```bash
uv run python scripts/preview_scene.py
```

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

`preview_scene.py` 默认启用相机和 Kit visualizer。使用 `--viz none` 可以关闭 visualizer；`--max-steps` 必须是正整数。运行以下命令可以查看完整 launcher 参数：

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
    left_robot_cfg=left.build_articulation_cfg(),
    right_robot_cfg=right.build_articulation_cfg(),
    config_path="configs/scene/default.yml",
)
scene = InteractiveScene(scene_cfg)
```

工厂函数参数：

| 参数 | 必需 | 作用 |
|---|---:|---|
| `left_robot_cfg` | 是 | 左侧机器人 `ArticulationCfg`。 |
| `right_robot_cfg` | 是 | 右侧机器人 `ArticulationCfg`。 |
| `config_path` | 否 | 场景 YAML；默认是 `configs/scene/default.yml`。 |
| `num_envs` | 否 | 覆盖 YAML 中的 `runtime.num_envs`。 |
| `env_spacing_m` | 否 | 覆盖 YAML 中的 `runtime.env_spacing_m`。 |

`replicate_physics` 和 `clone_in_fabric` 当前始终读取 YAML 的 `runtime` 配置。

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

### `camera`

相机配置分为三部分：

- 支架资产及相对于桌面的安装位姿；
- 传感器相对于支架的局部位置和四元数；
- 图像尺寸、输出类型、内参、焦距和裁剪范围。

相机支架的 Z 坐标同样自动使用 `table_top_z`。默认传感器输出 `rgb` 和 `distance_to_image_plane`。

当前构建函数实际读取：

```text
stand_usd_path
stand_position_xy_m
stand_orientation_xyzw
width, height, update_period_s, data_types
intrinsic_matrix_px, focal_length_mm, clipping_range_m
sensor_local_position_m, sensor_local_orientation_xyzw, convention
```

以下字段目前只是 YAML 中的说明性元数据，尚未被 `_camera_cfg()` 使用：

```text
model
intrinsic_source
distortion_model
distortion_coefficients
```

因此当前相机不会根据 distortion 字段模拟镜头畸变。

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

- 场景配置路径和其中的本地资产相对路径都从仓库根目录解析。
- 包含 `://` 的资产路径会作为 URI 原样传给 Isaac Lab。
- 同一路径的 YAML 会在进程内缓存；编辑配置后应重启预览进程才能重新加载。
- 场景 YAML 当前使用普通映射读取，没有类似 `RobotProfile` 的 Pydantic schema。缺失字段或类型错误会在构建对应配置时直接报错。
- 默认 preset 依赖 `Assets/` 中的房间、材质、相机支架和 HDR 文件，以及这些资产的传递依赖。

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

这些能力属于目标架构中的 `ScenarioSpec`、Task 和评测层，目前尚未实现。机器人配置约定见 [`robot_profiles.md`](robot_profiles.md)，整体设计状态见 [`benchmark_architecture.md`](benchmark_architecture.md)。
