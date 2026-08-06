# ScaleBench

[English](README.md) | [简体中文](README.zh-CN.md)

ScaleBench 是一组配置优先的 Isaac Lab 基础组件，用于构建尺度相关的双臂操作场景。

项目将机器人语义、相机参数和场景参数保存在 YAML 中，在边界处严格校验各类 profile，再将其编译为原生 Isaac Lab 配置对象。当前实现聚焦于可复用的机器人与场景构建，还不是一套完整的任务、策略或评测流水线。

## 已实现能力

- **类型化机器人配置**：校验关节、TCP、执行器、平行夹爪和可选机器人相机，并生成全新的 Isaac Lab 配置。
- **可复用相机配置**：将相机光学与输出参数和场景、机器人内部的传感器位姿分离。
- **类型化场景元数据**：校验场景 preset 顶层结构，并公开环境局部坐标系中的任务物体放置范围。
- **可复用双臂场景**：组合房间、带纹理的地面和桌面、两台带腕部相机的机器人、顶视 RGB-D 相机及环境光。
- **纹理正确的程序化表面**：`UvCuboidCfg` 会写入 face-varying UV，使 MDL 材质能在长方体表面正确平铺。
- **可直接运行的场景预览**：既可以在 Isaac Sim 中查看放置区域与相机视锥，也可以执行短时间无界面冒烟验证。

> [!NOTE]
> `src/scale_bench` 目前只提供配置与场景基础层。任务、episode 调度、数据记录和 benchmark 报告尚未在该包中实现。

## 架构

```text
configs/robots/*.yml
        │
        ▼
  RobotProfile ── 校验 ──► ArticulationCfg ───────────┤
                └───────► robot CameraCfg ────────────┤
                                                      │
configs/scene/*.yml ──► SceneConfig ──────────────────┼─► DualArmTabletopSceneCfg
                                                      │
configs/cameras/*.yml ──► CameraProfile ──► CameraCfg ┤
                                                      │
  UvCuboidCfg ── 带纹理的地面和桌面 ──────────────────┘
                                                              │
                                                              ▼
                                                    Isaac Lab InteractiveScene
```

这一层边界有意保持精简：机器人特有信息放在 robot profile 中，场景特有信息放在 scene preset 中，下游代码只接收标准 Isaac Lab 配置对象。

## 环境要求

`pyproject.toml` 和 `uv.lock` 中的依赖配置面向以下环境：

- Python 3.12
- Isaac Sim 6.0.1
- PyTorch 2.10，使用 CUDA 12.8 wheels
- 使用 [`uv`](https://docs.astral.sh/uv/) 管理依赖

Isaac Lab 以本地 editable dependency 的形式从 `third_parties/IsaacLab` 加载；当前工作区已验证兼容的 checkout 是 `release/3.0.0-beta2`。机器还需要满足 Isaac Sim 对 GPU、驱动和操作系统的要求。本项目不会安装系统级 NVIDIA 驱动。

## 安装

克隆仓库，并将指定版本的 Isaac Lab 放到 `third_parties/IsaacLab`：

```bash
git clone https://github.com/CrysGate/learn-isaac.git
cd learn-isaac

mkdir -p third_parties
git clone --branch release/3.0.0-beta2 --depth 1 \
  https://github.com/isaac-sim/IsaacLab.git \
  third_parties/IsaacLab

uv sync --frozen
```

默认配置还要求项目资产包位于 `Assets/`。如果你的 checkout 中不包含这些资产，请补充以下文件，以及它们引用的纹理和其他 USD 依赖：

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

配置中的相对路径统一从仓库根目录解析，因此请保持资产目录结构不变。

## 快速开始

在 Isaac Sim 中打开默认双 Piper 场景：

```bash
uv run python scripts/preview_scene.py
```

执行两步无界面冒烟验证：

```bash
uv run python scripts/preview_scene.py --viz none --max-steps 2
```

无需修改 Python 代码即可替换左右机器人或场景配置：

```bash
uv run python scripts/preview_scene.py \
  --config configs/scene/default.yml \
  --left-robot-config configs/robots/piper.yml \
  --right-robot-config configs/robots/piper.yml \
  --device cuda:0
```

常用预览参数：

| 参数 | 作用 |
|---|---|
| `--config PATH` | 选择场景 YAML。 |
| `--left-robot-config PATH` | 选择左臂 robot profile。 |
| `--right-robot-config PATH` | 选择右臂 robot profile。 |
| `--device VALUE` | 选择 `cpu`、`cuda` 或 `cuda:0` 等具体设备。 |
| `--viz none` | 关闭 visualizer，以无界面方式运行。 |
| `--max-steps N` | 在指定仿真步数后退出。 |
| `--camera-frustum-length-m M` | 设置预览中相机视锥的显示长度，单位为米。 |

运行 `uv run python scripts/preview_scene.py --help` 可以查看 Isaac Lab launcher 的全部参数。

## 核心 API

### Robot profile

[`RobotProfile`](src/scale_bench/robots/robot_profile.py) 是 `configs/robots/*.yml` 与 Isaac Lab 之间的类型化边界：

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

只加载和校验 profile 不需要启动仿真器；调用 `build_articulation_cfg()` 前则应先启动 Isaac Lab `AppLauncher`。仓库中的 [`preview_scene.py`](scripts/preview_scene.py) 展示了正确的启动和 import 顺序。

`RobotProfile.load()` 会：

- 从仓库根目录解析相对路径；
- 拒绝未知字段和非有限数值；
- 要求机械臂、夹爪和执行器中的关节名唯一；
- 确保初始位置恰好覆盖所有已声明关节；
- 检查执行器覆盖关系，并禁止不同执行器组重复控制同一关节；
- 校验 TCP、平行夹爪和可选相机安装约定；
- 加载并校验机器人引用的相机 profile；
- 检查本地 USD 和可选 URDF 资产是否存在。

`build_articulation_cfg()` 每次都会返回一份新的 Isaac Lab `ArticulationCfg`。`build_camera_cfg()` 会在给定机器人根节点下创建相机；未配置相机时返回 `None`。当前机器人实现支持 implicit actuator、parallel-jaw gripper 和一台挂载相机。

### 相机 profile

[`CameraProfile`](src/scale_bench/sensors/camera_profile.py) 可以在不启动 Isaac Sim 的情况下校验可复用的相机光学和输出参数：

```python
from scale_bench.sensors import CameraProfile

profile = CameraProfile.load("configs/cameras/d435.yml")
```

相机 profile 负责图像尺寸、更新周期、数据类型、针孔内参、畸变元数据、焦距和裁剪范围。场景与机器人 profile 引用它，并分别保有自身资产内部的安装位姿。左右 Piper 腕部相机和顶视相机复用同一份 D435 profile。

### 场景模板

[`create_dual_arm_tabletop_scene_cfg()`](src/scale_bench/scenes/scene_template.py) 将两个机器人 profile 和场景 preset 组合起来：

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

这段代码应在 `AppLauncher` 完成 Isaac Sim 初始化后运行。

`SceneConfig` 会校验场景顶层结构，以及 `task_object_placement_area` 中有限且顺序正确的 XY 边界。任务构建器和可视化工具可以复用它的 `table_top_z_m` 属性与放置区域元数据，无需重复计算场景几何。

```python
from scale_bench.scenes import SceneConfig

scene_metadata = SceneConfig.load("configs/scene/default.yml")
placement_area = scene_metadata.task_object_placement_area
table_top_z_m = scene_metadata.table_top_z_m
```

场景包含：

- USD 房间和 dome light；
- 启用碰撞并带纹理的地面与桌面；
- 左右两套独立机器人安装位；
- 左右机器人各自挂载的 D435 风格 RGB-D 传感器；
- 相机支架和顶视 D435 风格 RGB-D 传感器；
- 可配置的环境数量、间距、物理复制和 Fabric cloning。

机器人底座和相机支架会根据计算得到的桌面高度放置，因此修改桌子高度后，安装在桌面上的资产仍会自动对齐。

### UV 长方体

[`UvCuboidCfg`](src/scale_bench/scenes/uv_cuboid.py) 在 Isaac Lab `CuboidCfg` 的基础上增加了 `uv_scale`。它先把几何和物理创建交给 Isaac Lab，再为六个表面写入 24 个 face-varying `st` 值，每个表面四个，从而得到可预测的材质平铺效果。

## 配置方法

### 添加机器人

1. 复制 [`configs/robots/piper.yml`](configs/robots/piper.yml)。
2. 修改资产路径、初始关节状态、运动学 frame、TCP、执行器组、夹爪语义和可选相机安装关系。
3. 四元数使用 `xyzw` 顺序，距离使用米。
4. 不启动 Isaac Sim，直接校验 profile：

   ```bash
   PYTHONPATH=src uv run python -c \
     'from scale_bench.robots import RobotProfile; p = RobotProfile.load("configs/robots/my_robot.yml"); print(p.name)'
   ```

5. 使用 `--left-robot-config` 或 `--right-robot-config` 在场景中预览机器人。

不要在场景代码中添加基于机器人名称的分支。如果新机器人需要尚不支持的执行器或末端执行器类型，应显式扩展 profile 模型。

### 自定义场景

复制 [`configs/scene/default.yml`](configs/scene/default.yml)，然后修改对应部分：

| 配置段 | 控制内容 |
|---|---|
| `room` | 房间 USD 和统一缩放。 |
| `ground`、`table` | 位姿、尺寸、材质、摩擦、恢复系数和 UV 平铺。 |
| `task_object_placement_area` | 环境局部 XY 平面上的任务物体放置范围。 |
| `robot_mounts` | 左右机器人底座相对于桌面的位姿。 |
| `camera` | 相机 profile 引用、支架位姿和传感器变换。 |
| `lighting` | HDR 环境纹理和光照强度。 |
| `runtime` | 环境数量、间距、物理复制和 Fabric cloning。 |

场景 YAML 会按路径在进程内缓存。修改场景配置后，请重启预览进程。

## 仓库结构

```text
src/scale_bench/
├── robots/
│   └── robot_profile.py    # 机器人 schema 及 articulation/camera 构建
├── scenes/
│   ├── scene_config.py     # 场景 YAML 与放置区域 schema
│   ├── scene_template.py   # 双臂桌面场景编译
│   └── uv_cuboid.py        # 带 face-varying UV 的长方体 spawner
└── sensors/
    └── camera_profile.py   # YAML schema、校验、CameraCfg 构建

configs/
├── cameras/d435.yml        # 可复用相机 profile
├── robots/piper.yml        # 参考机器人 profile
└── scene/default.yml       # 场景内位姿和环境配置

scripts/preview_scene.py    # 交互预览和无界面冒烟验证入口
```

项目使用 `src` 布局，但没有作为 package 安装（`tool.uv.package = false`）。仓库自带的预览脚本会把 `src` 加入 `sys.path`；运行你自己的独立脚本时，请设置 `PYTHONPATH=src`，或者显式添加该目录。

## 常见问题

- **出现 `Robot asset does not exist`**：检查 robot YAML 中的路径，并确认 `Assets/` 资产包完整。
- **找不到本地 Isaac Lab 依赖**：运行 `uv sync` 前确认 `third_parties/IsaacLab/source/...` 已存在。
- **自定义脚本出现 `No module named scale_bench`**：从仓库根目录使用 `PYTHONPATH=src` 启动。
- **多个环境的房间互相重叠**：增大 `runtime.env_spacing_m`，尤其是在修改房间缩放之后。

## 延伸阅读

- [Robot profile 约定](docs/robot_profiles.md)
- [场景模板说明](docs/scene_template.md)
- [Benchmark 架构方向](docs/benchmark_architecture.md)——这是设计目标，其中部分组件尚未实现

## 许可证

本项目使用 [MIT License](LICENSE)。
