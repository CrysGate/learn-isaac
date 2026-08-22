# ScaleBench

[English](README.md) | [简体中文](README.zh-CN.md)

ScaleBench 是一组配置优先的 Isaac Lab 基础组件，用于构建尺度相关的双臂操作场景。

项目将机器人语义、相机、场景、任务、仿真和环境参数保存在 YAML 中，在边界处严格校验，再将其编译为原生 Isaac Lab 配置对象。当前实现提供可复用的场景构建、一个支持最终状态评测的 seed/layout 驱动任务和正式的 manager-based 运行时入口，但还不是完整的策略或 benchmark 流水线。

## 已实现能力

- **类型化机器人配置**：校验关节、TCP、执行器、平行夹爪和可选机器人相机，并生成全新的 Isaac Lab 配置。
- **可复用相机配置**：将相机光学与输出参数和场景、机器人内部的传感器位姿分离。
- **类型化场景元数据**：校验场景 preset 的每个嵌套区块，并公开环境局部坐标系中的任务物体放置范围。
- **类型化仿真 preset**：只校验会定义 benchmark 行为的时间步、重力、渲染和操作稳定性参数，其余设置继承 Isaac Lab 默认值。
- **可复用双臂场景**：组合房间、带纹理的地面和桌面、两台带腕部相机的机器人、顶视 RGB-D 相机及环境光。
- **精简的 Task 接口与首个任务**：在不依赖仿真器的情况下描述任务资产与布局；`SortDollsBySize` 是第一个与机器人型号无关的案例。
- **Manager-based 环境入口**：通过可安全导入的公共 API 创建 `ScaleBenchEnv`，应用启动仍由调用方管理。
- **Profile 驱动的 action**：左右机械臂与夹爪均使用动态维度、物理单位的绝对命令关节目标。
- **具名 policy observation**：按 profile 顺序提供机器人状态和已配置相机的原始 RGB-D，不混入任务或评测真值。
- **任务专属评测 observation**：每个任务可以注册所需的具名仿真真值 term，并通过统一的环境方法评测指定环境。
- **纹理正确的程序化表面**：`UvCuboidCfg` 会写入 face-varying UV，使 MDL 材质能在长方体表面正确平铺。
- **可直接运行的场景预览**：既可以在 Isaac Sim 中查看放置区域与相机视锥，也可以执行短时间无界面冒烟验证。

> [!NOTE]
> 关节空间 Action、policy/evaluator Observation、任务专属最终状态评测与可选 Recorder Manager term 已接入。末端控制、episode 调度和 benchmark 报告仍留待后续实现。

## 架构

```text
机器人/场景/相机 YAML ──► config.loader / 纯模型 ─────────┐
任务 YAML + 场景上下文 ──► Task / TaskLayout ──────────────┤
sim YAML ────────────────► SimulationConfig ───────────────┤
env YAML ────────────────► EnvironmentConfig ──────────────┤
                                                           ▼
                                             api.create_env()
                                                           │
                                                           ▼
                                      内部 cfg ─────► ScaleBenchEnv
                                      reset() / step() / IO descriptor
```

这一层边界有意保持精简：机器人特有信息放在 robot config 中，场景特有信息放在 scene preset 中，应用只通过公共 API 获得初始化后的环境。

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

本 Git 仓库不包含运行默认配置所需的项目资产包。使用默认 preset 前，需要从项目维护者或团队资产存储中另行取得标准资产包并放到 `Assets/`，也可以在以下精确路径提供兼容资产。还必须包含这些文件引用的全部纹理和传递 USD 依赖：

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

配置引用相对于包含它的文件解析。预览脚本默认传入 `--asset-root .`，因此应从资产根目录运行，或显式指定其他根目录。

## 快速开始

在 Isaac Sim 中打开默认双 Piper 场景：

```bash
uv run python scripts/preview_scene.py
```

通过稳定的 task ID 预览任务场景：

```bash
uv run python scripts/preview_scene.py --task sort_dolls_by_size
```

生成并导出可复现布局，或在之后直接加载：

```bash
uv run python scripts/preview_scene.py --task sort_dolls_by_size \
  --seed 42 --export-layout layouts/sort_dolls_by_size/42.json
uv run python scripts/preview_scene.py --task sort_dolls_by_size \
  --layout layouts/sort_dolls_by_size/42.json
```

执行两步无界面冒烟验证：

```bash
uv run python scripts/preview_scene.py --viz none --max-steps 2
```

无需修改 Python 代码即可替换左右机器人或场景配置：

```bash
uv run python scripts/preview_scene.py \
  --config configs/scene/default.yml \
  --sim-config configs/sim/default.yml \
  --env-config configs/envs/default.yml \
  --left-robot-config configs/robots/piper.yml \
  --right-robot-config configs/robots/piper.yml \
  --device cuda:0
```

常用预览参数：

| 参数 | 作用 |
|---|---|
| `--config PATH` | 选择场景 YAML。 |
| `--sim-config PATH` | 选择仿真、渲染和 PhysX 参数。 |
| `--env-config PATH` | 选择控制 decimation 和 reset 生命周期参数。 |
| `--task TASK_ID` | 在公共场景上添加任务资产（目前支持 `sort_dolls_by_size`）。 |
| `--seed N` | 确定性生成任务布局；默认使用零。 |
| `--layout PATH` | 从已导出的 layout JSON 加载任务资产位姿。 |
| `--export-layout PATH` | 保存本次生成或加载的任务布局。 |
| `--asset-root PATH` | 使用明确的根目录解析资产引用。 |
| `--left-robot-config PATH` | 选择左臂 robot profile。 |
| `--right-robot-config PATH` | 选择右臂 robot profile。 |
| `--device VALUE` | 选择 `cpu`、`cuda` 或 `cuda:0` 等具体设备。 |
| `--viz none` | 关闭 visualizer，以无界面方式运行。 |
| `--max-steps N` | 在指定环境步数后退出。 |
| `--camera-frustum-length-m M` | 设置预览中相机视锥的显示长度，单位为米。 |

运行 `uv run python scripts/preview_scene.py --help` 可以查看 Isaac Lab launcher 的全部参数。

## 核心 API

### 配置层

[`load_config()`](src/scale_bench/config/loader.py) 是唯一的 YAML/JSON 加载边界。六类不可变纯 Python 模型只包含数据和局部校验：

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

模型禁止未知字段、冻结属性、拒绝非有限数值，并且不读取文件或构造 Isaac Lab 对象。配置引用相对于包含它的配置文件解析；资产引用在提供 `asset_root` 时相对于该目录解析，否则相对于配置文件目录解析。绝对路径保持不变，本地资产缺失时错误会包含源文件与字段位置。当前只支持本地配置与资产路径。`num_envs`、间距、克隆、控制和 reset 设置属于 `EnvironmentConfig`，不再属于 `SceneConfig`。

默认仿真 preset 以 120 Hz 运行 physics，每四步渲染一次，即 30 Hz。材质、Fabric、日志、solver iteration 和 GPU buffer 仍继承当前安装的 Isaac Lab 默认值。

### 环境运行时

[`create_env()`](src/scale_bench/api.py) 接收已加载的 `RobotConfig`、`SceneConfig`、`SimulationConfig`、`EnvironmentConfig` 和可选的 Task layout 来源。它只在 Isaac Sim 启动后调用函数时延迟导入适配层：

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
    # 执行 policy，用最新最终状态评测，并在 reset 前导出。
    results = env.evaluate()
    env.complete_episodes(
        success=[results[env_id].success for env_id in range(env.num_envs)]
    )
finally:
    env.close()
    simulation_app.close()
```

`ScaleBenchEnv` 继承 Isaac Lab 的 `ManagerBasedEnv`，是 `SimulationContext`、`InteractiveScene`、reset、step 和清理操作的唯一所有者；`AppLauncher` 及其 application 仍由调用方持有。runtime IO 元数据由 `isaaclab/runtime/io_descriptors.py` 从初始化后的 manager 和 sensor 计算。使用 `base_seed` 时，环境 `i` 在配置期一次性获得由 `base_seed + i` 生成的布局，之后的全量或局部 reset 都恢复该布局。也可以通过 `layouts` 传入一个布局并广播，或传入恰好 `num_envs` 个布局。`info["episode"]` 返回受影响的环境 ID 及稳定 layout seed。

数据记录通过 `RecordingConfig` 显式启用。默认 term 将初始相对场景状态、原始与处理后 action，以及关节观测写入 HDF5；相机 RGB-D 通过 `record_camera_observations=True` 独立开启，逐步场景真值也必须显式开启。`complete_episodes()` 写入 success 并导出指定环境的 buffer，必须在这些环境下一次 reset 前调用。`overwrite_existing=False` 时会自动递增已占用的名称（`rollout.hdf5`、`rollout_1.hdf5`……）；`overwrite_existing=True` 才会明确复用请求的名称。

Camera、robot、scene、simulation、task、manager 和 environment 的原生 cfg 实现统一位于 [`scale_bench.isaaclab`](src/scale_bench/isaaclab)。重构前的 `envs`、`scenes`、`robots`、`sensors` 和 `sim` 导入路径已删除；应用代码只使用纯配置模型和 `scale_bench.api`。

### 机器人配置

[`RobotConfig`](src/scale_bench/config/models/robot.py) 在不依赖 Isaac Sim 的情况下校验机器人语义。通过 `load_config()` 加载时会：

- 相对于机器人 YAML 解析配置引用，并相对于显式资产根解析资产引用；
- 拒绝未知字段和非有限数值；
- 要求机械臂、夹爪和执行器中的关节名唯一；
- 确保初始位置恰好覆盖所有已声明关节；
- 检查执行器覆盖关系，并禁止不同执行器组重复控制同一关节；
- 校验 TCP、平行夹爪和可选相机安装约定；
- 检查本地 USD 和可选 URDF 资产是否存在。

### 相机配置

[`CameraConfig`](src/scale_bench/config/models/camera.py) 负责图像尺寸、更新周期、数据类型、针孔内参、畸变元数据、焦距、裁剪范围和坐标约定：

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.camera import CameraConfig

camera = load_config("configs/cameras/d435.yml", CameraConfig)
```

场景与机器人配置引用它，并分别保有自身资产内部的安装位姿。左右 Piper 腕部相机和顶视相机复用同一份 D435 配置。

### 场景配置

`SceneConfig` 校验静态场景区块，包括资产引用、有限位姿、正数尺寸、材质参数、单位四元数、相机坐标约定和有序 XY 放置边界。它的 `table_top_z_m` 属性与放置区域元数据可直接复用，无需重复计算场景几何。

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.scene import SceneConfig

scene_metadata = load_config("configs/scene/default.yml", SceneConfig, asset_root=".")
placement_area = scene_metadata.task_object_placement_area
table_top_z_m = scene_metadata.table_top_z_m
```

Isaac Sim 启动后，`create_env()` 会将已加载的场景、机器人、相机与环境配置组合为原生双臂场景。

场景包含：

- USD 房间和 dome light；
- 启用碰撞并带纹理的地面与桌面；
- 左右两套独立机器人安装位；
- 左右机器人各自挂载的 D435 风格 RGB-D 传感器；
- 相机支架和顶视 D435 风格 RGB-D 传感器；
- 由 `EnvironmentConfig` 提供的环境数量、间距、物理复制和 Fabric cloning。

机器人底座和相机支架会根据计算得到的桌面高度放置，因此修改桌子高度后，安装在桌面上的资产仍会自动对齐。

### Task

`Task` Protocol 只公开任务身份、instruction，以及由上下文驱动的布局生成和校验。`RigidObjectTask` 复用 metadata、确定性桌面采样和 layout JSON 行为，但不持有 `SceneConfig`。`SortDollsBySize` 只声明套娃资产和尺寸排序目标。原生 `RigidObjectCfg` 由适配层的 TaskBuilder 构建。

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

调用 `task.resolve_layout(context, layout_path=...)` 会恢复并校验保存的精确位姿。将该 layout 传给 `create_env(..., task=task, layouts=(layout,))` 后，environment builder 会从 `SceneConfig` 派生同一上下文，选择内置 TaskBuilder，并注册新的原生资产和 evaluator observation cfg。要改用其他任务 YAML，应先将其加载为 `SortDollsBySizeConfig`。Piper/cuRobo 规划、机器人分工、数据记录和应用生命周期仍留在 Task 层之外；任务专属的成功评测由 Task 自身实现。

### UV 长方体

[`UvCuboidCfg`](src/scale_bench/isaaclab/spawners/uv_cuboid.py) 在 Isaac Lab `CuboidCfg` 的基础上增加了 `uv_scale`。它先把几何和物理创建交给 Isaac Lab，再为六个表面写入 24 个 face-varying `st` 值，每个表面四个，从而得到可预测的材质平铺效果。

## 配置方法

### 添加机器人

1. 复制 [`configs/robots/piper.yml`](configs/robots/piper.yml)。
2. 修改资产路径、初始关节状态、运动学 frame、TCP、执行器组、夹爪语义和可选相机安装关系。
3. 四元数使用 `xyzw` 顺序，距离使用米。
4. 不启动 Isaac Sim，直接校验 profile：

   ```bash
   PYTHONPATH=src uv run python -c \
     'from scale_bench.config.loader import load_config; from scale_bench.config.models.robot import RobotConfig; print(load_config("configs/robots/my_robot.yml", RobotConfig, asset_root=".").name)'
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

环境数量、间距、物理复制和 Fabric cloning 在 `configs/envs/*.yml` 中配置。

### 自定义仿真

复制 [`configs/sim/default.yml`](configs/sim/default.yml) 即可创建仿真 preset。时间步与重力位于顶层，`render` 选择观测质量，`physx` 只保留当前由运行行为证明有必要的一个操作稳定性覆盖项。其他参数跟随 Isaac Lab 默认值；只有 benchmark 需求证明某项必须变化时，才应把它提升为公共配置。使用 `--sim-config` 选择 preset，使用 `--device` 做临时的机器相关覆盖。

复制 [`configs/envs/default.yml`](configs/envs/default.yml) 可以修改环境克隆、机械臂 action 模式、control decimation、reset 重渲染次数、纹理等待和环境 seed。当前机械臂模式为 `joint_position`，该显式分派点为后续末端控制模式保留。builder 要求 render interval、control decimation 和相机更新周期共同描述同一个同步环境频率。

### 验证改动

运行 `uv run pytest` 可执行快速的配置、Task、builder、runtime contract 和依赖边界测试。初始化后的双环境 runtime 测试由 `integration` marker 隔离，因为它需要 Isaac Sim、受支持的 GPU 和外部资产包：

```bash
uv run pytest -m integration -q
```

该集成测试在子进程中通过公共 API 启动环境，覆盖 create/reset/step/close、渲染 RGB-D 观测、初始化后的 IO descriptor、每环境 layout seed 与 partial reset。可以通过 `SCALE_BENCH_ASSET_ROOT` 指定 checkout 外的资产包；Git worktree 会在可用时自动使用主 worktree 的 `Assets/`。

## 仓库结构

```text
src/scale_bench/
├── api.py                  # 延迟导入适配层的公共 create_env 入口
├── config/
│   ├── base.py             # 不可变模型基类与公共约束
│   ├── loader.py           # YAML/JSON 加载与错误包装
│   ├── paths.py            # 配置引用与资产引用语义
│   └── models/             # 纯 camera、robot、scene、sim、env 模型
├── isaaclab/
│   ├── builders/           # 纯数据到原生 cfg 的转换
│   ├── managers/           # Action、Observation、Event cfg 声明
│   ├── mdp/                # manager 运行时 term
│   ├── runtime/            # ScaleBenchEnv 与运行时 IO descriptor
│   └── spawners/           # 项目自定义原生 spawner
└── tasks/
    ├── common/             # Task 契约、布局、放置算法与刚体数据
    └── sort_dolls_by_size/ # 任务专用配置与规则

configs/
├── cameras/d435.yml        # 可复用相机 profile
├── envs/default.yml        # 克隆、控制与 reset 生命周期参数
├── robots/piper.yml        # 参考机器人 profile
├── scene/default.yml       # 静态场景资产和局部位姿
├── sim/default.yml         # 仿真、渲染与 PhysX 参数
└── tasks/sort_dolls_by_size.yml

scripts/preview_scene.py    # 交互预览和无界面冒烟验证入口
```

项目使用 `src` 布局，但没有作为 package 安装（`tool.uv.package = false`）。仓库自带的预览脚本会把 `src` 加入 `sys.path`；运行你自己的独立脚本时，请设置 `PYTHONPATH=src`，或者显式添加该目录。

## 常见问题

- **出现 `Robot asset does not exist`**：检查 robot YAML 中的路径，并确认 `Assets/` 资产包完整。
- **找不到本地 Isaac Lab 依赖**：运行 `uv sync` 前确认 `third_parties/IsaacLab/source/...` 已存在。
- **自定义脚本出现 `No module named scale_bench`**：从仓库根目录使用 `PYTHONPATH=src` 启动。
- **多个环境的房间互相重叠**：增大环境配置中的 `env_spacing_m`，尤其是在修改房间缩放之后。

## 延伸阅读

- [Robot profile 约定](docs/robot_profiles.md)
- [场景模板说明](docs/scene_template.md)
- [当前 Benchmark 架构与边界](docs/benchmark_architecture.md)

## 许可证

本项目使用 [MIT License](LICENSE)。
