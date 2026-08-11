# ScaleBench 当前架构

本文只描述已经实现的边界。正式环境入口、Action Manager 与 policy Observation Manager 已经建立；EE 控制、Evaluator、Runner 和 Recorder 尚未实现。

## 当前组件

| 组件 | 职责 |
|---|---|
| `RobotConfig` | 与仿真器无关地校验机器人、关节、TCP、夹爪和挂载相机数据。 |
| `CameraConfig` | 与仿真器无关地校验可复用的相机光学和输出参数。 |
| `SceneConfig` | 校验 scene YAML，公开桌面高度和 `task_object_placement_area`。 |
| `DualArmTabletopSceneCfg` | 描述公共房间、桌面、双臂、相机和灯光。 |
| `Task` / `PlacementContext` | 定义任务身份与布局接口，并只传递布局所需的场景值。 |
| `RigidObjectTask` | 复用刚体 metadata、seed 布局生成、校验和 JSON 导入导出。 |
| `scale_bench.isaaclab.builders` | 将纯配置与 Task layout 转换为新的 Isaac Lab cfg。 |
| `scale_bench.isaaclab.managers` | 声明 action、observation 和 event manager cfg。 |
| `SortDollsBySize` | 当前唯一具体任务，声明套娃资产、instruction 和尺寸目标顺序。 |
| `EnvironmentConfig` | 校验环境数量、间距、克隆、control decimation、reset 和 seed。 |
| `scale_bench.api.create_env()` | 延迟导入适配层并返回初始化后的正式环境。 |
| `build_environment_cfg()` | 将纯配置、Task layout 来源和 manager 配置编译为原生 EnvCfg。 |
| `ScaleBenchEnvCfg` | 适配层内部使用的完整 `ManagerBasedEnvCfg`。 |
| `ResetTaskLayout` | 在 reset 时为指定 `env_id` 恢复其固定的初始 layout。 |
| `ScaleBenchEnv` | 唯一持有仿真生命周期，并从实际运行对象导出元数据。 |
| `scripts/preview_scene.py` | 通过 `ScaleBenchEnv` 预览公共场景或指定 seed/layout 的任务场景。 |

当前迁移后的主要责任目录为：

```text
src/scale_bench/
├── config/                 # 纯配置模型、loader 与本地路径解析
├── isaaclab/
│   ├── builders/           # camera/robot/scene/simulation/task/environment
│   ├── managers/           # action/observation/event cfg 声明
│   ├── mdp/                # manager 运行时 term
│   ├── runtime/            # 环境生命周期与初始化后 IO descriptor
│   └── spawners/           # 项目自定义 Isaac Lab spawner
└── tasks/                  # 不导入 Isaac Lab 的任务与布局逻辑
```

Task 目录当前包含：

```text
src/scale_bench/tasks/
├── common/
│   ├── task.py
│   ├── layout.py
│   ├── placement.py
│   └── rigid_object.py
└── sort_dolls_by_size/
    ├── config.py
    └── task.py

configs/
├── cameras/d435.yml
├── envs/default.yml
├── robots/piper.yml
├── scene/default.yml
├── sim/default.yml
└── tasks/sort_dolls_by_size.yml
```

## 配置流

项目默认使用 `Config` 表示 Pydantic 数据配置、`Profile` 表示可复用的 Pydantic 规格，使用 `Cfg` 表示 Isaac Lab `configclass`。这是命名约定，不增加运行时命名校验。

```text
Robot/Camera/Scene YAML ─► load_config() ─► pure configs ──────────┐
Task YAML ───────────────► load_config() ─► Task ─► per-env layouts ┤
Sim YAML ────────────────► load_config() ─► SimulationConfig ──────┤
Env YAML ────────────────► load_config() ─► EnvironmentConfig ─────┤
                                                                    ▼
                                                scale_bench.api
                                                                    │
                                                                    ▼
                                      adapter builders / ScaleBenchEnvCfg
                                                                    │
                                                                    ▼
                                                         ScaleBenchEnv
                                              reset() / step() / close()
                                                   │             │
                                 stable per-env layout      actual IO metadata
```

`DualArmTabletopSceneCfg` 是公共 `InteractiveSceneCfg`。Task 不持有 `SceneConfig`，不创建任务专用 SceneCfg 子类，也不注册 native asset cfg。environment builder 从 `SceneConfig` 派生不可变 `PlacementContext`；公共 API 收到 `base_seed` 时，它为每个环境生成初始 layout；收到一个 `layouts` 元素时广播给所有环境，收到 `num_envs` 个元素时按 `env_id` 分配。TaskBuilder 将 layout 转换为资产 cfg 并加入公共 scene。

## Env 入口

`configs/envs/default.yml` 管理环境运行时与 action 模式，不复制 Sim 或 Scene 配置：

```yaml
control_decimation: 4
arm_action_mode: joint_position
num_rerenders_on_reset: 1
wait_for_textures: true
seed: 0
```

默认 `SimulationConfig` 以 120 Hz 推进 physics，`EnvironmentConfig` 每 4 个 physics step 执行一次环境 step，因此环境、render 和三个相机都以 30 Hz 同步更新。builder 会在创建仿真前检查 `render_interval == control_decimation` 且每个相机 `update_period == step_dt`。

调用方启动 Isaac Sim 后，通过 `scale_bench.api.create_env()` 直接获得 `ScaleBenchEnv`；native `ScaleBenchEnvCfg` 与 builder 保持为适配层内部细节。`api.py` 在顶层只导入纯配置与 Task 类型，Isaac 适配层只在函数内部延迟导入，也不创建或关闭 `AppLauncher`。环境的 `get_IO_descriptors` 由 `isaaclab/runtime/io_descriptors.py` 补充并校验，它复用 Isaac Lab 原生 action、observation、articulation 和 scene descriptor，并从已经初始化的 env、sim 与 camera sensor 计算 runtime timing，不保留第二份构建期事实来源。

`tests/isaaclab/test_headless_runtime.py` 以独立子进程启动 Isaac Sim，自动化验证双环境 create/reset/step/close、实际 RGB-D 观测与 IO descriptor、`base_seed + env_id` 布局分配，以及 partial reset 只恢复指定环境。该测试通过 `pytest -m integration` 显式运行。

Task 环境在配置期按 `base_seed + env_id` 一次性生成每个环境的确定性 layout，或直接接收一个/每环境一个显式 layout。reset event 不采样、不推进 seed，只恢复本次 reset 环境原有的 layout。`info["episode"]` 返回对应的 `env_ids`、`task_id`、`instruction` 和稳定的 `layout_seeds`。

Action Manager 按 `left_arm | left_gripper | right_arm | right_gripper` 组织 term。机械臂与夹爪都直接使用 Isaac Lab `JointPositionAction`，关节顺序和维度来自各自 `RobotConfig`，并接收物理单位的绝对关节目标；夹爪目标由配置的开合位置范围约束。`arm_action_mode` 是 policy 输出模式与控制 term 的显式分派边界，后续 EE 控制在此扩展。

Observation Manager 的 `policy` group 不拼接，包含左右机械臂与夹爪实际 joint position，以及场景中已配置相机的原始 RGB-D。任务物体和评测真值不进入该 group。环境启动时会对照 manager 实际解析结果检查 IO descriptor 的 term 维度、关节顺序和相机输出。

## Task 接口

Task 公共入口只包含任务信息、布局生成和校验：

```python
from scale_bench.config.loader import load_config
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.sort_dolls_by_size.config import SortDollsBySizeConfig
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize

task_config = load_config(
    "configs/tasks/sort_dolls_by_size.yml",
    SortDollsBySizeConfig,
    asset_root=".",
)
context = PlacementContext.from_scene_config(scene_config)
task = SortDollsBySize(task_config)

task.task_id
task.instruction
task.target_order_small_to_large  # SortDollsBySize 的具体任务目标

layout = task.resolve_layout(context, seed=42)
layout.save("layouts/sort_dolls_by_size/42.json")
```

也可以从文件恢复：

```python
layout = task.resolve_layout(
    context,
    layout_path="layouts/sort_dolls_by_size/42.json",
)
```

`seed` 与 `layout_path` 互斥；两者都不传时使用 seed `0`。`resolve_layout()` 只解析和校验，`save()` 显式执行文件写入。将 Task 和 layout 交给 environment builder 后，由内置 `RigidObjectTaskBuilder` 构造并注册原生资产 cfg。

共同逻辑按职责拆分在 `tasks/common`：

- `config.loader` 加载任务 YAML，`rigid_object.py` 加载资产 metadata；
- `task.py` 定义不依赖具体仿真框架的 Task Protocol；
- `layout.py` 定义可序列化布局，`placement.py` 定义上下文与算法；
- 根据 seed 使用局部随机数生成器采样，不修改进程全局随机状态；
- 用 metadata 的 XY 外接圆半径约束采样范围；
- 检查完整 footprint 位于 `task_object_placement_area` 内；
- 检查物体间不重叠且满足最小间距；
- 根据桌面高度与物体高度计算 upright 初始 Z；
- 导入和导出稳定、可读的 layout JSON。

具名 `RigidObjectCfg` 的构造与公共 SceneCfg 注册集中在 `isaaclab/builders/rigid_object_task.py` 和 `isaaclab/builders/environment.py`。

具体任务包只保留套娃配置 schema、资产命名和目标尺寸顺序。

## Layout 文件

布局格式保持简单、可读，并记录生成它的 seed。下面是当前默认配置按 seed `42` 生成的完整、可加载示例：

```json
{
  "task_id": "sort_dolls_by_size",
  "seed": 42,
  "assets": {
    "doll_00000": {
      "position_m": [0.12504033868560732, -0.19098493572006767, 0.8330000000000001],
      "orientation_xyzw": [0.0, 0.0, -0.6493780073399296, 0.7604657806786722]
    },
    "doll_00001": {
      "position_m": [-0.2526231806044581, 0.023942516469479008, 0.8230000000000001],
      "orientation_xyzw": [0.0, 0.0, 0.527043424660673, 0.8498383543486075]
    },
    "doll_00002": {
      "position_m": [0.3641644929281297, -0.18571772961623295, 0.8130000000000001],
      "orientation_xyzw": [0.0, 0.0, -0.2428374969504492, 0.9700669822619676]
    },
    "doll_00003": {
      "position_m": [-0.444078306836691, -0.14691233162800832, 0.803],
      "orientation_xyzw": [0.0, 0.0, 0.016823340091961164, 0.9998584775997802]
    },
    "doll_00004": {
      "position_m": [0.14393616538985488, -0.03380694294210679, 0.793],
      "orientation_xyzw": [0.0, 0.0, -0.7696301475203746, 0.6384899654871378]
    }
  }
}
```

加载时会重新执行所有约束校验，而不是盲目信任文件。任务 ID 和资产名称集合必须与当前任务完全一致。当前格式没有 `schema_version` 字段；如果以后需要不兼容地修改 layout schema，应先增加显式版本和迁移规则。

## 当前边界

Task 只负责配置期的任务资产和初始布局。它不会：

- 选择或控制机器人；
- 启动 Isaac Sim；`AppLauncher` 由应用负责，`SimulationContext` 由 `ScaleBenchEnv` 负责；
- 实现 Policy、规划器或成功判定；
- 管理 episode reset、step 或数据记录。

后续出现第二个刚体任务时，优先复用 `RigidObjectTask` 中已经稳定的 metadata 与布局行为。只有新任务确实需要不同的资产类型或放置规则时，才增加新的小接口；不创建任务专用 SceneCfg 继承树，也不把仿真生命周期下放给 Task。
