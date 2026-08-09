# ScaleBench 当前架构

本文只描述已经实现的边界。正式环境入口、Action Manager 与 policy Observation Manager 已经建立；EE 控制、Evaluator、Runner 和 Recorder 尚未实现。

## 当前组件

| 组件 | 职责 |
|---|---|
| `RobotProfile` | 从 YAML 校验机器人、关节、TCP、夹爪和挂载相机，并生成 Isaac Lab 配置。 |
| `CameraProfile` | 校验可复用的相机光学和输出参数，并生成 `CameraCfg`。 |
| `SceneConfig` | 校验 scene YAML，公开桌面高度和 `task_object_placement_area`。 |
| `DualArmTabletopSceneCfg` | 描述公共房间、桌面、双臂、相机和灯光。 |
| `TaskDefinition` | 统一任务资产加载、seed 布局生成、布局校验和 JSON 导入导出。 |
| `SortDollsBySize` | 当前唯一具体任务，声明套娃资产、instruction 和尺寸目标顺序。 |
| `EnvRuntimeConfig` | 校验 arm action 模式、control decimation、reset 重渲染、纹理等待和环境 seed。 |
| `create_env_cfg()` | 将 Scene、Task layout 来源、Sim 和 manager 配置编译为原生 EnvCfg。 |
| `ScaleBenchEnvCfg` | 可直接交给 Isaac Lab 的完整 `ManagerBasedEnvCfg`。 |
| `ResetTaskLayout` | 在 reset 时为指定 `env_id` 恢复其固定的初始 layout。 |
| `ScaleBenchEnv` | 唯一持有仿真生命周期，并从实际运行对象导出元数据。 |
| `scripts/preview_scene.py` | 通过 `ScaleBenchEnv` 预览公共场景或指定 seed/layout 的任务场景。 |

目录保持为最小结构：

```text
src/scale_bench/
├── envs/
│   ├── action_cfg.py
│   ├── env_cfg.py
│   ├── events.py
│   ├── mdp/
│   │   └── observations.py
│   ├── observation_cfg.py
│   ├── runtime_config.py
│   └── scale_bench_env.py
├── robots/robot_profile.py
├── scenes/
│   ├── scene_config.py
│   ├── scene_template.py
│   └── uv_cuboid.py
├── sensors/camera_profile.py
└── tasks/
    ├── base.py
    └── sort_dolls_by_size.py

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
Robot/Camera/Scene YAML ──► profiles ──► DualArmTabletopSceneCfg ──┐
Task YAML + seed/layout ──► TaskDefinition ─► per-env layouts/assets ┤
Sim YAML ────────────────► SimConfig ─► SimulationCfg ─────────────┤
Env YAML ────────────────► EnvRuntimeConfig ───────────────────────┤
                                                                   ▼
                                                     create_env_cfg()
                                                                   │
                                                                   ▼
                                                        ScaleBenchEnvCfg
                                                                   │
                                                                   ▼
                                                        ScaleBenchEnv
                                             reset() / step() / close()
                                                  │             │
                                stable per-env layout      actual IO metadata
```

`DualArmTabletopSceneCfg` 是公共 `InteractiveSceneCfg`。Task 不创建任务专用 SceneCfg 子类。调用方传入 `task_layout_seed` 时，`create_env_cfg()` 为每个环境生成初始 layout；传入一个 `task_layouts` 元素时广播给所有环境，传入 `num_envs` 个元素时按 `env_id` 分配。Task 仍只负责 layout 生成、校验和资产字段注册。

## Env 入口

`configs/envs/default.yml` 管理环境运行时与 action 模式，不复制 Sim 或 Scene 配置：

```yaml
control_decimation: 4
arm_action_mode: joint_position
num_rerenders_on_reset: 1
wait_for_textures: true
seed: 0
```

默认 SimConfig 以 120 Hz 推进 physics，EnvRuntimeConfig 每 4 个 physics step 执行一次环境 step，因此环境、render 和三个相机都以 30 Hz 同步更新。builder 会在创建仿真前检查 `render_interval == control_decimation` 且每个相机 `update_period == step_dt`。

`create_env_cfg()` 直接返回 `ScaleBenchEnvCfg`，调用方通过 `ScaleBenchEnv(env_cfg)` 创建环境。环境的 `get_IO_descriptors` 复用 Isaac Lab 原生 action、observation、articulation 和 scene descriptor，并从已经初始化的 env、sim 与 camera sensor 计算 runtime timing，不保留第二份构建期事实来源。

Task 环境在配置期按 `base_seed + env_id` 一次性生成每个环境的确定性 layout，或直接接收一个/每环境一个显式 layout。reset event 不采样、不推进 seed，只恢复本次 reset 环境原有的 layout。`info["episode"]` 返回对应的 `env_ids`、`task_id`、`instruction` 和稳定的 `layout_seeds`。

Action Manager 按 `left_arm | left_gripper | right_arm | right_gripper` 组织 term。机械臂与夹爪都直接使用 Isaac Lab `JointPositionAction`，关节顺序和维度来自各自 RobotProfile，并接收物理单位的绝对关节目标；夹爪目标由 profile 的开合位置范围约束。`arm_action_mode` 是 policy 输出模式与控制 term 的显式分派边界，后续 EE 控制在此扩展。

Observation Manager 的 `policy` group 不拼接，包含左右机械臂与夹爪实际 joint position，以及场景中已配置相机的原始 RGB-D。任务物体和评测真值不进入该 group。环境启动时会对照 manager 实际解析结果检查 IO descriptor 的 term 维度、关节顺序和相机输出。

## Task 接口

当前公共入口只有任务信息和一次资产装载操作：

```python
task.task_id
task.instruction
task.target_order_small_to_large  # SortDollsBySize 的具体任务目标

layout = task.resolve_layout(seed=42)
layout.save("layouts/sort_dolls_by_size/42.json")
task.add_assets_to_scene(scene_cfg, layout)
```

也可以从文件恢复：

```python
layout = task.resolve_layout(
    layout_path="layouts/sort_dolls_by_size/42.json",
)
task.add_assets_to_scene(scene_cfg, layout)
```

`seed` 与 `layout_path` 互斥；两者都不传时使用 seed `0`。`resolve_layout()` 只解析和校验，`save()` 显式执行文件写入，`add_assets_to_scene()` 只注册已解析 layout 对应的资产。

共同逻辑集中在 `tasks/base.py`：

- 加载任务 YAML 和资产 metadata；
- 根据 seed 使用局部随机数生成器采样，不修改进程全局随机状态；
- 用 metadata 的 XY 外接圆半径约束采样范围；
- 检查完整 footprint 位于 `task_object_placement_area` 内；
- 检查物体间不重叠且满足最小间距；
- 根据桌面高度与物体高度计算 upright 初始 Z；
- 构造具名 `RigidObjectCfg` 并加入公共 SceneCfg；
- 导入和导出稳定、可读的 layout JSON。

具体任务文件只保留套娃配置 schema、资产命名和目标尺寸顺序。

## Layout 文件

布局格式保持简单、可读，并记录生成它的 seed。下面是当前默认配置按 seed `42` 生成的完整、可加载示例：

```json
{
  "task_id": "sort_dolls_by_size",
  "seed": 42,
  "assets": {
    "doll_00000": {
      "position_m": [0.1668683782229724, -0.13473224691440094, 0.8330000000000001],
      "orientation_xyzw": [0.0, 0.0, -0.6493780073399296, 0.7604657806786722]
    },
    "doll_00001": {
      "position_m": [-0.3356599591598113, 0.25806032001048207, 0.8230000000000001],
      "orientation_xyzw": [0.0, 0.0, 0.527043424660673, 0.8498383543486075]
    },
    "doll_00002": {
      "position_m": [0.48181836323958327, -0.11398302145887892, 0.8130000000000001],
      "orientation_xyzw": [0.0, 0.0, -0.2428374969504492, 0.9700669822619676]
    },
    "doll_00003": {
      "position_m": [-0.58513914100527, -0.04225283792710749, 0.803],
      "orientation_xyzw": [0.0, 0.0, 0.016823340091961164, 0.9998584775997802]
    },
    "doll_00004": {
      "position_m": [0.18890149672371181, 0.15242842720869737, 0.793],
      "orientation_xyzw": [0.0, 0.0, -0.7696301475203746, 0.6384899654871378]
    }
  }
}
```

加载时会重新执行所有约束校验，而不是盲目信任文件。任务 ID 和资产名称集合必须与当前任务完全一致。当前格式没有 `schema_version` 字段；如果以后需要不兼容地修改 layout schema，应先增加显式版本和迁移规则。

## 当前边界

Task 只负责配置期的任务资产和初始布局。它不会：

- 选择或控制机器人；
- 启动 Isaac Sim 或创建 `SimulationContext`，这些由 `ScaleBenchEnv` 负责；
- 实现 Policy、规划器或成功判定；
- 管理 episode reset、step 或数据记录。

后续出现第二个任务时，优先复用 `TaskDefinition` 中已经稳定的刚体与布局行为。只有新任务确实需要不同的资产类型或放置规则时，才增加新的小接口；不创建任务专用 SceneCfg 继承树，也不把仿真生命周期下放给 Task。
