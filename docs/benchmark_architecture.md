# ScaleBench 当前架构

本文只描述已经实现的边界。Action、Observation、Evaluator、Runner 和 Recorder 尚未实现，不在当前 Task 层中预留空模块。

## 当前组件

| 组件 | 职责 |
|---|---|
| `RobotProfile` | 从 YAML 校验机器人、关节、TCP、夹爪和挂载相机，并生成 Isaac Lab 配置。 |
| `CameraProfile` | 校验可复用的相机光学和输出参数，并生成 `CameraCfg`。 |
| `SceneConfig` | 校验 scene YAML，公开桌面高度和 `task_object_placement_area`。 |
| `DualArmTabletopSceneCfg` | 描述公共房间、桌面、双臂、相机和灯光。 |
| `TaskDefinition` | 统一任务资产加载、seed 布局生成、布局校验和 JSON 导入导出。 |
| `SortDollsBySize` | 当前唯一具体任务，声明套娃资产、instruction 和尺寸目标顺序。 |
| `scripts/preview_scene.py` | 预览公共场景或指定 seed/layout 的任务场景。 |

目录保持为最小结构：

```text
src/scale_bench/
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
├── robots/piper.yml
├── scene/default.yml
└── tasks/sort_dolls_by_size.yml
```

## 配置流

```text
Robot YAML ──► RobotProfile ──► ArticulationCfg ─────┐
                            └─► robot CameraCfg ─────┤
                                                    │
Scene YAML ──► SceneConfig ─────────────────────────┼─► DualArmTabletopSceneCfg ─► 公共场景预览
                                                    │               │
Camera YAML ──► CameraProfile ──► CameraCfg ────────┘               │
                                                                    │
Task YAML ──► SortDollsBySize ──────────────────────────────────────┤
seed 或 layout JSON ────────────────────────────────────────────────┤
                                                                    ▼
                                                add_assets_to_scene(scene_cfg)
                                                                    │
                                                                    ▼
                                          task-extended InteractiveSceneCfg
                                                                    │
                                                                    ▼
                                                  InteractiveScene 任务预览
```

`DualArmTabletopSceneCfg` 是公共 `InteractiveSceneCfg`。Task 不创建任务专用 SceneCfg 子类，也不复制公共配置；它直接向调用方传入的配置实例增加任务资产字段。Isaac Lab 的 `InteractiveScene` 会正常解析这些动态字段。

## Task 接口

当前公共入口只有任务信息和一次资产装载操作：

```python
task.task_id
task.instruction
task.target_order_small_to_large  # SortDollsBySize 的具体任务目标

layout = task.add_assets_to_scene(
    scene_cfg,
    seed=42,
    export_layout_path="layouts/sort_dolls_by_size/42.json",
)
```

也可以从文件恢复：

```python
layout = task.add_assets_to_scene(
    scene_cfg,
    layout_path="layouts/sort_dolls_by_size/42.json",
)
```

`seed` 与 `layout_path` 互斥；两者都不传时使用 seed `0`。该方法返回实际加入场景的 `TaskLayout`。

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
- 启动 Isaac Sim 或创建 `SimulationContext`；
- 实现 Policy、规划器或成功判定；
- 管理 episode reset、step 或数据记录。

后续出现第二个任务时，优先复用 `TaskDefinition` 中已经稳定的刚体与布局行为。只有新任务确实需要不同的资产类型或放置规则时，才增加新的小接口；不提前创建 SceneCfg 继承树、registry 或空的运行时框架。
