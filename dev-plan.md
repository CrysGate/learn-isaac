# ScaleBench 开发计划

## 当前状态

### 第一步：配置驱动的公共场景（已完成）

通过 YAML 组合场景、机器人和相机，生成可供多个任务共享的 `DualArmTabletopSceneCfg`。当前公共场景包含房间、地面、桌面、双臂、三台相机和灯光，并公开任务物体放置区域。

### 第二步：最小 Task 接口与首个任务（已完成）

当前 Task 层只负责配置期的任务信息、资产和初始布局：

- `TaskDefinition` 提供 instruction、确定性 seed 布局、layout JSON 导入导出、放置校验和刚体配置构建；
- `SortDollsBySize` 声明五个套娃资产和从小到大的目标顺序；
- Task 接收公共 `InteractiveSceneCfg` 实例，直接增加具名 `RigidObjectCfg` 字段；
- `add_assets_to_scene()` 返回实际使用的 `TaskLayout`，不会创建或返回 `TaskSceneCfg` 子类；
- `scripts/preview_scene.py` 可以预览公共场景，也可以按 seed 或 layout 文件预览任务场景；
- 自动化测试覆盖公共 Task 契约、布局复现、边界与间距校验、资产注册和 layout 回放。

当前 Task 层不实现 reset、step、Observation、Action、Evaluator、Recorder 或 episode 生命周期。这些能力将在环境运行时边界明确后逐步加入。

```text
Robot / Camera / Scene YAML
             │
             ▼
  DualArmTabletopSceneCfg
             │
             ├──────────────┐
             │              │
             │     Task YAML + seed/layout
             │              │
             │              ▼
             │       TaskDefinition
             │              │
             └──────┬───────┘
                    ▼
       task-extended InteractiveSceneCfg
                    │
                    ▼
           InteractiveScene preview
```

## 当前进行阶段

### 第三步：Env 配置与唯一仿真上下文（进行中）

当前已经完成配置驱动的仿真参数边界：

- `configs/sim/default.yml` 只管理 device、physics timestep、gravity、render interval、渲染质量，以及一个与操作稳定性直接相关的 PhysX 覆盖项；
- `SimConfig` 在启动 Isaac Sim 前完成严格校验，并构建全新的原生 `SimulationCfg`；
- 默认 preset 使用 120 Hz physics 和 30 Hz render；
- `scripts/preview_scene.py` 通过 `--sim-config` 使用该配置，显式 `--device` 和 `--rendering_mode` 仍可作为机器相关的命令行覆盖；
- 其余材质、Fabric、日志、solver iteration 和 GPU buffer 参数保持 Isaac Lab 当前版本的原生默认值，不在项目中复制整套后端配置；
- sim YAML 不管理 `num_envs`、环境间距或 task layout，这些仍由 Scene/Task 各自拥有。

配置流：

```text
Sim YAML ──► SimConfig ──► Isaac Lab SimulationCfg
                                  │
                                  ├─► 当前：scene preview
                                  └─► 后续：EnvCfg.sim
```

本阶段接下来需要：

- 定义组合公共场景、Task、SimulationCfg 和运行时 manager 的 EnvCfg builder；
- 由 `ManagerBasedEnv` 或等价环境入口创建唯一的 `SimulationContext`；
- 明确 reset、step 和场景访问的所有权，避免 Task、Runner 和脚本重复管理仿真生命周期。

目标结构：

```text
env
├── env.sim
│   └── SimulationContext
├── env.scene
│   └── InteractiveScene
├── env.action_manager
│   └── ActionManager
├── env.observation_manager
│   └── ObservationManager
├── env.event_manager
│   └── EventManager
└── env.recorder_manager
    └── RecorderManager
```

## 后续阶段

### 第四步：统一的seed管理，当前修改了num_envs之后每个场景的layout都是完全一样的，需要实现多个num_envs有不同的layout布局，即先正常克隆场景，在sim.reset之后为每个env单独生成layout,再通过write_root_pose_to_sim_index()写入各物体的位置

### 第五步：Action Profile

Action 同时依赖机器人语义和 policy 控制方式。计划支持：

- joint position；
- EE pose；
- EE delta pose。

控制关节、末端 link、TCP 和夹爪语义应来自 `RobotProfile`，不在 Task 或 policy 适配代码中按机器人名称分支。

### 第六步：Observation 数据边界

- Policy Observation 只公开 policy 被允许使用的传感器和状态；
- Evaluator 可以读取 Scene 仿真真值；
- 两者使用明确分层的数据接口，避免评测真值泄漏给 policy。

### 第七步：Task 运行时逻辑与 Evaluator

在 Env 边界稳定后，再为任务增加：

- reset/randomization 约定；
- 目标状态；
- 成功、失败和 timeout 判定；
- 指标计算；
- Task 与 Evaluator 的绑定方式。

这些能力不应塞入当前只负责配置期资产和布局的 `TaskDefinition`，除非后续接口设计证明它们属于同一个稳定抽象。

### 第八步：Episode Runner 与 Recorder

Runner 统一执行：

```text
reset → observe → policy.act → apply_action → step → evaluate → record
```

并负责 episode 生命周期、timeout、结果汇总和批量运行。Recorder 只记录已定义的数据边界，不反向影响 Policy 或 Evaluator。

## 长期架构原则

- **Task**：声明任务实体、初始布局、instruction 和目标语义；运行时职责在接口稳定后再扩展。
- **Env**：持有仿真、场景、传感器及 action/observation/event managers。
- **Policy**：读取公开 observation 和 instruction，输出 action。
- **Evaluator**：读取允许的仿真真值，计算成功、失败和指标，不影响 Policy。
- **Runner**：管理 episode 调度、timeout、汇总和记录。
