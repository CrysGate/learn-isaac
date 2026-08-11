# ScaleBench 开发计划

## 当前状态

### 第一步：配置驱动的公共场景（已完成）

通过 YAML 组合场景、机器人和相机，生成可供多个任务共享的 `DualArmTabletopSceneCfg`。当前公共场景包含房间、地面、桌面、双臂、三台相机和灯光，并公开任务物体放置区域。

### 第二步：最小 Task 接口与首个任务（已完成）

当前 Task 层只负责配置期的任务信息、资产和初始布局：

- `RigidObjectTask` 提供 instruction、确定性 seed 布局、layout JSON 导入导出和放置校验；
- `SortDollsBySize` 声明五个套娃资产和从小到大的目标顺序；
- `resolve_layout()` 生成或读取并校验 `TaskLayout`，文件导出由调用方显式执行；
- `RigidObjectTaskBuilder` 接收已解析 layout，并由 environment builder 向公共 `InteractiveSceneCfg` 增加具名 `RigidObjectCfg` 字段；
- `scripts/preview_scene.py` 可以预览公共场景，也可以按 seed 或 layout 文件预览任务场景；
- 自动化测试覆盖 Task 契约、布局复现、边界与间距、资产注册和 layout 回放。

Task 层不拥有 reset、step、Action、Observation、Evaluator、Recorder 或 episode 生命周期。reset、step 与 Action/Observation/Recorder Manager 已由 Env 层实现；Evaluator 和 episode 调度仍属于后续阶段。

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
             │       RigidObjectTask
             │              │
             └──────┬───────┘
                    ▼
       task-extended InteractiveSceneCfg
                    │
                    ▼
           InteractiveScene preview
```

## 最近完成阶段

### 第三步：Env 配置与唯一仿真上下文（已完成）

本阶段已经完成：

- `configs/sim/default.yml` 只管理 device、physics timestep、gravity、render interval、渲染质量，以及一个与操作稳定性直接相关的 PhysX 覆盖项；
- `SimulationConfig` 在启动 Isaac Sim 前完成严格校验，由 adapter builder 构建全新的原生 `SimulationCfg`；
- 默认 preset 使用 120 Hz physics 和 30 Hz render；
- `configs/envs/default.yml` 与 `EnvironmentConfig` 管理 control decimation、reset 重渲染、纹理等待和环境 seed；
- `scale_bench.api.create_env()` 组合 RobotConfig、SceneConfig、Task layout 来源、SimulationConfig 和 manager 配置并返回正式环境；
- 内部 `ScaleBenchEnvCfg` 提供原生 `ManagerBasedEnvCfg`，并正式接入由 RobotConfig 和 CameraConfig 编译得到的 Action/Observation manager term；
- `ScaleBenchEnv` 是 `SimulationContext`、Scene、reset、step 和 close 的唯一所有者，并从实际运行对象生成 runtime IO metadata；
- 配置期为每个 `env_id` 准备固定的初始 layout，reset Event term 只负责恢复；
- builder 在启动前校验 physics、render、control 和 camera update period 同步；
- `scripts/preview_scene.py` 已改用正式环境入口，不再直接创建或推进 `SimulationContext`；
- `pytest -m integration` 在隔离子进程中覆盖双环境生命周期、渲染观测、runtime descriptor、布局 seed 分配和 partial reset；
- 其余材质、Fabric、日志、solver iteration 和 GPU buffer 参数保持 Isaac Lab 当前版本的原生默认值，不在项目中复制整套后端配置；
- sim YAML 不管理 `num_envs`、环境间距或 task layout，这些仍由 Scene/Task 各自拥有。

配置流：

```text
Scene / Task / Robot ─────────────┐
Sim YAML ──► SimulationConfig ─────┤
Env YAML ──► EnvironmentConfig ───┼─► create_env() ─► ScaleBenchEnv
manager 配置 ─────────────────────┘
```

实现结构：

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

## 运行时阶段

### 第四步：统一的 seed 管理（已完成）

使用 seed 创建任务环境时，配置期按 `base_seed + env_id` 为每个环境一次性生成确定性的独立 layout；也可以显式传入一个 layout 广播给所有环境，或传入 `num_envs` 个 layout 按 `env_id` 分配。Event Manager term 在全量或局部 reset 时只通过 `write_root_pose_to_sim_index()` 恢复对应环境的初始对象位姿，不再采样或推进 seed。

### 第五步：Action Profile（已完成第一版）

Action 同时依赖机器人语义和 policy 控制方式。当前已经接入 joint position；后续计划支持：

- EE pose；
- EE delta pose。

当前 Action Manager 按 `left_arm | left_gripper | right_arm | right_gripper` 组织动态维度的绝对关节位置 term，并保留 `RobotConfig` 中的关节顺序。控制关节、末端 link、TCP 和夹爪语义来自 `RobotConfig`，不在 Task 或 policy 适配代码中按机器人名称分支。

### 第六步：Observation 数据边界（已完成第一版）

- Policy Observation 已通过不拼接的具名 term 公开左右机器人状态及已配置相机的原始 RGB-D；
- 任务物体和评测真值不进入 policy group；
- Evaluator 将通过独立接口读取 Scene 仿真真值，该接口仍待后续实现。

### 第七步：Task 运行时逻辑与 Evaluator

在 Env 边界稳定后，再为任务增加：

- reset/randomization 约定；
- 目标状态；
- 成功、失败和 timeout 判定；
- 指标计算；
- Task 与 Evaluator 的绑定方式。

这些能力不应塞入当前只负责配置期资产和布局的 `RigidObjectTask`，除非后续接口设计证明它们属于同一个稳定抽象。

### 第八步：Episode Runner 与 Recorder（Recorder 第一版已完成）

Runner 统一执行：

```text
reset → observe → policy.act → apply_action → step → evaluate → record
```

并负责 episode 生命周期、timeout、结果汇总和批量运行。Recorder 第一版已通过原生 Recorder Manager 接入；Runner 尚未实现。Recorder 只记录已定义的数据边界，不反向影响 Policy 或 Evaluator。

## 长期架构原则

- **Task**：声明任务实体、初始布局、instruction 和目标语义；运行时职责在接口稳定后再扩展。
- **Env**：持有仿真、场景、传感器及 action/observation/event managers。
- **Policy**：读取公开 observation 和 instruction，输出 action。
- **Evaluator**：读取允许的仿真真值，计算成功、失败和指标，不影响 Policy。
- **Runner**：管理 episode 调度、timeout、汇总和记录。
