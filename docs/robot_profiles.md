# RobotProfile

`RobotProfile` 是机器人 YAML 与 Isaac Lab `ArticulationCfg` 之间的类型化边界。它负责保存机器人资产、初始状态、运动学语义、执行器和夹爪约定，并在创建仿真对象前集中完成校验。

```text
configs/robots/*.yml
        │
        ▼
RobotProfile.load() ──► Pydantic 校验 ──► RobotProfile
                                                │
                                                ▼
                              build_articulation_cfg()
                                                │
                                                ▼
                                  Isaac Lab ArticulationCfg
```

当前参考配置是 [`configs/robots/piper.yml`](../configs/robots/piper.yml)，实现位于 [`src/scale_bench/robots/robot_profile.py`](../src/scale_bench/robots/robot_profile.py)。

## 加载与使用

只加载和校验 YAML 不需要启动 Isaac Sim：

```python
from scale_bench.robots import RobotProfile

profile = RobotProfile.load("configs/robots/piper.yml")
print(profile.name)
```

项目使用 `src` 布局但不安装自身 package，因此独立脚本需要设置 `PYTHONPATH=src`：

```bash
PYTHONPATH=src uv run python -c \
  'from scale_bench.robots import RobotProfile; p = RobotProfile.load("configs/robots/piper.yml"); print(p.name)'
```

`build_articulation_cfg()` 会导入 Isaac Lab 仿真模块，所以调用前必须先通过 `AppLauncher` 初始化 Isaac Sim。仓库中的 [`scripts/preview_scene.py`](../scripts/preview_scene.py) 展示了完整的启动顺序。

在已经启动的 Isaac Lab 进程中：

```python
from scale_bench.robots import RobotProfile

profile = RobotProfile.load("configs/robots/piper.yml")
robot_cfg = profile.build_articulation_cfg(
    prim_path="{ENV_REGEX_NS}/Robot",
)
```

每次调用都会创建一份新的 `ArticulationCfg`，因此同一个 profile 可以安全地用于左右机器人或多个场景。`prim_path` 可以省略，由场景模板在挂载机器人时设置。

## YAML 结构

### 顶层字段

| 字段 | 必需 | 作用 |
|---|---:|---|
| `name` | 是 | 非空机器人标识。当前只作为数据使用，不参与类型分支。 |
| `usd_path` | 是 | 用于生成 articulation 的 USD 路径。 |
| `urdf_path` | 否 | 可选 URDF 参考路径；会检查存在性，但当前不用于生成 articulation。 |
| `scale` | 否 | 三轴正数缩放；缺省时使用 USD 原始尺度。 |
| `fixed_base` | 否 | 是否固定根节点，默认 `true`。 |
| `disable_gravity` | 否 | 是否关闭机器人刚体重力，默认 `false`。 |
| `self_collisions` | 否 | 是否启用自碰撞，默认 `false`。 |
| `initial_joint_positions` | 是 | 所有已声明机械臂和夹爪关节的初始位置。 |
| `kinematics` | 是 | 机械臂关节顺序、基座、末端和 TCP 语义。 |
| `actuators` | 是 | 一个或多个 implicit actuator 组。 |
| `gripper` | 是 | 平行夹爪状态和命令语义。 |

所有相对文件路径都从仓库根目录解析。包含 `://` 的路径会作为远端或 Omniverse URI 原样保留，不进行本地文件存在性检查。

### `kinematics`

```yaml
kinematics:
  base_body: base_link
  arm_joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  ee_body: link6
  tcp:
    parent_frame: gripper_center
    position_m: [-0.04, 0.0, 0.0]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

- `arm_joint_names` 的顺序是 benchmark 采用的机械臂关节顺序。
- `tcp.position_m` 使用米，`orientation_xyzw` 使用 `xyzw` 顺序且必须是单位四元数。
- `base_body`、`ee_body` 和 TCP 当前保存为下游控制与评测语义；`build_articulation_cfg()` 不会额外创建 TCP frame。

### `actuators`

每个 actuator 组都会转换为 `ImplicitActuatorCfg`：

```yaml
actuators:
  arm:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    stiffness: null
    damping: null
    effort_limit_sim: null
    velocity_limit_sim: null
```

`stiffness`、`damping`、`effort_limit_sim` 和 `velocity_limit_sim` 支持：

- 非负标量；
- 按关节名设置的非负数值映射；
- `null`，将 `None` 传给 Isaac Lab，以保留 USD 或 Isaac Lab 的已有设置。

### `gripper`

当前只支持平行夹爪：

```yaml
gripper:
  joint_names: [gripper_joint, joint8]
  command_joint_names: [gripper_joint]
  finger_body_names: [link7, link8]
  min_aperture_m: 0.0
  max_aperture_m: 0.1
  closed_positions:
    gripper_joint: 0.0
  open_positions:
    gripper_joint: 0.05
```

`joint_names` 描述完整夹爪状态，`command_joint_names` 只包含需要直接下发命令的关节，因此可以表达 mimic 或被动关节。开合位置映射必须恰好覆盖所有命令关节。

## 校验约定

`RobotProfile.load()` 会拒绝以下配置：

- 任意模型中出现未声明字段；
- 空名称、非有限数值、负执行器参数或非正缩放；
- 机械臂、夹爪或单个 actuator 内部存在重复关节名；
- 机械臂关节与夹爪关节重叠；
- `initial_joint_positions` 没有恰好覆盖机械臂和夹爪关节的并集；
- 不同 actuator 组控制同一关节，或 actuator 引用了未声明关节；
- 机械臂关节或夹爪命令关节没有 actuator；
- 夹爪命令关节不是夹爪状态关节的子集；
- `open_positions` 或 `closed_positions` 的键与命令关节不完全一致；
- 两个 finger body 相同，或最大开口不大于最小开口；
- TCP 四元数不是单位四元数；
- 本地 USD 或 URDF 文件不存在。

YAML 读取和 schema 校验错误会包装为带 profile 路径的 `ValueError`；本地资产检查错误会在 `ValueError` 中给出解析后的资产路径。

## 生成的 `ArticulationCfg`

转换过程会设置：

- `UsdFileCfg` 的 USD 路径和可选缩放；
- `fix_root_link`、`enabled_self_collisions` 和 `disable_gravity`；
- 按“机械臂关节、夹爪关节”顺序组织的初始关节位置；
- 每个 YAML actuator 对应的 `ImplicitActuatorCfg`；
- 调用方提供的可选 `prim_path`。

URDF、TCP、末端 body 和 finger body 当前不会直接写入 `ArticulationCfg`；它们是后续控制器、观测和评测所需的机器人语义。

## 新增机器人

1. 复制 [`configs/robots/piper.yml`](../configs/robots/piper.yml)。
2. 从 USD/URDF 核对准确的 joint、body 和 frame 名称。
3. 修改资产路径、初始状态、运动学、actuator 与夹爪字段。
4. 运行不启动仿真器的 profile 校验：

   ```bash
   PYTHONPATH=src uv run python -c \
     'from scale_bench.robots import RobotProfile; p = RobotProfile.load("configs/robots/my_robot.yml"); print(p.name)'
   ```

5. 在实际场景中执行无界面冒烟验证：

   ```bash
   uv run python scripts/preview_scene.py \
     --left-robot-config configs/robots/my_robot.yml \
     --right-robot-config configs/robots/piper.yml \
     --viz none \
     --max-steps 2
   ```

目前仓库没有独立的 `RobotProfile` pytest 测试文件，上述 schema 加载和场景冒烟测试是现有验证入口。

## 当前支持边界

- 机器人通过 `UsdFileCfg` 生成，URDF 仅作为可选元数据和存在性检查。
- actuator 类型固定为 `ImplicitActuatorCfg`。
- 末端执行器类型固定为 parallel-jaw gripper。
- 不应在场景或任务中增加基于机器人名称的特殊分支。出现新的 actuator 或末端类型时，应扩展 profile schema 和转换逻辑。

场景侧用法见 [`scene_template.md`](scene_template.md)。
