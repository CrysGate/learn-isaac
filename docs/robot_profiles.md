# RobotConfig

`RobotConfig` 是纯 Python 机器人配置模型，负责机器人资产引用、初始状态、运动学语义、执行器、夹爪和相机安装约定。它不读取文件，也不创建 Isaac Lab 对象。

```text
configs/robots/*.yml
        │
        ▼
load_config() ──► 路径解析 / Pydantic 校验 ──► RobotConfig
                                                    │
                                                    ▼
                                      过渡期 Isaac Lab builder
```

当前参考配置是 [`configs/robots/piper.yml`](../configs/robots/piper.yml)，纯模型位于 [`src/scale_bench/config/models/robot.py`](../src/scale_bench/config/models/robot.py)。旧的 [`RobotProfile`](../src/scale_bench/robots/robot_profile.py) 暂时保留为兼容门面。

## 加载与使用

只加载和校验 YAML 不需要启动 Isaac Sim：

```python
from scale_bench.config.loader import load_config
from scale_bench.config.models.robot import RobotConfig

config = load_config("configs/robots/piper.yml", RobotConfig, asset_root=".")
print(config.name)
```

项目使用 `src` 布局但不安装自身 package，因此独立脚本需要设置 `PYTHONPATH=src`：

```bash
PYTHONPATH=src uv run python -c \
  'from scale_bench.config.loader import load_config; from scale_bench.config.models.robot import RobotConfig; print(load_config("configs/robots/piper.yml", RobotConfig, asset_root=".").name)'
```

Isaac Lab 构造逻辑仍位于过渡期 builder 中，调用前必须先通过 `AppLauncher` 初始化 Isaac Sim。仓库中的 [`scripts/preview_scene.py`](../scripts/preview_scene.py) 展示了完整启动顺序。兼容门面仍支持旧调用：

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
| `fixed_base` | 否 | 是否固定根节点，默认 `true`。 |
| `disable_gravity` | 否 | 是否关闭机器人刚体重力，默认 `false`。 |
| `self_collisions` | 否 | 是否启用自碰撞，默认 `false`。 |
| `initial_joint_positions` | 是 | 所有已声明机械臂和夹爪关节的初始位置。 |
| `kinematics` | 是 | 机械臂关节顺序、基座、末端和 TCP 语义。 |
| `actuators` | 是 | 一个或多个 implicit actuator 组。 |
| `gripper` | 是 | 平行夹爪状态和命令语义。 |
| `camera` | 否 | 相机参数 profile 引用及相对机器人资产的安装位姿。 |

`profile_path` 等配置引用相对于当前机器人 YAML 解析。资产引用在传入 `asset_root` 时相对于该根目录解析，否则相对于当前 YAML 解析。绝对路径原样保留；当前只支持本地路径，并检查资产存在性。

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

### `camera`

机器人相机的光学参数继续复用独立 `CameraProfile`，robot profile 只保存安装关系：

```yaml
camera:
  profile_path: ../cameras/d435.yml
  parent_prim_path: link6/camera
  sensor_prim_name: D435Sensor
  position_m: [0.0, 0.0, 0.0]
  orientation_xyzw: [1.0, 0.0, 0.0, 0.0]
  convention: opengl
```

- `parent_prim_path` 是相对于机器人根 prim 的路径，必须指向机器人 USD 中已存在的安装 frame。
- `sensor_prim_name` 是在该 frame 下创建的 USD Camera prim 名称。
- `position_m` 和 `orientation_xyzw` 是相机相对于父 frame 的局部位姿；四元数必须归一化。
- `convention` 只能是 `opengl`、`ros` 或 `world`。
- Piper 的 `link6/camera` 已由资产放置在腕部实际相机安装位，局部绕 X 轴旋转 180 度后符合 USD/OpenGL 相机轴约定。

`load_config()` 会解析 `profile_path`，引用的相机文件由需要它的加载或构建入口按 `CameraConfig` 校验。在 Isaac Lab 配置阶段，兼容门面仍可按机器人根路径构建传感器：

```python
camera_cfg = profile.build_camera_cfg(
    robot_prim_path="{ENV_REGEX_NS}/Robot",
)
```

未配置 `camera` 时该方法返回 `None`。

## 校验约定

`RobotConfig` 和 `load_config()` 会拒绝以下配置：

- 任意模型中出现未声明字段；
- 空名称、非有限数值或负执行器参数；
- 机械臂、夹爪或单个 actuator 内部存在重复关节名；
- 机械臂关节与夹爪关节重叠；
- `initial_joint_positions` 没有恰好覆盖机械臂和夹爪关节的并集；
- 不同 actuator 组控制同一关节，或 actuator 引用了未声明关节；
- 机械臂关节或夹爪命令关节没有 actuator；
- 夹爪命令关节不是夹爪状态关节的子集；
- `open_positions` 或 `closed_positions` 的键与命令关节不完全一致；
- 两个 finger body 相同，或最大开口不大于最小开口；
- TCP 或相机四元数不是单位四元数；
- 相机父 prim 路径不是合法相对路径，传感器 prim 名不合法，或坐标约定不受支持；
- 本地 USD 或 URDF 文件不存在。

YAML/JSON 读取、schema 校验和本地资产错误统一包装为 `ConfigLoadError`，信息包含源文件、字段位置和解析后的路径。

## 生成的 `ArticulationCfg`

转换过程会设置：

- `UsdFileCfg` 的 USD 路径；机器人始终使用资产原始尺寸；
- `fix_root_link`、`enabled_self_collisions` 和 `disable_gravity`；
- 按“机械臂关节、夹爪关节”顺序组织的初始关节位置；
- 每个 YAML actuator 对应的 `ImplicitActuatorCfg`；
- 调用方提供的可选 `prim_path`。

URDF、TCP、末端 body 和 finger body 当前不会直接写入 `ArticulationCfg`；它们是后续控制器、观测和评测所需的机器人语义。

## 生成的 `CameraCfg`

`build_camera_cfg()` 将机器人根路径、`parent_prim_path` 和 `sensor_prim_name` 拼成完整 prim path，再复用 camera profile 的分辨率、输出类型、针孔内参和裁剪范围，最后写入机器人 YAML 中的局部位姿。场景模板分别传入 `LeftRobot` 与 `RightRobot` 根路径，因此同一份 Piper profile 会生成两台独立、随各自腕部运动的相机。

## 新增机器人

1. 复制 [`configs/robots/piper.yml`](../configs/robots/piper.yml)。
2. 从 USD/URDF 核对准确的 joint、body 和 frame 名称。
3. 修改资产路径、初始状态、运动学、actuator、夹爪与可选相机安装字段。
4. 运行不启动仿真器的 profile 校验：

   ```bash
   PYTHONPATH=src uv run python -c \
     'from scale_bench.config.loader import load_config; from scale_bench.config.models.robot import RobotConfig; print(load_config("configs/robots/my_robot.yml", RobotConfig, asset_root=".").name)'
   ```

5. 在实际场景中执行无界面冒烟验证：

   ```bash
   uv run python scripts/preview_scene.py \
     --left-robot-config configs/robots/my_robot.yml \
     --right-robot-config configs/robots/piper.yml \
     --viz none \
     --max-steps 2
   ```

运行 `uv run pytest tests/config` 可执行纯配置模型、路径和依赖边界测试。新增机器人时还应验证相机配置引用、USD 挂载 frame，以及左右场景相机的 prim path；实际渲染仍应通过场景冒烟测试验证。

## 当前支持边界

- 机器人通过 `UsdFileCfg` 生成，URDF 仅作为可选元数据和存在性检查。
- actuator 类型固定为 `ImplicitActuatorCfg`。
- 末端执行器类型固定为 parallel-jaw gripper。
- 不应在场景或任务中增加基于机器人名称的特殊分支。出现新的 actuator 或末端类型时，应扩展 profile schema 和转换逻辑。

场景侧用法见 [`scene_template.md`](scene_template.md)。
