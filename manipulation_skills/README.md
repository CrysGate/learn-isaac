# Piper 原子操作

这个目录提供一组与任务判定分离、按仿真步推进的操作模板：

| 原子操作 | 入口 | 主要阶段 |
| --- | --- | --- |
| 笛卡尔移动 | `move_to_pose()` | 插值移动、反馈收敛 |
| 夹爪开合 | `open_gripper()` / `close_gripper()` | 命令、开度验证 |
| 抓取 | `pick()` | 张开、预抓取、接近、闭合、抬升、验证 |
| 放置 | `place()` | 预放置、下降、释放、撤离、验证 |
| 直线插入 | `insert()` | 预插入、插入、可选释放、撤离、验证 |
| 持物旋转 | `rotate()` | 抬升、旋转、验证 |
| 回零 | `home()` | 关节空间插值、收敛验证 |

`SkillSequence` 可以按顺序组合这些操作，并且只在前一个操作成功后才创建
下一个操作。这一点很重要，因为 `place()`、`insert()` 和 `rotate()` 会在创建时
读取当前的“物体到 TCP”相对变换。

## 调用方式

每个入口都会返回一个有状态的 skill。调用方负责循环调用 `tick()`，再把产生的
完整动作送入环境：

```python
from manipulation_skills import PickConfig, pick

skill = pick(
    env,
    piper_profile,
    robot="right",
    object_name="doll_00002",
    config=PickConfig(lift_distance_m=0.10),
)

while not skill.done:
    step = skill.tick()
    observation, info = env.step(step.action)
```

`step.phase` 给出当前阶段；`step.done` 为真时，通过 `step.succeeded` 和
`step.message` 判断操作结果。

`PiperRuntime` 是机器人适配层。它通过 `env.scene["left_robot"]` 或
`env.scene["right_robot"]` 读取机械臂，通过 `env.scene[object_name]` 读取刚体，
并用 action manager 的命名切片构造双臂环境所需的完整动作。TCP 世界坐标目标先
转换到机器人基坐标，再由差分 IK 生成六个关节目标；未被选择的机械臂保持当前
关节位置。

没有传入 `GraspCandidate` 时，Piper 适配器会根据机器人基座和物体中心生成向下
倾斜的侧抓位姿。自定义 grasp 是物体坐标系下的 TCP 位姿和 TCP 接近轴。技能和
Isaac Lab 数学函数中的四元数使用标量在前的 `(w, x, y, z)` 顺序；
`GraspCandidate.orientation_object_xyzw` 是为兼容已有接口保留的旧字段名。

## 现有资产任务

`demo_atomic_tasks.py` 使用现有双 Piper 场景和五个套娃资产，包含五个独立任务：

- `move_home`：TCP 上移后回到配置中的 home 关节位置。
- `gripper`：闭合再张开，并检查最终开度。
- `pick`：抓取套娃并检查物体抬升距离。
- `pick_place`：抓取、移动到无碰撞邻近位置、释放，并检查最终物体位姿。
- `reorient`：抓取后绕世界 Z 轴旋转 45 度，并检查最终朝向。

默认 `--robot auto` 会为物体任务选择水平距离更近的机械臂。也可以显式传入
`--robot left` 或 `--robot right`；显式选择的机械臂不会因目标不可达而自动切换。
桌面放置任务使用 `PlaceConfig.release_clearance_m` 在目标上方释放，避免把桌面接触
反力误判为 IK 不收敛；自由落体稳定后仍按原始目标位姿验收。

运行全部任务：

```bash
.venv/bin/python manipulation_skills/demo_atomic_tasks.py \
    --tasks move_home gripper pick pick_place reorient \
    --robot auto \
    --headless \
    --viz none
```

只运行一个抓取并查看分阶段日志：

```bash
.venv/bin/python manipulation_skills/demo_piper_pick.py \
    --robot auto \
    --object doll_00002 \
    --headless
```

当前套娃资产没有孔、槽或配合件，因此 `insert()` 只在快速假运行时测试中验证
状态机和坐标变换，没有把普通放置伪装成物理插入测试。增加插孔资产后，应为任务
额外验证插入深度、横向偏差、朝向、接触力以及释放后的稳定时间。

## 任务模板

新增任务时保持三层职责分离：

1. 重置环境并选择物体、机械臂和目标位姿。
2. 用 lazy factory 组成 `SkillSequence`，逐步执行直到完成或超时。
3. 使用 `LiftGoal`、`ObjectPoseGoal` 或 `JointPositionGoal` 独立检查最终仿真状态，
   不把 skill 自己的成功标志当作任务成功。

快速测试不启动 Isaac Sim：

```bash
.venv/bin/python -m pytest -q tests/manipulation_skills/test_atomic_skills.py
```

GPU 集成测试入口是
`tests/isaaclab/test_atomic_skill_tasks.py::test_atomic_skills_on_existing_piper_and_doll_assets`。

需要记录单次抓取相机数据时：

```bash
.venv/bin/python manipulation_skills/demo_piper_pick.py \
    --robot auto \
    --dataset-name piper_pick \
    --record-cameras
```

该实现是分段笛卡尔轨迹加差分 IK，不包含全局避障、碰撞感知抓取规划和力控插入。
真实任务应在原子操作外增加可达性筛选、碰撞检查、失败重试和安全限位。
