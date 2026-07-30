# 双 Piper 三相机套娃排序仿真与专家数据采集

你是负责在当前仓库中直接工作的机器人仿真工程师。请在
`/home/ogyco/learn-isaac` 中设计、实现、运行并验证一套双 Piper
机械臂、三台 Intel RealSense D435 RGB-D 相机的仿真与 cuRobo
专家数据采集程序。

不要只给建议、伪代码或未经运行的代码。持续推进、检查运行结果并修复问题，
直到下面定义的完整验收条件成立。

## 1. 最终目标

在 Isaac Sim/Isaac Lab 中搭建指定场景，每个 episode 在桌面上随机放置
5 个外观相同、尺寸不同的套娃。两个 Piper 机械臂都要参与操作，使用
cuRobo 生成无碰撞专家轨迹，把套娃从小到大排列成一条经过桌面中央、
平行于桌子宽度方向的直线，随后两个机械臂自动归位。

执行期间同步采集机器人状态、实际下发动作、三路 RGB、三路深度图、
物体状态和完整场景元数据。数据只有在恢复其初始场景并直接重放记录动作后
仍能成功完成任务时，才允许标记为成功数据。

实现优先级严格为：

1. 正确。
2. 简单。
3. 干净、易读。
4. 在满足前三项之后再考虑性能和扩展性。

## 2. 不可违反的约束

### 2.1 环境

- 当前 `uv` 环境已经安装并配置好 Isaac Sim 5.1.0、Isaac Lab 和 cuRobo。
- 直接使用仓库现有 `.venv`、`pyproject.toml`、`uv.lock` 和本地 editable
  依赖。
- 禁止重新安装、升级或降级 Isaac Sim、Isaac Lab、cuRobo、PyTorch
  或 CUDA 相关依赖。
- 禁止修改 `pyproject.toml` 和 `uv.lock`。
- 必须以仓库实际安装版本的 API 为准。

### 2.2 单 Python 文件

所有实现只能放在一个 Python 文件中：

`/home/ogyco/learn-isaac/dual_piper_sort.py`

禁止为了实现任务再创建其他 `.py`、YAML、JSON、配置模块、测试模块、
包目录或生成后的机器人配置文件。cuRobo 所需的 Piper 配置、碰撞球、
关节映射、场景参数和相机参数均放在这个 Python 文件中，或者直接读取
已经存在的资产/URDF。

以下文件不违反“单 Python 文件”约束：

- 本 `goal.md`。
- 过程日志 `dual-piper-dev-log.md`。
- 程序生成的 HDF5 episode、运行日志、预览图片和诊断产物。

不要过度工程化。优先使用少量普通函数和简单数据结构，不要建立插件、
任务注册系统、大型继承层次、配置框架或无必要的抽象。如果 Isaac Sim
直接 API 更容易正确跑通，就不要为了形式强行套用复杂的 Isaac Lab
ManagerBased 环境。

### 2.3 保护现有工作区

- 开始每个小任务前运行 `git status --short`。
- 当前工作区可能已有用户创建或未跟踪的资产，不能删除、移动、格式化或
  顺手提交与当前小任务无关的文件。
- 禁止使用 `git reset --hard`、`git checkout --` 等破坏性命令。
- 每次只暂存本次确实修改的文件，禁止使用不加选择的 `git add .`。
- 不修改原始 USD、USDZ、URDF、纹理和 MDL 资产。

## 3. 开发日志和 Git 提交

每完成一个可独立验证的小任务，必须立即按以下顺序处理：

1. 运行与该小任务对应的最小验证命令。
2. 在 `/home/ogyco/learn-isaac/dual-piper-dev-log.md` 末尾追加一条记录。
3. 检查 `git diff` 和 `git status --short`。
4. 只暂存该任务涉及的文件。
5. 立即创建一个单一目的、可读的 Git commit。

日志不能等到最后一起补写，也不要重写之前的记录。每条记录至少包含：

```markdown
## YYYY-MM-DD HH:MM — 小任务名称

- 目标：
- 完成内容：
- 修改文件：
- 运行命令：
- 验证结果：
- 问题与处理：
- 参考资料：没有则写“无”
- Commit message：
```

只有经过最小验证的阶段性成果才能作为“完成的小任务”提交。失败的尝试、
重要报错、被否定的方案和最终解决方法也要及时记录，避免重复踩坑。

推荐按以下小任务推进，每一项完成后分别记录并提交：

1. 固化场景常量并完成资产/prim/关节检查。
2. 只搭建房间、HDR、地面和桌子的场景 smoke test。
3. 加载双 Piper，验证关节、夹爪、安装位姿和自动归位。
4. 创建三台 RGB-D 相机并验证同步 RGB/深度输出。
5. 随机生成五个套娃并验证无重叠、稳定和可复现。
6. 在一个 Piper 上跑通 cuRobo 到达、抓取、搬运和放置。
7. 跑通两个机械臂顺序协作和完整五物体排序。
8. 加入同步数据记录并生成一个原始成功 episode。
9. 加入从磁盘恢复场景和动作重放验证。
10. 完成 headed 与 headless 集成验收。

如果实际依赖关系要求调整小任务边界，可以调整，但仍须保持“小步验证、
写日志、立即 commit”的节奏。

## 4. 权威资料要求

遇到 API、坐标系、相机、PhysX、USD、Isaac Lab 或 cuRobo 问题时：

1. 先检查当前仓库、本地安装包源码、随当前版本提供的示例和文档。
2. 需要联网查询时，只优先使用 NVIDIA Isaac Sim 官方文档、Isaac Lab
   官方文档/仓库、cuRobo 官方文档和 NVLabs/cuRobo 官方仓库等一手来源。
3. 不使用博客、论坛转载、CSDN 或未经核实的答案作为关键实现依据。
4. 官方网页与本地安装版本有差异时，以本地 Isaac Sim 5.1.0 和当前
   editable 源码实际 API 为准。
5. 把真正影响实现决策的文档名称、链接或本地源码路径写入开发日志。

不能以“可能是版本问题”结束任务；要通过最小可复现检查确定实际 API
和错误原因。

## 5. 固定资产

所有资产路径均以仓库根目录
`/home/ogyco/learn-isaac` 为基准，也要在程序顶部保留清晰的绝对路径
解析逻辑。

### 5.1 Piper

USD：

`Assets/Robots/piper/Piper.usd`

URDF：

`Assets/Robots/piper/piper_description/urdf/piper.urdf`

两台机械臂必须分别引用同一个 `Piper.usd`，使用不同 prim：

- `/World/Robots/LeftPiper`
- `/World/Robots/RightPiper`

现有 `Assets/Robots/piper/robot_config.yml` 可以作为关节和相机安装信息
的参考，但不能假设它已经是完整可用的 cuRobo MotionGen 配置。

### 5.2 房间和 HDR

房间：

`Assets/Room/Simple_Room_nolight/simple_room_nolight.usd`

HDR：

`Assets/Background/brown_photostudio_02_4k.hdr`

房间作为静态背景几何加载到独立 prim。检查引用后的 stage，显式禁用或
移除房间资产中任何残留的 authored light，避免其与统一环境光重复。

使用 HDR 创建 DomeLight，作为场景环境背景和主要环境照明。DomeLight
的强度、旋转、纹理路径和可见性策略集中定义为常量，并写入 episode
元数据。不要加载额外的默认灯光。

### 5.3 顶部相机支架

`Assets/Object/RoboDojo/Geometry/camera_stand/00000/object.usd`

它是支架几何资产，不等于相机传感器。作为静态物体放置在桌子周围，使
顶部相机能够俯视完整桌面，同时不与双臂工作空间发生碰撞。

支架最终位姿必须放在程序顶部常量中。由于当前尚未指定支架位姿，应先
读取该 USD 的坐标系和包围盒，在 headed 模式下进行最小调整并记录最终值，
不能把试调值散落在代码中。

### 5.4 套娃

根目录：

`Assets/Object/RoboDojo/Rigid/matryoshka_dolls`

只使用以下五个资产：

| ID | 资产 | 高度 |
|---|---|---:|
| 0 | `00000/object.usdz` | `0.13 m` |
| 1 | `00001/object.usdz` | `0.11 m` |
| 2 | `00002/object.usdz` | `0.09 m` |
| 3 | `00003/object.usdz` | `0.07 m` |
| 4 | `00004/object.usdz` | `0.05 m` |

这五个目录的元数据 UUID 相同，代表同一外观对象的五种尺寸。不要从全部
25 个目录中随机混选不同 UUID 的物体，也不要再对这五个资产施加随机缩放。

按元数据配置合理的质量和摩擦；如果 USDZ 已有物理属性，先检查再决定是否
覆盖。碰撞近似必须足以稳定抓取，不能只添加视觉模型。

## 6. 世界坐标和固定场景参数

统一使用：

- 米制单位。
- Z-up 世界坐标系。
- 四元数顺序为 `[qw, qx, qy, qz]`。

### 6.1 桌子

桌子使用带静态碰撞体的自定义 Cube。

| 参数 | 固定值 |
|---|---:|
| 中心位置 | `[0.0, -0.05, 0.74]` |
| 姿态 `[qw,qx,qy,qz]` | `[1.0, 0.0, 0.0, 0.0]` |
| 尺寸 `[x,y,z]` | `[1.4, 1.1, 0.05]` |
| X 范围 | `[-0.70, 0.70]` |
| Y 范围 | `[-0.60, 0.50]` |
| 桌面高度 | `0.765 m` |

桌子局部 X 是长度方向，局部 Y 是宽度方向。不要在实现中交换 X/Y。

桌面材质：

`Assets/Material/material_0122/Mahogany_Planks.mdl`

材质入口：

`Mahogany_Planks`

### 6.2 双臂安装

| 项目 | 位置 `[x,y,z]` | 姿态 `[qw,qx,qy,qz]` |
|---|---:|---:|
| 左臂基座 | `[-0.3, -0.45, 0.765]` | `[0.707,0,0,0.707]` |
| 右臂基座 | `[0.3, -0.45, 0.765]` | `[0.707,0,0,0.707]` |

要求：

- 两个基座沿桌子 X 方向左右分布，间距 `0.6 m`。
- 两个基座都位于桌面高度 `z=0.765`。
- 两个基座都在桌子负 Y 一侧，`y=-0.45`。
- 两台机械臂姿态相同，不做镜像旋转。
- 给定四元数表示绕世界 Z 轴约 `+90°`；使用前可以做数值归一化，
  但不能改变其代表的姿态。
- 配置意图是让两台机械臂从负 Y 侧朝桌面内部的 `+Y` 方向工作。
- 必须通过末端位姿或简单 cuRobo 可达性测试验证这一方向，不能只凭视觉猜测。

### 6.3 地面

地面使用带静态碰撞体的自定义 Cube：

- 厚度固定为 `0.1 m`。
- 地面上表面为世界 `z=0`，因此中心 `z=-0.05`。
- X/Y 尺寸应覆盖整个房间工作区域，默认使用 `[6.0, 6.0, 0.1]`；
  若房间包围盒证明需要调整，只修改程序顶部常量并在日志中说明。
- 不要再添加 default ground plane。

地面材质：

`Assets/Material/material_0564/Wood_Tiles_Fineline.mdl`

默认材质入口：

`Wood_Tiles_Fineline`

## 7. 三台 Intel RealSense D435 相机

创建三台逻辑上模拟 D435 的 RGB-D 相机：

- `left_wrist_camera`
- `right_wrist_camera`
- `overhead_camera`

Piper USD 中的 `link6/camera` 是腕部相机安装 Xform，不是真正的 USD
Camera。分别在左右机械臂对应安装位下创建实际 Camera prim，并验证
其光轴确实看到夹爪前方的抓取区域。

顶部 Camera 创建在相机支架坐标系下，朝向桌面中心
`[0.0, -0.05, 0.765]`，画面需要覆盖整个物体随机区和最终排列区域。

为保持第一版简单，三台相机都使用对齐的 pinhole RGB-D 近似模型，不宣称
完整模拟真实 D435 的双目基线和所有噪声。默认参数：

- 分辨率 `640 × 480`。
- 输出频率 `30 Hz`。
- RGB 保存为 `uint8`，通道顺序在元数据中明确。
- 深度使用 `distance_to_image_plane` 或当前版本等价的 Z-depth，
  保存为 `float32` 米制数据。
- 近远裁剪范围应覆盖腕部近距离抓取和顶部完整桌面。
- RGB、深度、状态和动作使用明确且可对应的时间戳。
- 保存每台相机的内参。
- 顶部相机保存静态外参；两个腕部相机保存逐帧世界外参。
- 明确记录无效深度值的表示方式。

相机模型参数、安装变换和朝向全部集中在文件顶部。先用 headed 模式保存
一组预览图，确认画面方向、覆盖范围和深度数值正确，再进入数据采集。

## 8. Episode 初始化与随机化

每个 episode 使用显式 seed，并设置 NumPy、PyTorch、cuRobo/相关 GPU
随机源和可用的仿真随机源。保存所有实际使用的 seed。

五个套娃以直立姿态随机放到桌面有效区域，要求：

- 完全位于桌面范围内，并为最大物体半径和抓取预留安全边界。
- 不与两个机械臂基座、夹爪、支架和其他物体相交。
- 物体之间保留明确的最小间隙。
- 初始位置不能已经满足最终排序。
- XY 和 yaw 随机；不随机缩放，不随机倾倒。
- 使用有限次数的拒绝采样，不能无限循环。
- 生成后运行若干物理步使其稳定。
- 只有全部物体线速度、角速度和姿态通过稳定检查后才能开始规划。

episode 元数据必须保存采样前 seed、五个物体的资产 ID、初始 pose、
初始线速度/角速度以及稳定后的实际状态。

## 9. 排序任务定义

桌子局部坐标与世界坐标方向一致：

- X：桌子长度方向。
- Y：桌子宽度方向。
- Z：向上。

最终直线：

- 平行于桌子宽度，即沿世界/桌子局部 Y 轴。
- 经过桌面中央，因此直线的 X 坐标为 `0.0`。
- 直线中心的 Y 坐标为桌子中心 `-0.05`。
- 物体底部位于桌面 `z=0.765`。

从桌子局部 `-Y` 到 `+Y`，按尺寸从小到大排列：

`00004 → 00003 → 00002 → 00001 → 00000`

目标中心间距根据相邻物体包围半径加安全间隙计算，并使五个物体整体关于
`y=-0.05` 对称。不能仅按固定中心间距导致大物体互相接触。

默认任务成功判据：

- 每个物体中心 XY 到对应目标的误差不超过 `0.02 m`。
- 每个物体保持直立，倾角不超过 `10°`。
- 五个物体尺寸顺序正确。
- 五个中心到目标直线的垂直误差不超过 `0.02 m`。
- 相邻物体不接触、不重叠。
- 所有物体仍在桌面上。
- 经过稳定等待后，每个物体线速度和角速度低于程序顶部定义的阈值。
- 两台机械臂最终回到各自 home joint position，最大关节误差不超过
  `0.02 rad`。
- 两个夹爪最终处于打开状态。

成功检查必须是一个可复用函数，原始执行和重放执行使用完全相同的判据。

## 10. 双臂协作策略

第一版以稳定跑通为目标，不要求两个机械臂同时运动。采用顺序协作：

- 两个机械臂都必须至少成功搬运一个套娃。
- 一只机械臂执行时，另一只保持 home 或经过验证的安全等待姿态。
- 根据目标/初始位置可达性和规划代价分配物体，但结果要确定且可记录。
- 若默认分配失败，可以在有限次数内尝试另一机械臂或重新规划。
- 不能退化成一台机械臂完成全部任务。

需要检查机械臂自碰撞、机械臂与桌子/地面/支架/房间碰撞、机械臂之间碰撞、
机械臂与未抓取/已放置物体碰撞。顺序运动不能成为忽略另一台机械臂碰撞
几何的理由。

## 11. cuRobo 专家轨迹

必须真正调用 cuRobo 进行六个机械臂关节的运动规划，不能用直线插值、
硬编码关节序列或 Isaac IK 冒充 cuRobo 专家轨迹。

在唯一 Python 文件中完成或构造：

- Piper URDF 和 USD 关节名称映射。
- base link、末端执行器/抓取中心和 home configuration。
- 关节限位、速度和加速度限制。
- 自碰撞配置与碰撞球。
- 左右机械臂各自的 world/base 变换。
- cuRobo MotionGen 配置。
- 场景碰撞世界和动态更新。

使用前要检查 URDF 中 `link6`、`gripper_center`、夹爪关节和 USD articulation
的实际对应关系。末端规划 frame 必须与真实夹爪抓取中心一致，不能仅因为
旧配置写了 `link6` 就忽略夹爪偏移。

每次搬运至少包含：

1. 安全等待/home。
2. pre-grasp。
3. grasp。
4. 闭合夹爪。
5. 验证物体已经被夹住。
6. lift。
7. pre-place。
8. place。
9. 打开夹爪。
10. retreat。

cuRobo 负责机械臂六关节轨迹；夹爪动作由脚本显式控制并与轨迹一起记录。
被抓物体应作为 attached object 或等价的随动碰撞体参与规划。放下后及时
更新碰撞世界，使已经排好的物体成为后续规划障碍物。

规划世界至少包含：

- 桌子。
- 地面。
- 相机支架。
- 必要的房间碰撞体。
- 另一台机械臂当前姿态的碰撞近似。
- 所有未抓取物体。
- 所有已放置物体。

所有规划都要有有限超时和有限重试次数。规划失败、执行偏差过大、抓取失败、
发生不允许的碰撞或物体掉落时，该次尝试不能标记为成功。

## 12. 控制、归位与仿真节拍

把物理、控制、渲染和相机频率集中定义。默认：

- Physics：`120 Hz`。
- Control：`30 Hz`。
- Camera/Render：`30 Hz`。

cuRobo 轨迹要插值或重采样到实际控制周期。记录的是最终实际下发给
articulation controller 的 action，而不是只记录规划器稀疏 waypoint。

所有五个物体放置完成并稳定后，两台机械臂都使用 cuRobo 规划无碰撞轨迹
返回 home，打开夹爪，然后执行最终成功检查。

## 13. HDF5 数据格式

当前环境已包含 `h5py`，每个 episode 使用 HDF5 保存。保持 schema 简单、
明确，并在根 attributes 中写 `schema_version`。

至少保存：

### 静态/元数据

- episode ID、创建时间、accepted 状态和失败原因。
- 所有 seed。
- Isaac Sim、Isaac Lab、cuRobo、Python 版本。
- 所有资产路径。
- 世界单位和坐标约定。
- 桌子、地面、支架、房间、DomeLight 参数。
- 两台机械臂 base pose、home joint position、关节顺序。
- 五个物体 ID、尺寸、初始 pose、目标 pose。
- 三台相机分辨率、频率、内参、裁剪范围和深度定义。
- physics/control/render dt。

### 逐帧数据

- 仿真时间和 frame index。
- 左右机械臂 joint position、joint velocity。
- 左右机械臂实际 joint action。
- 左右夹爪状态和 action。
- 左右末端执行器 world pose。
- 五个物体 world pose、线速度和角速度。
- task phase、当前操作者和当前物体 ID。
- 三台相机同步 RGB。
- 三台相机同步深度。
- 两台腕部相机逐帧 world pose/extrinsic。

RGB、深度、状态和 action 的第一维必须能通过 frame index/timestamp 明确
对应。不要把三路图片松散写成没有索引关系的一堆 PNG。

文件先写入临时/未确认状态；只有重放验证通过后才把 episode 标记为
`accepted=true`。原始执行失败或重放失败的数据可以保留诊断信息，但不能
进入成功数据计数。

## 14. 重放验证

“成功数据”的定义不是“记录时成功”，而是同时满足：

1. 原始 cuRobo 专家执行成功。
2. 原始执行通过完整任务成功检查。
3. 两台机械臂原始执行后成功归位。
4. 从 HDF5 中读取记录信息，销毁并重新建立干净场景，或等价地进行经过
   验证的完整 reset。
5. 恢复机器人初始关节状态、夹爪状态、物体初始 pose/速度、场景参数、
   相机参数、seed 和所有 dt。
6. 重放时禁止重新调用 cuRobo 规划；必须按原时间顺序直接下发 HDF5
   中记录的实际 joint/gripper action。
7. 重放完成后使用与原始执行完全相同的成功检查。
8. 重放后两个机械臂同样成功归位。

只有原始执行和重放都成功，才能设置：

```text
expert_success = true
replay_success = true
accepted = true
```

重放失败必须保存明确原因，不能为了提高成功率而在重放期间重新规划、
偷偷修正物体 pose 或放宽成功阈值。

## 15. 唯一 Python 文件的命令行

在 `dual_piper_sort.py` 内使用简单 `argparse`，至少提供：

```bash
uv run python dual_piper_sort.py --mode scene --headless
uv run python dual_piper_sort.py --mode cameras
uv run python dual_piper_sort.py --mode demo
uv run python dual_piper_sort.py --mode collect --episodes N --seed S --headless
uv run python dual_piper_sort.py --mode replay --episode /path/to/episode.h5 --headless
uv run python dual_piper_sort.py --mode validate --episode /path/to/episode.h5
```

要求：

- 默认 `--mode demo` 采集一条原始成功数据并立即执行重放验证。
- `collect` 的 `N` 表示最终得到的 accepted episode 数，不是尝试次数。
- 设置整个 episode 和单次规划的最大时间/步数，避免无限运行。
- headed 与 headless 使用同一套任务逻辑。
- `scene`、`cameras`、`validate` 同时充当嵌入在单文件内的 smoke/自动化检查，
  不再创建独立测试文件。
- 失败时返回非零退出码，并输出可以定位阶段、机械臂、物体和原因的错误。

建议只使用以下级别的简单组织：

```text
常量和少量 dataclass
create_scene()
create_robots()
create_cameras()
spawn_objects()
build_curobo_planners()
plan_and_execute_pick_place()
record_frame()
check_success()
save_episode()
restore_and_replay()
main()
```

可以根据实际 API 调整函数名，但不要把一个文件写成难以追踪的大型框架。

## 16. 验证顺序

必须从低风险检查逐步推进：

1. 静态检查：路径、USD default prim、stage 单位、up axis、材质入口。
2. 场景 headed 检查：桌子/地面/房间/HDR/支架位置和光照。
3. 双臂检查：base pose、关节名、限位、夹爪开合、home。
4. 相机检查：三路 RGB/深度 shape、dtype、方向、覆盖和数值范围。
5. 随机物体检查：固定 seed 可复现、无重叠、稳定。
6. 单臂 cuRobo 检查：至少一个物体完整抓放。
7. 双臂完整专家执行。
8. HDF5 schema 和同步检查。
9. 从记录状态进行动作重放。
10. 至少一次 headed 集成验证和一次 headless 集成验证。

不能用“脚本能 import”“某个函数单测通过”或“cuRobo 返回 success”代替完整
物理执行、图像采集和重放成功。

## 17. 最终验收条件

完成任务前必须提供当前运行产生的权威证据，证明：

- 只新增了一个实现 Python 文件 `dual_piper_sort.py`。
- 指定房间、HDR、桌面/地面材质、相机支架、双 Piper 和五个套娃均实际加载。
- 桌子和双臂使用本文固定位置、尺寸和姿态。
- 两个机械臂都至少搬运一个物体。
- 所有机械臂运动段来自 cuRobo，并经过碰撞检查。
- 五个物体按 `00004 → 00003 → 00002 → 00001 → 00000` 排列。
- 排列直线经过桌面中心且沿 Y/桌宽方向。
- 两台机械臂最后自动归位并打开夹爪。
- 三台相机同步产生 RGB 和米制深度。
- HDF5 包含规定的状态、动作、图像和元数据。
- 至少一条 episode 的原始执行成功。
- 同一条 episode 从记录的初始状态重建后，直接重放记录 action 仍成功。
- headed 和 headless 至少各完成一次相应集成验证。
- `dual-piper-dev-log.md` 已逐任务记录工作、命令、结果、问题和权威资料。
- 每个已经完成的小任务都有对应的、未混入无关文件的 Git commit。

最终回复必须简洁列出：

- 唯一实现文件的路径。
- 运行命令。
- 成功 episode 路径和 schema version。
- 三路 RGB/深度的 shape、dtype、频率和深度单位。
- 五个物体各自的最终位置误差。
- 两台机械臂最大归位误差。
- 原始执行和重放的成功检查结果。
- 相关 commit 列表。
- 尚存的真实限制；不能把未验证项目描述成已完成。
