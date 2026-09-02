# Scripts

以下命令都从仓库根目录运行。参数列表以各脚本的 `--help` 为准。

## 场景预览

`preview_scene.py` 创建真实 `ScaleBenchEnv`，用于交互预览、layout 检查和有界运行：

```bash
uv run python scripts/preview_scene.py \
  --task sort_dolls_by_size \
  --seed 42
```

支持的 task 为 `sort_dolls_by_size` 和 `single_object_pick_and_place`。`--seed` 与 `--layout` 互斥；`--export-layout` 保存本次布局。

无界面检查：

```bash
uv run python scripts/preview_scene.py \
  --task single_object_pick_and_place \
  --viz none \
  --max-steps 2
```

## Policy 运行

`run_policy_rollout.py` 使用套娃任务验证 policy、fixed-batch scheduler、episode evaluator 和可选 HDF5 记录：

```bash
uv run python scripts/run_policy_rollout.py \
  --num-envs 2 \
  --episodes 3 \
  --max-steps 8 \
  --record-output outputs/policy-smoke \
  --viz none
```

默认 policy 保持 reset 关节位置，并按 seed 在不同 step 结束。传入 `--left-joint4-offset-rad` 时会执行一次真实 `MoveToJoints` command，用于检查 action adapter。

## 专家数据生成

`run_demo_generation.py` 支持五种 program：

- `pick`：抓取一个物体。
- `pick-and-place`：抓取并放置一个物体。
- `expert`：执行 Task 提供的完整专家程序。
- `grasp-diagnostics`：只检查 AnyGrasp 候选，不执行运动。
- `collect-grasps`：逐个规划并真实执行 AnyGrasp 候选，只保存完成完整
  pick-and-place 的抓取姿态。

完整 bottle task：

```bash
HEADLESS=1 uv run python scripts/run_demo_generation.py \
  --task single_object_pick_and_place \
  --program expert \
  --base-seed 101 \
  --num-envs 2 \
  --episodes 2 \
  --max-steps 1200 \
  --viz kit \
  --record-output outputs/bottle-pick-place \
  --dataset-name bottle_pick_place \
  --record-camera-observations \
  --replay
```

HEADLESS=1 uv run python scripts/run_demo_generation.py \
  --task single_object_pick_and_place \
  --program grasp-diagnostics \
  --base-seed 101 \
  --num-envs 1 \
  --episodes 1 \
  --max-steps 1200 \
  --viz kit \
  --open3d

seed 范围固定为 `[base-seed, base-seed + episodes)`，增加 `--num-envs` 只改变并行 slot 数。末尾输出成功数量、总数和成功率。

需要保存 episode 时增加：

```bash
  --record-output outputs/bottle-pick-place \
  --dataset-name bottle_pick_place
```

需要同时记录左腕、右腕和俯视相机的 RGB-D 时，再增加
`--record-camera-observations`。不传该开关时只记录关节等默认观测，避免产生大量图像数据；该开关要求同时传入 `--record-output`。无显示器采集相机时必须使用 `HEADLESS=1 --viz kit`，使 reset 阶段的 rerender 生成有效 RTX 帧；`--viz none` 不适用于相机数据采集。

`run_demo_generation.py` 使用 CuRobo 规划真实轨迹。默认 `--grasp-source scene` 使用 Scene 中的 AnyGrasp；套娃任务可以传 `--grasp-source catalog` 使用离线候选。

### 采集物理验证的抓取姿态

`collect-grasps` 只用于 `single_object_pick_and_place`。每次 AnyGrasp
推理后，程序会把本地几何过滤通过的每个 candidate 放进独立 episode；该
episode 的规划器只能看到这一条 candidate。`--grasp-arm` 默认是 `auto`，每个
seed 按物体到左右 robot base 的距离选臂，并在拍摄、过滤、规划和执行期间固定
使用选出的机械臂；显式传 `left` 或 `right` 时则始终使用指定机械臂。只有
完整抓取、搬运、释放和最终稳定性验证都成功的 candidate 才会保存：

```bash
HEADLESS=1 uv run python scripts/run_demo_generation.py \
  --task single_object_pick_and_place \
  --program collect-grasps \
  --base-seed 101 \
  --episodes 1 \
  --num-envs 1 \
  --max-steps 1200 \
  --viz kit
```

此模式下 `--episodes` 表示独立的 AnyGrasp 采集轮数，而不是最终 candidate
episode 数；每轮使用 `[base-seed, base-seed + episodes)` 中对应的物体布局。
`--num-envs` 控制 candidate episode 的并行 slot 数。

结果写到物体 USD 同目录的 `<usd文件名>_grasps.yml`，例如
`Assets/Object/Rigid/bottle/bottle_grasps.yml`。文件使用项目现有
`GraspCatalogConfig` 格式，姿态为 `T_object_tcp`、四元数顺序为 `xyzw`。文件
已存在时会先校验 robot、TCP、approach distance 和对象名，再保留原数据并为
本次成功项分配连续 `candidate_id` 后原子追加；本轮没有成功项时文件保持不变。

### CuRobo 碰撞模型可视化

规划调试时传入 `--visualize-curobo`。程序会正常完成规划和实际执行以取得各阶段的真实状态，但运行过程中不显示碰撞标记；episode 结束后会在 Kit Viewer 中打开阶段浏览器：

```bash
uv run python scripts/run_demo_generation.py \
  --task single_object_pick_and_place \
  --program pick-and-place \
  --num-envs 1 \
  --episodes 1 \
  --max-steps 1200 \
  --visualize-curobo
```

该开关会自动启用 Kit，不需要另外传入 `--viz kit`。可视化目前只支持 `--num-envs 1`，不能与 `--headless`、`--viz none` 或不包含 Kit 的其他显式 visualizer 配置一起使用。

- 蓝色球：当前求解机械臂的 CuRobo collision spheres。
- 橙色球：当前夹持物的 collision spheres。
- 黄色半透明盒体：桌面 collision cuboid。
- 红色半透明盒体：场景物体 collision cuboids。
- 灰色半透明盒体：相机支架 USD 中启用碰撞的几何体包围盒。
- 绿色半透明盒体：另一机械臂在当前 CuRobo world 中的 collision cuboids。

浏览器不会逐个列出失败的抓取候选或放置朝向，一次运行最多显示七个阶段。最终方案使用的阶段标为 `selected`；尚未成功的阶段保留最后一次实际送入 CuRobo 的尝试，并标为 `attempted`，可用于检查失败现场。使用 `<` 和 `>` 依次查看 `pre_grasp`、`grasp`、`lift`、`pre_place`、`place`、`retreat`、`clear` 中实际进入过的阶段；标题会显示阶段序号、名称、状态和当前机械臂。浏览期间仿真时间线暂停，点击 `Close` 后程序退出。

这些标记是各阶段规划起点的碰撞快照，不是轨迹动画；它们不参与物理，也不会写入相机观测。

采集后逐条打开 Kit 回放：

```bash
uv run python scripts/run_demo_generation.py \
  --program expert \
  --record-output outputs/curobo-expert \
  --dataset-name curobo_expert \
  --replay \
  --viz none
```

`--replay` 要求 `--record-output`。采集与每个 GUI replay 分别运行在独立进程；关闭当前 Kit 窗口后才会启动下一个 episode。

AnyGrasp 设置和诊断命令见 [`docs/anygrasp.md`](../docs/anygrasp.md)。

## Episode 回放

`replay_episode.py` 恢复录制初态、重放全部 action，并核对 seed、layout、步数和重新评测的 success：

```bash
uv run python scripts/replay_episode.py \
  outputs/bottle-pick-place/bottle_pick_place.hdf5 \
  --task single_object_pick_and_place \
  --episode-name demo-seed-101 \
  --viz kit
```

数据集只有一个 episode 时可省略 `--episode-name`。回放不会再次记录。

## 浏览 HDF5 录制

`view_hdf5.py` 使用 NiceGUI 在本机浏览器中展示多个 episode、可选的 RGB-D 相机、录制属性和逐帧状态。每个关节都有独立的连续轨迹图，并与视频或纯数据播放同步：

```bash
uv run python scripts/view_hdf5.py \
  outputs/bottle-pick-place/bottle_pick_place.hdf5
```

打开命令输出的 `http://127.0.0.1:8765`；端口占用时传入 `--port`。浏览器只监听本机回环地址。无相机观测的录制可直接播放关节和逐帧数据；存在相机观测时必须成对包含 RGB 和 depth，且系统需要安装 `ffmpeg` 才能生成播放视频。

## 导出相机视频

`export_hdf5_camera_videos.py` 从一个 HDF5 group 导出 RGB 和伪彩色深度 MP4：

```bash
uv run python scripts/export_hdf5_camera_videos.py \
  outputs/bottle-pick-place/bottle_pick_place.hdf5 \
  --demo demo_demo-seed-101 \
  --camera overhead
```

`--camera` 可选 `left_robot`、`right_robot` 或 `overhead`。默认帧率从记录元数据推导；`--depth-min-m` 和 `--depth-max-m` 只控制深度视频显示范围，不修改原始数据。

## AnyGrasp 服务

`run_anygrasp_service.py` 在安装了 AnyGrasp SDK 的远端环境运行协议 v3 服务：

```bash
python scripts/run_anygrasp_service.py \
  --checkpoint_path /absolute/path/to/checkpoint.tar \
  --host 0.0.0.0 \
  --port 5001
```

部署后检查 `GET /health` 返回 `protocol_version: 3`。`run_demo_generation.py --program grasp-diagnostics --open3d` 会先显示实际发送给服务的二维 RGB-D，再显示返回候选的彩色点云。`view_anygrasp_open3d.py` 是该命令启动的隔离查看进程，通常不直接调用。

## 生成 CuRobo 配置

`generate_curobo_robot_config.py` 从机器人配置和 URDF 生成 collision YAML、XRDF 和 metrics：

```bash
uv run python scripts/generate_curobo_robot_config.py \
  --robot-config configs/robots/piper.yml \
  --output configs/robots/curobo/piper.yml
```

生成需要 CUDA。`--reuse-generated PATH --device cpu` 只能复核已有生成物的结构和 CPU 几何指标，不能替代真实 CuRobo load 与规划验收。
