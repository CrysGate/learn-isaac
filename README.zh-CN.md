# ScaleBench

[English](README.md) | [简体中文](README.zh-CN.md)

ScaleBench 是一个配置驱动的 Isaac Lab 双臂操作项目。它把机器人、相机、场景、任务、仿真和环境参数保存在 YAML 中，并提供场景预览、任务评测、专家数据生成、策略运行和 episode 回放入口。

当前包含两个任务：

- `sort_dolls_by_size`：将五个套娃按尺寸排列到固定槽位。
- `single_object_pick_and_place`：将随机位置的 bottle 直立放到固定槽位。

两个任务都支持确定性 seed、layout 导入导出和最终状态评测。专家链路支持 CuRobo 规划，以及 AnyGrasp 在线抓取或机器人配置中的离线抓取 catalog。

## 环境

- Python 3.12
- Isaac Sim 6.0.1
- Isaac Lab `release/3.0.0-beta2`（当前验证提交 `6a7acb0`）
- CuRobo（当前验证提交 `8e734f3`）
- CUDA 12.8 对应的 PyTorch 2.10
- [`uv`](https://docs.astral.sh/uv/)

项目通过 `pyproject.toml` 引用本地 `third_parties/IsaacLab` 和 `third_parties/curobo`：

```bash
mkdir -p third_parties
git clone --branch release/3.0.0-beta2 \
  https://github.com/isaac-sim/IsaacLab.git third_parties/IsaacLab
git clone https://github.com/NVlabs/curobo.git third_parties/curobo
git -C third_parties/IsaacLab checkout 6a7acb0
git -C third_parties/curobo checkout 8e734f3

uv sync --frozen
```

默认配置还依赖未纳入 Git 的 `Assets/` 资产包，包括 Piper USD/URDF、房间、材质、相机支架、HDR、套娃和 bottle。资产路径以 `configs/` 中的 YAML 为准；缺少资产时配置加载会报告具体字段和路径。

## 快速开始

预览任务场景：

```bash
uv run python scripts/preview_scene.py --task sort_dolls_by_size
```

使用固定 seed 做两步无界面运行检查：

```bash
uv run python scripts/preview_scene.py \
  --task single_object_pick_and_place \
  --seed 42 \
  --viz none \
  --max-steps 2
```

导出并恢复同一个布局：

```bash
uv run python scripts/preview_scene.py \
  --task sort_dolls_by_size \
  --seed 42 \
  --export-layout layouts/sort_dolls_by_size/42.json

uv run python scripts/preview_scene.py \
  --task sort_dolls_by_size \
  --layout layouts/sort_dolls_by_size/42.json
```

运行完整专家链路：

```bash
uv run python scripts/run_demo_generation.py \
  --task single_object_pick_and_place \
  --program expert \
  --num-envs 1 \
  --episodes 1 \
  --max-steps 1200 \
  --viz none
```

默认场景使用 AnyGrasp 服务。服务部署和诊断见 [AnyGrasp 文档](docs/anygrasp.md)。使用套娃离线 catalog 时增加 `--grasp-source catalog`。

## 主要入口

| 入口 | 用途 |
|---|---|
| `scripts/preview_scene.py` | 预览场景、检查布局、执行有界运行。 |
| `scripts/run_policy_rollout.py` | 验证 policy、fixed-batch 调度和记录链路。 |
| `scripts/run_demo_generation.py` | 运行 skill expert、CuRobo 和数据记录。 |
| `scripts/replay_episode.py` | 恢复 HDF5 初态、重放 action 并重新评测。 |
| `scripts/view_hdf5.py` | 在浏览器中检查录制的 episode、相机和状态。 |
| `scripts/export_hdf5_camera_videos.py` | 导出 RGB 和深度视频。 |
| `scripts/generate_curobo_robot_config.py` | 从机器人配置生成 CuRobo 碰撞配置。 |

完整命令示例见 [scripts/README.md](scripts/README.md)。所有脚本都应从仓库根目录通过 `uv run` 启动；参数以各脚本的 `--help` 为准。

## 配置边界

- `configs/robots/`：关节、TCP、执行器、夹爪、相机挂载、URDF 和抓取 catalog。
- `configs/cameras/`：图像尺寸、输出类型、内参和裁剪范围。
- `configs/scene/`：静态场景、机器人安装位、推理相机、AnyGrasp 和光照。
- `configs/tasks/`：任务资产、布局约束、目标槽位和成功阈值。
- `configs/sim/`：物理步长、重力、渲染和必要的 PhysX 覆盖。
- `configs/envs/`：环境数量、间距、控制频率、克隆和 reset 行为。

配置模型严格拒绝未知字段。配置引用相对于所在配置文件解析；资产引用在传入 `asset_root` 时相对于该目录解析。距离单位为米，四元数顺序为 `xyzw`。

## 运行时

`scale_bench.api.create_env()` 是公共环境入口。调用方先启动 Isaac Sim，再传入已经加载的配置、具体 Task，以及 `base_seed` 或 `layouts`。`ScaleBenchEnv` 负责仿真、scene 和 manager 生命周期。

Episode 运行时分为两条链路：

- `PolicyRolloutRunner` 接收 policy observation，输出关节 action。
- `DemoGenerationRunner` 将 task expert 展开为 skill request，经 planner 和 command executor 执行。

两条链路共享调度、评测、终止和记录逻辑。HDF5 同时保存初始状态、action、评测结果和终止原因，可由 `replay_episode.py` 检查一致性。

## 验证改动

项目不编写单元测试；改动后运行对应的真实链路。最低限度先加载配置，再执行有界环境运行：

```bash
uv run python -c \
  'from scale_bench.config.loader import load_config; from scale_bench.config.models.environment import EnvironmentConfig; print(load_config("configs/envs/default.yml", EnvironmentConfig))'

uv run python scripts/preview_scene.py \
  --task sort_dolls_by_size \
  --viz none \
  --max-steps 2
```

涉及 planner、抓取或 recorder 时，还应运行对应的 demo generation 和 replay 命令。

## 进一步阅读

- [AnyGrasp](docs/anygrasp.md)
- [抓取数据生成](src/grasp_data_gen/README.zh-CN.md)
- [OBJ 转 USD](src/assets_gen/README.zh-CN.md)

## 许可证

[MIT License](LICENSE)
