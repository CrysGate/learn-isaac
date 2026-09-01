# 抓取数据生成器

[English](README.md) | [简体中文](README.zh-CN.md)

在 Isaac Sim 中生成对向抓取候选，按桌面任务约束进行过滤，并通过“闭合并保持”的
物理仿真验证每个可行候选。默认配置面向 Piper 夹爪，直接从完整机器人 USD 中组装
夹爪，无需单独准备夹爪资产。

## 处理流程

1. 从机器人 USD 中组装配置指定的夹爪 link 和 joint。
2. 在物体局部坐标系中采样对向抓取 TCP 位姿。
3. 剔除不满足接近方向、支撑面或腕部相机约束的位姿。
4. 在仿真中闭合并保持夹爪，剔除空抓、不同步或不稳定的抓取。
5. 导出完整诊断信息，以及按分数降序排列的有效抓取。

夹爪组装、采样、桌面过滤和物理验证阈值定义在
[`piper.yml`](piper.yml)；机器人语义和源 USD 来自
[`configs/robots/piper.yml`](../../configs/robots/piper.yml)。

## 生成抓取

先按[仓库 README](../../README.zh-CN.md) 完成安装，然后在仓库根目录运行：

```bash
uv run python -m grasp_data_gen.generate_grasps \
  --object-usd Assets/Object/Rigid/matryoshka_dolls/00000/object.usdz \
  --output-dir outputs/grasp_data/piper/00000
```

生成过程默认以无界面模式运行；添加 `--gui` 可观察物理评估。命令还支持
`--num-orientations` 和 `--seed`，使用 `--help` 可查看当前默认值。

批量处理一个目录下的物体时，使用 `--object-dir` 代替 `--object-usd`：

```bash
uv run python -m grasp_data_gen.generate_grasps \
  --object-dir Assets/Object/Rigid/matryoshka_dolls \
  --output-dir outputs/grasp_data/piper
```

程序会递归查找 `.usd`、`.usda`、`.usdc` 和 `.usdz` 文件，并为每个物体创建
独立的输出子目录。比如输入 `00000/object.usdz` 会输出到 `00000/`；平铺的
`mug.usd` 会输出到 `mug/`。输出目录必须位于输入目录之外，避免再次运行时把
生成的评估场景当成输入。单个物体失败不会中断批次中的其他物体，命令会在结束时
打印成功/失败汇总，并在存在失败时返回非零退出码。

## 输出文件

- `evaluation_stage.usda`：组装后的夹爪与物体评估场景。
- `report.yaml`：所有候选、评估指标及淘汰原因。
- `successful_grasps.yaml`：按分数降序排列的有效抓取，并包含可视化所需的
  闭合夹爪 link 位姿。

保存的 TCP 位姿为 `T_object_tcp`，位置单位为米，四元数顺序为 `xyzw`。
运行时可按下式转换到世界坐标系：

```text
T_world_tcp = T_world_object @ T_object_tcp
```

离线生成只验证夹爪与物体的接触以及桌面可行性。将物体放入实际运行场景后，仍需
检查完整机械臂 IK、接近路径碰撞和任务特定约束。

## 可视化结果

无需重新生成，即可查看已通过验证的抓取：

```bash
uv run python -m grasp_data_gen.visualize_grasps \
  --grasp-file outputs/grasp_data/piper/00000/successful_grasps.yaml
```

查看器默认显示得分最高的抓取。使用上一项/下一项控件逐个浏览，或点击 `All`
叠加显示全部有效位姿。

## 导出 ScaleBench catalog

使用显式的物体名映射，将生成结果转换为运行时使用的紧凑 schema。源文件候选数超过
上限时，导出器先保留最高分姿态，再按姿态角覆盖选择其余项，最后恢复得分顺序：

```bash
uv run python -m grasp_data_gen.export_scale_bench_catalog \
  --robot-config configs/robots/piper.yml \
  --output outputs/grasp_data/piper/catalog_candidate.yml \
  --max-candidates-per-object 32 \
  --object-grasp doll_00000=outputs/grasp_data/piper/00000/successful_grasps.yaml \
  --object-grasp doll_00001=outputs/grasp_data/piper/00001/successful_grasps.yaml
```

任务需要的每个物体都必须传一项 `--object-grasp`。导出器会拒绝机器人 TCP 父帧、
位置、方向或接近距离不一致的生成结果。导出文件只是待验证 catalog：孤立夹爪评估
分数不等价于完整操作质量，只有通过多 seed 真实 PickAndPlace 后才能将它配置为机器人
的默认 catalog。
