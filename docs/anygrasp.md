# AnyGrasp 接入

专家链路可以通过外部 AnyGrasp HTTP 服务实时生成抓取候选。模型常驻服务端 GPU，ScaleBench 通常经 SSH 隧道访问 `http://127.0.0.1:5001`。

## 启动服务

远端环境需要安装 AnyGrasp SDK 并准备 checkpoint，然后从仓库运行协议 v3 入口：

```bash
python scripts/run_anygrasp_service.py \
  --checkpoint_path /absolute/path/to/checkpoint.tar \
  --host 0.0.0.0 \
  --port 5001
```

本机验证：

```bash
curl http://127.0.0.1:5001/health
```

响应必须包含：

```json
{
  "protocol_version": 3,
  "capabilities": ["target_pixel_indices", "aligned_rgb"],
  "status": "ok"
}
```

协议 v3 在完整场景点云上做碰撞检测，同时使用目标像素生成 `region_steering`。旧服务可能忽略目标区域，候选质量和性能都不可作为当前验收结果。

## Scene 配置

默认值位于 [`configs/scene/default.yml`](../configs/scene/default.yml)：

```yaml
anygrasp:
  service_url: http://127.0.0.1:5001
  request_timeout_s: 60.0
  capture_distance_m: 0.4
  depth_trunc_m: 2.0
  top_k: 200
  min_score: 0.0
  collision_detection: true
  dense_grasp: false
  approach_distance_m: 0.08
  target_margin_m: 0.015
  minimum_target_points: 128
  minimum_point_height_above_table_m: 0.002
  minimum_tcp_height_above_table_m: 0.015
  maximum_open_axis_vertical_dot: 0.35
```

这些参数分别控制服务连接、采集距离、点云范围、候选数量，以及本地目标归属、桌面净空和夹爪方向过滤。

完全省略 `anygrasp` 时，运行时使用机器人配置的 `grasp_catalog_path`。服务错误或无有效候选不会自动回退到 catalog，当前 skill 会明确失败。

## 运行流程

每次 pick 执行一次抓取推理：

1. `arm="auto"` 根据目标到左右机器人 base 的距离选择机械臂；显式 `left` 或 `right` 则固定使用该臂。
2. 从所选臂一侧的目标斜上方采集，目标点少于 `minimum_target_points` 时只从正上方重拍一次。`single_object_pick_and_place` 额外只保留 TCP 位于物体上半部的抓取，避免瓶底附近的实际抓取关系被直接平移到目标位姿。曾验证过固定正上方 `0.4 m` 的任务视角，但 seed 101 返回的 24 个有效候选在 Piper 的真实 CuRobo IK 中全部不可达，因此它只适合作为点数不足时的采集回退，而不作为该任务的首选视角。
3. 客户端发送同一帧 RGB-D、内参和目标像素索引。深度单位已经是米，因此请求使用 `scale: 1.0`。
4. 服务返回相机光学坐标系中的候选；客户端将它们变换到环境局部世界系。
5. 本地按分数、夹爪最大开度、目标包围盒、桌面净空和开合轴方向过滤。
6. Planner 对候选及其平行夹爪 180 度等价姿态做 IK 和碰撞检查，选择分数最高的完整可行轨迹。
7. 闭合后从实时物体与 TCP 位姿重测 `T_object_tcp`，再规划搬运和放置。

相机刷新不调用 `env.step()`，不会推进 episode，也不会写入 recorder。默认只请求一次服务；目标点不足时的重拍发生在请求之前。

## 坐标约定

Isaac Lab 的相机 optical pose 直接用于 AnyGrasp 的 `+Z` 向前、`+Y` 向下坐标系。AnyGrasp tip 按官方定义计算：

```text
tip = translation + depth * rotation_matrix[:, 0]
```

检测 tip 先解释为机器人 `tcp.parent_frame`，再应用 `RobotConfig.kinematics.tcp` 得到 benchmark TCP。运行时不硬编码 Piper offset；修改 TCP 时必须同步验证在线抓取和离线 catalog。

## 运行与诊断

执行单次真实 pick：

```bash
uv run python scripts/run_demo_generation.py \
  --program pick \
  --num-envs 1 \
  --episodes 1 \
  --max-steps 1200 \
  --viz none
```

检查单帧原始候选，不执行机器人动作：

```bash
uv run python scripts/run_demo_generation.py \
  --program grasp-diagnostics \
  --base-seed 107 \
  --object-name doll_00004 \
  --grasp-arm left \
  --diagnostics-output outputs/anygrasp-seed107.json \
  --viz none
```

增加 `--open3d` 会先并排显示客户端实际发送的 RGB 图和米制深度图，再依次显示完整候选、服务最高分候选和本地过滤后的最高分候选。深度图中的黑色像素是请求中序列化为 `0` 的无效或截断深度；有效深度按当前帧范围从近处红色映射到远处蓝色。每个窗口都会阻塞，关闭当前窗口后才会显示下一个，只用于交互诊断。

常见失败顺序：先检查 `/health` 协议版本，再检查目标点数量和相机分辨率，最后查看候选过滤状态与 CuRobo 可达性。低分辨率 `d435_smoke.yml` 适合运行链路检查，不适合作为小物体抓取质量基准。
