# Dual Piper 开发日志

## 2026-07-30 22:30 — 固化完整任务提示词

- 目标：把双 Piper 三相机套娃排序平台的资产、固定布局、单文件实现、数据重放、日志和 Git 工作流整理为可直接执行的完整规格。
- 完成内容：编写 `goal.md`，加入桌子与双臂精确位姿、地面参数、房间与 HDR、三相机、cuRobo、HDF5、动作重放验收、逐任务日志和逐任务提交要求。
- 修改文件：`goal.md`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short --branch`；检查现有 goal/log；`.venv/bin/python -c "import h5py; print(h5py.__version__)"`。
- 验证结果：确认两个目标文档原本为空；确认当前环境含 `h5py 3.16.0`；未修改或暂存现有未跟踪资产。
- 问题与处理：工作区已有大量未跟踪资产，因此后续提交必须使用路径限定，禁止 `git add .`。
- 参考资料：本轮只使用用户给定参数和当前仓库/环境检查，无外部资料。
- Commit message：`docs: define dual Piper simulation goal`

## 2026-07-30 22:57 — 允许独立测试代码文件

- 目标：把实现文件上限从一个 Python 文件调整为最多两个 Python 文件。
- 完成内容：保留 `dual_piper_sort.py` 作为主实现，新增唯一允许的 `test_dual_piper_sort.py` 测试文件规格；同步补充快速测试、headless 集成测试、文件上限和最终验收要求。
- 修改文件：`goal.md`、`dual-piper-dev-log.md`。
- 运行命令：检查 `goal.md` 中所有“单 Python/唯一 Python/独立测试”条款；复核 Git diff 和文件上限表述。
- 验证结果：主实现和测试代码最多使用两个固定 Python 文件，禁止第三个任务 Python 文件；测试文件不得复制实现或替代最终物理验收。
- 问题与处理：`goal.md` 在本轮开始前已有用户删除标题后空行的未提交修改，予以保留。
- 参考资料：无。
- Commit message：`docs: allow one dedicated simulation test file`

## 2026-07-30 23:14 — 固化场景常量并检查资产、prim 与关节

- 目标：在不修改原始资产的前提下，固化世界、桌子、地面、双臂、相机和五个指定套娃的第一版常量，并以 Isaac Sim 5.1.0 实际 USD API 检查资产组成。
- 完成内容：新增 `dual_piper_sort.py` 和唯一测试入口 `test_dual_piper_sort.py`；集中定义绝对资产解析、米制/Z-up/wxyz 约定、固定桌子与双臂位姿、仿真频率和 D435 近似参数；解析 Piper URDF、套娃 metadata 和 MDL 导出入口；新增 `--mode audit`，实际打开 Piper、房间、支架和五个 USDZ，验证 default prim、单位、up axis、包围盒、关节、刚体和碰撞。确认房间仍有 `/World/simple_room/RectLight`，后续场景加载必须禁用；确认 Piper 的 `link6/camera` 只是 Xform、`gripper_center` 位于 `link6` 后 0.1358 m；确认套娃已有根刚体和凸分解碰撞，但未 authored mass/physics material，后续按 metadata 覆盖 `0.05 kg` 与摩擦 `0.45`。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；`uv run python test_dual_piper_sort.py --mode fast`；`uv run python dual_piper_sort.py --mode audit --headless`；`uv run python test_dual_piper_sort.py --mode integration --headless`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check -- dual_piper_sort.py test_dual_piper_sort.py`。
- 验证结果：快速测试 5/5 通过；Isaac USD 集成测试连同快速测试共 6/6 通过；CLI 输出 `ASSET_AUDIT_OK` 并以退出码 0 完成；所有八个 USD/USDZ 均为 `metersPerUnit=1.0`、Z-up，Piper USD 的九个关节与 URDF 映射一致；语法编译与 diff whitespace 检查通过。
- 问题与处理：首次用 `UsdLux.Light` 检查灯光时发现当前 USD bindings 不提供该基类，改为检查实际 light type name；快速测试首次因 metadata 的 `0.11000000000000001` 与字面量 `0.11` 精确比较失败，改为有界浮点比较；发现 Isaac Sim 5.1.0 的 `SimulationApp.close()` 会直接终止进程并吞掉其后的输出，因此把报告和测试断言全部放到关闭之前并显式 flush，避免“退出码为 0 但测试未运行”的假通过。
- 参考资料：`Assets/Robots/piper/piper_description/urdf/piper.urdf`；`Assets/Robots/piper/Piper.usd`；`Assets/Object/RoboDojo/Rigid/matryoshka_dolls/00000..00004/metadata.json`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py`。
- Commit message：`feat: audit dual Piper simulation assets`

## 2026-07-30 23:22 — 搭建房间、HDR、地面、桌子和顶部支架

- 目标：实现可复用的静态场景搭建与严格验证入口，并分别完成 headed/headless smoke test。
- 完成内容：实现 `create_scene()`、`validate_scene()` 和有限帧 viewport 预览；引用指定房间和顶部相机支架，按支架源包围盒把 0.613 m 横梁放在 `y=0.50,z=1.55`，使资产自带的相机外壳位于桌面中心上方；创建固定尺寸的静态 Cube 桌子和地面并绑定 `Mahogany_Planks`、`Wood_Tiles_Fineline` 的明确 MDL entry；停用房间残留 `RectLight`，创建唯一 HDR DomeLight；验证 stage 单位、up axis、dt、世界包围盒、静态碰撞、引用、灯光唯一性、HDR 属性和 MDL shader subIdentifier。将生成目录加入 `.gitignore`。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`.gitignore`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；`uv run python test_dual_piper_sort.py --mode fast`；`uv run python dual_piper_sort.py --mode scene --headless`；`uv run python dual_piper_sort.py --mode scene`；`uv run python test_dual_piper_sort.py --mode integration --headless`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check -- dual_piper_sort.py test_dual_piper_sort.py .gitignore`。
- 验证结果：fast 5/5、集成 7/7 通过；headed 与 headless 均输出 `SCENE_SMOKE_OK` 且退出码为 0；两张预览均为 RGBA `1280×720`，目视确认房间、带纹理地面、桌子和顶部横梁/相机外壳位置合理；stage 报告只有 `/World/Scene/EnvironmentLight` 一个 active light；桌面包围盒精确为 `[-0.7,-0.6,0.715]..[0.7,0.5,0.765]`，地面为 `[-3,-3,-0.1]..[3,3,0]`，支架组合后有 12 个 collision prim。
- 问题与处理：首次 headless viewport capture 在 MDL 首次编译时耗时约 34 秒，但有限 240 render-step 上限内完成；Mahogany MDL 报告已有资产内部的 float-to-float2 compiler warning，不影响 shader 创建、entry 验证和渲染，未修改原始 MDL。
- 参考资料：`third_parties/IsaacLab/source/isaaclab/isaaclab/sim/spawners/lights/lights.py`；`third_parties/IsaacLab/source/isaaclab/isaaclab/sim/spawners/materials/visual_materials.py`；`.venv/lib/python3.11/site-packages/isaacsim/extscache/omni.kit.viewport.utility-1.1.2+69cbf6ad/omni/kit/viewport/utility/__init__.py`。
- Commit message：`feat: build static dual Piper scene`

## 2026-07-30 23:31 — 加载双 Piper 并验证关节、夹爪、安装位姿和归位

- 目标：在固定桌面位姿加载两个独立 Piper articulation，确定真实仿真 DOF 映射，验证夹爪 mimic、home 驱动、安装方向和 headed/headless 几何。
- 完成内容：实现 `create_robots()`、有限步 action/convergence helper、`validate_robots_at_home()` 和 `exercise_and_validate_robots()`；两个 prim 均引用同一 `Piper.usd`，使用固定 base pose；用实际 articulation controller 闭合/打开 `gripper_joint`，确认 `joint8` 跟随；给左右 joint1 分别施加 `+0.2/-0.2 rad` 后自动返回 home 并等待额外 60 个稳定步；读取 `gripper_center`、两指和 base 的实际 world pose，验证两臂都朝世界 `+Y` 的桌面内部工作；新增 `--mode robots` 和 headed/headless 预览。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；两次 `uv run python - <<'PY' ...` 最小 articulation/控制探针；`uv run python test_dual_piper_sort.py --mode fast`；`uv run python dual_piper_sort.py --mode robots --headless`；`uv run python dual_piper_sort.py --mode robots`；`uv run python test_dual_piper_sort.py --mode integration --headless`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check -- dual_piper_sort.py test_dual_piper_sort.py`。
- 验证结果：fast 6/6、集成 9/9 通过；headed/headless 均输出 `ROBOT_SMOKE_OK` 且退出码 0；两臂 base 实际位置分别为约 `[-0.3,-0.45,0.765]`、`[0.3,-0.45,0.765]`，姿态与归一化后的 `+90° Z` 四元数一致；两臂 home `gripper_center` 从 base 向世界 `+Y` 前伸约 `0.51236 m`；夹爪约 45 physics steps 从 `0.080 m` 开距闭到约 `6e-6 m` 并重新打开；两臂回 home 的最大关节误差约 `0.000661 rad`，远低于 `0.02 rad`；headed 预览目视确认两个基座、等待姿态、支架和桌面互不穿插。
- 问题与处理：首个探针沿用旧示例假定只有 7 DOF，`set_joint_positions` 报 `(1,7)` 到 `(1,8)` shape mismatch；实际 USD/PhysX 暴露顺序为六个 arm joints、`gripper_joint`、`joint8` 共 8 DOF，改为初始化全部 8 DOF，但 action 只下发六臂关节加 `gripper_joint`，让 `joint8` 由 mimic schema 跟随。USD 的 `joint8` 下限为 `-0.007 m`，与 URDF 的 `0 m` 不同，因此以实际 USD limit 记录，控制目标仍限制在 `[0,0.05] m`。
- 参考资料：`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.api/isaacsim/core/api/robots/robot.py`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/single_articulation.py`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py`；`Assets/Robots/piper/Piper.usd`。
- Commit message：`feat: validate dual Piper articulations`

## 2026-07-30 23:42 — 挂载并验证三路逻辑 D435 RGB-D 相机

- 目标：在两个 `link6/camera` helper 和顶部实体支架下创建三台可真实出图的 640×480、30 Hz 逻辑 D435，固化标定、深度语义、挂载层级与视野验收。
- 完成内容：实现 `create_cameras()`、相机 world look-at、RGB-D 统一读帧和 `validate_and_capture_cameras()`；腕部相机严格使用资产 `robot_config.yml` 指定的局部 X 轴 180° USD 光轴转换，顶部相机放入支架自带 D455 外壳并看向桌面中心；三路均设置 pinhole `f=1.93 mm`、水平 aperture `2.65 mm`、RGB uint8/RGB、image-plane depth float32/m、非有限深度转 NaN；验证 render product 的真实时间戳、内外参、裁剪、非空画面、深度范围、目标投影和 prim parent；保存 headed/headless RGB 与深度预览。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；`uv run python - <<'PY' ...` 读取双腕 helper world pose 和支架各 mesh world bbox；`uv run python test_dual_piper_sort.py --mode fast`；两次 `uv run python test_dual_piper_sort.py --mode integration --headless`；`uv run python dual_piper_sort.py --mode cameras --headless`；`uv run python dual_piper_sort.py --mode cameras`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check`。
- 验证结果：fast 7/7、最终集成 11/11 通过；headed/headless 均输出 `CAMERA_SMOKE_OK` 且退出码 0；三路实际 RGB shape/dtype 均为 `[480,640,3] uint8`，depth 均为 `[480,640] float32`、单位 m；三路时间戳中位周期均为 `0.033333335 s`；实际内参为 `fx=fy=466.1132,cx=320,cy=240`；左右腕目标像素约为 `[320,389.26]`、光轴对夹爪中心 cosine `0.95236`，顶部桌心约为 `[320,240]`、cosine 接近 1；顶部有效深度比例 100%，两腕各 33.33%，其余按契约转 NaN；目视确认腕部画面同时包含双指和前方桌面，顶部画面覆盖中央桌面与两臂边缘。
- 问题与处理：第一次集成测试的所有实现侧相机验收已经通过，但测试末尾用 `math.isclose` 默认的近零容差比较 `0.033333335` 与 `1/30`，造成单个假失败；失败退出时未调用 `SimulationApp.close()`，SyntheticData graph 在 Python atexit 又产生 shutdown crash。把测试容差改为与实现一致的 `5 ms` 后 11/11 正常通过并由 `close()` 干净退出。Camera 初始化时会报告默认 aperture 被调整为 4:3 方像素 aperture，这是设置明确 D435 aperture 前的正常中间状态，最终读回值和内参均已严格验证。
- 参考资料：`Assets/Robots/piper/robot_config.yml`；`Assets/Robots/piper/piper_description/urdf/piper.urdf`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.utils/isaacsim/core/utils/rotations.py`。
- Commit message：`feat: add three D435 RGB-D cameras`

## 2026-07-30 23:53 — 确定性生成五个套娃并验证刚体落稳

- 目标：只使用 `00000..00004` 五个同 UUID、不同尺寸的 USDZ，以显式 seed 生成直立、无重叠、非已排序的初始布局，并在真实 PhysX 中验证质量、摩擦、碰撞和稳定状态。
- 完成内容：实现尺寸感知的最终目标点计算，使占用外缘关于 `y=-0.05` 对称且相邻中心距离等于两半径加 `0.025 m`；实现最大每物体 500 次的 largest-first rejection sampling，随机 XY/yaw、不缩放不倾倒，限制中央随机区、桌边余量、机器人基座排除区和 `0.04 m` 初始表面间隙；统一设置 Python、NumPy、PyTorch、CUDA/cuRobo torch seed；引用五个真实 USDZ，在根刚体覆盖 metadata `0.05 kg`，给每个 `/collision/model` 绑定静/动摩擦 `0.45` 的 physics material；实现连续 30 步速度、倾角、桌面高度、边界和物间隙稳定检查；新增 `--mode dolls`，同时保存顶视 RGB-D 和场景总览，并验证顶部相机覆盖随机区四角与全部五个目标点。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；`uv run python test_dual_piper_sort.py --mode fast`；三次 `uv run python test_dual_piper_sort.py --mode integration --headless`；两次 `uv run python dual_piper_sort.py --mode dolls --seed 20260730 --headless`；两次 `uv run python dual_piper_sort.py --mode dolls --seed 20260730`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check`。
- 验证结果：fast 9/9、最终集成 14/14 通过；最终 headed/headless 均输出 `DOLL_SMOKE_OK` 且退出码 0；seed `20260730` 两种模式产生相同布局和物理指标；初始最小表面间隙 `0.08538 m`、最小桌边余量 `0.19872 m`、最小基座排除余量 `0.30912 m`，五个目标误差均远大于 `0.02 m`，确认初态非已排序；38 physics steps（`0.3167 s`）后达到连续 30 步稳定，最大倾角约 `0.02852°`、最大线速度约 `0.00204 m/s`、最大角速度约 `0.01339 rad/s`；五个刚体质量均读回约 `0.0500000007 kg`、动摩擦约 `0.449999988`，各有真实 convex collision `/collision/model`；顶置相机中随机区四角投影约落在 `u=98.8..547.0,v=52.7..335.9` 内，五个最终目标也全部在画面内；headed 总览目视确认五个尺寸正确、直立、分散且没有与双臂/桌边/支架相交。
- 问题与处理：首次物理集成的实现侧 mass 容差和全部稳定检查已通过，但测试用 `math.isclose` 默认精度比较 PhysX float32 `0.050000000745` 与 `0.05` 导致假失败，并在未 close 的失败退出路径再次触发 SyntheticData shutdown crash；将测试改为与实现相同的 `1e-7` 容差后正常通过。最终稳定状态保留轻微非零速度，不用强制写零制造假稳定，实际值均显著低于顶部阈值。
- 参考资料：五个 `Assets/Object/RoboDojo/Rigid/matryoshka_dolls/00000..00004/metadata.json` 和 `object.usdz`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/single_rigid_prim.py`；`third_parties/IsaacLab/source/isaaclab/isaaclab/sim/spawners/materials/physics_materials.py`；`third_parties/IsaacLab/source/isaaclab/isaaclab/sim/utils/prims.py`。
- Commit message：`feat: add deterministic matryoshka initialization`

## 2026-07-31 00:09 — 修正顶部支架、顶置相机与双臂收拢位

- 目标：严格采用用户给出的支架和杆顶相机世界位姿，保证支架底座落在桌面上且位于两臂之间，并让两台 Piper 保持资产定义的收拢等待状态。
- 完成内容：将静态支架根位置固定为 `[0,-0.47,0.765]`、姿态固定为归一化后的 `[0.707,-0.707,0,0]`，按旋转后的八个源包围盒角点验证世界包围盒，并明确拒绝任何后代 `RigidBodyAPI`；将顶置相机固定为 `[0,-0.41,1.308]` 和归一化后的 USD 相机姿态 `[0.9659258,0.2588190,0,0]`，直接读回验证其 USD-frame 世界位姿；将六轴 home 改为仓库 Piper 示例使用的全零收拢态，并在真实 PhysX 中验证关节误差和末端前向范围；在不改固定相机参数的前提下，把随机中心区收窄至 `x=[-0.22,0.22]、y=[-0.22,0.17]`，按最大娃娃的完整三维包围范围验证随机边界及全部目标位均完整入镜。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；headed 零位 monkeypatch 探针；`uv run python test_dual_piper_sort.py --mode fast`；`uv run python dual_piper_sort.py --mode scene --headless`；`uv run python dual_piper_sort.py --mode robots --headless`；三次 `uv run python dual_piper_sort.py --mode cameras --headless`；`uv run python dual_piper_sort.py --mode dolls --seed 20260730 --output-dir dual_piper_output/stand_pose_fix --headless`；`uv run python dual_piper_sort.py --mode dolls --seed 20260730 --output-dir dual_piper_output/stand_pose_fix`；`uv run python test_dual_piper_sort.py --mode integration --headless`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check`。
- 验证结果：fast 10/10、集成 15/15 通过；scene、robots、cameras、headed/headless dolls 均输出对应 `*_SMOKE_OK` 且退出码 0；支架世界包围盒约为 `[-0.15,-0.515,0.765]..[0.15,-0.402566,1.378153]`，有 12 个 collision prim、0 个 rigid body，底面精确落在桌面 `z=0.765`；顶置相机实际 USD 位姿约为 `[0,-0.409999996,1.307999969]`、`[0.965925872,0.258818984,0,0]`；两臂最终最大 home 误差约 `0.000080 rad`，末端从底座向桌面前伸约 `0.191426 m`；最大娃娃在随机区与全部目标位的完整边界至少保留约 `10.74 px` 画面余量；headed 总览目视确认支架竖直、底座位于两臂之间、两臂水平收拢，顶视 RGB 中五个娃娃均完整可见。
- 问题与处理：固定相机后，旧随机中心范围首先让底面角点投影到 `u=-9.84/649.84 px`；升级为完整娃娃边界检查后又发现旧 `y=0.28` 上界会让最大娃娃顶部投影到 `v=-33.36 px`。保留用户指定的相机位姿和 D435 内参，只缩小非固定随机采样区，随后完整三维边界检查通过。已有 MDL float-to-float2 与 Camera 初始 aperture 警告不影响最终读回参数和验收。
- 参考资料：用户提供的支架与相机精确位姿；`basic/piper.py`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py`。
- Commit message：`fix: place camera stand and retract Piper arms`

## 2026-07-31 00:48 — 在单个 Piper 上完成真实 cuRobo 抓取搬运

- 目标：使用当前环境实际安装的 cuRobo，在一个 Piper 上跑通带完整场景碰撞、真实关节下发、夹爪接触、附着物碰撞、抬升、搬运、放置和自动归位的端到端物理 smoke test。
- 完成内容：以内存字典定义六轴 Piper cuRobo robot config、保守 link collision spheres、自碰撞忽略对、锁定的主动夹爪关节和四个 attached-object sphere slots；把桌子、地面、支架包围盒、房间墙面、另一台 Piper 当前 FK 碰撞球以及未抓取套娃加入每次规划世界；使用 `MotionPlanner` 的位置 IK 生成有限候选，按工具 +X 向下及首选抓取姿态排序，再对每个候选调用真实 `plan_cspace`，明确按 joint name 提取六轴 30 Hz 插值轨迹。为避免 Isaac Sim 5.1 内置 Warp 1.8.2 与当前 cuRobo 所需 Warp 1.15.0 在同一解释器冲突，在同一个 `dual_piper_sort.py --mode planner-worker` 中实现常驻 JSON 管道规划进程，设置固定响应前缀和 60 秒超时，不新增第三个 Python/配置文件。实现 cuRobo attachment manager 的内缩双球娃体模型；夹爪真实闭合接触后，在当前相对位姿创建 PhysX `FixedJoint`，同时给 cuRobo 附着碰撞体，搬运和放置均保持抓取端姿态，释放后移除两种附着、退离并由 cuRobo 规划回零位。新增 `motion` 与 `pick` smoke mode，分别验证自由空间往返和 seed `20260730` 的 `00001` 单娃最终目标放置。
- 修改文件：`dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；多次 `uv run python - <<'PY' ...` 最小 cuRobo FK、自碰撞、IK、c-space、exact-pose、attached-sphere 和完整五段路径探针；`uv run python dual_piper_sort.py --mode motion --headless --planner-seed 1`；`uv run python dual_piper_sort.py --mode pick --headless --seed 20260730 --planner-seed 1`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`uv run python -m unittest -v test_dual_piper_sort.StaticAssetAndConstantTests`。
- 验证结果：隔离 worker 的纯 GPU 规划返回 121 帧真实轨迹；Isaac 自由空间往返的末端到达误差 `0.000208 m`，两段峰值关节跟踪误差约 `0.0473 rad`、末帧约 `0.0008 rad`。单娃完整执行输出 `CUROBO_PICK_PLACE_SMOKE_OK` 并退出码 0：预抓取/抓取/抬升/横移/放置均由 cuRobo 成功规划；夹爪中心到娃体中心误差 `0.028410 m`，小于该娃 `0.031065 m` 半径；物体实际抬升 `0.128654 m`；约束状态下放置误差 `0.002299 m`，释放落稳后最终误差 `0.001289 m`；七段轨迹峰值关节跟踪误差最大约 `0.0483 rad`，低于 `0.08 rad` 上限；物体最终直立稳定且双臂回到收拢零位。快速测试 10/10、语法编译通过；最终 headless 预览目视确认支架落桌、单娃移入中央目标且双臂归位。
- 问题与处理：直接在已启动 Isaac 的进程导入 cuRobo 首先因 Warp 1.8 的 `wp.func` 不接受 `module` 参数失败；反向预载 Warp 1.15 虽能创建规划器，却导致 Isaac 多个扩展缺少旧 `warp.types.array`、`np_dtype_to_warp_type` 和 `warp.context`，因此否定该方案并采用进程隔离。保守碰撞球最初在全零折叠态误报 link1/link2 与 link4/link5 相交，只对实测零位邻接对加入 ignore，其他自碰撞仍开启。精确竖直末端四元数受 Piper 腕限位影响不稳定，改为位置 IK 候选加真实 c-space 验证；抓取解若只追求最向下会落在腕限位并无法保持姿态抬升，最终让抓取解优先靠近已验证的预抓取姿态，端点姿态差约 `0.0964 rad`，随后精确姿态抬升/搬运均通过。attached sphere 与桌面恰好接触时会触发 `0.005 m` activation distance，给规划碰撞球相对实体半径内缩 `0.007 m`，保留实体碰撞和 PhysX 接触不变。一次使用 `pytest -m 'not integration'` 时因现有 unittest 集成类未使用 pytest marker 而误收集并报缺少未启动的 `omni`，改用测试文件既有的快速 unittest 类，10/10 通过。
- 参考资料：`third_parties/curobo/curobo/_src/motion/motion_planner.py`；`third_parties/curobo/curobo/_src/collision/attachment_manager.py`；`third_parties/curobo/curobo/_src/solver/solver_ik.py`；`third_parties/curobo/curobo/_src/types/tool_pose.py`；`.venv/lib/python3.11/site-packages/isaacsim/extscache/omni.warp.core-1.8.2+...`；`Assets/Robots/piper/piper_description/urdf/piper.urdf`。
- Commit message：`feat: execute cuRobo Piper pick and place`

## 2026-07-31 01:21 — 完成双臂顺序协作五物体排序

- 目标：让左右 Piper 都参与，以有限、确定、可记录的 cuRobo/PhysX 动作序列把五个随机套娃从小到大排到桌面中央宽度方向目标线，保持物体长期稳定并让双臂自动归位。
- 完成内容：把单娃抓放参数化为任意 asset/active arm；按初始 X 分配左右臂并保证两臂均有任务，seed `20260730` 的分配为左臂 `00004/00001`、右臂 `00003/00002/00000`。加入目标高位姿态探针、抓取中心补偿、64 个有限 IK 候选、附着体抬升/横移/下降的 exact-pose 首选和最大 9° bounded-orientation 回退；每次回退仍由真实 cuRobo position IK + `plan_cspace` 完成，随后以仿真实际 tool/object offset 重算后续目标，物体运输倾角和最终倾角分别严格检查。把抓取接触点提高到物体中心上方 `min(0.035, 0.35*height)`，避免斜抓时指爪碰桌；桌面释放只张到物体半径加每指 `0.006 m`，垂直退离后再完全张开，避免扫到邻居。根据实际目标邻域冲突选择有限顺序 `00004 → 00003 → 00002 → 00000 → 00001`，逐步打印所有已放物体的误差。为消除 0.05 kg 娃体在桌面接触上的长期数值抖动，在 `world.reset()` 后显式写入并读回 PhysX linear/angular damping `0.1`、sleep threshold `0.005`、stabilization threshold `0.001`；保留原有速度/倾角验收阈值。新增最终五目标、尺寸顺序、两臂参与和 home 验证，以及 cuRobo config/双臂分配的纯快速测试。
- 修改文件：`dual_piper_sort.py`、`test_dual_piper_sort.py`、`dual-piper-dev-log.md`。
- 运行命令：`git status --short`；多次纯 GPU `5 objects × 2 arms` 可达性矩阵、目标兼容姿态、64-seed 和 attachment 分段探针；多次 `uv run python dual_piper_sort.py --mode sort --headless --seed 20260730 --planner-seed 1` 有限诊断运行；`uv run python dual_piper_sort.py --mode dolls --headless --seed 20260730`；Isaac 内 `physxRigidBody:sleepThreshold` property-stack/reset 时序探针；最终同一 full-sort 命令；`uv run python -m unittest -v test_dual_piper_sort.StaticAssetAndConstantTests`；`uv run python -m py_compile dual_piper_sort.py test_dual_piper_sort.py`；`git diff --check`。
- 验证结果：最终 headless 运行输出 `CUROBO_FULL_SORT_OK` 并退出码 0；左右臂均参与，五个物体均完成真实闭合、附着碰撞规划、搬运、释放和归位。每步完成后的最终误差分别为：`00004=0.001349 m`、`00003=0.000260 m`、`00002=0.000282 m`、`00000=0.000470 m`、`00001=0.000326 m`，长期复检最大目标误差仍为 `0.001349 m`，远低于 `0.02 m`；最终五体线/角速度全部为 0，最大倾角为 `00004` 的 `7.05266°`，低于 `10°`；五个 Y 中心按 `00004..00000` 严格递增、X 均位于中央线误差内，两臂最终 home 误差通过。最终预览目视确认中央尺寸序列、支架落桌和两臂收拢。快速测试 12/12、语法和 whitespace 检查通过。
- 问题与处理：初版固定抓取姿态在右臂高位横移不可达；单纯改用目标兼容姿态会让斜抓在桌面碰撞，改为提高实体内接触带并扩大有限 IK 候选。原先先放大娃会使尚未抓的中央娃与已放目标只剩约 `0.016 m` 表面间隙，改为先清理目标邻域。exact-pose 在部分抬升/横移/下降段受 Piper 腕限位拒绝，加入最大 9° 的有限位置回退并在每段后用实际倾角复核；`00004` 最终运输倾角约 `7.05°`。最初五体长期复检因根 prim 的 sleep threshold 为 0 持续接触抖动；首次在 `world.reset()` 前 `Create...Attr` 会被 `SingleRigidPrim/world.reset` 重写，经属性栈探针确认后改为 reset 后 `.Set`，实际读回 `0.0049999999/0.00100000005` 并进入睡眠。最初 `00003 → 00004` 顺序会因后者斜姿态下降把前者推移约 `0.022 m`；交换为 `00004 → 00003` 后两者以及后续所有已放物体误差不再变化。一轮验证在第三物体处收到外部 SIGTERM 143，日志无 Python/CUDA/PhysX 错误且无残留 GPU 进程，原样重跑后完整通过。
- 参考资料：`third_parties/curobo/curobo/_src/motion/motion_planner.py`；`third_parties/curobo/curobo/_src/collision/attachment_manager.py`；`third_parties/IsaacLab/source/isaaclab/isaaclab/sim/schemas/schemas.py`；`third_parties/IsaacLab/source/isaaclab/isaaclab/sim/schemas/schemas_cfg.py`；`.venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/single_rigid_prim.py`。
- Commit message：`feat: sort all dolls with both Piper arms`
