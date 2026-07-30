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
