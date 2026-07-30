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
