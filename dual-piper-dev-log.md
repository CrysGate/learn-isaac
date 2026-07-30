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
