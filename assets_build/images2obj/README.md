# 腾讯混元 3D 自动化生成

基于 Playwright 实现的网页端自动化工具，用于将图像批量生成并下载为 3D 模型资产。

## ✨ 功能特性 (Features)

- [x] **登录态持久化**：首次扫码后自动记录并保存登录上下文，后续运行免扫码。
- [x] **全自动工作流**：支持自动上传正视图、背视图、左视图、右视图（目录中存在对应文件时会上传），触发生成任务，并等待下载最终的 `.obj` 模型文件。
- [ ] **并发处理 (TODO)**：引入多线程/异步机制，支持多组资产的批量并发生成，进一步提升效率。

## 📦 快速安装 (Installation)

本项目依赖于 [Playwright](https://playwright.tw/python/docs/intro) 进行浏览器自动化驱动。请在你的 Python 环境中执行以下命令：

```bash
# 1. 安装 Playwright 核心库
pip install playwright

# 2. 安装所需的浏览器内核及底层依赖
playwright install
```

### 「Host system is missing dependencies」与堆栈输出

`playwright install` 结束时若出现 **Playwright Host validation warning** 以及一段 Node 堆栈，通常表示：

- **浏览器已下载完成**（安装命令仍可能以退出码 `0` 结束）；
- 本机 **缺少运行 Chromium 所需的系统动态库**（GTK、X11、字体等），首次 `launch()` 可能报错。

**推荐修复（需要 root，且需与 `pip install playwright` 使用同一 Python）：**

```bash
# 只装 Chromium 相关系统依赖，体积与改动相对最小
sudo "$(command -v python3)" -m playwright install-deps chromium
```

若 `python3` 不是装 Playwright 的那个解释器，请改成你的 conda 路径，例如：

```bash
sudo /path/to/miniconda3/bin/python -m playwright install-deps chromium
```

**RHEL / Rocky / AlmaLinux 9 等（无 apt）**：官方提示里的 `apt-get` 不适用。优先仍用上面的 `install-deps`（Playwright 会按发行版调用 `dnf`/`yum`）。若无法使用 `sudo`，可请管理员安装与 Chromium/GTK3 相关的常见依赖包（如 `gtk3`、`libXcomposite`、`mesa-libgbm`、`alsa-lib`、`atk`、`pango` 等）。

**仅在确认系统库已齐、但校验误报时使用**（跳过校验，不安装缺失库）：

```bash
export PLAYWRIGHT_SKIP_VALIDATE_HOST_REQUIREMENTS=1
playwright install   # 或运行你的脚本
```

若跳过校验后浏览器仍启动失败，说明确实缺库，请回到 `install-deps`。