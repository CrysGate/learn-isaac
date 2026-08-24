# assets_build 使用说明

本文档说明 `assets_build/` 目录中现有脚本和资产的用途，以及从输入图片生成多视图图片、再生成 3D 模型的基本流程。

## 目录作用

`assets_build/` 当前更像一个资产生成流水线原型，主要完成：

1. 从单张物体图片生成前、后、左、右四张视图。
2. 使用 Hunyuan3D 多视图模型从四视图生成白模。
3. 使用 Hunyuan3D 贴图模型生成带纹理的 3D 模型。

主要文件如下：

- `main.py`：批量遍历待处理图片，并调用多视图生成接口。
- `multi_view_api/multi_view.py`：OpenAI 兼容 image-to-image 接口封装，用于生成四视图图片。
- `multi_view_api/mulyi_view_3d.py`：读取四视图图片，调用 Hunyuan3D 生成 3D 模型。
- `utils/checkpoints_download.py`：下载 Hunyuan3D 模型权重的辅助脚本。
- `img_wait_process/`：待处理输入图片目录。
- `assets/`：多视图输出目录。
- `demo_white_mesh_mv.*`：示例白模输出。
- `demo_textured_mv.*`、`material.mtl`、`material_0.png`：示例贴图模型输出。

## 输入目录规范

`main.py` 依赖固定的相对路径，因此应从 `assets_build/` 目录内运行。

输入图片需要放在：

```text
assets_build/img_wait_process/<类别>/<变体>/<图片文件>
```

例如当前仓库已有：

```text
assets_build/img_wait_process/cola/300/300ml_cola.png
assets_build/img_wait_process/cola/500/500ml_cola.png
```

每个 `<类别>/<变体>/` 目录下可以放一张或多张 `jpg`、`jpeg` 或 `png` 图片。

## 环境准备

当前代码依赖以下几类环境：

- Python 3。
- `openai`：用于调用 OpenAI 兼容图片生成接口。
- `pydantic`：部分 OpenAI SDK 版本会依赖。
- `huggingface_hub`：用于下载模型权重。
- `torch`、`torchvision`：Hunyuan3D 推理依赖。
- `Pillow`：读取图片。
- `hy3dgen`：Hunyuan3D 的 Python 包。
- OpenGL / Mesa 相关系统库：Hunyuan3D 贴图和渲染依赖可能需要。

原始 `Readme.md` 中给出的安装步骤是作者环境备忘，包含固定绝对路径和镜像源。实际使用时需要按当前机器环境调整。

建议从仓库根目录进入 `assets_build/`（不要用写死的机器绝对路径）：

```bash
cd assets_build
```

然后按本机环境安装依赖。示例：

```bash
pip install openai pydantic huggingface_hub pillow
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
```

Hunyuan3D 相关依赖需要依赖完整的 Hunyuan3D 源码。当前仓库中的 `assets_build/Hunyuan3D-2` 是一个 gitlink 占位，但缺少 `.gitmodules`，本地目录也是空的。因此如果要运行 3D 生成流程，需要先补齐 Hunyuan3D 源码和依赖。

## 密钥和模型路径

源码中目前仍存在硬编码的 API key、HF token 等。`utils/checkpoints_download.py` 已默认将 `HF_HOME` 与下载目录指向**仓库根目录**下的 `checkpoints_download/`（仍可用环境变量覆盖）。实际使用时建议把密钥改成环境变量，不要把真实密钥写入代码或文档。

推荐约定：

```bash
export HF_ENDPOINT="https://hf-mirror.com"
export HF_TOKEN="<your_huggingface_token>"
export HF_HOME="${HF_HOME:-$PWD/checkpoints_download}"
export HY3DGEN_MODELS="/path/to/hunyuan3d/models"
export IMAGE_API_BASE_URL="<your_openai_compatible_base_url>"
export IMAGE_API_KEY="<your_image_api_key>"
export IMAGE_API_MODEL="<your_image_generation_model>"
```

当前代码尚未读取 `IMAGE_API_*` 环境变量。如果要按上述方式使用，需要先调整 `multi_view_api/multi_view.py` 中的 `DEFAULT_BASE_URL`、`DEFAULT_MODEL`、`DEFAULT_API_KEY`。

## 第一步：生成四视图图片

确认输入图片已经按目录规范放好后，从 `assets_build/` 目录运行：

```bash
cd assets_build
python3 main.py
```

脚本会遍历：

```text
img_wait_process/<类别>/<变体>/
```

并输出到：

```text
assets/<类别>/<变体>/<图片文件名>_views/
```

每个样本会生成：

```text
<图片文件名>_front.png
<图片文件名>_back.png
<图片文件名>_left.png
<图片文件名>_right.png
```

如果这四个方向文件都已经存在，`main.py` 会自动跳过该图片。

当前生成顺序是：

1. 用原始图片生成正视图 `front.png`。
2. 用正视图生成背视图 `back.png`。
3. 用正视图和背视图生成左视图 `left.png`。
4. 用正视图和背视图生成右视图 `right.png`。

## 第二步：下载或准备 Hunyuan3D 权重

如果使用 Hugging Face 下载权重，可以参考：

```bash
cd assets_build
python3 utils/checkpoints_download.py
```

需要注意：

- 该脚本默认把 `HF_HOME` 与下载目录设为仓库根下的 `checkpoints_download/`（可用环境变量覆盖）。
- 默认下载 `tencent/Hunyuan3D-2.1`。
- 如果本机无法访问 Hugging Face，需要配置镜像或提前离线准备权重。
- 运行前应把 token 和路径改成自己的环境配置。

## 第三步：生成 3D 模型

`multi_view_api/mulyi_view_3d.py` 当前写死读取旧版示例路径：

```text
assets/cola/300/multi_views/front.png
assets/cola/300/multi_views/left.png
assets/cola/300/multi_views/right.png
assets/cola/300/multi_views/back.png
```

从 `assets_build/` 目录运行：

```bash
cd assets_build
python3 multi_view_api/mulyi_view_3d.py
```

脚本会：

1. 读取四张视图图片。
2. 加载 `tencent/Hunyuan3D-2mv` 的多视图形状生成 pipeline。
3. 使用固定随机种子 `12345` 生成白模。
4. 导出 `demo_white_mesh_mv.glb`。
5. 加载 `tencent/Hunyuan3D-2` 的贴图 pipeline。
6. 使用多视图图片生成贴图。
7. 导出 `demo_textured_mv.glb`。

如需处理其他类别或变体，需要修改脚本中的 `images` 字典。例如改为 `cola/500`：

```python
images = {
    "front": "./assets/cola/500/multi_views/front.png",
    "left": "./assets/cola/500/multi_views/left.png",
    "right": "./assets/cola/500/multi_views/right.png",
    "back": "./assets/cola/500/multi_views/back.png",
}
```

如果使用新版 `main.py` 生成的输出，也需要把文件名改成带图片名前缀的路径，例如
`assets/cola/500/500ml_cola_views/500ml_cola_front.png`。
`multi_view_3d.py` 已支持从这类带前缀的四视图目录中自动识别
`*_front.png`、`*_left.png`、`*_right.png` 和 `*_back.png`。

## 当前仓库已有示例

输入图片：

```text
img_wait_process/cola/300/300ml_cola.png
img_wait_process/cola/500/500ml_cola.png
```

多视图输出：

```text
assets/cola/300/300ml_cola_views/300ml_cola_front.png
assets/cola/300/300ml_cola_views/300ml_cola_back.png
assets/cola/300/300ml_cola_views/300ml_cola_left.png
assets/cola/300/300ml_cola_views/300ml_cola_right.png
assets/cola/500/500ml_cola_views/500ml_cola_front.png
assets/cola/500/500ml_cola_views/500ml_cola_back.png
assets/cola/500/500ml_cola_views/500ml_cola_left.png
assets/cola/500/500ml_cola_views/500ml_cola_right.png
```

3D 示例输出：

```text
demo_white_mesh_mv.glb
demo_white_mesh_mv.obj
demo_textured_mv.glb
demo_textured_mv.obj
material.mtl
material_0.png
```

## 已知问题和注意事项

- `multi_view_api/multi_view.py` 中直接硬编码了图片生成接口地址、模型名和 API key，建议改为读取环境变量。
- `utils/checkpoints_download.py` 中直接硬编码了 Hugging Face token 和本机绝对路径，建议改为读取环境变量。
- `main.py` 依赖当前工作目录，建议始终从 `assets_build/` 目录运行。
- `multi_view_api/mulyi_view_3d.py` 当前只处理 `assets/cola/300/multi_views/`，不是通用批处理脚本。
- `multi_view_api/mulyi_view_3d.py` 中图片先被转换为 `RGBA`，后续 `if image.mode == 'RGB'` 分支不会触发，背景去除逻辑实际不会执行。
- `multi_view_api/multi_view.py` 文件底部的 `__main__` 示例调用参数名不匹配，直接运行该文件会报错。日常应通过 `main.py` 调用。
- `assets_build/Hunyuan3D-2` 是 gitlink，但仓库没有 `.gitmodules`，需要手动补齐对应源码后才能按原始 README 中的 Hunyuan3D 示例运行。
- 仓库中包含已生成的模型、图片、`.DS_Store` 和 `__pycache__`，如果要长期维护，建议后续整理 `.gitignore`。

## 推荐的后续整理方向

为了让这套流程更容易复现，可以进一步做以下改造：

1. 把 API key、HF token、模型路径改为环境变量读取。
2. 给 `mulyi_view_3d.py` 增加命令行参数，例如 `--class cola --variant 300`。
3. 把 3D 生成输出放到 `assets/<类别>/<变体>/models/`，避免所有样本覆盖同一组 `demo_*` 文件。
4. 补齐 Hunyuan3D 子模块或在 README 中明确安装来源。
5. 增加 `requirements.txt` 或 `environment.yml`，固定可复现依赖。
