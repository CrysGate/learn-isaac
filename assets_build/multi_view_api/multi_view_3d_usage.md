# `multi_view_3d.py` 使用说明

## 1. 脚本作用

`multi_view_3d.py` 用腾讯混元 3D 的 OpenAI 兼容接口，把一张参考图生成 3D 模型，并把结果整理到统一的资产目录中。

当前脚本支持：

- 单张图片生成 3D
- 批量处理多个商品 / 多个规格
- 默认 `3` 并发提交任务
- 自动把结果落到 `assets/<class>/<scale>/3D/`
- 自动整理 `OBJ / MTL / 纹理`

这个脚本现在走的是**单视图图生 3D**，不再使用 `MultiViewImages`。

## 2. 依赖和前置条件

脚本本身只依赖 Python 标准库，不要求额外安装第三方包。

运行前需要准备：

- 可用的混元 3D API Key
- 已经能访问 `https://api.ai3d.cloud.tencent.com`

先设置环境变量：

```bash
export HUNYUAN3D_API_KEY='你的_API_KEY'
```

如果你想覆盖默认接口地址，也可以设置：

```bash
export HUNYUAN3D_BASE_URL='https://api.ai3d.cloud.tencent.com'
```

## 3. 输入规则

脚本的核心参数是：

```bash
--input-path
```

它支持以下几种输入形式。

### 3.1 单张图片

例如：

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola/300/300ml_cola.png
```

### 3.2 一个规格目录

例如：

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola/300
```

如果目录里只有一张图片，就直接处理这张图。

### 3.3 一个类别目录

例如：

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola
```

这时脚本会递归找到里面每个规格目录下的图片，并批量处理。

### 3.4 整个待处理根目录

例如：

```bash
python multi_view_3d.py --input-path ../img_wait_process
```

这时会递归处理所有类别和规格。

### 3.5 已经生成过多视图的目录

例如：

```bash
python multi_view_3d.py --input-path ../assets/cola/300/multi_views
```

这时脚本会优先取 `front.png/jpg/jpeg/webp` 作为单视图输入。

### 3.6 同时传多个输入

例如：

```bash
python multi_view_3d.py \
  --input-path ../img_wait_process/cola ../img_wait_process/apple
```

脚本会把这几个输入展开后一起处理。

## 4. 输入发现逻辑

当 `--input-path` 指向一个目录时，脚本按下面顺序找图片：

1. 如果目录下有 `multi_views/`，就继续去这个子目录里找。
2. 如果目录下有 `front.*`，优先用它。
3. 如果目录下直接只有一张图片，就用这张。
4. 如果目录下直接有多张图片，就把这些图片都当作独立任务。
5. 如果目录下没有图片，就递归扫描子目录。

支持的图片后缀：

- `.png`
- `.jpg`
- `.jpeg`
- `.webp`

## 5. 输出目录规则

默认不传 `--output-root` 时，脚本会自动把结果放到 `assets/<class>/<scale>/3D/`。

例如：

```text
img_wait_process/cola/300/300ml_cola.png
```

会自动输出到：

```text
assets/cola/300/3D/
```

如果输入本来就在 `assets` 目录下，例如：

```text
assets/cola/300/multi_views/front.png
```

输出仍然会放到：

```text
assets/cola/300/3D/
```

如果你想把所有输出重定向到别的根目录，可以显式指定：

```bash
python multi_view_3d.py \
  --input-path ../img_wait_process/cola \
  --output-root ../tmp_assets
```

这时类似结果会落到：

```text
../tmp_assets/cola/300/3D/
../tmp_assets/cola/500/3D/
```

## 6. 输出文件结构

每个规格目录下的 `3D/` 目录里，脚本会尽量整理出下面这些文件：

```text
assets/cola/300/3D/
  300ml_cola_3d.obj
  300ml_cola_3d.mtl
  300ml_cola_material_0.png
```

说明：

- `<原图名>_3d.obj`：主模型文件
- `<原图名>_3d.mtl`：材质文件
- `<原图名>_material_0.png` 等：纹理图片，数量可能不止一张

这样即使同一个 `scale/3D/` 目录下同时写入多个任务，也不会因为文件名相同而互相覆盖。

如果接口返回的是 `OBJ zip`，脚本会自动解压、重命名并修正材质引用。脚本会忽略额外的 `glb`、预览图、单独附件等旁路文件，不会把它们落到输出目录。

## 7. 并发规则

脚本默认：

```bash
--max-concurrency 3
```

也就是同时最多提交 `3` 个任务。

例如：

```bash
python multi_view_3d.py --input-path ../img_wait_process --max-concurrency 3
```

如果要串行跑，可以改成：

```bash
python multi_view_3d.py --input-path ../img_wait_process --max-concurrency 1
```

注意：

- 脚本层面支持 3 并发
- 真正能否同时跑满，还受腾讯混元 3D 账号并发额度影响
- 如果接口侧默认并发就是 3，那么这个设置是合理的默认值

## 8. 常用命令

### 9.1 先只看会处理哪些任务

```bash
python multi_view_3d.py --input-path ../img_wait_process --dry-run
```

这个命令不会真正提交 API，只会打印：

- 一共发现多少任务
- 每个任务的输入图
- 每个任务会输出到哪个目录

建议批量跑之前先执行一次。

### 9.2 单张图生成 3D

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola/300/300ml_cola.png
```

### 9.3 整个类别批量处理

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola
```

### 9.4 整个根目录批量处理

```bash
python multi_view_3d.py --input-path ../img_wait_process
```

### 9.5 只生成白模

```bash
python multi_view_3d.py \
  --input-path ../img_wait_process/cola \
  --generate-type Geometry
```

### 9.6 指定并发数

```bash
python multi_view_3d.py \
  --input-path ../img_wait_process \
  --max-concurrency 3
```

## 9. 主要参数说明

### `--input-path`

一个或多个输入路径。可以是：

- 单张图片
- 一个规格目录
- 一个类别目录
- 整个 `img_wait_process`
- `assets/.../multi_views`

### `--output-root`

可选。指定一个新的输出根目录。

### `--model`

可选值：

- `3.0`
- `3.1`

默认是：

```text
3.1
```

### `--generate-type`

可选值：

- `Normal`
- `Geometry`
- `LowPoly`
- `Sketch`

当前脚本默认是：

```text
Normal
```

注意：

- `3.1` 不支持 `LowPoly`
- `3.1` 不支持 `Sketch`

### `--face-count`

控制目标面数，默认：

```text
500000
```

### `--enable-pbr`

开启 PBR 相关输出。

### `--submit-timeout`

提交任务的 HTTP 超时时间，默认：

```text
300
```

### `--query-timeout`

查询任务状态的 HTTP 超时时间，默认：

```text
120
```

### `--timeout`

单个任务从提交到等待完成的总超时时间，默认：

```text
1800
```

### `--max-concurrency`

脚本层面的最大并发任务数，默认：

```text
3
```

### `--dry-run`

只预览任务，不真正提交。

## 10. 推荐使用方式

### 场景一：先看任务，再正式跑

```bash
python multi_view_3d.py --input-path ../img_wait_process --dry-run
python multi_view_3d.py --input-path ../img_wait_process --max-concurrency 3
```

### 场景二：只跑某一个类

```bash
python multi_view_3d.py --input-path ../img_wait_process/apple
```

### 场景三：只重跑某一个规格

```bash
python multi_view_3d.py --input-path ../img_wait_process/apple/300
```

### 场景四：用已经生成的 `front.png` 重跑 3D

```bash
python multi_view_3d.py --input-path ../assets/apple/300/multi_views
```

## 11. 常见问题

### Q1：为什么传一个目录也能跑？

因为脚本会自动展开目录，去找里面的单张图、`front.*` 或更深层子目录里的图片。

### Q2：为什么建议先 `--dry-run`？

因为批量任务一多，先确认“发现到了哪些任务”和“输出会写到哪里”更安全，尤其是你一次处理整个 `img_wait_process` 时。

### Q3：如果一个任务失败，会不会影响全部？

不会立即中断其他已提交任务。脚本会继续等其他任务完成，最后统一汇总失败项，并以非零退出码结束。

### Q4：为什么目录里最后是 `<原图名>_3d.obj` 而不是原始下载文件名？

脚本会把接口返回的 `OBJ zip` 解压并整理成统一命名，方便后续程序按固定规则读取，不用再猜文件名。

### Q5：如果接口没有返回 OBJ 怎么办？

脚本会报错：

```text
No OBJ result found in response
```

这代表当前接口返回格式和脚本预期不一致，需要再看接口回包。

## 12. 一个完整示例

假设目录结构是：

```text
img_wait_process/
  cola/
    300/
      300ml_cola.png
    500/
      500ml_cola.png
```

先预览：

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola --dry-run
```

再正式跑：

```bash
python multi_view_3d.py --input-path ../img_wait_process/cola --max-concurrency 3
```

跑完后结果会整理成：

```text
assets/
  cola/
    300/
      3D/
        300ml_cola_3d.obj
        300ml_cola_3d.mtl
        300ml_cola_material_0.png
    500/
      3D/
        500ml_cola_3d.obj
        500ml_cola_3d.mtl
        500ml_cola_material_0.png
```
