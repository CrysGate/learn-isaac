# Scripts

以下命令均在项目根目录运行。

## `preview_scene.py`

在 Isaac Sim 中预览公共场景：

```bash
uv run python scripts/preview_scene.py
```

预览指定任务，并使用固定 seed 生成布局：

```bash
uv run python scripts/preview_scene.py \
  --task sort_dolls_by_size \
  --seed 44
```

无界面冒烟测试可以使用：

```bash
uv run python scripts/preview_scene.py \
  --task sort_dolls_by_size \
  --seed 44 \
  --viz none \
  --max-steps 10
```

脚本还支持通过 `--layout` 加载布局、通过 `--export-layout` 导出布局，
以及通过 `--config`、`--sim-config` 和 `--env-config` 指定配置文件。

## `export_hdf5_camera_videos.py`

从录制的 HDF5 episode 中导出 RGB 和伪彩色深度 MP4：

```bash
uv run python scripts/export_hdf5_camera_videos.py \
  outputs/datasets/piper_pick.hdf5 \
  --demo demo_0 \
  --camera right_robot
```

`--camera` 可选 `left_robot`、`right_robot` 或 `overhead`。默认输出目录为
`<HDF5 文件名>_videos/`，也可以通过 `--output-dir` 指定。输出帧率默认从
HDF5 元数据推导。

如需固定深度视频的显示范围：

```bash
uv run python scripts/export_hdf5_camera_videos.py \
  outputs/datasets/piper_pick.hdf5 \
  --depth-min-m 0.1 \
  --depth-max-m 3.0
```

深度 MP4 仅用于可视化；原始 `float32` 米制深度仍保存在 HDF5 中。
运行任一脚本时添加 `--help` 可以查看全部参数。
