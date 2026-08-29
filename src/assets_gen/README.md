# OBJ to USD Asset Conversion

[简体中文](README.zh-CN.md)

`convert_obj_to_usd.py` uses Isaac Sim's Asset Converter to process OBJ assets
in batches and generate standardized USD files for rigid-body simulation. It
can also read mass and dimensions from XLSX metadata, scale each `x/y/z` axis
independently, and export the aligned geometry as OBJ files.

## Requirements

The script requires Isaac Sim 6.0.1 and the project dependencies. It starts
`SimulationApp` in headless mode. The `pxr`, `omni`, and `pymeshlab` modules
must therefore be imported only after the Isaac Sim runtime is initialized.

```bash
uv run python src/assets_gen/convert_obj_to_usd.py \
  --assets-root ./assets \
  --folders ./assets/vase ./assets/mug \
  --metadata-xlsx ./assets/sample.xlsx \
  --output-root ./converted-obj
```

When `--folders` is omitted, the script scans `<assets-root>/vase`. When
`--metadata-xlsx` is omitted, it reads `<assets-root>/sample.xlsx`. The default
OBJ output directory is `<assets-root>/../rigid assets`. All defaults are
portable relative paths rather than machine-specific user paths.

## Processing Pipeline

1. Optionally extract ZIP archives under the input folders. Git LFS pointers
   are skipped, and archive members are checked for path traversal.
2. Recursively discover OBJ files and deduplicate them by resolved path.
3. Use MeshLab to simplify meshes that exceed `--target-faces`.
4. Generate USD with Asset Converter, measure the source AABB, and match the
   corresponding metadata record.
5. Create `/root/{_materials,visual,collision}`. Source hierarchy transforms,
   physical dimension scaling, and up-axis alignment are baked into the visual
   and collision meshes. The baked mesh AABB is then centered on `/root`, so
   the rigid-body root represents the geometry center.
6. Author rigid-body properties, mass, and `scale_x/scale_y/scale_z` on
   `/root`; apply PhysX convex-decomposition colliders to the collision mesh;
   then author `real_x/real_y/real_z` on `/root`.
7. Save `Aligned.usd` and export its aligned geometry as `Aligned.obj`.

Each source directory receives an `Aligned.usd`. Exported OBJ files preserve
the directory layout relative to `--assets-root`, for example:

```text
assets/vase/001/model.obj
assets/vase/001/Aligned.usd
converted-obj/vase/001/Aligned.obj
```

## Metadata Format

Only the first worksheet is read. The first row must contain headers using the
following supported names:

- Name: `name`, `object`, `名称`, or `物体`
- Mass: `mass(kg)`, `mass(g)`, `mass`, `weight`, `重量`, or `质量`
- Dimensions: `x`, `y`, and `z`

The name cell may be left empty on consecutive rows; the previous non-empty
name is reused. Dimensions are interpreted as centimeters by default. Change
this with `--dimension-unit m|cm|mm`. The mass unit is inferred from the column
header first, with `--metadata-mass-unit auto|g|kg` as the fallback.

When metadata is unavailable, the script uses `--mass` (default `0.1 kg`) and
`--scale` (default `1.0`). Each axis is resolved independently, so a missing
dimension falls back only for that axis.

## Options

| Option | Description |
| --- | --- |
| `--assets-root PATH` | Asset root; defaults to `assets` |
| `--folders PATH ...` | Directories to scan |
| `--metadata-xlsx PATH` | Metadata workbook |
| `--output-root PATH` | Root directory for exported OBJ files |
| `--target-faces N` | MeshLab face limit; defaults to `1000` |
| `--mass KG` | Fallback mass when metadata is unavailable |
| `--scale VALUE` | Fallback scale when dimension metadata is unavailable |
| `--force` | Replace an existing `Aligned.usd` |
| `--max-models N` | Maximum successful conversions; `0` means unlimited |
| `--extract-zips` | Recursively extract ZIP files before conversion |

`--scale-axis` remains available for backward compatibility. Scaling is always
computed independently for all three axes, and passing a value other than
`auto` produces a deprecation warning.

## Limitations

- Asset Converter is configured to merge meshes. If conversion does not
  produce exactly one mesh, processing stops for that asset to avoid creating
  an invalid collider.
- `Aligned.usd` and `Aligned.obj` form one conversion result. A failed reverse
  OBJ export marks the asset as failed rather than reporting partial success.
- A failed asset does not stop the batch. The final summary reports discovered,
  converted, skipped, and failed counts.
