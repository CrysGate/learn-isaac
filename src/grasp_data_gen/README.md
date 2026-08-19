# Grasp data generator

[English](README.md) | [简体中文](README.zh-CN.md)

Generate antipodal grasp candidates in Isaac Sim, filter them for tabletop use,
and validate each feasible candidate with close-and-hold physics. The default
recipe targets the Piper gripper and assembles it directly from the full robot
USD, so no separate gripper asset is required.

## Pipeline

1. Assemble the configured gripper links and joints from the robot USD.
2. Sample object-local antipodal TCP poses.
3. Reject poses that violate approach, support-plane, or wrist-camera constraints.
4. Close and hold the gripper in simulation; reject empty, uneven, or unstable grasps.
5. Export complete diagnostics and accepted grasps sorted by score.

Gripper assembly, sampling, tabletop filtering, and validation thresholds are
defined in [`piper.yml`](piper.yml). Robot semantics and the source USD come
from [`configs/robots/piper.yml`](../../configs/robots/piper.yml).

## Generate grasps

Install the project as described in the [repository README](../../README.md),
then run from the repository root:

```bash
uv run python -m grasp_data_gen.generate_grasps \
  --object-usd Assets/Object/Rigid/matryoshka_dolls/00000/object.usdz \
  --output-dir outputs/grasp_data/piper/00000
```

Generation is headless by default. Use `--gui` to watch the physics evaluation.
The CLI also accepts `--num-orientations` and `--seed`; run it with `--help` for
the current defaults.

Use `--object-dir` instead of `--object-usd` to process a directory of objects:

```bash
uv run python -m grasp_data_gen.generate_grasps \
  --object-dir Assets/Object/Rigid/matryoshka_dolls \
  --output-dir outputs/grasp_data/piper
```

The directory is searched recursively for `.usd`, `.usda`, `.usdc`, and `.usdz`
files, and each object gets an independent output subdirectory. For example,
`00000/object.usdz` maps to `00000/`, while a flat `mug.usd` maps to `mug/`.
The output directory must be outside the input directory so generated stages
cannot become inputs on a later run. Failure of one object does not stop the
remaining batch; the command prints a summary at the end and returns a non-zero
exit code if any object failed.

## Outputs

- `evaluation_stage.usda`: the assembled gripper/object evaluation stage.
- `report.yaml`: every candidate, evaluation metric, and rejection reason.
- `successful_grasps.yaml`: accepted grasps sorted by descending score, including
  closed-gripper link poses used by the viewer.

Stored TCP poses are `T_object_tcp`, in metres, with quaternions ordered as
`xyzw`. Convert one to the runtime world frame with:

```text
T_world_tcp = T_world_object @ T_object_tcp
```

Generation checks gripper/object contact and tabletop feasibility only. Full-arm
IK, approach-path collisions, and task-specific constraints still need to be
checked after placing the object in the runtime scene.

## Visualize results

Inspect accepted grasps without rerunning generation:

```bash
uv run python -m grasp_data_gen.visualize_grasps \
  --grasp-file outputs/grasp_data/piper/00000/successful_grasps.yaml
```

The viewer opens the highest-scoring grasp. Use the previous/next controls to
step through results, or `All` to overlay every accepted pose.
