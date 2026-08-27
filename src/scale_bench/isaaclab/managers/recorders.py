"""Build native Isaac Lab recorder configuration from pure settings."""

from __future__ import annotations

from dataclasses import MISSING
from pathlib import Path

import isaaclab.envs.mdp as mdp
from isaaclab.envs.mdp.recorders.recorders_cfg import (
    InitialStateRecorderCfg,
    PostStepProcessedActionsRecorderCfg,
    PostStepStatesRecorderCfg,
    PreStepActionsRecorderCfg,
)
from isaaclab.managers import (
    DatasetExportMode,
    ObservationTermCfg,
    RecorderManagerBaseCfg,
    RecorderTermCfg,
)
from isaaclab.utils.configclass import configclass

from scale_bench.config.models.recording import RecordingConfig
from scale_bench.isaaclab.managers.observations import ObservationsCfg
from scale_bench.isaaclab.mdp.observations import camera_image, gripper_joint_pos
from scale_bench.isaaclab.mdp.recorders import PolicyObservationsRecorder, SemanticEventsRecorder

_EXPORT_MODES = {
    "all": DatasetExportMode.EXPORT_ALL,
    "succeeded_only": DatasetExportMode.EXPORT_SUCCEEDED_ONLY,
    "succeeded_failed_separate": (
        DatasetExportMode.EXPORT_SUCCEEDED_FAILED_IN_SEPARATE_FILES
    ),
}

@configclass
class PolicyObservationsRecorderCfg(RecorderTermCfg):
    """Configuration for named, dictionary-valued policy observations."""

    class_type: type[PolicyObservationsRecorder] = PolicyObservationsRecorder
    observation_names: tuple[str, ...] = MISSING


@configclass
class SemanticEventsRecorderCfg(RecorderTermCfg):
    """Configuration for per-frame skill and command text."""

    class_type: type[SemanticEventsRecorder] = SemanticEventsRecorder


@configclass
class RecordersCfg(RecorderManagerBaseCfg):
    """Recorder terms enabled for one ScaleBench environment."""

    initial_state: InitialStateRecorderCfg | None = None
    actions: PreStepActionsRecorderCfg | None = None
    processed_actions: PostStepProcessedActionsRecorderCfg | None = None
    policy_observations: PolicyObservationsRecorderCfg | None = None
    scene_state: PostStepStatesRecorderCfg | None = None
    semantic_events: SemanticEventsRecorderCfg | None = None


def build_recorders_cfg(
    recording_config: RecordingConfig | None,
    observations_cfg: ObservationsCfg,
) -> RecorderManagerBaseCfg:
    """Compile optional recording settings into a native recorder cfg."""

    if recording_config is None:
        return RecorderManagerBaseCfg(
            dataset_export_mode=DatasetExportMode.EXPORT_NONE
        )

    output_dir = recording_config.output_dir.expanduser().resolve()
    dataset_name = _resolve_dataset_name(recording_config, output_dir)
    policy_observation_names = _policy_observation_names(
        observations_cfg,
        include_joints=recording_config.record_joint_observations,
        include_cameras=recording_config.record_camera_observations,
    )
    return RecordersCfg(
        dataset_export_dir_path=str(output_dir),
        dataset_filename=dataset_name,
        dataset_export_mode=_EXPORT_MODES[recording_config.export_mode],
        export_in_record_pre_reset=False,
        export_in_close=False,
        dataset_compression=recording_config.compression,
        initial_state=(
            InitialStateRecorderCfg()
            if recording_config.record_initial_state
            else None
        ),
        actions=(
            PreStepActionsRecorderCfg()
            if recording_config.record_actions
            else None
        ),
        processed_actions=(
            PostStepProcessedActionsRecorderCfg()
            if recording_config.record_processed_actions
            else None
        ),
        policy_observations=(
            PolicyObservationsRecorderCfg(
                observation_names=policy_observation_names
            )
            if policy_observation_names
            else None
        ),
        scene_state=(
            PostStepStatesRecorderCfg()
            if recording_config.record_scene_state
            else None
        ),
        semantic_events=(
            SemanticEventsRecorderCfg()
            if recording_config.record_semantic_events
            else None
        ),
    )


def _policy_observation_names(
    observations_cfg: ObservationsCfg,
    *,
    include_joints: bool,
    include_cameras: bool,
) -> tuple[str, ...]:
    names = []
    for name, term_cfg in vars(observations_cfg.policy).items():
        if not isinstance(term_cfg, ObservationTermCfg):
            continue
        if term_cfg.func is camera_image:
            if include_cameras:
                names.append(name)
        elif term_cfg.func in {mdp.joint_pos, gripper_joint_pos}:
            if include_joints:
                names.append(name)
    return tuple(names)


def _resolve_dataset_name(
    recording_config: RecordingConfig,
    output_dir: Path,
) -> str:
    if output_dir.exists() and not output_dir.is_dir():
        raise NotADirectoryError(f"recording output is not a directory: {output_dir}")
    if recording_config.overwrite_existing:
        return recording_config.dataset_name

    suffix = 0
    while True:
        candidate = recording_config.dataset_name
        if suffix > 0:
            candidate = f"{candidate}_{suffix}"
        dataset_names = [candidate]
        if recording_config.export_mode == "succeeded_failed_separate":
            dataset_names.append(f"{candidate}_failed")
        if not any(
            (output_dir / f"{dataset_name}.hdf5").exists()
            for dataset_name in dataset_names
        ):
            return candidate
        suffix += 1


__all__ = [
    "PolicyObservationsRecorderCfg",
    "RecordersCfg",
    "SemanticEventsRecorderCfg",
    "build_recorders_cfg",
]
