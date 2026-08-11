"""Build native rigid-object cfgs for metadata-backed tasks."""

from __future__ import annotations

from collections.abc import Mapping

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg

from scale_bench.tasks.common.layout import AssetPlacement, TaskLayout
from scale_bench.tasks.common.rigid_object import RigidObjectTask
from scale_bench.tasks.common.task import Task


class RigidObjectTaskBuilder:
    """Convert pure task data into fresh Isaac Lab rigid-object cfgs."""

    def build_assets(
        self,
        task: Task,
        layout: TaskLayout,
    ) -> Mapping[str, RigidObjectCfg]:
        if not isinstance(task, RigidObjectTask):
            raise TypeError("RigidObjectTaskBuilder requires a RigidObjectTask")
        task.validate_asset_layout(layout)
        return {
            name: self._build_asset_cfg(task, name, layout.assets[name])
            for name in task.assets
        }

    @staticmethod
    def _build_asset_cfg(
        task: RigidObjectTask,
        name: str,
        placement: AssetPlacement,
    ) -> RigidObjectCfg:
        asset = task.assets[name]
        metadata = task.metadata[name]
        physics = task.config.physics
        return RigidObjectCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Task/Objects/{name}",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=placement.position_m,
                rot=placement.orientation_xyzw,
            ),
            spawn=sim_utils.UsdFileCfg(
                usd_path=asset.usd_path,
                mass_props=sim_utils.MassPropertiesCfg(mass=metadata.mass),
                rigid_props=sim_utils.PhysxRigidBodyPropertiesCfg(
                    linear_damping=physics.linear_damping,
                    angular_damping=physics.angular_damping,
                    sleep_threshold=physics.sleep_threshold,
                    stabilization_threshold=physics.stabilization_threshold,
                ),
                physics_material=RigidBodyMaterialBaseCfg(
                    static_friction=metadata.friction,
                    dynamic_friction=metadata.friction,
                    restitution=physics.restitution,
                ),
            ),
        )


__all__ = ["RigidObjectTaskBuilder"]
