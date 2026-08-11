"""Rules and layout behavior for sorting nesting dolls by physical size."""

from __future__ import annotations

from typing import ClassVar

from scale_bench.tasks.common.rigid_object import RigidObjectTask

from .config import DollAssetConfig, SortDollsBySizeConfig


class SortDollsBySize(RigidObjectTask):
    """Place nesting dolls and expose their target size ordering."""

    TASK_ID: ClassVar[str] = "sort_dolls_by_size"

    def __init__(self, config: SortDollsBySizeConfig) -> None:
        assets = {f"doll_{doll.asset_id}": doll for doll in config.dolls}
        super().__init__(config, assets)
        self._dolls_by_name: dict[str, DollAssetConfig] = assets

        heights = [metadata.size[2] for metadata in self.metadata.values()]
        if len(heights) != len(set(heights)):
            raise ValueError("doll asset heights must be unique")

    @property
    def target_order_small_to_large(self) -> tuple[str, ...]:
        """Return asset IDs ordered by metadata height."""

        names = sorted(
            self.assets,
            key=lambda name: self.metadata[name].size[2],
        )
        return tuple(self._dolls_by_name[name].asset_id for name in names)


__all__ = ["SortDollsBySize"]
