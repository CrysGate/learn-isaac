"""Single-object pick-and-place task package."""

from .config import SingleObjectPickAndPlaceConfig
from .task import SingleObjectPickAndPlace

__all__ = [
    "SingleObjectPickAndPlace",
    "SingleObjectPickAndPlaceConfig",
]
