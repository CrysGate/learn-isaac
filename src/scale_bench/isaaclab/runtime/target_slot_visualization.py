"""Viewport-only target-slot line geometry."""

from __future__ import annotations

import math

Point = tuple[float, float, float]
Line = tuple[Point, Point]
Color = tuple[float, float, float, float]
LineGroup = tuple[list[Line], Color, float]

TARGET_SLOT_COLORS: tuple[Color, ...] = (
    (0.0, 0.8, 1.0, 1.0),
    (0.2, 1.0, 0.35, 1.0),
    (1.0, 0.85, 0.0, 1.0),
    (1.0, 0.35, 0.1, 1.0),
    (1.0, 0.2, 0.75, 1.0),
)


def target_slot_line_groups(
    target_positions_m: tuple[Point, ...],
    env_origins_m: tuple[Point, ...],
    table_top_z_m: float,
) -> list[LineGroup]:
    """Build colored tabletop rings and crosses for target centers."""

    z_m = table_top_z_m + 0.006
    radius_m = 0.008
    segment_count = 24
    groups: list[LineGroup] = []
    for slot_index, (x_m, y_m, _) in enumerate(target_positions_m):
        slot_lines: list[Line] = []
        for origin in env_origins_m:
            center = (origin[0] + x_m, origin[1] + y_m, origin[2] + z_m)
            ring: list[Point] = []
            for index in range(segment_count):
                angle = 2.0 * math.pi * index / segment_count
                ring.append(
                    (
                        center[0] + radius_m * math.cos(angle),
                        center[1] + radius_m * math.sin(angle),
                        center[2],
                    )
                )
            slot_lines.extend(
                (ring[index], ring[(index + 1) % segment_count])
                for index in range(segment_count)
            )
            slot_lines.extend(
                (
                    (
                        (center[0] - radius_m, center[1], center[2]),
                        (center[0] + radius_m, center[1], center[2]),
                    ),
                    (
                        (center[0], center[1] - radius_m, center[2]),
                        (center[0], center[1] + radius_m, center[2]),
                    ),
                )
            )
        color = TARGET_SLOT_COLORS[slot_index % len(TARGET_SLOT_COLORS)]
        groups.append((slot_lines, color, 5.0))
    return groups


__all__ = [
    "Color",
    "Line",
    "Point",
    "target_slot_line_groups",
]
