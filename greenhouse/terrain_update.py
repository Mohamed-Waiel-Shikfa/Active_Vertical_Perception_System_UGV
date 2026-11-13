# SPDX-License-Identifier: GPL-3.0-only
"""Validation shared by runtime transactions and their focused regression tests."""
import math
from dataclasses import asdict
from typing import TYPE_CHECKING, Any

from greenhouse.config import Config

if TYPE_CHECKING:
    from greenhouse.terrain import Terrain

RUNTIME_FIELDS = (
    "ground_resolution", "ground_height_range", "ground_headland",
    "ground_bump_rms", "ground_bump_scale", "ground_friction",
    "slip_longitudinal", "slip_lateral",
)


def proposed_config(current: dict[str, Any], values: dict[str, Any]) -> dict[str, Any]:
    """Merge terrain/contact ``values`` over the ``current`` config dict and return the validated result.

    Raises ValueError for any key outside ``RUNTIME_FIELDS`` or any value ``Config.validate`` rejects.
    """
    if not isinstance(values, dict) or set(values) - set(RUNTIME_FIELDS):
        raise ValueError("Only the eight listed terrain/contact fields can be applied")
    result = dict(current)
    result.update(values)
    return asdict(Config(**result).validate())


def support_points(robot: dict[str, Any]) -> list[tuple[float, float]]:
    """World-frame ``(x, y)`` metres of an 11 x 7 grid spanning 0.60 x 0.54 m around the robot.

    ``robot`` holds world ``x``/``y`` in metres and ``yaw`` in radians; the grid is laid out in the
    body frame (+x forward, +y left) and rotated by ``yaw``.
    """
    # Covers chassis, wheels/fenders and the inboard rear-deck payload.
    yaw = robot["yaw"]
    return [(robot["x"] + dx * math.cos(yaw) - dy * math.sin(yaw),
             robot["y"] + dx * math.sin(yaw) + dy * math.cos(yaw))
            for dx in (-0.30, -0.24, -0.18, -0.12, -0.06, 0, 0.06, 0.12, 0.18, 0.24, 0.30)
            for dy in (-0.27, -0.18, -0.09, 0, 0.09, 0.18, 0.27)]


def validate_support(surface: "Terrain", config: dict[str, Any], robot: dict[str, Any]) -> None:
    """Check ``surface`` covers the robot footprint and the house corners plus 0.1 m (world frame).

    ``config`` is a config dict (metres). Raises ValueError from ``Terrain.sample`` if any
    point is off the grid.
    """
    for x, y in support_points(robot):
        surface.sample(x, y)
    for x in (-config["house_length"] / 2 - 0.1, config["house_length"] / 2 + 0.1):
        for y in (-config["house_width"] / 2 - 0.1, config["house_width"] / 2 + 0.1):
            surface.sample(x, y)
