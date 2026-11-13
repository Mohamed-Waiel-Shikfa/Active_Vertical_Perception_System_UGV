# SPDX-License-Identifier: GPL-3.0-only
import math
from pathlib import Path
from typing import Any

import numpy as np

from greenhouse.mesh import Mesh, read_leaf_mesh


def tomato_mesh(
    source: Path | str, height: float, radius: float,
) -> tuple[list[tuple[str, Mesh, tuple[float, ...], str | None]], dict[str, Any]]:
    """Build the tomato plant from the leaf asset at ``source``, baked to ``height`` and ``radius`` metres.

    Plant frame: origin at the stem base, +z up. Returns ``(parts, info)``: ``write_dae`` parts
    as ``(name, mesh, rgba, texture)`` and a dict of build measurements for the manifest.
    """
    source_vertices, leaves = read_leaf_mesh(source)
    source_min = float(source_vertices[:, 2].min())
    source_height = float(np.ptp(source_vertices[:, 2]))
    source_radius = float(np.linalg.norm(source_vertices[:, :2], axis=1).max())
    foliage, stems, fruit, calyces = Mesh(), Mesh(), Mesh(), Mesh()
    crowns = ((0.25, 0.16, 1.0, 0.0), (0.55, 0.18, 0.92, 1.4),
              (0.85, 0.19, 0.76, 2.7), (1.17, 0.23, 0.54, 4.1))
    for bottom, crown_height, width, yaw in crowns:
        for points, uv, direction in leaves:
            transformed = points.copy()
            radial = (points[:, :2] @ direction)[:, None] * direction
            transformed[:, :2] = radial * 0.75 + (points[:, :2] - radial) * 2.2
            transformed[:, :2] *= 0.35 * width / source_radius
            x, y = transformed[:, 0].copy(), transformed[:, 1].copy()
            transformed[:, 0] = x * math.cos(yaw) - y * math.sin(yaw)
            transformed[:, 1] = x * math.sin(yaw) + y * math.cos(yaw)
            transformed[:, 2] = bottom + (points[:, 2] - source_min) * crown_height / source_height
            foliage.triangle(transformed, uv, double_sided=True)
    stems.tube((0, 0, 0), (0, 0, 1.36), 0.012, sides=10)
    fruit_count = 0
    for level, yaw in ((0.50, 0.5), (0.70, 2.2), (0.91, 3.8), (1.12, 5.1)):
        arm = np.array((0.19 * math.cos(yaw), 0.19 * math.sin(yaw), level + 0.1))
        stems.tube((0, 0, level + 0.02), arm, 0.006)
        for i in range(3):
            side = (i - 1) * 0.063
            center = (arm[0] - side * math.sin(yaw), arm[1] + side * math.cos(yaw),
                      level - (0.045 if i == 1 else 0.0))
            fruit.fruit(center, 0.046)
            stems.tube(arm, (center[0], center[1], center[2] + 0.04), 0.0025, sides=5)
            for leaf in range(5):
                theta = leaf * 2 * math.pi / 5
                calyces.triangle(((center[0], center[1], center[2] + 0.045),
                                 (center[0] + 0.035 * math.cos(theta),
                                  center[1] + 0.035 * math.sin(theta), center[2] + 0.036),
                                 (center[0] + 0.008 * math.cos(theta + 1),
                                  center[1] + 0.008 * math.sin(theta + 1), center[2] + 0.042)),
                                double_sided=True)
            fruit_count += 1
    parts = [
        ("foliage", foliage, (0.2, 0.43, 0.08, 1), "../materials/textures/leaf.png"),
        ("stems", stems, (0.20, 0.34, 0.08, 1), None),
        ("tomatoes", fruit, (0.85, 0.045, 0.015, 1), None),
        ("calyces", calyces, (0.10, 0.31, 0.025, 1), None),
    ]
    all_vertices = np.vstack([mesh.vertices for _, mesh, _, _ in parts])
    raw_height = float(np.ptp(all_vertices[:, 2]))
    raw_radius = float(np.linalg.norm(all_vertices[:, :2], axis=1).max())
    base = float(all_vertices[:, 2].min())
    for _, mesh, _, _ in parts:
        values = np.asarray(mesh.vertices)
        values[:, :2] *= radius / raw_radius
        values[:, 2] = (values[:, 2] - base) * height / raw_height
        mesh.vertices = values.tolist()
    return parts, {
        "source_height_m_after_scene_scale": source_height,
        "source_radius_m_after_scene_scale": source_radius,
        "crowns": len(crowns),
        "leaf_blades_per_crown": len({tuple(direction) for _, _, direction in leaves}),
        "leaf_radial_scale": 0.75,
        "leaf_width_scale": 2.2,
        "fruit_count": fruit_count,
        "final_bake_scale": [radius / raw_radius, radius / raw_radius, height / raw_height],
        "triangles": sum(len(mesh.faces) for _, mesh, _, _ in parts),
    }
