# SPDX-License-Identifier: GPL-3.0-only
from dataclasses import dataclass
from types import SimpleNamespace
from typing import TYPE_CHECKING

import cv2
import numpy as np

from third_party.virtual_maize_field.terrain import Field2DGenerator

if TYPE_CHECKING:
    from greenhouse.config import Config


@dataclass
class Terrain:
    """Square height grid in the world frame (metres, +z up).

    ``heights[j, i]`` (metres) sits at world ``(origin + i * resolution, origin + j * resolution)``,
    so the row index grows with +y. ``pixels`` is upstream's uint8 image before relief
    refinement, with image rows top-down (row 0 is the +y edge).
    """
    pixels: np.ndarray
    heights: np.ndarray
    resolution: float
    origin: float

    def sample(self, x: float, y: float) -> float:
        """Surface height in metres at world ``(x, y)`` metres, on the same triangles as the collision mesh.

        Raises ValueError if the point lies outside the grid.
        """
        u, v = (x - self.origin) / self.resolution, (y - self.origin) / self.resolution
        n = self.heights.shape[0]
        if not (0 <= u <= n - 1 and 0 <= v <= n - 1):
            raise ValueError(f"position ({x}, {y}) lies outside the terrain mesh")
        i, j = min(int(u), n - 2), min(int(v), n - 2)
        a, b = u - i, v - j
        h00, h10 = self.heights[j, i], self.heights[j, i + 1]
        h01, h11 = self.heights[j + 1, i], self.heights[j + 1, i + 1]
        # Exactly the same diagonal as the emitted collision triangles.
        if b <= a:
            return float(h00 * (1 - a) + h10 * (a - b) + h11 * b)
        return float(h00 * (1 - b) + h11 * a + h01 * (b - a))

    def mesh(self) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        """Return ``(vertices, triangles)``: world-frame vertices in metres and vertex-index triples.

        The first ``n * n`` vertices are the surface grid (row-major, +x fastest, top faces wound
        counter-clockwise seen from +z); the rest close the sides and bottom at z = -0.2 m.
        """
        n = self.heights.shape[0]
        vertices = [
            (self.origin + x * self.resolution, self.origin + y * self.resolution,
             float(self.heights[y, x]))
            for y in range(n) for x in range(n)
        ]
        triangles = []
        for y in range(n - 1):
            for x in range(n - 1):
                a = y * n + x
                triangles.extend(((a, a + 1, a + n + 1), (a, a + n + 1, a + n)))
        # Close the perimeter below the lowest surface; there is no flat upper collider.
        rim = list(range(n)) + [y * n + n - 1 for y in range(1, n)]
        rim += list(range(n * n - 2, n * (n - 1) - 1, -1))
        rim += [y * n for y in range(n - 2, 0, -1)]
        bottom = len(vertices)
        for index in rim:
            x, y, _ = vertices[index]
            vertices.append((x, y, -0.2))
        center = len(vertices)
        vertices.append((0, 0, -0.2))
        for k, top in enumerate(rim):
            next_k = (k + 1) % len(rim)
            triangles.extend(((top, bottom + k, bottom + next_k),
                              (top, bottom + next_k, rim[next_k]),
                              (center, bottom + next_k, bottom + k)))
        return vertices, triangles


def wheel_scale_relief(heights: np.ndarray, resolution: float, rms: float,
                       scale: float) -> tuple[np.ndarray, float]:
    """Refine a metre height grid to <= 0.075 m spacing and add ~``rms`` m relief at ``scale`` m.

    ``resolution`` is the input grid spacing in metres. Returns ``(heights, spacing_m)``, unchanged
    when ``rms`` is 0. Raises ValueError if the refined grid exceeds 512 samples a side or the
    input has no wheel-scale variation.
    """
    if rms == 0:
        return heights, resolution
    subdivisions = max(1, int(np.ceil(resolution / 0.075)))
    size = (len(heights) - 1) * subdivisions + 1
    if size > 512:
        raise ValueError("refined terrain exceeds the 512x512 mesh budget")
    coordinates = np.arange(size, dtype=np.float32) / subdivisions
    x, y = np.meshgrid(coordinates, coordinates)
    refined = cv2.remap(heights, x, y, cv2.INTER_CUBIC, borderMode=cv2.BORDER_REFLECT_101)
    spacing = resolution / subdivisions

    def smooth(sigma):
        return cv2.GaussianBlur(refined, (0, 0), sigma / spacing,
                                borderType=cv2.BORDER_REFLECT_101)

    # Upstream's coarse octaves dominate its range. Recover irregular patches
    # from its own samples at wheel/axle scales, not a new random height field.
    patches = smooth(scale) - smooth(3 * scale)
    detail = smooth(0.55 * scale) - smooth(1.25 * scale)
    patch_rms, detail_rms = float(patches.std()), float(detail.std())
    if min(patch_rms, detail_rms) <= 1e-12:
        raise ValueError("upstream terrain has no usable wheel-scale height variation")
    relief = patches / patch_rms + 0.25 * detail / detail_rms
    relief = 2.5 * rms * np.tanh(relief / 2.5)
    surface = smooth(0.09) + relief
    surface += heights.mean() - surface.mean()
    return surface, spacing


def generate_terrain(config: "Config") -> Terrain:
    """Build the seeded ground under the house with upstream ``Field2DGenerator`` plus wheel-scale relief.

    The grid is centred on the world origin (house centre). Raises ValueError if it exceeds the
    512x512 budget or does not cover the whole greenhouse.
    """
    params = {
        "ground_ditch_depth": config.ground_height_range / 2,
        "ground_elevation_max": config.ground_height_range,
        "ground_headland": config.ground_headland,
        "ground_resolution": config.ground_resolution,
    }
    # ditch_depth = elevation/2 makes upstream's field_height exactly zero.
    # This removes the outdoor perimeter ditch, not the multiscale roughness.
    wd = SimpleNamespace(structure={"params": params}, rng=np.random.default_rng(config.seed))
    generator = Field2DGenerator(wd)
    positions = np.asarray(config.plant_positions(), dtype=float)
    generator.rows = list(np.split(positions, config.row_count))
    generator.object_placements = np.empty((0, 2))
    span = max(np.ptp(positions[:, 0]), np.ptp(positions[:, 1])) + 2 * config.ground_headland + 1
    if 2 ** np.ceil(np.log2(np.ceil(span / config.ground_resolution))) > 512:
        raise ValueError("terrain exceeds the 512x512 grid budget; increase ground_resolution")
    generator.generate_ground()
    heights = np.flipud(generator.heightmap).astype(float) * generator.heightmap_elevation / 255
    origin = -(len(heights) - 1) * generator.resolution / 2
    if max(config.house_length, config.house_width) / 2 + 0.1 > -origin:
        raise ValueError("ground_headland must be increased to support the entire greenhouse")
    heights, resolution = wheel_scale_relief(
        heights, generator.resolution, config.ground_bump_rms, config.ground_bump_scale)
    return Terrain(generator.heightmap, heights, float(resolution), float(origin))
