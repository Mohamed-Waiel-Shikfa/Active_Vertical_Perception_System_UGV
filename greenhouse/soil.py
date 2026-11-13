# SPDX-License-Identifier: GPL-3.0-only
"""Local soil albedo with cavity tint registered to the actual collision surface."""
from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np

SOIL_LINEAR_RGB = np.array((0.34, 0.24, 0.135))
TEXTURE_SIZE = 2048

if TYPE_CHECKING:
    from greenhouse.terrain import Terrain


def write_soil_texture(terrain: "Terrain", seed: int, path: Path | str) -> dict[str, Any]:
    """Write a ``TEXTURE_SIZE``-square sRGB PNG covering the whole terrain grid; return its manifest entry.

    Image column 0 is the -x edge and row 0 the +y edge of the grid. Raises OSError if the PNG
    cannot be written.
    """
    heights = terrain.heights
    neighborhood = cv2.GaussianBlur(
        heights, (0, 0), 0.35 / terrain.resolution, borderType=cv2.BORDER_REFLECT_101)
    # Darker settled soil in real hollows, lighter soil on real crowns. This is
    # an albedo/cavity cue, not a normal map or a second displacement surface.
    relative_height = heights - neighborhood
    tint = 1 + 0.12 * np.tanh(relative_height / 0.008)
    coordinate = np.linspace(0, len(heights) - 1, TEXTURE_SIZE, dtype=np.float32)
    x, y = np.meshgrid(coordinate, coordinate)
    tint = cv2.remap(tint, x, y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    rng = np.random.default_rng(seed ^ 0x534F494C)
    grain = rng.standard_normal((TEXTURE_SIZE, TEXTURE_SIZE)).astype(np.float32)
    grain = cv2.GaussianBlur(grain, (0, 0), 0.65)
    grain /= grain.std()
    tint *= 1 + 0.045 * np.tanh(grain)
    tint /= tint.mean()
    linear = np.clip(tint[:, :, None] * SOIL_LINEAR_RGB, 0, 1)
    srgb = np.where(linear <= 0.0031308, 12.92 * linear,
                    1.055 * np.power(linear, 1 / 2.4) - 0.055)
    # PNG rows run downward; mesh UV v runs along world +Y.
    rgb = np.flipud(np.rint(255 * srgb).astype(np.uint8))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise OSError(f"failed to save local soil texture: {path}")
    return {
        "size_pixels": TEXTURE_SIZE,
        "metres_per_pixel": (len(heights) - 1) * terrain.resolution / (TEXTURE_SIZE - 1),
        "linear_mean_rgb": linear.mean(axis=(0, 1)).tolist(),
        "cavity_neighborhood_m": 0.35,
        "cavity_tint_strength": 0.12,
        "grain_strength": 0.045,
        "normal_map": False,
    }


def soil_uv(vertices: Sequence[tuple[float, float, float]],
            terrain: "Terrain") -> list[tuple[float, float]]:
    """UV per world-frame vertex (metres): u grows with +x, v with +y, both in [0, 1] at texel centres."""
    span = (len(terrain.heights) - 1) * terrain.resolution
    # Map the unique whole-field atlas to texel centres, without repeat seams.
    return [
        ((x - terrain.origin) / span * (TEXTURE_SIZE - 1) / TEXTURE_SIZE + 0.5 / TEXTURE_SIZE,
         (y - terrain.origin) / span * (TEXTURE_SIZE - 1) / TEXTURE_SIZE + 0.5 / TEXTURE_SIZE)
        for x, y, _ in vertices
    ]
