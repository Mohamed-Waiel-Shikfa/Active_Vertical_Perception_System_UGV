# SPDX-License-Identifier: GPL-3.0-only
import argparse
import hashlib
import json
import math
import shutil
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from greenhouse.config import Config, load_config
from greenhouse.mesh import Mesh, read_baked_vertices, write_dae
from greenhouse.plants import tomato_mesh
from greenhouse.soil import soil_uv, write_soil_texture
from greenhouse.terrain import generate_terrain
from greenhouse.world import greenhouse_world, plant_model, terrain_model, write_xml

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "third_party" / "virtual_maize_field"


def verify_upstream() -> dict[str, Any]:
    """Check the vendored virtual_maize_field files against ``provenance.json``; return that JSON.

    Raises ValueError if any file's SHA-256 differs.
    """
    provenance = json.loads((UPSTREAM / "provenance.json").read_text(encoding="utf-8"))
    for filename, expected in provenance["files"].items():
        actual = hashlib.sha256((UPSTREAM / "upstream" / filename).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"vendored upstream asset changed: {filename} ({actual})")
    return provenance


def generate(config: Config, output: Path | str) -> dict[str, Any]:
    """Generate all world assets under ``output`` and return the manifest written to ``manifest.json``.

    Positions in the manifest are world-frame metres (house centre origin, +x along rows, +z up)
    and yaws radians. Raises ValueError on an invalid config or failed check, OSError on write failure.
    """
    config.validate()
    provenance = verify_upstream()
    output = Path(output)
    ground_dir = output / "models" / "greenhouse_ground"
    plant_dir = output / "models" / "greenhouse_tomato"
    terrain = generate_terrain(config)
    ground_vertices, ground_faces = terrain.mesh()
    soil_info = write_soil_texture(
        terrain, config.seed, ground_dir / "materials" / "textures" / "soil.png")
    write_dae(ground_dir / "meshes" / "terrain.dae",
              [("soil", Mesh(ground_vertices, ground_faces, soil_uv(ground_vertices, terrain)),
                (1, 1, 1, 1), "../materials/textures/soil.png")])
    terrain_model(ground_dir, config.ground_friction)
    leaf_dir = UPSTREAM / "upstream" / "models" / "unknown_weed"
    parts, plant_info = tomato_mesh(leaf_dir / "meshes" / "unknown_weed.dae",
                                   config.plant_height, config.plant_radius)
    plant_mesh = plant_dir / "meshes" / "tomato.dae"
    write_dae(plant_mesh, parts)
    textures = plant_dir / "materials" / "textures"
    textures.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(leaf_dir / "materials" / "textures" / "unknown_weed.png", textures / "leaf.png")
    measured = read_baked_vertices(plant_mesh)
    measured_radius = float(np.linalg.norm(measured[:, :2], axis=1).max())
    collision_radius = math.ceil((measured_radius - 1e-8) * 1000000) / 1000000
    measured_height = float(np.ptp(measured[:, 2]))
    if abs(measured_height - config.plant_height) > 1e-8:
        raise ValueError("baked tomato height does not match requested height")
    if config.row_spacing - 2 * collision_radius < 1.2 - 1e-8:
        raise ValueError("measured canopy collision leaves less than 1.2 m clear")
    plant_model(plant_dir, measured_height, collision_radius)
    plant_info.update({
        "bounds_min_m": measured.min(axis=0).tolist(),
        "bounds_max_m": measured.max(axis=0).tolist(),
        "height_m": measured_height,
        "max_horizontal_radius_m": measured_radius,
        "collision_radius_m": collision_radius,
        "sdf_mesh_scale": [1, 1, 1],
    })
    rng = np.random.default_rng(config.seed + 1)
    placements = []
    for index, (x, y) in enumerate(config.plant_positions()):
        yaw = float(rng.uniform(-math.pi, math.pi))
        y_values = measured[:, 0] * math.sin(yaw) + measured[:, 1] * math.cos(yaw) + y
        placements.append({
            "name": f"tomato_{index:03d}", "x": x, "y": y, "z": terrain.sample(x, y), "yaw": yaw,
            "foliage_min_y": float(y_values.min()), "foliage_max_y": float(y_values.max()),
        })
    row_size = len(placements) // config.row_count
    row_bounds = []
    for row in range(config.row_count):
        values = placements[row * row_size:(row + 1) * row_size]
        row_bounds.append((
            min(min(p["foliage_min_y"], p["y"] - collision_radius) for p in values),
            max(max(p["foliage_max_y"], p["y"] + collision_radius) for p in values),
        ))
    clearances = [row_bounds[i + 1][0] - row_bounds[i][1] for i in range(config.row_count - 1)]
    write_xml(output / "worlds" / "greenhouse.sdf", greenhouse_world(config, placements))
    if not cv2.imwrite(str(ground_dir / "heightmap.png"), terrain.pixels):
        raise OSError("failed to save the upstream terrain heightmap")
    heights = {
        "origin": terrain.origin, "resolution": terrain.resolution,
        "heights": terrain.heights.tolist(),
    }
    (output / "terrain.json").write_text(json.dumps(heights, separators=(",", ":")), encoding="utf-8")
    # Sample the entire padded standard-Jackal footprint, not only its centre.
    footprint = [terrain.sample(config.spawn_x + dx, config.spawn_y + dy)
                 for dx in np.linspace(-0.52, 0.52, 13)
                 for dy in np.linspace(-0.52, 0.52, 13)]
    surface_vertices = np.asarray(ground_vertices)[:terrain.heights.size]
    report = {
        "schema_version": 1,
        "world": "greenhouse",
        "config": asdict(config),
        "upstream_revision": provenance["revision"],
        "terrain": {
            "algorithm": "Field2DGenerator.generate_ground (unchanged method) + wheel-scale band relief",
            "upstream_shape": list(terrain.pixels.shape),
            "bump_rms_m": config.ground_bump_rms,
            "bump_scale_m": config.ground_bump_scale,
            "shape": list(terrain.heights.shape),
            "resolution_m": terrain.resolution,
            "surface_bounds_min_m": surface_vertices.min(axis=0).tolist(),
            "surface_bounds_max_m": surface_vertices.max(axis=0).tolist(),
            "height_range_m": float(np.ptp(terrain.heights)),
            "standard_deviation_m": float(terrain.heights.std()),
            "triangles": len(ground_faces),
            "collision_matches_visual": True,
            "appearance": soil_info,
        },
        "plant": plant_info,
        "layout": {
            "plant_count": len(placements), "clear_aisles_m": clearances,
            "clear_side_aisle_m": config.side_clearance,
            "clear_headland_m": config.end_clearance,
            "clear_door_width_m": config.door_width - 0.08,
        },
        "spawn": {"x": config.spawn_x, "y": config.spawn_y,
                  "z": max(footprint) + config.spawn_clearance, "yaw": config.spawn_yaw},
        "plants": placements,
    }
    report["files_sha256"] = {
        path.relative_to(output).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.rglob("*")) if path.is_file() and path.name != "manifest.json"
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    """CLI entry point: generate from ``--config`` into ``--output``; exits with status 2 on failure."""
    parser = argparse.ArgumentParser(description="Generate local greenhouse assets using pinned virtual_maize_field terrain.")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "greenhouse.json")
    parser.add_argument("--output", type=Path, default=ROOT / "generated")
    args = parser.parse_args()
    try:
        report = generate(load_config(args.config), args.output)
    except (ValueError, OSError) as error:
        parser.exit(2, f"greenhouse generation failed: {error}\n")
    print(json.dumps({key: report[key] for key in ("world", "terrain", "plant", "layout", "spawn")}, indent=2))


if __name__ == "__main__":
    main()
