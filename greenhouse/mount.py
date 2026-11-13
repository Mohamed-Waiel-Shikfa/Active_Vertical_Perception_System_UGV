# SPDX-License-Identifier: GPL-3.0-only
"""Prototype rear-DECK bracket, exact slots and conservative 1 mm gusset steps."""
import argparse
import json
import struct
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from greenhouse.actuator import load_actuator


def bracket_mesh(pitch: float = 120.0,
                 clearance: float = 0.4) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    """Voxel-surface mesh of the rear-DECK bracket; every length, `pitch` and `clearance` included, is in mm.

    Frame: origin at the 150 x 150 mm plate centre on its underside, +Z up, matching the fpb30_mount link.
    Returns (vertices, triangles): vertex coordinates and vertex-index triples.
    """
    bolt = (5.0 + clearance) / 2
    rail = (4.0 + clearance) / 2
    xs = sorted(set(np.arange(-75.0, 76.0).tolist() +
                    [s * pitch / 2 + d for s in (-1, 1) for d in (-6, 6)]))
    ys = sorted(set([-75., -25., -22., -19., 19., 22., 25., 75.] +
                    [s * pitch / 2 + d for s in (-1, 1) for d in (-bolt, bolt)] +
                    [s * 8.5 + d for s in (-1, 1) for d in (-rail, rail)]))
    zs = sorted(set(np.arange(0., 199.).tolist() + [28-rail, 28+rail, 166-rail, 166+rail]))
    x, y, z = np.meshgrid((np.array(xs[:-1])+xs[1:])/2,
                          (np.array(ys[:-1])+ys[1:])/2,
                          (np.array(zs[:-1])+zs[1:])/2, indexing="ij")
    material = z < 8
    material |= (x >= -60) & (x <= -50) & (abs(y) <= 22) & (z < 198)
    material |= (x >= -60) & (x <= -5) & (abs(y) >= 19) & (abs(y) <= 25) & (z < 8 + (-5-x)*2.8)
    for sx in (-1, 1):
        for sy in (-1, 1):
            material &= ~((abs(x-sx*pitch/2) < 6) & (abs(y-sy*pitch/2) < bolt) & (z < 8))
    for sy in (-1, 1):
        for h in (28, 166):
            material &= ~((x >= -60) & (x <= -50) & (abs(y-sy*8.5) < rail) & (abs(z-h) < rail))
    vertices, triangles, indices = [], [], {}
    def vertex(index):
        point = (xs[index[0]], ys[index[1]], zs[index[2]])
        if point not in indices:
            indices[point] = len(vertices)
            vertices.append(point)
        return indices[point]
    corners = (
        ((0,0,0),(0,0,1),(0,1,1),(0,1,0)),
        ((1,0,0),(1,1,0),(1,1,1),(1,0,1)),
        ((0,0,0),(1,0,0),(1,0,1),(0,0,1)),
        ((0,1,0),(0,1,1),(1,1,1),(1,1,0)),
        ((0,0,0),(0,1,0),(1,1,0),(1,0,0)),
        ((0,0,1),(1,0,1),(1,1,1),(0,1,1)),
    )
    for face, (axis, shift) in enumerate(((0,1),(0,-1),(1,1),(1,-1),(2,1),(2,-1))):
        neighbor = np.roll(material, shift, axis=axis)
        edge = [slice(None)]*3
        edge[axis] = 0 if shift == 1 else -1
        neighbor[tuple(edge)] = False
        for cell in np.argwhere(material & ~neighbor):
            ids = [vertex(tuple(cell + corner)) for corner in corners[face]]
            triangles.extend(((ids[0],ids[1],ids[2]), (ids[0],ids[2],ids[3])))
    return vertices, triangles


def write_stl(path: Path | str, vertices: Sequence[Sequence[float]], triangles: Sequence[Sequence[int]]) -> None:
    """Write a binary STL; coordinates are stored as given (mm for the bracket, as the header says)."""
    points = np.asarray(vertices)
    with Path(path).open("wb") as stream:
        stream.write(b"FPB30 rear deck prototype; units millimetres".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(triangles)))
        for face in triangles:
            a, b, c = points[list(face)]
            normal = np.cross(b-a, c-a)
            normal /= np.linalg.norm(normal)
            stream.write(struct.pack("<12fH", *normal, *a, *b, *c, 0))


def main() -> None:
    """CLI: write rear_deck_bracket.stl (mm) and mount.json into --output for the --config actuator."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = load_actuator(args.config)
    args.output.mkdir(parents=True, exist_ok=True)
    vertices, triangles = bracket_mesh(config.deck_hole_pitch_mm, config.print_clearance_mm)
    write_stl(args.output / "rear_deck_bracket.stl", vertices, triangles)
    report = {
        "units": "mm", "bounds": [np.min(vertices,axis=0).tolist(),np.max(vertices,axis=0).tolist()],
        "triangles": len(triangles), "deck_hole_pitch_mm": config.deck_hole_pitch_mm,
        "deck_thread": "M5x0.8", "rail_m4_pitch_mm": 17,
        "rail_attachment_heights_mm": [28,166], "print_clearance_mm": config.print_clearance_mm,
        "status": "Prototype: measure deck origin and rail interfaces; not fit/load certified",
    }
    (args.output / "mount.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
