# SPDX-License-Identifier: GPL-3.0-only
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

COLLADA = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA}


def numbers(values: Iterable[float]) -> str:
    """Space-separated ``values`` with nine decimals, as COLLADA and SDF text fields expect."""
    return " ".join(f"{float(value):.9f}" for value in values)


@dataclass
class Mesh:
    """Indexed triangle mesh in metres, +z up, in the caller's local frame; ``uv`` is per vertex."""
    vertices: list = field(default_factory=list)
    faces: list = field(default_factory=list)
    uv: list = field(default_factory=list)

    def triangle(self, points: Sequence[Sequence[float]] | np.ndarray,
                 texcoords: Sequence[Sequence[float]] | np.ndarray | None = None,
                 double_sided: bool = False) -> None:
        """Append one triangle of three ``(x, y, z)`` metre points, with optional per-corner UVs.

        ``double_sided`` also appends the reversed-winding copy.
        """
        start = len(self.vertices)
        self.vertices.extend(tuple(p) for p in points)
        self.faces.append((start, start + 1, start + 2))
        if texcoords is not None:
            self.uv.extend(texcoords)
        if double_sided:
            self.triangle(list(reversed(points)),
                          list(reversed(texcoords)) if texcoords is not None else None)

    def tube(self, start: Sequence[float] | np.ndarray, end: Sequence[float] | np.ndarray,
             radius: float, sides: int = 8) -> None:
        """Append a capped ``sides``-gon tube of ``radius`` m between metre points ``start`` and ``end``."""
        start, end = np.asarray(start, float), np.asarray(end, float)
        axis = end - start
        axis /= np.linalg.norm(axis)
        tangent = np.cross(axis, [0, 0, 1] if abs(axis[2]) < 0.95 else [1, 0, 0])
        tangent /= np.linalg.norm(tangent)
        bitangent = np.cross(axis, tangent)
        rings = [
            [point + radius * (math.cos(i * 2 * math.pi / sides) * tangent +
                               math.sin(i * 2 * math.pi / sides) * bitangent)
             for i in range(sides)]
            for point in (start, end)
        ]
        for i in range(sides):
            j = (i + 1) % sides
            self.triangle((rings[0][i], rings[0][j], rings[1][j]))
            self.triangle((rings[0][i], rings[1][j], rings[1][i]))
            self.triangle((start, rings[0][j], rings[0][i]))
            self.triangle((end, rings[1][i], rings[1][j]))

    def fruit(self, center: Sequence[float], radius: float, rings: int = 6, sectors: int = 10) -> None:
        """Append a sphere at ``center`` (metres) of ``radius`` m, squashed to 0.9 * ``radius`` along z."""
        def point(ring, sector):
            phi, theta = math.pi * ring / rings, 2 * math.pi * sector / sectors
            return (center[0] + radius * math.sin(phi) * math.cos(theta),
                    center[1] + radius * math.sin(phi) * math.sin(theta),
                    center[2] + 0.9 * radius * math.cos(phi))

        for ring in range(rings):
            for sector in range(sectors):
                a, b = point(ring, sector), point(ring, sector + 1)
                c, d = point(ring + 1, sector + 1), point(ring + 1, sector)
                if ring > 0:
                    self.triangle((a, d, b))
                if ring < rings - 1:
                    self.triangle((b, d, c))


def read_leaf_mesh(path: Path | str) -> tuple[np.ndarray, list[tuple[np.ndarray, np.ndarray, np.ndarray]]]:
    """Read the pinned Z-up leaf COLLADA with its node matrix and unit applied (metres).

    Returns ``(vertices, faces)``: an (N, 3) array, and per triangle its (3, 3) corners, (3, 2) UVs
    and the unit xy direction of its leaf blade away from the z axis. Raises ValueError on an
    unexpected asset.
    """
    root = ET.parse(path).getroot()
    if root.findtext("c:asset/c:up_axis", namespaces=NS) != "Z_UP":
        raise ValueError("pinned leaf asset must use Z_UP")
    geometry = root.find("c:library_geometries/c:geometry/c:mesh", NS)
    if geometry is None:
        raise ValueError("leaf asset has no geometry")
    sources = {}
    for source in geometry.findall("c:source", NS):
        array = np.fromstring(source.findtext("c:float_array", namespaces=NS), sep=" ")
        stride = int(source.find("c:technique_common/c:accessor", NS).get("stride"))
        sources[source.get("id")] = array.reshape(-1, stride)
    position_id = geometry.find("c:vertices/c:input", NS).get("source")[1:]
    matrix_text = root.findtext("c:library_visual_scenes/c:visual_scene/c:node/c:matrix", namespaces=NS)
    # COLLADA stores matrices column-major. The pinned asset has a 0.10223 scale.
    matrix = np.fromstring(matrix_text, sep=" ").reshape(4, 4).T
    positions = sources[position_id]
    transformed = (matrix @ np.column_stack((positions, np.ones(len(positions)))).T).T[:, :3]
    transformed *= float(root.find("c:asset/c:unit", NS).get("meter", "1"))
    triangles = geometry.find("c:triangles", NS)
    inputs = {node.get("semantic"): node for node in triangles.findall("c:input", NS)}
    width = max(int(node.get("offset")) for node in inputs.values()) + 1
    indices = np.fromstring(triangles.findtext("c:p", namespaces=NS), dtype=int, sep=" ").reshape(-1, 3, width)
    texture = sources[inputs["TEXCOORD"].get("source")[1:]]
    vertex_indices = indices[:, :, int(inputs["VERTEX"].get("offset"))]
    parents = list(range(len(transformed)))

    def component(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for face in vertex_indices:
        for index in face[1:]:
            parents[component(int(index))] = component(int(face[0]))
    groups = {}
    for index in range(len(transformed)):
        groups.setdefault(component(index), []).append(index)
    directions = {}
    for key, group in groups.items():
        direction = transformed[group, :2].mean(axis=0)
        length = np.linalg.norm(direction)
        if length < 1e-10:
            raise ValueError("pinned leaf component has no radial direction")
        directions[key] = direction / length
    return transformed, [
        (transformed[face[:, int(inputs["VERTEX"].get("offset"))]],
         texture[face[:, int(inputs["TEXCOORD"].get("offset"))]],
         directions[component(int(face[0, int(inputs["VERTEX"].get("offset"))]))])
        for face in indices
    ]


def write_dae(path: Path | str,
              parts: list[tuple[str, Mesh, tuple[float, ...], str | None]]) -> None:
    """Write baked metre/Z-up geometry, with no external scene transforms.

    ``parts`` are ``(name, mesh, rgba 0-1, texture path or None)``. Raises ValueError if a
    textured mesh has a UV count different from its vertex count.
    """
    ET.register_namespace("", COLLADA)

    def element(parent, tag, text=None, **attributes):
        node = ET.SubElement(parent, f"{{{COLLADA}}}{tag}", attributes)
        if text is not None:
            node.text = str(text)
        return node

    root = ET.Element(f"{{{COLLADA}}}COLLADA", version="1.4.1")
    asset = element(root, "asset")
    contributor = element(asset, "contributor")
    element(contributor, "author", "Jackal greenhouse contributors; attributed source assets")
    element(asset, "created", "2026-09-11T00:00:00")
    element(asset, "modified", "2026-09-11T00:00:00")
    element(asset, "unit", name="meter", meter="1")
    element(asset, "up_axis", "Z_UP")
    effects, images = element(root, "library_effects"), element(root, "library_images")
    materials, geometries = element(root, "library_materials"), element(root, "library_geometries")
    scenes = element(root, "library_visual_scenes")
    scene = element(scenes, "visual_scene", id="Scene", name="Scene")
    for name, mesh, color, texture in parts:
        effect = element(effects, "effect", id=f"{name}-effect")
        profile = element(effect, "profile_COMMON")
        if texture:
            image = element(images, "image", id=f"{name}-image")
            element(image, "init_from", texture)
            surface = element(element(profile, "newparam", sid=f"{name}-surface"), "surface", type="2D")
            element(surface, "init_from", f"{name}-image")
            sampler = element(element(profile, "newparam", sid=f"{name}-sampler"), "sampler2D")
            element(sampler, "source", f"{name}-surface")
        phong = element(element(profile, "technique", sid="common"), "phong")
        element(element(phong, "ambient"), "color", numbers(color))
        diffuse = element(phong, "diffuse")
        if texture:
            element(diffuse, "texture", texture=f"{name}-sampler", texcoord="UVMap")
            transparent = element(phong, "transparent", opaque="A_ONE")
            element(transparent, "texture", texture=f"{name}-sampler", texcoord="UVMap")
        else:
            element(diffuse, "color", numbers(color))
        element(element(phong, "specular"), "color", "0.08 0.08 0.08 1")
        element(element(phong, "shininess"), "float", "16")
        material = element(materials, "material", id=f"{name}-material", name=name)
        element(material, "instance_effect", url=f"#{name}-effect")
        geometry = element(geometries, "geometry", id=f"{name}-geometry", name=name)
        body = element(geometry, "mesh")
        vertices, faces = np.asarray(mesh.vertices, float), np.asarray(mesh.faces, int)
        normals = np.zeros_like(vertices)
        face_normals = np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                                vertices[faces[:, 2]] - vertices[faces[:, 0]])
        for corner in range(3):
            np.add.at(normals, faces[:, corner], face_normals)
        lengths = np.linalg.norm(normals, axis=1)
        normals[lengths > 0] /= lengths[lengths > 0, None]
        normals[lengths == 0] = (0, 0, 1)
        arrays = [("positions", vertices, ("X", "Y", "Z")),
                  ("normals", normals, ("X", "Y", "Z"))]
        if mesh.uv:
            if len(mesh.uv) != len(vertices):
                raise ValueError("each textured vertex must have a UV coordinate")
            arrays.append(("uv", np.asarray(mesh.uv), ("S", "T")))
        for label, array, axes in arrays:
            source = element(body, "source", id=f"{name}-{label}")
            element(source, "float_array", numbers(array.flat),
                    id=f"{name}-{label}-array", count=str(array.size))
            accessor = element(element(source, "technique_common"), "accessor",
                               source=f"#{name}-{label}-array",
                               count=str(len(array)), stride=str(len(axes)))
            for axis in axes:
                element(accessor, "param", name=axis, type="float")
        verts = element(body, "vertices", id=f"{name}-vertices")
        element(verts, "input", semantic="POSITION", source=f"#{name}-positions")
        tris = element(body, "triangles", count=str(len(faces)), material=f"{name}-material")
        element(tris, "input", semantic="VERTEX", source=f"#{name}-vertices", offset="0")
        element(tris, "input", semantic="NORMAL", source=f"#{name}-normals", offset="1")
        if mesh.uv:
            element(tris, "input", semantic="TEXCOORD", source=f"#{name}-uv", offset="2", set="0")
        # gz-common counts input semantics, not max(offset)+1. Distinct offsets
        # are required even when position/normal/UV share the same vertex index.
        stride = 3 if mesh.uv else 2
        element(tris, "p", " ".join(str(int(index)) for index in faces.flat for _ in range(stride)))
        node = element(scene, "node", id=f"{name}-node", name=name, type="NODE")
        instance = element(node, "instance_geometry", url=f"#{name}-geometry")
        binding = element(element(instance, "bind_material"), "technique_common")
        bound = element(binding, "instance_material", symbol=f"{name}-material", target=f"#{name}-material")
        if mesh.uv:
            element(bound, "bind_vertex_input", semantic="UVMap", input_semantic="TEXCOORD", input_set="0")
    element(element(root, "scene"), "instance_visual_scene", url="#Scene")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def read_baked_vertices(path: Path | str) -> np.ndarray:
    """All position vertices of a ``write_dae`` file as an (N, 3) array in metres.

    Raises ValueError if the file has a scene transform or is not in metres.
    """
    root = ET.parse(path).getroot()
    if root.findall(".//c:matrix", NS) or root.findall(".//c:scale", NS):
        raise ValueError("generated geometry unexpectedly contains an unbaked transform")
    if root.find("c:asset/c:unit", NS).get("meter") != "1":
        raise ValueError("generated geometry must be in metres")
    return np.vstack([
        np.fromstring(node.text, sep=" ").reshape(-1, 3)
        for node in root.findall(".//c:source/c:float_array", NS)
        if node.get("id").endswith("-positions-array")
    ])
