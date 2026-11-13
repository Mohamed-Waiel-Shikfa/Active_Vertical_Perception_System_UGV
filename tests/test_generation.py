import ast
import hashlib
import json
import math
import tempfile
import unittest
from collections import Counter
from dataclasses import replace
from pathlib import Path
from xml.etree import ElementTree as ET

import cv2
import numpy as np

from greenhouse.config import load_config
from greenhouse.generate import ROOT, UPSTREAM, generate, verify_upstream
from greenhouse.mesh import NS, read_baked_vertices
from greenhouse.terrain import Terrain, generate_terrain
from greenhouse.world import SYSTEMS


class GenerationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temp.name) / "assets"
        cls.config = load_config(ROOT / "config" / "greenhouse.json")
        cls.report = generate(cls.config, cls.output)
        cls.terrain = generate_terrain(cls.config)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_exact_upstream_method_not_a_reimplementation(self):
        def method(path):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            return next(node for node in ast.walk(tree)
                        if isinstance(node, ast.FunctionDef) and node.name == "generate_ground")
        original = method(UPSTREAM / "upstream" / "virtual_maize_field" / "world_generator" / "field_2d_generator.py")
        subset = method(UPSTREAM / "terrain.py")
        self.assertEqual(ast.dump(original), ast.dump(subset))
        self.assertEqual(verify_upstream()["revision"], "e525d70c3c73fcdfae9c77bb8e4ca03bf3377ac3")

    def test_generation_is_byte_deterministic(self):
        second = Path(self.temp.name) / "repeat"
        report = generate(self.config, second)
        self.assertEqual(self.report, report)
        for name in self.report["files_sha256"]:
            self.assertEqual((self.output / name).read_bytes(), (second / name).read_bytes(), name)

    def test_seed_changes_real_terrain(self):
        changed = generate_terrain(replace(self.config, seed=43))
        self.assertFalse(np.array_equal(self.terrain.heights, changed.heights))

    def test_gentle_nonzero_physical_terrain(self):
        terrain = self.terrain
        self.assertGreater(np.ptp(terrain.heights), 0.025)
        self.assertLess(np.ptp(terrain.heights), 0.055)
        self.assertGreater(terrain.heights.std(), 0.003)
        self.assertLessEqual(terrain.resolution, 0.075)
        self.assertLess(max(np.max(abs(np.diff(terrain.heights, axis=axis))) / terrain.resolution
                            for axis in (0, 1)), 0.4)
        vertices, faces = terrain.mesh()
        edges = Counter(tuple(sorted(edge)) for a, b, c in faces for edge in ((a, b), (b, c), (c, a)))
        self.assertTrue(all(count == 2 for count in edges.values()), "terrain must be watertight")
        self.assertEqual(len(vertices) - len(edges) + len(faces), 2)

    def test_wheel_scale_relief_not_blanket_amplitude_scaling(self):
        original = generate_terrain(replace(self.config, ground_bump_rms=0))
        self.assertAlmostEqual(np.ptp(original.heights), self.config.ground_height_range, places=8)
        self.assertEqual(original.heights.shape, (128, 128))
        self.assertEqual(self.terrain.heights.shape, (255, 255))
        self.assertAlmostEqual(self.terrain.origin, original.origin)
        self.assertAlmostEqual(self.terrain.heights.mean(), original.heights.mean(), places=12)
        difference = []
        original_difference = []
        for x in np.arange(-8, 8, 0.2):
            for y in (-2.0, 0.0, 2.0):
                difference.append(abs(self.terrain.sample(x, y + 0.187795)
                                      - self.terrain.sample(x, y - 0.187795)))
                original_difference.append(abs(original.sample(x, y + 0.187795)
                                               - original.sample(x, y - 0.187795)))
        self.assertGreater(np.quantile(difference, 0.95), 0.003)
        self.assertLess(np.quantile(difference, 0.95), 0.015)
        self.assertGreater(np.median(difference), np.median(original_difference))
        heights, resolution = self.terrain.heights, self.terrain.resolution
        dx, dy = np.diff(heights, axis=1) / resolution, np.diff(heights, axis=0) / resolution
        slopes = np.hypot(dx[:-1, :], dy[:, :-1])
        self.assertLess(np.quantile(slopes, 0.95), math.tan(math.radians(9)))
        self.assertLess(slopes.max(), math.tan(math.radians(20)))

    def test_collision_is_the_visual_surface_not_a_flat_floor(self):
        root = ET.parse(self.output / "models" / "greenhouse_ground" / "model.sdf")
        collision, visual = root.find(".//collision/geometry"), root.find(".//visual/geometry")
        self.assertEqual(ET.tostring(collision), ET.tostring(visual))
        self.assertEqual(collision.findtext("mesh/scale"), "1 1 1")
        for path in self.output.rglob("*.sdf"):
            self.assertFalse(ET.parse(path).findall(".//plane"), str(path))
        mesh = read_baked_vertices(self.output / "models" / "greenhouse_ground" / "meshes" / "terrain.dae")
        np.testing.assert_allclose(mesh[:self.terrain.heights.size, 2],
                                   self.terrain.heights.flat, atol=5e-10)

    def test_soil_appearance_does_not_change_verified_geometry(self):
        root = ET.parse(self.output / "models" / "greenhouse_ground" / "meshes" / "terrain.dae")
        from greenhouse.mesh import Mesh, write_dae
        vertices, faces = self.terrain.mesh()
        reference = Path(self.temp.name) / "untextured.dae"
        write_dae(reference, [("soil", Mesh(vertices, faces), (0.34, 0.24, 0.135, 1), None)])
        plain = ET.parse(reference)
        for label in ("positions", "normals"):
            values = root.find(f".//c:float_array[@id='soil-{label}-array']", NS)
            data = np.fromstring(values.text, sep=" ", dtype="<f8")
            expected = np.fromstring(plain.findtext(f".//c:float_array[@id='soil-{label}-array']", namespaces=NS), sep=" ")
            np.testing.assert_array_equal(data, expected)
        triangles = root.find(".//c:triangles", NS)
        indices = np.fromstring(triangles.findtext("c:p", namespaces=NS), sep=" ", dtype="<i8")
        vertices = indices.reshape(-1, 3)[:, 0]
        np.testing.assert_array_equal(vertices.reshape(-1, 3), faces)

    def test_local_soil_texture_is_bound_and_follows_real_cavities(self):
        directory = self.output / "models" / "greenhouse_ground"
        root = ET.parse(directory / "model.sdf")
        self.assertEqual(root.findtext(".//visual/material/diffuse"), "1 1 1 1")
        self.assertEqual(root.findtext(".//visual/material/pbr/metal/albedo_map"),
                         "model://greenhouse_ground/materials/textures/soil.png")
        self.assertEqual(root.findtext(".//visual/material/pbr/metal/metalness"), "0")
        self.assertEqual(root.findtext(".//visual/material/pbr/metal/roughness"), "0.95")
        self.assertFalse(root.findall(".//normal_map"))
        self.assertEqual(root.findtext(".//collision/surface/friction/ode/mu"), "0.9")
        self.assertEqual(root.findtext(".//collision/surface/friction/ode/mu2"), "0.9")
        image = cv2.cvtColor(cv2.imread(str(directory / "materials" / "textures" / "soil.png")),
                             cv2.COLOR_BGR2RGB) / 255
        linear = np.where(image <= 0.04045, image / 12.92, ((image + 0.055) / 1.055) ** 2.4)
        np.testing.assert_allclose(linear.mean(axis=(0, 1)), (0.34, 0.24, 0.135), atol=0.001)
        h = self.terrain.heights
        relief = h - cv2.GaussianBlur(h, (0, 0), 0.35 / self.terrain.resolution,
                                     borderType=cv2.BORDER_REFLECT_101)
        # Compare the cavity-scale signal, not the independent fine soil grain.
        cavity_signal = cv2.GaussianBlur(linear[:, :, 0], (0, 0), 5)
        registered = cv2.resize(np.flipud(cavity_signal), h.shape[::-1])
        self.assertGreater(np.corrcoef(relief.ravel(), registered.ravel())[0, 1], 0.9)
        mesh = ET.parse(directory / "meshes" / "terrain.dae")
        self.assertEqual(mesh.findtext(".//c:library_images/c:image/c:init_from", namespaces=NS),
                         "../materials/textures/soil.png")
        uv = np.fromstring(mesh.findtext(".//c:float_array[@id='soil-uv-array']", namespaces=NS),
                           sep=" ").reshape(-1, 2)
        n = h.shape[0]
        self.assertAlmostEqual(uv[0, 0], 0.5 / 2048, places=8)
        self.assertAlmostEqual(uv[n - 1, 0], 1 - 0.5 / 2048, places=8)
        self.assertAlmostEqual(uv[(n - 1) * n, 1], 1 - 0.5 / 2048, places=8)

    def test_triangle_sampler_not_bilinear_proxy(self):
        terrain = Terrain(np.zeros((2, 2)), np.array(((0.0, 0.0), (0.0, 1.0))), 1, 0)
        self.assertAlmostEqual(terrain.sample(0.75, 0.25), 0.25)
        self.assertAlmostEqual(terrain.sample(0.25, 0.75), 0.25)
        with self.assertRaises(ValueError):
            terrain.sample(-0.01, 0)

    def test_measured_tomato_geometry_and_recognizable_parts(self):
        plant = self.output / "models" / "greenhouse_tomato"
        vertices = read_baked_vertices(plant / "meshes" / "tomato.dae")
        self.assertAlmostEqual(np.ptp(vertices[:, 2]), 1.4, places=8)
        self.assertAlmostEqual(vertices[:, 2].min(), 0, places=8)
        self.assertLessEqual(np.linalg.norm(vertices[:, :2], axis=1).max(), 0.35000001)
        root = ET.parse(plant / "meshes" / "tomato.dae")
        names = {node.get("name") for node in root.findall(".//c:geometry", NS)}
        self.assertEqual(names, {"foliage", "stems", "tomatoes", "calyces"})
        self.assertEqual(self.report["plant"]["fruit_count"], 12)
        self.assertEqual(self.report["plant"]["crowns"], 4)
        self.assertGreaterEqual(self.report["plant"]["leaf_blades_per_crown"], 6)
        self.assertGreater(self.report["plant"]["leaf_width_scale"], 2)
        self.assertLess(self.report["plant"]["triangles"], 4000)
        self.assertEqual(ET.parse(plant / "model.sdf").findtext(".//mesh/scale"), "1 1 1")

    def test_clear_aisles_include_foliage_and_collision(self):
        self.assertEqual(self.report["layout"]["plant_count"], 68)
        for aisle in self.report["layout"]["clear_aisles_m"]:
            self.assertAlmostEqual(aisle, 1.3)
            self.assertGreaterEqual(aisle, 1.2)
        self.assertGreaterEqual(self.report["layout"]["clear_side_aisle_m"], 1.2)
        self.assertGreaterEqual(self.report["layout"]["clear_headland_m"], 2.0)
        radius = self.report["plant"]["collision_radius_m"]
        self.assertGreaterEqual(radius + 1e-8, self.report["plant"]["max_horizontal_radius_m"])

    def test_plants_touch_the_sampled_triangle_surface(self):
        for plant in self.report["plants"]:
            self.assertAlmostEqual(plant["z"], self.terrain.sample(plant["x"], plant["y"]), places=9)
        spawn = self.report["spawn"]
        self.assertGreater(spawn["z"], self.terrain.sample(spawn["x"], spawn["y"]) + 0.08)

    def test_local_resource_closure_and_modern_plugins(self):
        for path in self.output.rglob("*.sdf"):
            root = ET.parse(path)
            for link in root.findall(".//link"):
                names = [node.get("name") for node in link if node.get("name")]
                self.assertEqual(len(names), len(set(names)), f"duplicate SDF child names in {path}")
            for uri in root.findall(".//uri") + root.findall(".//albedo_map"):
                self.assertTrue(uri.text.startswith("model://"), uri.text)
                target = self.output / "models" / uri.text.removeprefix("model://")
                self.assertTrue(target.exists(), str(target))
            for plugin in root.findall(".//plugin"):
                self.assertIn(plugin.get("name"), {f"gz::sim::systems::{name}" for name in SYSTEMS})
                self.assertTrue(plugin.get("filename").startswith("gz-sim-"))
        for path in self.output.rglob("*.dae"):
            mesh = ET.parse(path)
            for triangle in mesh.findall(".//c:triangles", NS):
                offsets = [int(node.get("offset")) for node in triangle.findall("c:input", NS)]
                self.assertEqual(offsets, list(range(len(offsets))), "gz-common requires distinct input offsets")
                indices = triangle.findtext("c:p", namespaces=NS).split()
                self.assertEqual(len(indices), int(triangle.get("count")) * 3 * len(offsets))
            for image in mesh.findall(".//c:library_images/c:image/c:init_from", NS):
                self.assertTrue((path.parent / image.text).is_file(), image.text)
        world = ET.parse(self.output / "worlds" / "greenhouse.sdf")
        self.assertEqual(world.find("world").get("name"), "greenhouse")

    def test_bad_config_fails_explicitly(self):
        for key, value in (("seed", True), ("row_count", 4.0), ("plant_height", float("nan")),
                           ("ground_height_range", 0), ("row_spacing", 1.7),
                           ("ground_bump_rms", 0.03), ("ground_bump_scale", 0.1),
                           ("spawn_y", 4.8), ("house_length", 13), ("house_width", 7)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                replace(self.config, **{key: value}).validate()
        with self.assertRaisesRegex(ValueError, "plant"):
            replace(self.config, spawn_x=-6, spawn_y=1).validate()
        path = Path(self.temp.name) / "invalid.json"
        path.write_text(json.dumps({"unrecognized_option": 1}))
        with self.assertRaisesRegex(ValueError, "unknown"):
            load_config(path)

    def test_partial_config_takes_missing_keys_from_the_bundled_json(self):
        for name in ("greenhouse-alternate.json", "soil-baseline.json"):
            path = ROOT / "tests" / "fixtures" / name
            with self.subTest(fixture=name):
                overrides = json.loads(path.read_text())
                self.assertEqual(load_config(path), replace(self.config, **overrides))


if __name__ == "__main__":
    unittest.main()
