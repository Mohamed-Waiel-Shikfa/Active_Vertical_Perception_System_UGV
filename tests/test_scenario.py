"""Scenario schema, merging, object models, terrain resting poses and the run-state file."""
import copy
import json
import math
import tempfile
import unittest
from dataclasses import asdict
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from greenhouse import scenario as sc
from greenhouse.actuator import load_actuator
from greenhouse.config import load_config
from greenhouse.terrain import Terrain

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "scenario.example.json"
HEIGHTS = {"fertiliser_block": 0.15, "fallen_tomato": 0.07, "fallen_plant": 0.012, "robot": 0.25, "person": 1.75}


def rotation(roll, pitch, yaw):
    """SDF extrinsic roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def surface(function, size=24.0, resolution=0.05):
    n = int(size / resolution) + 1
    origin = -size / 2
    grid = origin + np.arange(n) * resolution
    x, y = np.meshgrid(grid, grid)
    return Terrain(np.empty((0, 0)), function(x, y), resolution, origin)


class ScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = asdict(load_config(ROOT / "config" / "greenhouse.json"))
        cls.actuator = asdict(load_actuator(ROOT / "config" / "actuator.json"))
        cls.example = json.loads(EXAMPLE.read_text())

    def check(self, data):
        return sc.validate(data, self.world, self.actuator)

    def rejects(self, change, message):
        data = copy.deepcopy(self.example)
        change(data)
        with self.assertRaisesRegex(ValueError, message):
            self.check(data)

    def test_example_uses_every_type_and_an_event_of_each_action(self):
        scenario, world, actuator = self.check(self.example)
        self.assertEqual({o["type"] for o in scenario["objects"]}, set(sc.TYPES))
        self.assertEqual({e["action"] for e in scenario["events"]}, {"add", "move", "remove"})
        self.assertEqual(world["seed"], 7)
        self.assertEqual(world["ground_bump_rms"], 0.004)
        self.assertEqual(scenario["objects"][1]["rz"], 0.0)  # rz defaults to 0

    def test_merge_seed_start_and_overrides_over_the_selected_configs(self):
        data = dict(copy.deepcopy(self.example), start={"x": -7.0, "y": 0.5, "rz": 90},
                    actuator={"speed_m_s": 0.15})
        base = dict(self.world, row_length=11.0)  # e.g. from a --config file
        _, world, actuator = sc.validate(data, base, self.actuator)
        self.assertEqual((world["spawn_x"], world["spawn_y"]), (-7.0, 0.5))
        self.assertAlmostEqual(world["spawn_yaw"], math.pi / 2)
        self.assertEqual((world["row_length"], world["ground_bump_rms"]), (11.0, 0.004))
        self.assertEqual((actuator["speed_m_s"], actuator["stroke_m"]), (0.15, self.actuator["stroke_m"]))

    def test_prepare_writes_configs_the_generators_accept(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            sc.prepare(EXAMPLE, ROOT / "config" / "greenhouse.json", ROOT / "config" / "actuator.json", out)
            self.assertEqual(load_config(out / sc.WORLD_FILE).seed, 7)
            self.assertEqual(load_actuator(out / sc.ACTUATOR_FILE).gimbal_stabilise, (True, True))
            state = sc.read_state(out)
            self.assertEqual(state["objects"], [])
            self.assertEqual([e["fired_sim_time"] for e in state["events"]], [None] * 3)
            bad = out / "bad.json"
            bad.write_text(json.dumps(dict(self.example, actuator={"camera_mass_kg": 5.0})))
            with self.assertRaisesRegex(ValueError, "actuator"):
                sc.prepare(bad, ROOT / "config" / "greenhouse.json", ROOT / "config" / "actuator.json", out)

    def test_strict_validation_names_the_problem(self):
        cases = [
            (lambda d: d.update(colour="red"), "unknown scenario key"),
            (lambda d: d.pop("seed"), "needs 'seed'"),
            (lambda d: d.update(version=2), "version must be 1"),
            (lambda d: d.update(seed=-1), "seed must be"),
            (lambda d: d["greenhouse"].update(seed=3), "top-level 'seed' and 'start'"),
            (lambda d: d["greenhouse"].update(spawn_x=0.0), "top-level"),
            (lambda d: d["greenhouse"].update(rows=3), "greenhouse: unknown config key"),
            (lambda d: d["actuator"].update(stroke=1), "actuator: unknown config key"),
            (lambda d: d["greenhouse"].update(ground_friction=9), "greenhouse/start: ground_friction"),
            (lambda d: d.update(start={"x": -6.0, "y": 1.0, "rz": 0}), "greenhouse/start: spawn intersects a plant"),
            (lambda d: d["start"].pop("rz"), r"start: missing key\(s\) \['rz'\]"),
            (lambda d: d["objects"][0].update(id="1bag"), r"objects\[0\].id must match"),
            (lambda d: d["objects"][0].update(type="cow"), r"objects\[0\].type must be one of"),
            (lambda d: d["objects"][0].update(x="8"), r"objects\[0\].x must be a finite number"),
            (lambda d: d["objects"][0].update(z=0), r"objects\[0\]: unknown key"),
            (lambda d: d["objects"][1].update(id="bag1"), "used twice"),
            (lambda d: d["objects"][0].update(x=8.8), "outside the greenhouse walls"),
            (lambda d: d["objects"][1].update(x=-7.2, y=0.3), "overlaps the robot start"),
            (lambda d: d["events"][0].update(action="teleport"), r"events\[0\].action must be one of"),
            (lambda d: d["events"][0].update(t=-1), "must be >= 0"),
            (lambda d: d["events"][0].pop("y"), "needs at least one of x, y, rz"),
            (lambda d: d["events"][1].update(id="bag1"), "id already exists"),
            (lambda d: d["events"][2].update(id="nobody"), "no such object"),
            (lambda d: d["events"][1].pop("type"), r"missing key\(s\) \['type'\]"),
            (lambda d: d["events"][2].update(type="person"), "unknown key"),
        ]
        for change, message in cases:
            with self.subTest(message=message):
                self.rejects(change, message)

    def test_events_are_replayed_in_time_order_ties_keep_file_order(self):
        events = [{"t": 9.0, "n": 1}, {"t": 3.0, "n": 2}, {"t": 9.0, "n": 3}, {"t": 0.0, "n": 4}]
        self.assertEqual([e["n"] for e in sc.ordered_events(events)], [4, 2, 1, 3])
        later_add = dict(self.example, events=[{"t": 50, "action": "remove", "id": "late"},
                                               {"t": 10, "action": "add", "id": "late", "type": "person",
                                                "x": 0.0, "y": 0.0}])
        self.check(later_add)  # file order differs from time order: still valid
        early_remove = copy.deepcopy(later_add)
        early_remove["events"][0]["t"] = 5
        with self.assertRaisesRegex(ValueError, "no such object"):
            self.check(early_remove)
        moved_out = dict(self.example, events=[{"t": 1, "action": "move", "id": "robot2", "x": 8.9}])
        with self.assertRaisesRegex(ValueError, "outside the greenhouse walls"):
            self.check(moved_out)

    def test_each_type_is_a_static_sdf_model_resting_on_its_frame_origin(self):
        for kind, height in HEIGHTS.items():
            with self.subTest(kind=kind):
                root = ET.fromstring(sc.model_sdf(kind, "obj_1"))
                model = root.find("model")
                self.assertEqual(model.get("name"), "scenario_obj_1")
                self.assertEqual(model.findtext("static"), "true")
                collisions, visuals = model.findall("link/collision"), model.findall("link/visual")
                self.assertEqual(len(collisions), len(sc.TYPES[kind]["shapes"]))
                self.assertEqual(len(visuals), len(collisions))
                low, high = math.inf, -math.inf
                for node in collisions:
                    x, y, z, roll, pitch, _ = map(float, node.findtext("pose").split())
                    geometry = node.find("geometry")[0]
                    if geometry.tag == "box":
                        half = float(geometry.findtext("size").split()[2]) / 2
                    elif geometry.tag == "sphere":
                        half = float(geometry.findtext("radius"))
                    else:
                        radius, length = float(geometry.findtext("radius")), float(geometry.findtext("length"))
                        half = radius if abs(roll) > 1 or abs(pitch) > 1 else length / 2
                    low, high = min(low, z - half), max(high, z + half)
                self.assertAlmostEqual(low, 0.0, places=9)
                self.assertAlmostEqual(high, height, places=9)
                self.assertNotIn("\n", sc.model_sdf(kind, "obj_1"))

    def test_rest_pose_follows_a_slope_without_sinking(self):
        def plane(x, y):
            return 0.02 + 0.04 * x - 0.03 * y
        terrain = surface(plane)
        for kind in sc.TYPES:
            for yaw in (0.0, 0.5, -2.0):
                with self.subTest(kind=kind, yaw=yaw):
                    pose = sc.rest_pose(terrain, kind, 1.3, -0.7, yaw)
                    hx, hy = sc.TYPES[kind]["footprint"]
                    turn = rotation(pose["roll"], pose["pitch"], pose["yaw"])
                    for u in (-hx, 0, hx):
                        for v in (-hy, 0, hy):
                            point = np.array((pose["x"], pose["y"], pose["z"])) + turn @ np.array((u, v, 0.0))
                            self.assertAlmostEqual(point[2], plane(point[0], point[1]), places=5)

    def test_rest_pose_on_bumps_touches_but_never_sinks(self):
        bumps = surface(lambda x, y: 0.01 * np.sin(7 * x) * np.cos(5 * y))
        for kind in ("fertiliser_block", "fallen_plant", "robot"):
            pose = sc.rest_pose(bumps, kind, 0.4, 0.2, 0.3)
            turn = rotation(pose["roll"], pose["pitch"], pose["yaw"])
            gaps = []
            for px, py in sc.footprint_points(kind, 0.4, 0.2, 0.3):
                local = turn.T @ (np.array((px, py, bumps.sample(px, py))) - (pose["x"], pose["y"], pose["z"]))
                gaps.append(-local[2])  # height of the bottom face above the ground there
            self.assertGreaterEqual(min(gaps), -1e-3, kind)  # tilt approximation only
            self.assertLess(min(gaps), 1e-3, kind)  # touches somewhere
        flat = surface(lambda x, y: np.full_like(x, 0.012))
        pose = sc.rest_pose(flat, "person", 0.0, 0.0, 1.0)
        self.assertAlmostEqual(pose["z"], 0.012)
        self.assertAlmostEqual(pose["roll"], 0.0)
        self.assertAlmostEqual(pose["pitch"], 0.0)

    def test_state_file_round_trip_is_atomic(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            self.assertEqual(sc.read_state(out)["objects"], [])
            state = {"version": 1, "source": None, "events": [],
                     "objects": [{"id": "a", "type": "person", "x": 1.0, "y": 2.0, "rz": 3.0,
                                  "model": "scenario_a", "z": 0.01}]}
            with sc.locked(out):
                sc.write_json(out / sc.STATE_FILE, state)
            self.assertEqual(sc.read_state(out), state)
            self.assertEqual(sorted(p.name for p in out.iterdir()), ["scenario-state.json", "scenario.lock"])

    def test_live_object_checks(self):
        self.assertEqual(sc.check_object({"id": "p", "type": "person", "x": 1, "y": 2})["rz"], 0.0)
        self.assertEqual(sc.check_move({"id": "p", "rz": 5}), {"id": "p", "rz": 5.0})
        for bad in ({"id": "p"}, {"id": "p", "z": 1}, {"x": 1}):
            with self.assertRaises(ValueError):
                sc.check_move(bad)
        with self.assertRaises(ValueError):
            sc.check_object({"id": "p", "type": "person", "x": 1, "y": True})


if __name__ == "__main__":
    unittest.main()
