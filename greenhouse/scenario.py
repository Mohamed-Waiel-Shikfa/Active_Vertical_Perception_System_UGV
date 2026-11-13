# SPDX-License-Identifier: GPL-3.0-only
"""Scenario files (schema v1), perturbation object models, terrain resting poses and the run-state file.

A scenario fixes a run: world seed, partial greenhouse/actuator overrides, robot start, static objects
and timed add/move/remove events. API units at the edges (x, y m; rz deg); SI inside.
Gazebo calls live in tools/scenario.py; everything here is pure and unit-tested.
"""
import argparse
import fcntl
import json
import math
import re
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from dataclasses import asdict, fields
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import numpy as np

from greenhouse.atomic import replace
from greenhouse.config import Config, load_config
from greenhouse.mesh import numbers
from greenhouse.world import child, material

VERSION = 1
MODEL_PREFIX = "scenario_"  # gz model name = prefix + id; never "robot" or ".../robot" (WorldState's Jackal match)
ID_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_-]{0,47}")
STATE_FILE = "scenario-state.json"
WORLD_FILE = "scenario-greenhouse.json"
ACTUATOR_FILE = "scenario-actuator.json"
RESERVED_GREENHOUSE = ("seed", "spawn_x", "spawn_y", "spawn_yaw")  # come from top-level seed / start
WALL_MARGIN = 0.05  # m between an object footprint and the house wall
START_ENVELOPE = 0.52  # m, padded J100 half-footprint radius (greenhouse.generate samples +/-0.52 m)
SAMPLE_SPACING = 0.05  # m between terrain samples under a footprint (finer than the 0.075 m mesh)

# Shapes in the model frame: origin at the footprint centre on the ground, +x forward, +z up.
# (name, "box" (sx, sy, sz) | "cylinder" (radius, length) | "sphere" (radius,), pose xyz rpy, RGBA)
# Sizes:
# - fertiliser_block: 25 kg woven-PP sack of granular fertiliser, about 0.60 x 0.40 x 0.15 m.
# - fallen_tomato: medium round greenhouse tomato, 60-80 mm diameter -> 0.07 m.
# - fallen_plant: 1.2 m stem segment of a high-wire tomato (stem ~12 mm) with four lying leaflets
#   ~0.28 x 0.14 m.
# - robot: Clearpath Jackal J100 envelope 0.508 x 0.430 x 0.250 m, wheel radius 0.098 m,
#   wheelbase 0.262 m (Clearpath J100 datasheet / jackal_description); static.
# - person: 1.75 m standing adult (ANSUR II mid-range stature): hip height 0.88 m, shoulder
#   breadth 0.40 m, head 0.22 m; static (an animated gz actor needs Fuel assets, v2).
Shape = tuple[str, str, tuple[float, ...], tuple[float, ...], tuple[float, ...]]
HALF_PI = math.pi / 2
BAG, RED, STEM, LEAF = (0.86, 0.85, 0.78, 1), (0.78, 0.08, 0.05, 1), (0.30, 0.45, 0.14, 1), (0.18, 0.42, 0.12, 1)
BODY, TYRE, CLOTH, LEGS, SKIN = (0.95, 0.72, 0.08, 1), (0.08, 0.08, 0.08, 1), (0.95, 0.45, 0.1, 1), (0.15, 0.2, 0.35, 1), (0.8, 0.62, 0.5, 1)
TYPES: dict[str, dict[str, Any]] = {
    "fertiliser_block": {"footprint": (0.30, 0.20), "shapes": [
        ("sack", "box", (0.60, 0.40, 0.15), (0, 0, 0.075, 0, 0, 0), BAG)]},
    "fallen_tomato": {"footprint": (0.035, 0.035), "shapes": [
        ("fruit", "sphere", (0.035,), (0, 0, 0.035, 0, 0, 0), RED)]},
    "fallen_plant": {"footprint": (0.60, 0.20), "shapes": [
        ("stem", "cylinder", (0.006, 1.2), (0, 0, 0.006, 0, HALF_PI, 0), STEM),
        *[(f"leaf_{i}", "box", (0.28, 0.14, 0.01), (x, side * 0.1, 0.005, 0, 0, side * 0.5), LEAF)
          for i, (x, side) in enumerate(((-0.42, 1), (-0.14, -1), (0.14, 1), (0.42, -1)))]]},
    "robot": {"footprint": (0.254, 0.215), "shapes": [
        ("chassis", "box", (0.508, 0.31, 0.185), (0, 0, 0.1575, 0, 0, 0), BODY),
        *[(f"wheel_{i}", "cylinder", (0.098, 0.05), (x, y, 0.098, HALF_PI, 0, 0), TYRE)
          for i, (x, y) in enumerate(((0.131, 0.19), (0.131, -0.19), (-0.131, 0.19), (-0.131, -0.19)))]]},
    "person": {"footprint": (0.15, 0.29), "shapes": [
        *[(f"leg_{i}", "cylinder", (0.075, 0.88), (0, y, 0.44, 0, 0, 0), LEGS) for i, y in enumerate((0.1, -0.1))],
        ("torso", "box", (0.24, 0.40, 0.58), (0, 0, 1.17, 0, 0, 0), CLOTH),
        *[(f"arm_{i}", "cylinder", (0.045, 0.62), (0, y, 1.15, 0, 0, 0), CLOTH) for i, y in enumerate((0.245, -0.245))],
        ("neck", "cylinder", (0.055, 0.07), (0, 0, 1.495, 0, 0, 0), SKIN),
        ("head", "sphere", (0.11,), (0, 0, 1.64, 0, 0, 0), SKIN)]},
}
OBJECT_KEYS = {"id", "type", "x", "y", "rz"}
EVENT_KEYS = {"add": {"t", "action", "id", "type", "x", "y", "rz"}, "move": {"t", "action", "id", "x", "y", "rz"},
              "remove": {"t", "action", "id"}}
TOP_KEYS = {"version", "seed", "greenhouse", "actuator", "start", "objects", "events"}


def model_name(object_id: str) -> str:
    """Gazebo model name of scenario object ``object_id``."""
    return MODEL_PREFIX + object_id


def model_sdf(kind: str, object_id: str) -> str:
    """Single-line SDF 1.9 of a static ``kind`` object named ``model_name(object_id)``; every shape has a
    matching collision and visual, so contact and future range sensors see it."""
    root = ET.Element("sdf", version="1.9")
    model = child(root, "model", name=model_name(object_id))
    child(model, "static", "true")
    link = child(model, "link", name="body")
    for name, geometry, size, pose, color in TYPES[kind]["shapes"]:
        for tag in ("collision", "visual"):
            node = child(link, tag, name=f"{name}_{tag}")
            child(node, "pose", numbers(pose))
            shape = child(child(node, "geometry"), geometry)
            if geometry == "box":
                child(shape, "size", numbers(size))
            else:
                child(shape, "radius", f"{size[0]:.9f}")
                if geometry == "cylinder":
                    child(shape, "length", f"{size[1]:.9f}")
            if tag == "visual":
                material(node, color)
    return ET.tostring(root, encoding="unicode")


def footprint_points(kind: str, x: float, y: float, yaw: float) -> np.ndarray:
    """(N, 2) world points in metres covering the type's footprint rectangle at (x, y), yaw rad."""
    hx, hy = TYPES[kind]["footprint"]
    u, v = np.meshgrid(np.linspace(-hx, hx, max(3, math.ceil(2 * hx / SAMPLE_SPACING) + 1)),
                       np.linspace(-hy, hy, max(3, math.ceil(2 * hy / SAMPLE_SPACING) + 1)))
    u, v = u.ravel(), v.ravel()
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + u * c - v * s, y + u * s + v * c))


def rest_pose(terrain: Any, kind: str, x: float, y: float, yaw: float) -> dict[str, float]:
    """World pose (m, rad) that rests a ``kind`` object at (x, y) with heading ``yaw`` on ``terrain``.

    A plane fitted to the surface under the footprint gives roll and pitch; z lifts the bottom by the
    largest residual, so no sampled point is above the bottom face (no sinking) and the gap elsewhere
    is only the sub-footprint relief. ``terrain`` needs ``sample(x, y)`` (greenhouse.terrain.Terrain).
    """
    points = footprint_points(kind, x, y, yaw)
    heights = np.array([terrain.sample(float(px), float(py)) for px, py in points])
    design = np.column_stack((np.ones(len(points)), points[:, 0] - x, points[:, 1] - y))
    (level, gx, gy), *_ = np.linalg.lstsq(design, heights, rcond=None)
    z = level + float(np.max(heights - design @ np.array((level, gx, gy))))
    c, s = math.cos(yaw), math.sin(yaw)
    forward, left = gx * c + gy * s, -gx * s + gy * c  # slope along the object's own x and y
    roll = math.asin(left / math.sqrt(1 + forward ** 2 + left ** 2))
    return {"x": x, "y": y, "z": float(z), "roll": roll, "pitch": math.atan2(-forward, 1.0), "yaw": yaw}


def _number(where: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{where} must be a finite number, got {value!r}")
    return float(value)


def _object(where: str, data: Any, keys: set[str], required: set[str]) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError(f"{where} must be a JSON object")
    unknown = set(data) - keys
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {sorted(unknown)}; valid: {sorted(keys)}")
    missing = required - set(data)
    if missing:
        raise ValueError(f"{where}: missing key(s) {sorted(missing)}")
    result = dict(data)
    if "id" in result and (not isinstance(result["id"], str) or not ID_PATTERN.fullmatch(result["id"])):
        raise ValueError(f"{where}.id must match {ID_PATTERN.pattern}, got {result['id']!r}")
    if "type" in result and result["type"] not in TYPES:
        raise ValueError(f"{where}.type must be one of {sorted(TYPES)}, got {result['type']!r}")
    for key in ("x", "y", "rz", "t"):
        if key in result:
            result[key] = _number(f"{where}.{key}", result[key])
    return result


def check_object(data: Any, where: str = "object") -> dict[str, Any]:
    """Validate one object {id, type, x, y[, rz]} (m, m, deg); returns it with rz (default 0) as floats."""
    return {"rz": 0.0, **_object(where, data, OBJECT_KEYS, {"id", "type", "x", "y"})}


def check_move(data: Any, where: str = "move") -> dict[str, Any]:
    """Validate a move {id[, x][, y][, rz]}; at least one coordinate is required."""
    result = _object(where, data, {"id", "x", "y", "rz"}, {"id"})
    if len(result) == 1:
        raise ValueError(f"{where}: give at least one of x, y, rz")
    return result


def check_inside(config: Config, kind: str, x: float, y: float, rz_deg: float, where: str = "object") -> None:
    """Raise ValueError unless the whole footprint lies inside the house walls (less WALL_MARGIN)."""
    points = footprint_points(kind, x, y, math.radians(rz_deg))
    if (np.abs(points[:, 0]).max() > config.house_length / 2 - WALL_MARGIN
            or np.abs(points[:, 1]).max() > config.house_width / 2 - WALL_MARGIN):
        raise ValueError(f"{where} ({kind} at {x:g}, {y:g}) reaches outside the greenhouse walls")


def ordered_events(events: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    """Events sorted by sim time ``t``; ties keep their file order."""
    return sorted(events, key=lambda event: event["t"])


def validate(scenario: Any, base_world: dict[str, Any], base_actuator: dict[str, Any]
             ) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Check a schema-v1 scenario strictly; return ``(scenario, world, actuator)``.

    ``base_world`` / ``base_actuator`` are complete config dicts (config/*.json merged with any
    --config / --actuator-config file). ``world`` is base + scenario ``greenhouse`` + seed + start,
    validated by greenhouse.config; ``actuator`` is base + scenario ``actuator`` (the caller validates
    it with greenhouse.actuator.load_actuator). Raises ValueError naming the first problem.
    """
    if not isinstance(scenario, dict):
        raise ValueError("scenario must be a JSON object")
    unknown = set(scenario) - TOP_KEYS
    if unknown:
        raise ValueError(f"unknown scenario key(s) {sorted(unknown)}; valid: {sorted(TOP_KEYS)}")
    for key in ("version", "seed", "start"):
        if key not in scenario:
            raise ValueError(f"scenario needs {key!r}")
    if scenario["version"] != VERSION or isinstance(scenario["version"], bool):
        raise ValueError(f"version must be {VERSION}, got {scenario['version']!r}")
    seed = scenario["seed"]
    if type(seed) is not int or not 0 <= seed < 2 ** 32:
        raise ValueError(f"seed must be an integer in [0, 2**32), got {seed!r}")
    start = _object("start", scenario["start"], {"x", "y", "rz"}, {"x", "y", "rz"})
    overrides = {}
    for section, valid in (("greenhouse", {f.name for f in fields(Config)}), ("actuator", set(base_actuator))):
        value = scenario.get(section, {})
        if not isinstance(value, dict):
            raise ValueError(f"{section} must be a JSON object of config overrides")
        bad = set(value) - valid
        if bad:
            raise ValueError(f"{section}: unknown config key(s) {sorted(bad)}")
        overrides[section] = value
    reserved = set(RESERVED_GREENHOUSE) & set(overrides["greenhouse"])
    if reserved:
        raise ValueError(f"greenhouse: {sorted(reserved)} come from the top-level 'seed' and 'start'")
    world = {**base_world, **overrides["greenhouse"], "seed": seed, "spawn_x": start["x"],
             "spawn_y": start["y"], "spawn_yaw": math.radians(start["rz"])}
    try:
        config = Config(**world).validate()
    except ValueError as error:
        raise ValueError(f"greenhouse/start: {error}") from None
    objects, events = scenario.get("objects", []), scenario.get("events", [])
    if not isinstance(objects, list) or not isinstance(events, list):
        raise ValueError("objects and events must be JSON lists")
    present: dict[str, dict[str, Any]] = {}
    checked_objects = []
    for index, data in enumerate(objects):
        where = f"objects[{index}]"
        item = check_object(data, where)
        if item["id"] in present:
            raise ValueError(f"{where}.id {item['id']!r} is used twice")
        check_inside(config, item["type"], item["x"], item["y"], item["rz"], where)
        if math.hypot(item["x"] - start["x"], item["y"] - start["y"]) < START_ENVELOPE + math.hypot(
                *TYPES[item["type"]]["footprint"]):
            raise ValueError(f"{where} ({item['id']}) overlaps the robot start pose")
        present[item["id"]] = item
        checked_objects.append(item)
    checked_events = []
    for index, data in enumerate(events):
        where = f"events[{index}]"
        if not isinstance(data, dict) or data.get("action") not in EVENT_KEYS:
            raise ValueError(f"{where}.action must be one of {sorted(EVENT_KEYS)}")
        action = data["action"]
        required = {"t", "action", "id"} | ({"type", "x", "y"} if action == "add" else set())
        event = _object(where, data, EVENT_KEYS[action], required)
        if event["t"] < 0:
            raise ValueError(f"{where}.t must be >= 0 simulated seconds")
        if action == "move" and not {"x", "y", "rz"} & set(event):
            raise ValueError(f"{where}: a move needs at least one of x, y, rz")
        if action == "add":
            event.setdefault("rz", 0.0)
        checked_events.append(event)
    # Replay ids in time order so an event never refers to an object that will not exist then.
    for event in ordered_events(checked_events):
        where = f"event at t={event['t']:g} ({event['action']} {event['id']})"
        if event["action"] == "add":
            if event["id"] in present:
                raise ValueError(f"{where}: id already exists")
            present[event["id"]] = event
        elif event["id"] not in present:
            raise ValueError(f"{where}: no such object at that time")
        elif event["action"] == "remove":
            del present[event["id"]]
            continue
        else:
            present[event["id"]] = {**present[event["id"]], **event}
        item = present[event["id"]]
        check_inside(config, item["type"], item["x"], item["y"], item["rz"], where)
    actuator = {**base_actuator, **overrides["actuator"]}
    checked = {"version": VERSION, "seed": seed, "greenhouse": overrides["greenhouse"],
               "actuator": overrides["actuator"], "start": start, "objects": checked_objects,
               "events": checked_events}
    return checked, asdict(config), actuator


def write_json(path: Path, data: Any) -> None:
    """Write ``data`` as indented JSON to ``path`` atomically (old-or-new for concurrent readers)."""
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    replace(temporary, path)


def read_state(directory: Path) -> dict[str, Any]:
    """The run's scenario-state.json, or an empty state if no object was added yet."""
    path = Path(directory) / STATE_FILE
    if not path.exists():
        return {"version": VERSION, "source": None, "objects": [], "events": []}
    return json.loads(path.read_text(encoding="utf-8"))


@contextmanager
def locked(directory: Path) -> Iterator[None]:
    """Hold the exclusive scenario lock: CLI, API and event runner each read-modify-write the state."""
    with (Path(directory) / "scenario.lock").open("w") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def prepare(scenario_path: Path, world_path: Path, actuator_path: Path, output: Path) -> dict[str, Any]:
    """Validate ``scenario_path`` over the selected configs and write the merged world and actuator
    JSON plus a fresh state file into ``output``; return the checked scenario."""
    from greenhouse.actuator import load_actuator

    raw = json.loads(Path(scenario_path).read_text(encoding="utf-8"))
    base_world = asdict(load_config(world_path))
    base_actuator = asdict(load_actuator(actuator_path))
    scenario, world, actuator = validate(raw, base_world, base_actuator)
    output = Path(output)
    write_json(output / ACTUATOR_FILE, actuator)
    try:
        load_actuator(output / ACTUATOR_FILE)
    except ValueError as error:
        raise ValueError(f"actuator: {error}") from None
    write_json(output / WORLD_FILE, world)
    write_json(output / STATE_FILE, {"version": VERSION, "source": str(scenario_path), "objects": [],
                                     "events": [dict(e, fired_sim_time=None) for e in scenario["events"]]})
    return scenario


def main() -> None:
    """CLI: ``prepare`` merges a scenario into the run configs (docker/start.sh); exits 2 on a bad file."""
    parser = argparse.ArgumentParser(description="Validate a scenario and write the merged run configs.")
    parser.add_argument("action", choices=("prepare",))
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--actuator-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        scenario = prepare(args.scenario, args.config, args.actuator_config, args.output)
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, f"scenario {args.scenario} rejected: {error}\n")
    print(f"Scenario: seed {scenario['seed']}, start {scenario['start']}, {len(scenario['objects'])} objects, "
          f"{len(scenario['events'])} timed events")


if __name__ == "__main__":
    main()
