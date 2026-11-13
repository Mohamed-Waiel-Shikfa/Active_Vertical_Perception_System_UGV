"""Live perturbation objects and scenario playback (in-container CLI; prints one JSON result).

  add TYPE --id ID --x X --y Y [--rz DEG]   spawn a static object resting on the terrain
  move ID [--x X] [--y Y] [--rz DEG]        set_pose it; omitted values are kept
  remove ID                                 delete it
  list                                      objects and timed events of this run
  save FILE                                 write the current run as a schema-v1 scenario
  run FILE                                  (docker/start.sh) spawn FILE's objects, then fire its events

State: $GREENHOUSE_STATE/scenario-state.json, replaced atomically under scenario.lock.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from greenhouse.config import Config
from greenhouse.scenario import (RESERVED_GREENHOUSE, STATE_FILE, VERSION, check_inside, check_move, check_object,
                                 locked, model_name, model_sdf, ordered_events, read_state, rest_pose, validate,
                                 write_json)
from greenhouse.terrain import Terrain
from greenhouse.transport import service, world_status

DIRECTORY = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))


def world_config() -> dict:
    """Greenhouse config in use: the live terrain revision's if one was applied, else the start-up one."""
    active = json.loads((DIRECTORY / "terrain-state.json").read_text())
    return active.get("config") or json.loads((DIRECTORY / "runtime.json").read_text())["world_config"]


def terrain() -> Terrain:
    """Current ground surface (same triangles as the collision mesh)."""
    active = json.loads((DIRECTORY / "terrain-state.json").read_text())
    data = json.loads((Path(active["assets_dir"]) / "terrain.json").read_text())
    return Terrain(np.empty((0, 0)), np.array(data["heights"]), data["resolution"], data["origin"])


def pose_text(pose: dict) -> str:
    """gz.msgs.Pose position/orientation fields for a pose in m and rad (extrinsic roll, pitch, yaw)."""
    r, p, y = (pose[k] / 2 for k in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (f"position {{ x: {pose['x']} y: {pose['y']} z: {pose['z']} }} "
            f"orientation {{ x: {sr*cp*cy-cr*sp*sy} y: {cr*sp*cy+sr*cp*sy} "
            f"z: {cr*cp*sy-sr*sp*cy} w: {cr*cp*cy+sr*sp*sy} }}")


def placed(item: dict) -> tuple[dict, dict]:
    """(state record, gz pose) for an object {id, type, x, y, rz} resting on the current terrain."""
    check_inside(Config(**world_config()), item["type"], item["x"], item["y"], item["rz"], item["id"])
    pose = rest_pose(terrain(), item["type"], item["x"], item["y"], math.radians(item["rz"]))
    record = {key: item[key] for key in ("id", "type", "x", "y", "rz")}
    return dict(record, model=model_name(item["id"]), z=round(pose["z"], 6)), pose


def add(state: dict, item: dict) -> dict:
    """Spawn ``item`` and append it to ``state``; the id must be new."""
    if any(o["id"] == item["id"] for o in state["objects"]):
        raise ValueError(f"object {item['id']!r} already exists")
    record, pose = placed(item)
    service("/world/greenhouse/create/blocking", "gz.msgs.EntityFactory",
            request=f"sdf: {json.dumps(model_sdf(item['type'], item['id']))} name: {json.dumps(record['model'])} "
                    f"allow_renaming: false pose {{ {pose_text(pose)} }}")
    state["objects"].append(record)
    return record


def find(state: dict, object_id: str) -> int:
    index = next((i for i, o in enumerate(state["objects"]) if o["id"] == object_id), None)
    if index is None:
        raise ValueError(f"no object {object_id!r}; current: {[o['id'] for o in state['objects']]}")
    return index


def move(state: dict, update: dict) -> dict:
    """Re-rest an existing object at the merged x, y, rz and set_pose it."""
    index = find(state, update["id"])
    record, pose = placed({**state["objects"][index], **update})
    service("/world/greenhouse/set_pose/blocking", "gz.msgs.Pose",
            request=f"name: {json.dumps(record['model'])} {pose_text(pose)}")
    state["objects"][index] = record
    return record


def remove(state: dict, object_id: str) -> dict:
    """Delete an existing object from Gazebo and from ``state``."""
    record = state["objects"][find(state, object_id)]
    service("/world/greenhouse/remove/blocking", "gz.msgs.Entity",
            request=f"name: {json.dumps(record['model'])} type: MODEL")
    state["objects"].remove(record)
    return record


def change(action, *arguments) -> dict:
    """Apply one action under the lock and persist the state; returns the action's record."""
    with locked(DIRECTORY):
        state = read_state(DIRECTORY)
        try:
            return action(state, *arguments)
        finally:
            write_json(DIRECTORY / STATE_FILE, state)


def wait_world(timeout: float = 900) -> dict:
    """World status once Gazebo answers and the Jackal exists; `timeout` in wall seconds."""
    end, last = time.monotonic() + timeout, None
    while time.monotonic() < end:
        try:
            status = world_status()
            if status.get("robot", {}).get("name"):
                return status
        except (RuntimeError, TimeoutError, subprocess.SubprocessError) as error:
            last = error
        time.sleep(0.5)
    raise TimeoutError(f"Gazebo world not ready for scenario objects: {last}")


def run(path: Path) -> int:
    """Spawn the scenario's objects once the world is up, then fire its events at their sim times."""
    scenario = json.loads(path.read_text())
    config = json.loads((DIRECTORY / "runtime.json").read_text())["world_config"]
    actuator = json.loads((DIRECTORY / "actuator.json").read_text())["config"]
    scenario, _, _ = validate(scenario, config, actuator)
    wait_world()
    for item in scenario["objects"]:
        print(json.dumps({"scenario": "add", **change(add, item)}), flush=True)
    for index, event in sorted(enumerate(scenario["events"]), key=lambda pair: pair[1]["t"]):
        while (now := world_status()["sim_time"]) < event["t"]:
            time.sleep(0.05)
        fields = {k: v for k, v in event.items() if k not in ("t", "action")}
        outcome = {"fired_sim_time": now}
        try:
            action = {"add": lambda s: add(s, fields), "move": lambda s: move(s, fields),
                      "remove": lambda s: remove(s, event["id"])}[event["action"]]
            outcome["result"] = change(action)
        except (OSError, ValueError, RuntimeError, TimeoutError) as error:
            outcome["error"] = str(error)
        with locked(DIRECTORY):
            state = read_state(DIRECTORY)
            if index < len(state["events"]):
                state["events"][index].update(outcome)
            write_json(DIRECTORY / STATE_FILE, state)
        print(json.dumps({"scenario": "event", "t": event["t"], "action": event["action"], **outcome}), flush=True)
    return 0


def save(path: Path) -> dict:
    """Write the running sim as a scenario: seed and configs in use, robot truth pose, current objects.

    Timed events are not carried over: the file is a snapshot that starts at sim time 0.
    """
    config = world_config()
    actuator = json.loads((DIRECTORY / "actuator.json").read_text())["config"]
    robot = world_status()["robot"]
    with locked(DIRECTORY):
        objects = [{k: o[k] for k in ("id", "type", "x", "y", "rz")} for o in read_state(DIRECTORY)["objects"]]
    scenario = {"version": VERSION, "seed": config["seed"],
                "greenhouse": {k: v for k, v in config.items() if k not in RESERVED_GREENHOUSE},
                "actuator": actuator,
                "start": {"x": round(robot["x"], 4), "y": round(robot["y"], 4),
                          "rz": round(math.degrees(robot["yaw"]), 3)},
                "objects": objects, "events": []}
    validate(scenario, config, actuator)
    write_json(path, scenario)
    return {"saved": str(path), "start": scenario["start"], "objects": len(objects)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    adding = commands.add_parser("add")
    adding.add_argument("type")
    adding.add_argument("--id", required=True)
    moving = commands.add_parser("move")
    moving.add_argument("id")
    for sub, required in ((adding, True), (moving, False)):
        sub.add_argument("--x", type=float, required=required, help="m, greenhouse frame")
        sub.add_argument("--y", type=float, required=required, help="m, greenhouse frame")
        sub.add_argument("--rz", type=float, help="deg, counter-clockwise from +x")
    commands.add_parser("remove").add_argument("id")
    commands.add_parser("list")
    commands.add_parser("save").add_argument("file", type=Path)
    commands.add_parser("run").add_argument("file", type=Path)
    args = parser.parse_args()
    values = {k: v for k, v in vars(args).items() if k in ("id", "type", "x", "y", "rz") and v is not None}
    try:
        if args.command == "add":
            result = change(add, check_object(values))
        elif args.command == "move":
            result = change(move, check_move(values))
        elif args.command == "remove":
            result = change(remove, args.id)
        elif args.command == "list":
            result = read_state(DIRECTORY)
        elif args.command == "save":
            result = save(args.file.resolve())
        else:
            return run(args.file)
    except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError) as error:
        print(f"scenario {args.command} failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2 if args.command == "list" else None))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
