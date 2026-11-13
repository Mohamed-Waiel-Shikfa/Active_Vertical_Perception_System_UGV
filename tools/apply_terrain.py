"""Checked pause-and-apply transaction for real terrain and DART wheel contact."""
import argparse
import fcntl
import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path
from xml.etree import ElementTree as ET

sys.path.insert(0, "/opt/greenhouse")
import numpy as np
from greenhouse.terrain import Terrain
from greenhouse.terrain_update import proposed_config, support_points, validate_support
from greenhouse.transport import service, settings, wheel_contact, world_status
from greenhouse.atomic import replace


def wait_status(predicate, timeout=20):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = world_status()
        if predicate(state):
            return state
        time.sleep(0.1)
    raise TimeoutError("world did not acknowledge the requested terrain state")


def pose_text(name, pose):
    r, p, y = (pose.get(k, 0.0) / 2 for k in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (f"name: {json.dumps(name)} position {{ x: {pose['x']} y: {pose['y']} z: {pose['z']} }} "
            f"orientation {{ x: {sr*cp*cy-cr*sp*sy} y: {cr*sp*cy+sr*cp*sy} "
            f"z: {cr*cp*sy-sr*sp*cy} w: {cr*cp*cy+sr*sp*sy} }}")


def update_poses(plants, robot):
    poses = [pose_text(p["name"], p) for p in plants] + [pose_text(robot["name"], robot)]
    service("/world/greenhouse/set_pose_vector/blocking", "gz.msgs.Pose_V",
            request=" ".join("pose { " + p + " }" for p in poses))


def model_file(assets, destination):
    root = ET.parse(assets / "models/greenhouse_ground/model.sdf")
    for node in root.findall(".//uri") + root.findall(".//albedo_map"):
        if node.text.startswith("model://greenhouse_ground/"):
            node.text = (assets / "models" / node.text.removeprefix("model://")).resolve().as_uri()
    root.write(destination, encoding="utf-8", xml_declaration=True)


def create_ground(name, path):
    service("/world/greenhouse/create/blocking", "gz.msgs.EntityFactory",
            request=f"sdf_filename: {json.dumps(str(path))} name: {json.dumps(name)} allow_renaming: false")


def remove_ground(name):
    service("/world/greenhouse/remove/blocking", "gz.msgs.Entity",
            request=f"name: {json.dumps(name)} type: MODEL")


def terrain(path):
    data = json.loads((path / "terrain.json").read_text())
    return Terrain(np.empty((0, 0)), np.array(data["heights"]), data["resolution"], data["origin"])


def apply_contact_only(directory, active, current, config, actuator, runtime):
    """Change the two native WheelSlip coefficients without reloading a mesh."""
    blocked = False
    paused = False
    was_paused = False
    nav_prepared = False
    try:
        if runtime.get("navigation_enabled"):
            from greenhouse.navigation_api import request
            request("prepare_update", {})
            nav_prepared = True
        before = world_status()
        if before["robot"].get("speed", 1) > 0.02 or before["robot"].get("angular_speed", 1) > 0.03:
            raise ValueError("Stop Jackal before changing contact")
        settings(actuator["prefix"], {"blocked": True})
        blocked = True
        was_paused = before["paused"]
        paused = True
        service("/world/greenhouse/control", "gz.msgs.WorldControl", request="pause: true")
        frozen = wait_status(lambda s: s["paused"])
        wheel_contact(frozen["robot"]["name"], config)
        applied = dict(active, config=config, revision=uuid.uuid4().hex[:12], contact_only=True,
                       before_sim_time=frozen["sim_time"], after_sim_time=frozen["sim_time"],
                       robot_id_before=frozen["robot"]["id"], robot_id_after=frozen["robot"]["id"],
                       collision_uri=frozen["grounds"][0]["collision_uri"],
                       visual_uri=frozen["grounds"][0]["visual_uri"], base_support_z_change=0)
        if nav_prepared:
            request("commit_update", {"assets_dir": active["assets_dir"], "revision": applied["revision"]})
        settings(actuator["prefix"], {"blocked": False})
        blocked = False
        service("/world/greenhouse/control", "gz.msgs.WorldControl", request=f"pause: {str(was_paused).lower()}")
        wait_status(lambda s: s["paused"] == was_paused and (was_paused or s["sim_time"] > frozen["sim_time"]))
        temporary = directory / "terrain-state.tmp"
        temporary.write_text(json.dumps(applied, indent=2)+"\n")
        replace(temporary,directory / "terrain-state.json")
        print("Applied native wheel contact; unchanged ground mesh and robot.", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        print(f"Contact update failed: {error}", file=sys.stderr)
        try:
            if paused:
                service("/world/greenhouse/control", "gz.msgs.WorldControl", request="pause: true")
                wait_status(lambda s: s["paused"])
                wheel_contact(before["robot"]["name"], current)
            if blocked or paused:
                settings(actuator["prefix"], {"blocked": False})
            if nav_prepared:
                request("rollback_update", {"assets_dir": active["assets_dir"], "revision": active.get("revision", "startup")})
            if paused:
                service("/world/greenhouse/control", "gz.msgs.WorldControl", request=f"pause: {str(was_paused).lower()}")
        except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError) as rollback:
            print(f"Contact rollback failed; forcing pause and blocking motion: {rollback}", file=sys.stderr)
            try:
                service("/world/greenhouse/control", "gz.msgs.WorldControl", request="pause: true")
                settings(actuator["prefix"], {"blocked": True})
            except (RuntimeError, subprocess.SubprocessError) as stopping:
                print(f"Emergency hold was not acknowledged: {stopping}", file=sys.stderr)
        return 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rms-mm", type=int)
    parser.add_argument("--values", help="JSON object containing the terrain/contact fields")
    parser.add_argument("--test-rollback", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    directory = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
    state_path = directory / "terrain-state.json"
    runtime = json.loads((directory / "runtime.json").read_text())
    actuator = json.loads((directory / "actuator.json").read_text())
    lock = (directory / "terrain-apply.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(2, "Another terrain update is in progress\n")
    active = json.loads(state_path.read_text())
    old_assets = Path(active["assets_dir"])
    old_manifest = json.loads((old_assets / "manifest.json").read_text())
    current = active.get("config", old_manifest["config"])
    values = json.loads(args.values) if args.values else {}
    if args.rms_mm is not None:
        values["ground_bump_rms"] = args.rms_mm / 1000
    config = proposed_config(current, values)
    if config == current and not args.test_rollback:
        print("Settings already applied; no change.", flush=True)
        return 0
    changed_fields = {key for key in config if current.get(key) != config[key]}
    if changed_fields <= {"slip_longitudinal", "slip_lateral"} and not args.test_rollback:
        return apply_contact_only(directory, active, current, config, actuator, runtime)
    staged = directory / "terrain-revisions" / uuid.uuid4().hex[:12]
    staged.mkdir(parents=True)
    (staged / "config.json").write_text(json.dumps(config))
    name = "greenhouse_ground_" + staged.name
    frozen, blocked, changed = None, False, False
    pause_requested = False
    was_paused = False
    nav_prepared = False
    try:
        if runtime.get("navigation_enabled"):
            from greenhouse.navigation_api import request
            request("prepare_update", {})
            nav_prepared = True
        state = world_status()
        if state["robot"].get("speed", 1) > 0.02 or state["robot"].get("angular_speed", 1) > 0.03:
            raise ValueError("Stop Jackal before applying terrain/contact")
        if len(state["grounds"])!=1 or state["grounds"][0]["name"]!=active["name"]:
            raise RuntimeError("Uncommitted support revision is present; reconcile it before another Apply")
        print("Preparing real terrain, soil and collider revision...", flush=True)
        env = dict(os.environ)
        env["PYTHONPATH"] = "/opt/greenhouse/python:/opt/greenhouse:" + env.get("PYTHONPATH", "")
        process = subprocess.run(["python3", "-m", "greenhouse.generate", "--config", str(staged / "config.json"),
                                  "--output", str(staged / "assets")], env=env, capture_output=True,
                                 text=True, timeout=90)
        if process.returncode:
            raise RuntimeError(process.stderr.strip())
        assets = staged / "assets"
        manifest = json.loads((assets / "manifest.json").read_text())
        old_surface, new_surface = terrain(old_assets), terrain(assets)
        state = world_status()
        if state["robot"].get("speed", 1) > 0.02 or state["robot"].get("angular_speed", 1) > 0.03:
            raise ValueError("Jackal moved during preparation; stop and Apply again")
        validate_support(old_surface, current, state["robot"])
        validate_support(new_surface, config, state["robot"])
        model_file(assets, staged / "new.sdf")
        model_file(old_assets, staged / "old.sdf")
        settings(actuator["prefix"], {"blocked": True})
        blocked = True
        was_paused = state["paused"]
        pause_requested = True
        service("/world/greenhouse/control", "gz.msgs.WorldControl", request="pause: true")
        frozen = wait_status(lambda s: s["paused"])
        validate_support(new_surface, config, frozen["robot"])
        journal={"phase":"paused_prepared","previous":active,"requested":config,
                 "before":frozen,"new_name":name}
        (staged/"transaction.json").write_text(json.dumps(journal,indent=2)+"\n")
        print("Paused: installing physical support and contact settings...", flush=True)
        changed = True
        create_ground(name, staged / "new.sdf")
        wait_status(lambda s: any(g["name"] == name and g.get("collision_uri")
                                 and g.get("collision_uri") == g.get("visual_uri") for g in s["grounds"]))
        if args.test_rollback:
            raise RuntimeError("Requested rollback after staging new support")
        remove_ground(active["name"])
        wheel_contact(frozen["robot"]["name"], config)
        robot = frozen["robot"]
        shift = max(new_surface.sample(x, y) - old_surface.sample(x, y) for x, y in support_points(robot))
        new_robot = dict(robot, z=robot["z"] + shift + 0.002)
        update_poses(manifest["plants"], new_robot)
        after = wait_status(lambda s: s["paused"] and len(s["grounds"]) == 1 and s["grounds"][0]["name"] == name
                            and s["robot"]["id"] == robot["id"])
        if after["sim_time"] != frozen["sim_time"]:
            raise RuntimeError("Physics advanced during the support transaction")
        applied = {"name": name, "assets_dir": str(assets), "config": config, "revision": staged.name,
                   "before_sim_time": frozen["sim_time"], "after_sim_time": after["sim_time"],
                   "robot_id_before": robot["id"], "robot_id_after": after["robot"]["id"],
                   "collision_uri": after["grounds"][0]["collision_uri"],
                   "visual_uri": after["grounds"][0]["visual_uri"], "base_support_z_change": shift + 0.002}
        journal.update({"phase":"physical_applied","physical":after,"applied":applied})
        (staged/"transaction.json").write_text(json.dumps(journal,indent=2)+"\n")
        if nav_prepared:
            request("commit_update", {"assets_dir": str(assets), "revision": staged.name})
        settings(actuator["prefix"], {"blocked": False})
        blocked = False
        service("/world/greenhouse/control", "gz.msgs.WorldControl", request=f"pause: {str(was_paused).lower()}")
        wait_status(lambda s: s["paused"] == was_paused and (was_paused or s["sim_time"] > after["sim_time"]))
        # Persist only once both physical support and navigation revision succeeded.
        journal["phase"]="ready_to_commit"
        (staged/"transaction.json").write_text(json.dumps(journal,indent=2)+"\n")
        temp = state_path.with_suffix(".tmp")
        temp.write_text(json.dumps(applied, indent=2) + "\n")
        replace(temp,state_path)
        print("Applied revision " + staged.name + " (session only); Go plans a fresh goal.", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        print(f"Update failed: {error}", file=sys.stderr, flush=True)
        uncertain=changed and (isinstance(error,(TimeoutError,subprocess.TimeoutExpired))
                               or "timed out" in str(error).lower())
        if uncertain:
            try:
                observed=world_status()
                if not observed["paused"]:
                    service("/world/greenhouse/control","gz.msgs.WorldControl",request="pause: true")
                    observed=wait_status(lambda s:s["paused"])
                journal.update({"phase":"uncertain_mutation","error":str(error),"observed":observed})
                (staged/"transaction.json").write_text(json.dumps(journal,indent=2)+"\n")
                print("Uncertain mutation: physics remains paused. Reconcile observed support/contact/map before rollback or resume.",
                      file=sys.stderr,flush=True)
            except (OSError,ValueError,RuntimeError,TimeoutError,subprocess.SubprocessError) as inspection:
                print(f"Could not confirm paused recovery state: {inspection}",file=sys.stderr,flush=True)
            return 1
        try:
            if changed:
                observed=world_status()
                if not observed["paused"]:
                    service("/world/greenhouse/control", "gz.msgs.WorldControl", request="pause: true")
                    observed=wait_status(lambda s: s["paused"])
                names = {g["name"] for g in observed["grounds"]}
                if active["name"] not in names:
                    create_ground(active["name"], staged / "old.sdf")
                if name in names:
                    remove_ground(name)
                wheel_contact(frozen["robot"]["name"], current)
                update_poses(old_manifest["plants"], frozen["robot"])
                wait_status(lambda s: len(s["grounds"]) == 1 and s["grounds"][0]["name"] == active["name"])
            if nav_prepared:
                request("rollback_update", {"assets_dir": str(old_assets), "revision": active.get("revision", "startup")})
            if blocked or changed:
                settings(actuator["prefix"], {"blocked": False})
            if pause_requested or frozen or changed:
                service("/world/greenhouse/control", "gz.msgs.WorldControl", request=f"pause: {str(was_paused).lower()}")
            print("Previous support/contact/map restored; no restart.", file=sys.stderr, flush=True)
        except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError) as rollback:
            try:
                service("/world/greenhouse/control", "gz.msgs.WorldControl", request="pause: true")
            except (RuntimeError, subprocess.SubprocessError):
                pass
            print(f"ROLLBACK FAILED: keep paused and inspect support before resuming: {rollback}", file=sys.stderr, flush=True)
        return 1
    finally:
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
