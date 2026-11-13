"""Bounded runtime-field verification against regenerated physical mesh and native revision."""
import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path

from greenhouse.transport import world_status


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    directory = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
    args.output.mkdir(parents=True, exist_ok=True)
    state_file = directory / "terrain-state.json"
    changes = [
        ("ground_resolution", 0.12), ("ground_height_range", 0.02),
        ("ground_headland", 3.5), ("ground_bump_rms", 0.006),
        ("ground_bump_scale", 0.24), ("ground_friction", 0.7),
    ]
    checks = []
    for key, value in changes:
        before = world_status()
        active = json.loads(state_file.read_text())
        old = Path(active["assets_dir"]) / "models/greenhouse_ground/meshes/terrain.dae"
        old_hash = hashlib.sha256(old.read_bytes()).hexdigest()
        process = subprocess.run(["python3", "/opt/greenhouse/tools/apply_terrain.py",
                                  "--values", json.dumps({key: value})],
                                 capture_output=True, text=True, timeout=165)
        (args.output / f"{key}.log").write_text(process.stdout+process.stderr)
        if process.returncode:
            raise RuntimeError(f"{key}: {process.stderr}")
        active = json.loads(state_file.read_text())
        after = world_status()
        assert active["config"][key] == value
        assert active["robot_id_before"] == active["robot_id_after"] == before["robot"]["id"]
        assert active["before_sim_time"] == active["after_sim_time"]
        assert after["sim_time"] >= before["sim_time"]
        assert active["collision_uri"] == active["visual_uri"]
        new = Path(active["assets_dir"]) / "models/greenhouse_ground/meshes/terrain.dae"
        new_hash = hashlib.sha256(new.read_bytes()).hexdigest()
        if key != "ground_friction":
            assert old_hash != new_hash
        checks.append({"field": key, "requested": value, "revision": active["revision"],
                       "mesh_changed": old_hash != new_hash, "robot_id": after["robot"]["id"],
                       "paused_clock": active["before_sim_time"]})
        (args.output/"result.json").write_text(json.dumps(checks,indent=2)+"\n")
    original = state_file.read_bytes()
    for values in ({"ground_resolution": 0.001}, {"ground_height_range": -0.1},
                   {"ground_headland": 2.0}, {"ground_bump_scale": "bad"}):
        process = subprocess.run(["python3", "/opt/greenhouse/tools/apply_terrain.py",
                                  "--values", json.dumps(values)], capture_output=True, text=True, timeout=100)
        assert process.returncode != 0
        assert state_file.read_bytes() == original
    rollback = subprocess.run(["python3","/opt/greenhouse/tools/apply_terrain.py",
                               "--values",'{"ground_bump_rms":0.004}',"--test-rollback"],
                              capture_output=True,text=True,timeout=150)
    assert rollback.returncode == 1
    assert state_file.read_bytes()==original
    assert len(world_status()["grounds"])==1
    (args.output/"rollback.log").write_text(rollback.stdout+rollback.stderr)
    print("All runtime fields changed their actual mesh/collider; invalid proposals and rollback preserved revision.")


if __name__ == "__main__":
    main()
