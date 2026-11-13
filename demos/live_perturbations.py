# SPDX-License-Identifier: GPL-3.0-only
"""Demo 2: live perturbations through the scripting API, seen by the lidar and the cameras.

Run:  bash run.sh --headless --viz --scenario demos/scenarios/disturbances.json --script demos/live_perturbations.py
(drop --headless to also watch in Gazebo). The scenario spawns five objects and plays timed events
(a worker walking down the y = -4 aisle, a tomato falling, a robot moving, a sack removed) while this
script parks in the centre aisle, drops each object type 1.5 m ahead of the lidar, records what the
lidar and camera 1 see, walks a person out of the way, then saves the whole world as a new scenario.

Writes results/perturbations-<time>/: visibility.json, a camera PNG per object, contact_sheet.png,
saved_scenario.json.
Replay the saved world:  bash run.sh --headless --viz --scenario demos/results/perturbations-<time>/saved_scenario.json
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np

from demo_ros import Listener
from self_driving_api import (add_object, capture, configure, drive_to, get_state, move_object, pose, remove_object,
                              save_scenario, wait)

configure(verbose=True)
listener = Listener()
out = Path("results") / time.strftime("perturbations-%Y%m%d-%H%M")
out.mkdir(parents=True, exist_ok=True)
STOP_X, AHEAD = -5.0, 1.5  # m: parking spot in the centre aisle; object distance from the lidar
LIDAR_X = 0.12             # m: scan origin ahead of base_link (config/actuator.json lidar mount)


def say(text: str) -> None:
    print(f"\n[demo] {text}", flush=True)


say("Driving to the centre aisle; the scenario's timed events play meanwhile (watch the worker walk).")
drive_to(STOP_X, 0.0, 0.0, speed=0.8)
say("Carriage up to 0.8 m and camera 1 turned to look ahead and down (at lift 0 camera 2 blocks its right half).")
pose({"lift": 800, "lift_speed": 0.2, "y1": -90, "p1": -35, "cam1_rot_speed": 60})
x = get_state()["pose"]["x"] + LIDAR_X + AHEAD
empty = listener.sector_min(10)
capture(1, out / "00_empty.png")
say(f"Empty aisle: closest lidar return within +-10 deg ahead = {empty:.2f} m")

rows = []
for kind, rz in (("person", 180), ("robot", 90), ("fertiliser_block", 90), ("fallen_plant", 90), ("fallen_tomato", 0)):
    record = add_object({"id": f"probe_{kind}", "type": kind, "x": x, "y": 0.0, "rz": rz})
    wait(1.0)  # sim s: let the new model render before the scan and the frame
    seen = listener.sector_min(10)
    image = f"{len(rows) + 1:02d}_{kind}.png"
    capture(1, out / image)
    visible = seen < empty - 0.2
    rows.append({"type": kind, "distance_m": round(AHEAD, 3), "resting_z_m": round(record["z"], 3),
                 "lidar_closest_ahead_m": round(seen, 3), "lidar_sees_it": visible, "camera1_image": image})
    say(f"{kind:17s} at {AHEAD} m ahead: lidar closest = {seen:.2f} m -> {'SEEN' if visible else 'NOT seen'}")
    remove_object(f"probe_{kind}")

say("A person stands in the aisle again, then steps aside in 0.15 m strides (move_object).")
add_object({"id": "probe_person", "type": "person", "x": x, "y": 0.0, "rz": 180})
for y in (0.15, 0.3, 0.45, 0.6):
    move_object({"id": "probe_person", "y": y})
    say(f"person at y = {y:.2f} m: lidar closest ahead = {listener.sector_min(10):.2f} m")
capture(1, out / "06_person_aside.png")
remove_object("probe_person")
pose({"lift": 0, "y1": 0, "p1": 0})

LAST_EVENT_T = 80.0  # sim s, after the last timed event in scenarios/disturbances.json
if listener.sim_now() < LAST_EVENT_T:
    say(f"Waiting until sim t = {LAST_EVENT_T:.0f} s for the scenario's remaining timed events (now {listener.sim_now():.0f} s).")
    wait(LAST_EVENT_T - listener.sim_now())

saved = save_scenario(out / "saved_scenario.json")
say(f"World saved as a scenario ({saved['objects']} objects, start {saved['start']}).")

frames = [cv2.resize(cv2.imread(str(out / name)), (480, 360)) for name in ["00_empty.png"] + [r["camera1_image"] for r in rows]]
cv2.imwrite(str(out / "contact_sheet.png"), np.vstack([np.hstack(frames[:3]), np.hstack(frames[3:])]))

summary = {"lidar_scan_height_above_base_link_m": 0.2465, "empty_aisle_closest_ahead_m": round(empty, 3),
           "objects": rows, "saved_scenario": "saved_scenario.json", "contact_sheet": "contact_sheet.png"}
(out / "visibility.json").write_text(json.dumps(summary, indent=2) + "\n")
say(f"Results in demos/{out}")
