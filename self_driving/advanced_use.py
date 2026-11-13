"""Advanced self_driving_api example: a small multi-height camera survey of the centre aisle.

Run with:  bash run.sh --script self_driving/advanced_use.py
Images and a CSV log land in self_driving/survey/ on the host (the script's folder is mounted).
"""
import csv
from pathlib import Path

from self_driving_api import (apply_terrain, capture, configure, drive_to, get_state, pose,
                              set_gimbal, set_lift, stop, wait, wait_ready)

# Tunables: raise instead of clamping out-of-range values, and demand a tighter camera settle.
configure(strict=True, gimbal_tolerance_deg=0.5, gimbal_settle_s=8.0)
wait_ready()  # already done by run.sh before this script starts; harmless to call again

start = get_state()
print("start pose:", start["pose"])
print("start terrain:", start["terrain"])

# Gentle, grippy floor for a clean survey; returns what is now applied (API names and units).
print("terrain now:", apply_terrain({"bump_band": 1.5, "ground_friction": 1.0, "slip_lat": 1.0}))

out = Path("survey")
out.mkdir(exist_ok=True)
with (out / "log.csv").open("w", newline="") as stream:
    log = csv.writer(stream)
    log.writerow(["stop", "lift_mm", "camera", "sim_time", "x", "y", "rz", "lift_measured_mm",
                  "pitch", "yaw", "roll", "image"])

    # Cameras face the plant rows on each side of the aisle (mounted at +/-90 deg yaw).
    pose({"p1": -10, "y1": 0, "r1": 0, "p2": -10, "y2": 0, "r2": 0,
          "cam1_rot_speed": 30, "cam2_rot_speed": 30, "stab1": True, "stab2": True})

    for stop_index, x in enumerate((-4.0, 2.0)):
        drive_to(x, 0.0, 0.0, speed=0.6, turn_speed=1.5)
        for lift_mm in (0, 700, 1400):
            set_lift(lift_mm, speed=0.15)
            wait(0.5)  # let the mast stop swaying (simulated seconds)
            for camera in (1, 2):
                name = f"stop{stop_index}_lift{lift_mm}_cam{camera}.png"
                capture(camera, out / name)
                state = get_state()
                p = state["pose"]
                log.writerow([stop_index, lift_mm, camera, f"{state['sim_time']:.2f}",
                              f"{p['x']:.3f}", f"{p['y']:.3f}", f"{p['rz']:.1f}", f"{p['lift']:.1f}",
                              f"{p[f'p{camera}']:.1f}", f"{p[f'y{camera}']:.1f}", f"{p[f'r{camera}']:.1f}",
                              name])

        # Slow, scan-like pitch sweep on camera 1 while the lift is high.
        for pitch in (-30, 15, -10):
            set_gimbal(1, pitch=pitch, speed=10)
        set_lift(0)  # never drive with the mast up

# Park: lift down, cameras level, back to the spawn point, then make sure nothing keeps moving.
pose({"lift": 0, "p1": 0, "p2": 0, "x": -7.5, "y": 0, "rz": 0, "drive_speed": 1.0})
stop()
print("survey written to", out.resolve())
