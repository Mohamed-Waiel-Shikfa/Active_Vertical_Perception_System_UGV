# SPDX-License-Identifier: GPL-3.0-only
"""Demo 1: a narrated tour of the Lichtblick hub (sensors, TF, cameras, system graph, odometry error).

Run:  bash run.sh --headless --viz --script demos/viz_tour.py
Then connect Lichtblick to ws://127.0.0.1:8767 with config/foxglove-layout.json. Each "[tour]" line
in the terminal names the tab to show while that step plays. About 3 min simulated.
"""
import math
import time

from demo_ros import Listener
from self_driving_api import configure, drive_to, get_state, pose, set_gimbal, set_lift, wait

configure(verbose=False)
listener = Listener()
listener.recording_errors = True


def say(text: str) -> None:
    print(f"\n[tour] {text}", flush=True)


def errors() -> str:
    rows = {key: rows[-1] for key, rows in listener.errors.items() if rows}
    return ", ".join(f"{key} {row[2]:.2f} m / {math.degrees(row[3]):+.0f} deg" for key, row in rows.items())


say("Connect Lichtblick now (ws://127.0.0.1:8767). Starting in 20 s wall.")
time.sleep(20)

say("TAB 'Robot and cameras': the whole Jackal in 3D, lidar scan (15 Hz) and both cameras, RGB + mono.")
say("Lift the FPB30 carriage to 1.2 m: watch fpb30_carriage and both camera frames rise in the TF tree.")
pose({"lift": 1200, "lift_speed": 0.15, "p1": -10, "p2": -10, "cam1_rot_speed": 40, "cam2_rot_speed": 40})
say("Gimbal sweep: camera 1 pans across the left row, camera 2 tilts over the right row.")
set_gimbal(1, yaw=-45, speed=30)
set_gimbal(1, yaw=45, speed=30)
set_gimbal(1, yaw=0, pitch=0, speed=60)
set_gimbal(2, pitch=-40, speed=30)
set_gimbal(2, pitch=20, speed=30)
set_gimbal(2, pitch=0, speed=60)
say("Lift back down: the lidar's left sector is shadowed by the gimbal below ~3 cm (known, documented).")
set_lift(0, speed=0.2)

say("TAB 'System graph': pick a subsystem (Sensors, Pose & TF, Navigation, Evaluation). 15 s pause.")
wait(15)

say("TAB 'Odometry vs truth': drive the centre aisle at 1 m/s. Truth, wheel odometry and EKF overlay.")
drive_to(6.5, 0.0, 0.0, speed=1.0, turn_speed=2.0)
say(f"End of aisle. Drift since spawn: {errors()}")

say("TAB 'IMU' then 'Odometry error': 180 deg spin. Gyro spikes; wheel/EKF yaw over-reads (skid slip).")
drive_to(6.5, 0.0, 180.0, speed=1.0, turn_speed=2.0)
say(f"After the spin: {errors()}")

say("Back down the next aisle (y = 2). The EKF error keeps growing: it fuses wheel yaw, not IMU yaw.")
pose({"x": 6.5, "y": 2.0, "oriented": False, "drive_speed": 1.0, "turn_speed": 2.0})
pose({"x": -6.5, "y": 2.0, "oriented": False})
p = get_state()["pose"]
say(f"Tour done. Final drift: {errors()}. Truth pose: x {p['x']:.2f} m, y {p['y']:.2f} m, rz {p['rz']:.0f} deg")
