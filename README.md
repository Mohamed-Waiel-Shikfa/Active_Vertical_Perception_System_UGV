# Jackal greenhouse simulation

A Clearpath Jackal (J100) drives through a physically uneven greenhouse with an FPB30
vertical actuator on its rear deck. Two RGB cameras on XF C-200T 3-axis stabilising gimbals
hang under the lift carriage (camera 1 looks left, camera 2 looks right). It is a test bed for
active vertical perception: moving cameras up and down plant rows from a ground robot.
Everything runs in one Docker container; the host needs no ROS.

## Quick start (Linux)

Needs Docker Engine usable by your user, Bash, and an X11 or Xwayland session for the window.
The pinned base image is linux/amd64, about 1.7 GB compressed; allow several GB of disk.

```bash
cd /path/to/jackal-greenhouse-sim
bash run.sh
```

The first run builds the image (several minutes), then opens Gazebo with the drive panel.
Later runs rebuild only what changed. `Ctrl+C` stops this container only.

```bash
bash run.sh --no-build                          # restart without building
bash run.sh --script self_driving/sample_use.py # run a scripted mission, sim stays open
bash run.sh --smoke-test                        # headless start + short drive, pass/fail
bash run.sh --help                              # every option, grouped, with examples
```

### How the options fit together

| You want to | Options | Notes |
|---|---|---|
| Drive it yourself | *(none)* | Builds if needed, opens the window. |
| Restart fast | `--no-build` | Refused if the image was built from other sources; then run without it, or add `--allow-stale-image`. Editing `config/greenhouse.json` or `config/actuator.json` never needs a rebuild. |
| Run without a window | `--headless` | Cameras still render, on the GPU if `/dev/dri` exists. Combine with `--script` for unattended runs. Not with `--rviz` or `--gui-test`. |
| Watch topics, plots and cameras | `--viz` | Starts foxglove_bridge on `ws://127.0.0.1:8767` (this computer only), headless or with the window. Open it in a recent Lichtblick and import `config/foxglove-layout.json`; its System graph tab (one robot subsystem at a time) needs the panel extension, installed once with `bash viz/subsystem-graph/install.sh` (Node and npm on the host), then a Lichtblick restart. The layout assumes namespace `j100_0000`; for another namespace edit its topic names. Details: [docs/TECHNICAL.md](docs/TECHNICAL.md#visualisation-hub---viz). |
| Script a mission | `--script FILE` | Only in the default run mode (window or `--headless`). |
| Replay a scenario | `--scenario FILE` | Seed, config overrides, robot start, obstacles and timed events from one JSON ([example](config/scenario.example.json), [schema](docs/TECHNICAL.md#scenarios-and-perturbation-objects)). Only in the default run mode. |
| Check everything starts | `--smoke-test` | Implies `--headless`; exits 0 on pass. At most 4 minutes. |
| Export the world only | `--generate-only` | Writes `SETUP_DIR/greenhouse/generated`; no ROS. |
| Only build | `--build-only` | Conflicts with `--no-build`. |
| GPU trouble | `--software-rendering` | CPU rendering; the sim then runs 2-4x slower than real time. |
| A second simulation | `--name gh2 --setup-dir DIR2` | Two runs must not share a name or a setup folder. |
| Your own robot.yaml | `--setup-dir DIR` | DIR must hold `robot.yaml` and be writable; run state goes to `DIR/greenhouse/`. |
| See what would run | `--dry-run` | Prints the docker commands, runs nothing. |

Setup folder: if `./robot.yaml` exists in this folder it is used; otherwise
`runtime/clearpath/` is used and seeded once from `config/robot.example.yaml`. An existing
`robot.yaml` is never overwritten. Robot namespace, stock wheel controllers and bridges are
kept as Clearpath generates them.

## Controls

Click the 3D scene to give it keyboard focus, then:

| Key | Action |
|---|---|
| Up / Down | Forward / reverse |
| Left / Right | Turn left / right |
| A / Q | Raise / lower the carriage |
| C | Cycle view: 3D -> camera 1 (left) -> camera 2 (right) -> 3D |
| P / R / Y | Hold to raise camera 1 pitch / roll / yaw at the key rotation speed |
| Alt + P / R / Y | Same for camera 2 |
| Shift (+ Alt) + P / R / Y | Lower instead of raise |
| Space / Esc | Stop base and lift |

Motion stops when you release the key or the window or an input field takes focus. Holding
opposite keys cancels that axis. C advances one view per press and skips a camera without
live frames. P / R / Y read Alt and Shift when the key goes down and stop at the +/-90 deg
limits; the panel fields follow the live angles.

Drive and turn speeds default to the panel maximums, 2.0 m/s and 4.0 rad/s; lower them under
**Robot** for careful driving. The base controller ramps at up to 10 m/s² and 20 rad/s²
(`config/platform_overrides.yaml`; Clearpath's J100 file uses 2 and 4), so it starts moving
within about 0.1 s of a key press and stops within 0.15-0.5 s. The rest of the feel is wheel
slip on the soil: turning reaches about 40% of the commanded yaw rate.

Terminal driving in a running container (same arrows, A/Q; 0.2 m/s and 0.5 rad/s by default;
motion stops 0.35 s after key repeat ends; Esc or Ctrl-C exits):

```bash
docker exec -it jackal_greenhouse /opt/greenhouse/tools/teleop.sh
```

## Configuration

`config/greenhouse.json` holds the layout and terrain and is the only place their default
values are written down. A `--config` file may list only the keys it changes. Firm-earth
defaults: 0.15 m sampling, 0.03 m base height range, 3 m ground padding, 3 mm bump-band RMS
at 0.18 m scale; tomatoes 1.4 m tall; clear aisles 1.3 m. This is rigid contact on a
heightfield, not deformable or calibrated soil.

The five terrain fields, ground friction and both wheel-slip compliances are live under
**Parameters**. **Apply terrain / contact** cancels navigation, pauses, replaces support and
contact, and refreshes the collision map. An error keeps the applied revision; an unsafe
rollback leaves physics paused. Ground padding is not crop headland distance.

`config/actuator.json` holds the FPB30 and both gimbal cameras, again as the single source of
defaults; `--actuator-config` files may be partial.

```json
{ "stroke_m": 1.5, "speed_m_s": 0.2, "force_limit_n": 30.0,
  "camera_mass_kg": 0.15, "gimbal_mass_kg": 0.125, "gimbal_offset_m": 0.065,
  "gimbal_mount_yaw_deg": [90.0, -90.0],
  "gimbal_initial_rpy_deg": [[0, 0, 0], [0, 0, 0]], "gimbal_stabilise": [true, true] }
```

The FPB30/B42 has 1500 mm travel and 1677 mm body length; module mass 3.9 kg. The 30 N
effort cap and component inertias are engineering assumptions, not a calibrated torque
specification. Rail length and stroke need a restart; the live lift target and speed are
limited to the installed stroke and payload envelope. The panel always uses the full stroke
(the `<ns>/greenhouse/lift/settings` service still accepts a narrower `lower`/`upper`).

The same file's `lidar` section adds a SICK TiM551 2D lidar on the front of the top plate,
published as `sensor_msgs/LaserScan` on `<ns>/sensors/lidar2d_0/scan` (frame `lidar2d_0_laser`).
A partial file may set only some lidar keys, e.g. `{"lidar": {"enabled": false}}` removes it.
Sensor rates, frames and noise: [docs/TECHNICAL.md](docs/TECHNICAL.md#sensors).

## Gimbal cameras

Both cameras hang camera-down under a crossbar on the lift carriage, 0.065 m either side of
the rail centre (the closest spacing at which they cannot touch). At the lowest lift position
they clear the deck by about 3.5 cm. Gimbal 1 faces the robot's left (+Y), gimbal 2 its right
(-Y); at roll/pitch/yaw 0 each camera looks level, straight out of its side. Each camera must
stay within the C-200T's 200 g payload (`camera_mass_kg` is per camera, default 0.15 kg).

Under **Parameters -> Gimbal cameras** each camera has Roll, Pitch, Yaw (degrees),
**Stabilise (horizon lock)** and its own **Apply**. Every axis is limited to +/-90 deg: pitch
positive tilts up; yaw is measured from the outward direction, positive counterclockwise seen
from above. **Key rotation speed (deg/s)**, default 5, sets how fast held P / R / Y keys turn.
With Stabilise on, roll and pitch hold relative to gravity and yaw follows the robot heading;
off, angles are relative to the robot (FPV). Settings last for the session; startup values
come from `gimbal_initial_rpy_deg` / `gimbal_stabilise`.

ROS: images on `<ns>/greenhouse/camera1/image` and `camera2/image` (+ `camera_info`), gimbal
state JSON on `<ns>/greenhouse/gimbal/state`, commands as `std_msgs/String` JSON on
`<ns>/greenhouse/gimbal/command`, e.g. `{"camera": 2, "roll": 0, "pitch": -20, "yaw": 15,
"stabilise": true}`. Gimbal and lift joint angles are on `platform/joint_states`.

## Navigate

Enter **X (m)**, **Y (m)** and **Rotation Z (degrees)** below Parameters and press **Go**.
Coordinates are fixed to the greenhouse: origin at the house centre, +X along the rows, +Y
across them, positive yaw counterclockwise. **Cancel**, an arrow key or Stop cancels the goal;
an idle keyboard does not fight navigation. After a terrain/contact Apply, press Go again.

Nav2 Jazzy plans with Smac 2D and follows the path with Regulated Pure Pursuit, using the
simulator's true pose and velocity rather than wheel odometry (there is no SLAM or lidar).
**Go** drives at the panel's **Drive (m/s)** on straights and turns at up to **Turn (rad/s)**.
It turns towards the path first, slows before row-end hairpins and the goal, and limits
wheelspin. A goal facing away from the arrival direction ends with a short docking move. Success
means the robot is at rest within 0.075 m and 0.07 rad (4°) of the goal. It does not predict
soil shear or guarantee recovery. From a terminal:
`nav_command.py go --x X --y Y --yaw DEG [--speed M_S] [--turn RAD_S]`.
Details: [navigation walkthrough](docs/navigation-stack.html) and
[beginner explainer](docs/navigation-explained.html).

## Scripting API

`self_driving/self_driving_api.py` is built into the image. Pass a script to the launcher; it
starts once the simulation is ready, prints a banner when it finishes or fails, and leaves the
simulation open:

```bash
bash run.sh --script self_driving/sample_use.py
```

```python
from self_driving_api import pose, apply_terrain
apply_terrain({"bump_band": 2, "ground_friction": 0.9})
pose({"x": 5, "y": 2, "rz": -180, "lift": 500, "p1": 30, "drive_speed": 1})
pose({"p1": 0})  # omitted keys keep their last value
pose({"x": 6.5, "y": 2, "oriented": False})  # keep the arrival heading (this pose only)
```

Each call blocks until the robot has physically arrived. Units: metres and degrees, lift in mm,
`lift_speed`/`drive_speed` in m/s, `turn_speed` in rad/s, gimbal `camN_rot_speed` in deg/s
(simulated time). Terrain keys and units match the GUI panel. Out-of-range values are clamped
with a warning (`configure(strict=True)` raises instead); unknown keys always raise. The
module docstring lists every key. `self_driving/advanced_use.py` shows `get_state`, `capture`
(camera PNG), `wait`, `stop`, `drive_to`/`set_lift`/`set_gimbal` and `configure`. The script's
folder is mounted read-write as its working directory, so outputs land next to it.

Perturbation objects (`fertiliser_block`, `fallen_tomato`, `fallen_plant`, `robot`, `person`)
can be added, moved and removed live, from a script (`add_object`, `move_object`,
`remove_object`, `save_scenario`) or from a host shell:

```bash
docker exec jackal_greenhouse /ros_entrypoint.sh python3 /opt/greenhouse/tools/scenario.py add person --id p1 --x -6 --y 0 --rz 90
docker exec jackal_greenhouse /ros_entrypoint.sh python3 /opt/greenhouse/tools/scenario.py move p1 --y -1
docker exec jackal_greenhouse /ros_entrypoint.sh python3 /opt/greenhouse/tools/scenario.py save /etc/clearpath/greenhouse/my-scenario.json
bash run.sh --scenario runtime/clearpath/greenhouse/my-scenario.json   # replay it later
```

Presentation demos (Lichtblick tour, live perturbations, baseline experiment) and their run
commands: [demos/README.md](demos/README.md).

## Printable mount

Editable source: `cad/rear_deck_mount.scad`; generated prototype:
`cad/generated/rear_deck_bracket.stl` (millimetres). The bare deck uses 120 mm square M5x0.8
holes; set `deck_hole_pitch_mm` to 80 for the optional PACS plate. Measure before printing and
keep the rear HMI / Motion Stop accessible. The selected startup mount is also written to
`SETUP_DIR/greenhouse/cad/`.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `image ... was built from different source` | You ran `--no-build` after changing code. Run without `--no-build`, or add `--allow-stale-image` deliberately. |
| `container 'jackal_greenhouse' already exists` | A previous run is still up. Stop it (`docker stop jackal_greenhouse`) or use `--name`. Nothing is deleted for you. |
| `a local X11/Xwayland DISPLAY ... is required` / `X11 socket ... is missing` | No local display (e.g. SSH). Use `--headless`. |
| Keys do nothing | Click the 3D scene: an input field or another window has focus. |
| Robot reacts slowly, everything feels sluggish | The sim runs slower than real time. Do not use `--software-rendering` unless the GPU fails; check `ls /dev/dri`. Lower Drive/Turn under **Robot** for gentler starts and stops. |
| Navigation stays "Starting" | Occasional Nav2 lifecycle race at startup; restart the container. |
| Build fails: `Locked compatible binary ... is unavailable` | ROS apt republished Nav2; refresh `config/nav-packages.lock.json` to the current builds of the same release. |
| Host runs low on memory | One simulation at a time. Copy the command from `bash run.sh --dry-run ...` and add `--memory 5g --memory-swap 5g`. |

Physics assumptions, dimensions, verification commands and implementation notes:
[technical reference](docs/TECHNICAL.md). Source and license attribution:
[third-party notices](THIRD_PARTY_NOTICES.md).
