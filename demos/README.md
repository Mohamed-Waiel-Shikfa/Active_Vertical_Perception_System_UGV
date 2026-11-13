# Demos (progress 2026-10-22 to 2026-11-05)

Run everything from `jackal-greenhouse-sim/`. One sim at a time (the `--viz` port is fixed).
Image is current: `--no-build` starts it immediately. Ctrl+C stops a sim.

## Lichtblick setup (once per session)

1. Start a demo with `--viz` (below).
2. Open Lichtblick → Open connection → Foxglove WebSocket → `ws://127.0.0.1:8767`.
3. Layout menu → Import from file → `config/foxglove-layout.json`.
   Tabs: Robot and cameras · System graph · Odometry vs truth · Odometry error · IMU.
   (System graph panel is installed; if missing: `bash viz/subsystem-graph/install.sh`, restart Lichtblick.)

## New demos

| # | Shows | Command | Time |
|---|---|---|---|
| 1 | Viz hub tour: 3D Jackal + TF + lidar, both cameras RGB/mono, System graph, odometry vs truth, IMU, EKF drift. Terminal prints `[tour]` lines naming the tab to show. | `bash run.sh --no-build --headless --viz --script demos/viz_tour.py` | ~3 min |
| 2 | Dynamic scenario + live perturbations: scenario spawns 5 objects and plays 7 timed events (worker walks, tomato falls, robot moves, sack removed); script drops each object type 1.5 m ahead, records lidar + camera, person steps aside, saves the world. | `bash run.sh --no-build --headless --viz --scenario demos/scenarios/disturbances.json --script demos/live_perturbations.py` | ~2 min |
| 2b | Replay the world saved by demo 2 | `bash run.sh --no-build --headless --viz --scenario demos/results/perturbations-<time>/saved_scenario.json` | — |
| 3 | Experiment: sensor audit, odometry drift on 2 terrains, lidar-only map scored vs truth | `bash run.sh --no-build --headless --viz --script demos/experiment_baseline.py` | ~12 min |

Drop `--headless` on 1 or 2 to also show the Gazebo window (objects visible there too).

## Existing demos (kept)

| Shows | Command |
|---|---|
| Interactive GUI: drive panel, Go to pose, terrain Apply | `bash run.sh --no-build` (arrows drive, A/Q lift, C cycles camera views, P/R/Y gimbal, Space stop) |
| Terminal teleop into a running sim | `docker exec -it jackal_greenhouse /opt/greenhouse/tools/teleop.sh` |
| Your contract script (serpentine all aisles) | `bash run.sh --no-build --script self_driving/sample_use.py` |
| Multi-height camera survey, PNGs + CSV in `self_driving/survey/` | `bash run.sh --no-build --script self_driving/advanced_use.py` |
| Example scenario (5 objects, 3 events) | `bash run.sh --no-build --scenario config/scenario.example.json` |
| Live object control from a shell | `docker exec jackal_greenhouse /ros_entrypoint.sh python3 /opt/greenhouse/tools/scenario.py add person --id p1 --x -6 --y 0 --rz 90` (also `move p1 --y -1`, `remove p1`, `list`, `save FILE`) |
| Nav goal from a shell | `docker exec jackal_greenhouse /ros_entrypoint.sh python3 /opt/greenhouse/tools/nav_command.py go --x 5 --y 2 --yaw 180` |
| Smoke test (pass/fail) | `bash run.sh --smoke-test` |
| Score any map vs truth | `docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/w -w /w jackal-greenhouse:jazzy python3 /opt/greenhouse/tools/score_map.py runtime/clearpath/greenhouse/nav-map/map.yaml demos/results/baseline-<time>/lidar_map.yaml --heatmap /w/heat.png` |
