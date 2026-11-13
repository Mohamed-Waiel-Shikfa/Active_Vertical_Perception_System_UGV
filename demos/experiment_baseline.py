# SPDX-License-Identifier: GPL-3.0-only
"""Demo 3 / experiment: sensor audit, odometry drift on two terrains, lidar-only map scored against truth.

Run:  bash run.sh --headless --viz --script demos/experiment_baseline.py     (needs --viz: error topics)
About 4 min simulated (two serpentine laps of all five aisles). Writes results/baseline-<time>/:
  report.md          human summary of everything below
  summary.json       all numbers
  sensors.json       header-stamp rates (sim Hz), lidar static noise, TF chain check, RTF
  drift_<cond>.csv   per-sample drift of wheel odometry and EKF vs truth (re-aligned at lap start)
  drift.png          planar drift vs distance travelled; trajectories.png  truth vs estimates
  lidar_map.yaml/.pgm  occupancy map from lap A scans on the TRUE pose; lidar_map_heatmap.png, lidar_map_score.json
Conditions: A = the default terrain; B = "wet soil" (more longitudinal and lateral wheel slip).
"""
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

from demo_ros import EST, LIDAR, NS, STATE, TRUTH, Listener
from self_driving_api import apply_terrain, configure, get_state, pose

configure(verbose=True)
listener = Listener()
out = Path("results") / time.strftime("baseline-%Y%m%d-%H%M")
out.mkdir(parents=True, exist_ok=True)
LIDAR_X = 0.12  # m, scan origin ahead of base_link (config/actuator.json lidar mount)
# Serpentine through all five aisles (same waypoints as self_driving/sample_use.py poses2).
LAP = [(-6.5, 0), (-6.5, 4), (6.5, 4), (6.5, 2), (-6.5, 2), (-6.5, 0), (6.5, 0), (6.5, -2), (-6.5, -2),
       (-6.5, -4), (6.5, -4)]
CONDITIONS = {"A_default": None,
              "B_wet_soil": {"slip_long": 0.3, "slip_lat": 1.6, "ground_friction": 0.7}}


def say(text: str) -> None:
    print(f"\n[experiment] {text}", flush=True)


# ---- 1. sensor audit at rest --------------------------------------------------------------------
def sensor_audit() -> dict:
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix
    ns = f"/{NS}"
    topics = {LIDAR["topic"]: LaserScan, f"{ns}/sensors/imu_0/data_raw": Imu, f"{ns}/sensors/imu_0/data": Imu,
              f"{ns}/platform/odom": Odometry, f"{ns}/platform/odom/filtered": Odometry,
              f"{ns}/greenhouse/camera1/image": Image, f"{ns}/greenhouse/camera2/image": Image,
              f"{ns}/greenhouse/camera1/camera_info": CameraInfo, f"{ns}/sensors/gps_0/fix": NavSatFix}
    stated = {LIDAR["topic"]: LIDAR["update_rate_hz"], f"{ns}/sensors/imu_0/data_raw": 50, f"{ns}/sensors/imu_0/data": 50,
              f"{ns}/platform/odom": 50, f"{ns}/platform/odom/filtered": 50, f"{ns}/greenhouse/camera1/image": 10,
              f"{ns}/greenhouse/camera2/image": 10, f"{ns}/greenhouse/camera1/camera_info": 10,
              f"{ns}/sensors/gps_0/fix": 10}
    w0, s0 = time.monotonic(), listener.sim_now()
    rates = listener.measure_rates(topics, 20.0)
    rtf = (listener.sim_now() - s0) / (time.monotonic() - w0)
    for topic, row in rates.items():
        row["stated_hz"] = stated[topic]

    listener.scans = []
    while len(listener.scans) < 60:
        time.sleep(0.1)
    scans, listener.scans = listener.scans[:60], None
    ranges = np.array([s.ranges for s in scans], dtype=float)
    steady = np.isfinite(ranges).all(axis=0)
    noise = ranges[:, steady].std(axis=0)

    import tf2_ros
    from rclpy.time import Time
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, listener.node, spin_thread=False)
    chains = {}
    for parent, child in (("greenhouse", "map"), ("map", "odom"), ("odom", "base_link"),
                          ("base_link", LIDAR["frame"]), ("greenhouse", LIDAR["frame"])):
        ok = False
        for _ in range(50):
            if buffer.can_transform(parent, child, Time()):
                ok = True
                break
            time.sleep(0.1)
        chains[f"{parent} -> {child}"] = ok
    return {"rtf_during_rates": round(rtf, 3), "rates": rates,
            "lidar_static_noise": {"scans": 60, "steady_beams": int(steady.sum()),
                                   "median_beam_std_m": round(float(np.median(noise)), 4),
                                   "p95_beam_std_m": round(float(np.percentile(noise, 95)), 4)},
            "tf_lookups": chains}


# ---- 2. drift on two terrains ---------------------------------------------------------------------
def compose_inverse(a, b):
    """a^-1 * b for planar poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))


def drift(rows: list[list[float]]) -> dict:
    """Re-align estimate and truth at the first row; drift = estimate motion minus truth motion."""
    est0, tru0 = rows[0][EST], rows[0][TRUTH]
    series, distance, last = [], 0.0, tru0
    for row in rows:
        e, t = compose_inverse(est0, row[EST]), compose_inverse(tru0, row[TRUTH])
        distance += math.hypot(row[TRUTH][0] - last[0], row[TRUTH][1] - last[1])
        last = row[TRUTH]
        series.append((distance, math.hypot(e[0] - t[0], e[1] - t[1]),
                       math.atan2(math.sin(e[2] - t[2]), math.cos(e[2] - t[2])), *row[EST], *row[TRUTH]))
    return series


def stats(series) -> dict:
    planar = np.array([s[1] for s in series])
    yaw = np.degrees(np.abs([s[2] for s in series]))
    return {"samples": len(series), "distance_m": round(series[-1][0], 2),
            "final_planar_m": round(float(planar[-1]), 3), "rms_planar_m": round(float(np.sqrt((planar ** 2).mean())), 3),
            "p95_planar_m": round(float(np.percentile(planar, 95)), 3), "max_planar_m": round(float(planar.max()), 3),
            "final_yaw_deg": round(float(np.degrees(series[-1][2])), 1), "max_abs_yaw_deg": round(float(yaw.max()), 1),
            "final_planar_per_metre": round(float(planar[-1] / max(series[-1][0], 1e-6)), 3)}


def lap(waypoints, record_scans: bool) -> dict:
    for key in listener.errors:
        listener.errors[key] = []
    listener.scans = [] if record_scans else None
    listener.recording_errors = True
    s0, w0, failures = listener.sim_now(), time.monotonic(), []
    for x, y in waypoints:
        try:
            pose({"x": x, "y": y, "oriented": False, "drive_speed": 1.0, "turn_speed": 2.0})
        except Exception as error:  # keep the lap going; the failure is part of the result
            failures.append({"goal": [x, y], "error": str(error)[:300]})
            say(f"goal ({x}, {y}) failed: {error}")
    listener.recording_errors = False
    scans, listener.scans = listener.scans, None
    sim_s, wall_s = listener.sim_now() - s0, time.monotonic() - w0
    return {"errors": {k: list(v) for k, v in listener.errors.items()}, "scans": scans or [],
            "sim_s": round(sim_s, 1), "wall_s": round(wall_s, 1), "rtf": round(sim_s / wall_s, 3), "failures": failures}


# ---- 3. lidar map on the true pose ---------------------------------------------------------------
def lidar_map(scans) -> dict:
    from greenhouse.map_score import FREE, OCCUPIED, UNKNOWN, Grid, load_map, save_map, score, write_heatmap
    truth = load_map(STATE / "nav-map" / "map.yaml")
    rows, cols = truth.cells.shape
    res, (ox, oy) = truth.resolution, truth.origin
    hits, misses = np.zeros(rows * cols), np.zeros(rows * cols)
    used = 0
    for scan in scans:
        p = listener.truth_at(scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9)
        if p is None:
            continue
        used += 1
        r = np.array(scan.ranges, dtype=float)
        a = p[2] + scan.angle_min + np.arange(len(r)) * scan.angle_increment
        hit = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        reach = np.where(hit, r, scan.range_max)
        sx, sy = p[0] + LIDAR_X * math.cos(p[2]), p[1] + LIDAR_X * math.sin(p[2])
        steps = np.arange(0.0, scan.range_max, res / 2)
        along = steps[None, :] < (reach[:, None] - res)  # free samples stop one cell short of the hit
        fx = sx + np.cos(a)[:, None] * steps[None, :]
        fy = sy + np.sin(a)[:, None] * steps[None, :]

        def index(x, y):
            c, rr = np.floor((x - ox) / res).astype(int), rows - 1 - np.floor((y - oy) / res).astype(int)
            ok = (c >= 0) & (c < cols) & (rr >= 0) & (rr < rows)
            return rr[ok] * cols + c[ok]
        np.add.at(misses, np.unique(index(fx[along], fy[along])), 1)
        np.add.at(hits, index(sx + np.cos(a[hit]) * r[hit], sy + np.sin(a[hit]) * r[hit]), 1)
    # shortcut: count-based occupancy (hit ratio >= 0.3), no log-odds or motion de-skew; fine for a baseline.
    seen = hits + misses
    cells = np.full(rows * cols, UNKNOWN, dtype=np.int8)
    cells[seen > 0] = FREE
    cells[(hits > 0) & (hits / np.maximum(seen, 1) >= 0.3)] = OCCUPIED
    candidate = Grid(cells.reshape(rows, cols), res, (ox, oy))
    save_map(candidate, out / "lidar_map.yaml")
    write_heatmap(truth, candidate, out / "lidar_map_heatmap.png", tolerance_cells=1)
    result = {"scans_used": used, **{f"k{k}": score(truth, candidate, k) for k in (0, 1, 2)}}
    (out / "lidar_map_score.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


# ---- 4. plots and report --------------------------------------------------------------------------
def plots(series: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colours = {"wheel_odom": "#E69F00", "ekf": "#0072B2"}
    fig, axes = plt.subplots(1, len(series), figsize=(6 * len(series), 4), sharey=True)
    for ax, (cond, sources) in zip(np.atleast_1d(axes), series.items()):
        for source, s in sources.items():
            ax.plot([r[0] for r in s], [r[1] for r in s], label=source, color=colours[source])
        ax.set_title(f"Planar drift vs truth: {cond}")
        ax.set_xlabel("distance travelled (truth, m)")
        ax.grid(alpha=0.3)
    np.atleast_1d(axes)[0].set_ylabel("planar error (m)")
    np.atleast_1d(axes)[0].legend()
    fig.tight_layout()
    fig.savefig(out / "drift.png", dpi=130)
    fig, axes = plt.subplots(1, len(series), figsize=(6 * len(series), 5))
    for ax, (cond, sources) in zip(np.atleast_1d(axes), series.items()):
        first = next(iter(sources.values()))
        ax.plot([r[6] for r in first], [r[7] for r in first], color="black", lw=2, label="truth")
        for source, s in sources.items():  # estimate re-aligned onto the truth start pose
            t0, e0 = s[0][6:9], s[0][3:6]
            pts = []
            for r in s:
                dx, dy, _ = compose_inverse(e0, r[3:6])
                c, sn = math.cos(t0[2]), math.sin(t0[2])
                pts.append((t0[0] + c * dx - sn * dy, t0[1] + sn * dx + c * dy))
            ax.plot(*zip(*pts), color=colours[source], label=source)
        ax.set_title(f"Trajectories (greenhouse frame): {cond}")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "trajectories.png", dpi=130)


def fmt(value) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def report(summary: dict) -> str:
    s = summary["sensors"]
    lines = [f"# Baseline experiment {summary['started']}", "",
             "Headless with `--viz`; record the host power profile (`/sys/firmware/acpi/platform_profile`) with these numbers.", "",
             "## Sensor rates at rest (header stamps, sim Hz)", "", "| Topic | Stated | Measured | Messages |", "|---|---|---|---|"]
    lines += [f"| `{t}` | {r['stated_hz']} | {r['hz']} | {r['messages']} |" for t, r in s["rates"].items()]
    n = s["lidar_static_noise"]
    lines += ["", f"RTF while measuring: {s['rtf_during_rates']}. Lidar static noise over {n['scans']} scans, "
              f"{n['steady_beams']} steady beams: median per-beam std {n['median_beam_std_m']} m, p95 {n['p95_beam_std_m']} m "
              "(model sigma 0.02 m).", "", "TF lookups: " + ", ".join(f"`{k}` {'ok' if v else 'MISSING'}" for k, v in s["tf_lookups"].items()),
              "", "## Odometry drift vs truth (re-aligned at each lap start)", "",
              "| Condition | Source | Distance m | Final m | RMS m | p95 m | Max m | Final yaw deg | Final m per m |", "|---|---|---|---|---|---|---|---|---|"]
    for cond, c in summary["drift"].items():
        for source, st in c["stats"].items():
            lines.append(f"| {cond} | {source} | {st['distance_m']} | {st['final_planar_m']} | {st['rms_planar_m']} | "
                         f"{st['p95_planar_m']} | {st['max_planar_m']} | {st['final_yaw_deg']} | {st['final_planar_per_metre']} |")
    lines += [""] + [f"- {cond}: terrain {c['terrain']}, {c['sim_s']} s sim, {c['wall_s']} s wall, RTF {c['rtf']}, "
                     f"failed goals: {len(c['failures'])}" for cond, c in summary["drift"].items()]
    m = summary["lidar_map"]
    lines += ["", "## Lidar-only occupancy map on the true pose (lap A) vs truth map", "",
              f"{m['scans_used']} scans. Coverage of truth free space: {fmt(m['k0']['coverage'])}; "
              f"symmetric boundary distance {fmt(m['k0']['boundary_distance_m']['symmetric_mean'])} m.", "",
              "| Tolerance (cells) | Precision | Recall | F1 | IoU |", "|---|---|---|---|---|"]
    for k in (0, 1, 2):
        r = m[f"k{k}"]
        lines.append(f"| {k} | {fmt(r['precision'])} | {fmt(r['recall'])} | {fmt(r['f1'])} | {fmt(r['iou'])} |")
    lines += ["", "Figures: `drift.png`, `trajectories.png`, `lidar_map_heatmap.png` (blue TP, orange FP, purple FN)."]
    return "\n".join(lines) + "\n"


say(f"Results go to demos/{out}")
summary = {"started": time.strftime("%Y-%m-%d %H:%M UTC")}
say("1/3 Sensor audit at rest (20 s wall of rates, 60 lidar scans, TF lookups).")
summary["sensors"] = sensor_audit()
(out / "sensors.json").write_text(json.dumps(summary["sensors"], indent=2) + "\n")

summary["drift"], series, scans_a = {}, {}, []
for index, (cond, terrain) in enumerate(CONDITIONS.items()):
    if terrain:
        say(f"Applying terrain {terrain}")
        apply_terrain(terrain)
    say(f"Back to the lap start {LAP[0]} (not recorded), so every condition drives the same lap.")
    pose({"x": LAP[0][0], "y": LAP[0][1], "rz": 90.0, "drive_speed": 1.0, "turn_speed": 2.0})
    say(f"2/3 Lap {cond}: {len(LAP) - 1} legs at 1 m/s.")
    result = lap(LAP[1:], record_scans=index == 0)
    if index == 0:
        scans_a = result["scans"]
    series[cond] = {k: drift(v) for k, v in result["errors"].items() if v}
    summary["drift"][cond] = {"terrain": get_state()["terrain"], "sim_s": result["sim_s"], "wall_s": result["wall_s"],
                              "rtf": result["rtf"], "failures": result["failures"],
                              "stats": {k: stats(v) for k, v in series[cond].items()}}
    with (out / f"drift_{cond}.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")  # repo enforces LF (.gitattributes)
        writer.writerow(["source", "distance_m", "planar_err_m", "yaw_err_rad", "est_x", "est_y", "est_yaw",
                         "truth_x", "truth_y", "truth_yaw"])
        for source, s in series[cond].items():
            writer.writerows([source, *(round(v, 5) for v in row)] for row in s)

say(f"3/3 Lidar map from {len(scans_a)} lap-A scans, scored against the truth nav map.")
summary["lidar_map"] = lidar_map(scans_a)
plots(series)
(out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
(out / "report.md").write_text(report(summary))
print((out / "report.md").read_text(), flush=True)
say(f"Done. Results in demos/{out}")
