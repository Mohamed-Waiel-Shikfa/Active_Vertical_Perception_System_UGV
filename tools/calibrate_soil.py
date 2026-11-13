"""Six bounded comparisons using synchronized native wheel/body physics, not odometry."""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, "/opt/greenhouse")
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from greenhouse.transport import service, wheel_contact, world_status
from apply_terrain import pose_text, terrain


def profile_stats(surface):
    result = {}
    for length in (1, 5):
        rms = []
        for y in (-2, 0, 2):
            for start in np.arange(-5.5, 5.5-length+0.01, length):
                xs = np.arange(start, start+length+0.0001, 0.01)
                z = np.array([surface.sample(x, y) for x in xs])
                trend = np.polyval(np.polyfit(xs-xs[0], z, 1), xs-xs[0])
                rms.append(float(np.sqrt(np.mean((z-trend)**2)))*1000)
        result[f"{length}m_detrended_rms_mm"] = {"median": float(np.median(rms)), "range": [min(rms), max(rms)]}
    h, r = surface.heights, surface.resolution
    gradients = np.hypot(np.diff(h, axis=1)[:-1]/r, np.diff(h, axis=0)[:, :-1]/r)
    result["height_range_mm"] = float(np.ptp(h)*1000)
    result["slope_degrees_p95_max"] = np.degrees(np.arctan(np.quantile(gradients, [0.95,1]))).tolist()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--no-baseline-arcs", action="store_true",
                        help="Use straight baseline/geometry comparisons, then straight+arc for the contact candidate")
    parser.add_argument("--start-phase", choices=("baseline", "geometry_only", "contact_010"), default="baseline")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    directory = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
    runtime = json.loads((directory / "runtime.json").read_text())
    rclpy.init(args=[])
    node = Node("firm_soil_measurement", parameter_overrides=[Parameter("use_sim_time", value=True)])
    samples, latest = [], {}
    def receive(message):
        value = json.loads(message.data)
        latest.update(value)
        samples.append(value)
    node.create_subscription(String, "/greenhouse/physics", receive, qos_profile_sensor_data)
    command = node.create_publisher(TwistStamped, runtime["cmd_vel"], 10)
    def publish(v=0.0, w=0.0):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.angular.z = float(v), float(w)
        command.publish(message)
    def wait(predicate, timeout=45):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
            if predicate():
                return
        raise TimeoutError("native physics sample/settling timeout")
    def advance(seconds, v=0.0, w=0.0):
        start = latest["sim_time"]
        deadline = time.monotonic()+270
        while latest["sim_time"] < start+seconds:
            if time.monotonic()>deadline:
                raise TimeoutError("bounded trial clock stalled")
            publish(v,w)
            rclpy.spin_once(node, timeout_sec=0.035)
        publish()
    report = {"source": "native Gazebo base_link and wheel link velocities, 50 Hz simulation timestamps",
              "normal_force_measurement": False,
              "lift_payload": json.loads((directory / "actuator.json").read_text())["config"], "cases": []}
    if args.start_phase != "baseline":
        report = json.loads((args.output / "comparison.json").read_text())
    started = False
    try:
        wait(lambda: (directory / "controller-ready.json").is_file(), 120)
        wait(lambda: latest.get("wheels") and len(latest["wheels"])==4, 75)
        for label, updates in (
            ("baseline", {}),
            ("geometry_only", {"ground_height_range": 0.03, "ground_bump_rms": 0.003}),
            ("contact_010", {"slip_longitudinal": 0.1, "slip_lateral": 1.0}),
        ):
            if label == args.start_phase:
                started = True
            if not started:
                continue
            if updates:
                publish()
                active = json.loads((directory/"terrain-state.json").read_text())
                current_surface = terrain(Path(active["assets_dir"]))
                reset = {"x": -7.5, "y": 0, "z": current_surface.sample(-7.5,0)+.09, "yaw": 0}
                service("/world/greenhouse/set_pose/blocking", "gz.msgs.Pose",
                        request=pose_text(runtime["robot_name"], reset))
                advance(3)
                wait(lambda: math.sqrt(sum(latest.get(k, 1)**2 for k in ("vx","vy","vz"))) < .01
                     and math.sqrt(sum(latest.get(k, 1)**2 for k in ("wx","wy","wz"))) < .02, 60)
                result = subprocess.run(["python3","/opt/greenhouse/tools/apply_terrain.py","--values",json.dumps(updates)],
                                        capture_output=True, text=True, timeout=150)
                (args.output/f"{label}-apply.log").write_text(result.stdout+result.stderr)
                if result.returncode:
                    raise RuntimeError(result.stderr)
            active = json.loads((directory/"terrain-state.json").read_text())
            assets = Path(active["assets_dir"])
            surface = terrain(assets)
            config = active.get("config", json.loads((assets/"manifest.json").read_text())["config"])
            modes = ("straight",) if args.no_baseline_arcs and label != "contact_010" else ("straight", "arc")
            for mode in modes:
                publish()
                pose = {"x": -7.5 if mode=="straight" else -7.25, "y": 0 if mode=="straight" else -0.9,
                        "z": 0.2, "yaw": 0 if mode=="straight" else math.pi/2}
                pose["z"] = max(surface.sample(pose["x"]+x,pose["y"]+y)
                                for x in (-.3,0,.3) for y in (-.27,0,.27))+.08
                service("/world/greenhouse/set_pose/blocking","gz.msgs.Pose",
                        request=pose_text(runtime["robot_name"],pose))
                advance(2)
                offset=len(samples); start=latest["sim_time"]
                advance(10,.2,0 if mode=="straight" else .2)
                trial=[s for s in samples[offset:] if s["sim_time"]-start>=2]
                advance(1)
                residuals=[]; low=[]; sides=[]
                rows=[]
                for sample in trial:
                    for wheel in sample["wheels"]:
                        rolling=.098*wheel["omega"]
                        residual=rolling-wheel["vx"]
                        if abs(rolling)>=.05:
                            residuals.append(abs(residual)/abs(rolling))
                        else:
                            low.append(abs(residual))
                        sides.append(abs(wheel["vy"]))
                        rows.append([sample["sim_time"],sample["x"],sample["y"],sample["yaw"],sample["vx"],
                                     sample["vy"],sample["wz"],wheel["name"],rolling,wheel["vx"],wheel["vy"],residual])
                with (args.output/f"{label}-{mode}.csv").open("w",newline="") as stream:
                    writer=csv.writer(stream);writer.writerow(["sim_time","x","y","yaw","base_vx","base_vy","base_wz",
                                                               "wheel","rolling_speed","wheel_center_vx","wheel_center_vy","rolling_residual"])
                    writer.writerows(rows)
                case={"label":label,"mode":mode,"samples":len(trial),"config":config,
                      "straight_or_arc_rolling_abs_ratio_median_p95":np.quantile(residuals,[.5,.95]).tolist() if residuals else None,
                      "low_speed_absolute_residual_p95":float(np.quantile(low,.95)) if low else None,
                      "wheel_center_lateral_speed_median_p95":np.quantile(sides,[.5,.95]).tolist(),
                      "mean_body_vx":float(np.mean([s["vx"] for s in trial])),
                      "mean_body_wz":float(np.mean([s["wz"] for s in trial])),
                      "profiles":profile_stats(surface)}
                report["cases"].append(case)
                (args.output/"comparison.json").write_text(json.dumps(report,indent=2)+"\n")
                print(json.dumps({k:case[k] for k in ("label","mode","mean_body_vx","mean_body_wz","straight_or_arc_rolling_abs_ratio_median_p95")}),flush=True)
        report["status"]="PASS"
    finally:
        for _ in range(5): publish();rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_node();rclpy.shutdown()
        (args.output/"comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
