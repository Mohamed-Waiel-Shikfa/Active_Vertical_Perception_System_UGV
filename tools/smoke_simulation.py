"""Observe and drive the full production launch, never a substitute robot/world."""
import argparse
import json
import math
import re
import statistics
import time
from pathlib import Path

import rclpy

from sim_checks import Observer, runtime_state, spin_until


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--launch-pid", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    state = runtime_state()
    manifest = json.loads((Path(state["assets_dir"]) / "manifest.json").read_text())
    deadline = time.monotonic() + args.timeout
    rclpy.init(args=[])
    node = Observer(state)
    result = {"status": "FAIL", "world": "greenhouse", "namespace": state["namespace"],
              "upstream_revision": manifest["upstream_revision"]}
    args.output.write_text(json.dumps({**result, "phase": "starting"}, indent=2) + "\n")
    try:
        spin_until(node, node.ready, deadline, args.launch_pid)
        settled_at = node.seconds + 2
        spin_until(node, lambda: node.seconds >= settled_at, deadline, args.launch_pid)
        start = tuple(node.pose)
        args.output.write_text(json.dumps({**result, "phase": "ready", "observed": node.diagnostics()}, indent=2) + "\n")
        if math.hypot(start[0] - manifest["spawn"]["x"], start[1] - manifest["spawn"]["y"]) > 0.3:
            raise RuntimeError(f"Jackal did not settle near the safe spawn: {start}")
        start_time, start_index = node.seconds, len(node.samples)
        while node.seconds - start_time < 8:
            if time.monotonic() >= deadline:
                raise TimeoutError("clock stalled during the simulated drive")
            if not node.ready():
                raise RuntimeError("a required live stream/controller disappeared during the drive")
            node.publish_speed(0.18)
            target = node.seconds + 0.025
            spin_until(node, lambda: node.seconds >= target, deadline, args.launch_pid)
        node.publish_speed()
        end = tuple(node.pose)
        drive_samples = node.samples[start_index:]
        if len(drive_samples) < 30:
            raise RuntimeError("insufficient physics pose samples during movement")
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        heights = [sample[3] for sample in drive_samples]
        rolls, pitches = [sample[4] for sample in drive_samples], [sample[5] for sample in drive_samples]
        z_range = max(heights) - min(heights)
        if not 0.5 < distance < 2.1:
            raise RuntimeError(f"implausible travel for 0.18 m/s over 8 s: {distance:.4f} m")
        if abs(end[1] - start[1]) > 0.45:
            raise RuntimeError("Jackal left the safe clear aisle")
        if not -0.01 <= min(heights) <= max(heights) <= 0.29:
            raise RuntimeError(f"Jackal is not supported on the terrain: z in {min(heights), max(heights)}")
        if not 0.0003 < z_range < 0.09:
            raise RuntimeError(f"terrain response is missing or excessive: z range {z_range:.6f} m")
        if max(abs(value) for value in rolls + pitches) > 0.2:
            raise RuntimeError("excessive roll/pitch for this gentle terrain")
        stop_at = node.seconds + 1.0
        while node.seconds < stop_at:
            node.publish_speed()
            spin_until(node, lambda: node.seconds >= stop_at or abs(node.odom.twist.twist.linear.x) < 0.02,
                       deadline, args.launch_pid)
            rclpy.spin_once(node, timeout_sec=0.05)
        if abs(node.odom.twist.twist.linear.x) >= 0.025:
            raise RuntimeError("robot did not stop after the zero teleop command")
        log = args.output.parent / "smoke-launch.log"
        errors = re.findall(
            r"(?im)^.*(?:Unable to find (?:uri|file)|Failed to load (?:system plugin|a plugin|mesh)|"
            r"Unable to load (?:mesh|texture)|Error Code [0-9]+|Downloading (?:model|resource)).*$",
            log.read_text(errors="replace"))
        if errors:
            raise RuntimeError("asset/plugin errors in production log: " + "\n".join(errors))
        result.update({
            "status": "PASS", "controllers": node.controllers, "start_pose": start, "end_pose": end,
            "distance_m": distance, "drive_sim_seconds": node.seconds - start_time - 1,
            "ground_truth_samples": len(drive_samples), "z_range_m": z_range,
            "roll_range_rad": max(rolls) - min(rolls), "pitch_range_rad": max(pitches) - min(pitches),
            "roll_std_rad": statistics.pstdev(rolls), "pitch_std_rad": statistics.pstdev(pitches),
            "imu_messages": node.imu_count, "stopped_odom_m_s": node.odom.twist.twist.linear.x,
        })
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        result["error"] = str(error)
        result["observed"] = node.diagnostics()
    finally:
        for _ in range(5):
            node.publish_speed()
            rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
