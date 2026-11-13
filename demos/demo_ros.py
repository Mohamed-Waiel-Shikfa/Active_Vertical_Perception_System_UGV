# SPDX-License-Identifier: GPL-3.0-only
"""ROS listeners shared by the demo scripts (run inside the sim container via run.sh --script).

One background node, wall clock (stamps come from the messages, in sim time), spun in its own thread
so self_driving_api calls keep blocking the main thread as usual.
"""
import bisect
import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

STATE = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
RUNTIME = json.loads((STATE / "runtime.json").read_text())
NS = RUNTIME["namespace"]
ROBOT = RUNTIME["robot_name"]  # child frame of the robot in /greenhouse/ground_truth
LIDAR = json.loads((STATE / "actuator.json").read_text())["lidar"]
ERROR_TOPICS = {"ekf": "/greenhouse/eval/ekf_error", "wheel_odom": "/greenhouse/eval/wheel_odom_error"}
# Fields of the error rows (tools/pose_error.py FIELDS): 6-8 aligned estimate x, y, yaw; 11-13 truth.
EST, TRUTH = slice(6, 9), slice(11, 14)
TRUTH_WINDOW_S = 900.0  # sim s of truth kept, enough to map a whole recorded lap afterwards


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Listener:
    """Latest scan, recorded error rows, truth pose history and per-topic header-stamp rates."""

    def __init__(self):
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Float64MultiArray
        from tf2_msgs.msg import TFMessage
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = Node(f"demo_listener_{os.getpid()}")
        self.lock = threading.Lock()
        self.scan = None
        self.scans = None          # list while recording scans
        self.errors = {key: [] for key in ERROR_TOPICS}
        self.recording_errors = False
        self.truth = []            # (t, x, y, yaw), greenhouse frame, last TRUTH_WINDOW_S
        self.rates = {}            # topic -> [count, first stamp, last stamp]
        self.node.create_subscription(LaserScan, LIDAR["topic"], self._on_scan, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/greenhouse/ground_truth", self._on_truth,
                                      qos_profile_sensor_data)
        for key, topic in ERROR_TOPICS.items():
            self.node.create_subscription(Float64MultiArray, topic, self._on_error(key), 50)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    # ---- callbacks ----------------------------------------------------------------------------
    def _on_scan(self, message):
        with self.lock:
            self.scan = message
            if self.scans is not None:
                self.scans.append(message)

    def _on_truth(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == ROBOT and transform.header.frame_id == "greenhouse":
                t, p = stamp(transform.header), transform.transform.translation
                with self.lock:
                    if not self.truth or t > self.truth[-1][0]:
                        self.truth.append((t, p.x, p.y, yaw_of(transform.transform.rotation)))
                        del self.truth[:bisect.bisect_left(self.truth, (t - TRUTH_WINDOW_S,))]

    def _on_error(self, key):
        def update(message):
            if self.recording_errors:
                with self.lock:
                    self.errors[key].append(list(message.data))
        return update

    # ---- queries ------------------------------------------------------------------------------
    def wait_scan(self, timeout: float = 10.0):
        """Next scan received after this call (wall-clock timeout, s)."""
        old, end = self.scan, time.monotonic() + timeout
        while time.monotonic() < end:
            if self.scan is not None and self.scan is not old:
                return self.scan
            time.sleep(0.02)
        raise TimeoutError(f"no scan on {LIDAR['topic']}")

    def sector_min(self, half_width_deg: float = 15.0, centre_deg: float = 0.0) -> float:
        """Closest finite lidar return (m) within +-half_width_deg of centre_deg (0 = straight ahead)."""
        scan = self.wait_scan()
        best = math.inf
        for i, r in enumerate(scan.ranges):
            angle = math.degrees(scan.angle_min + i * scan.angle_increment)
            if abs(angle - centre_deg) <= half_width_deg and math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                best = min(best, r)
        return best

    def sim_now(self) -> float:
        """Latest ground-truth stamp: sim time (s), advancing even while the API node is not spun."""
        with self.lock:
            return self.truth[-1][0] if self.truth else 0.0

    def truth_at(self, t: float):
        """Truth (x, y, yaw) interpolated at sim time t; None outside the buffer."""
        with self.lock:
            i = bisect.bisect_left(self.truth, (t,))
            if i == 0 or i >= len(self.truth):
                return None
            (t0, *a), (t1, *b) = self.truth[i - 1], self.truth[i]
        f = (t - t0) / (t1 - t0)
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * dyaw

    def measure_rates(self, topics: dict[str, type], seconds: float) -> dict[str, dict]:
        """Subscribe to {topic: msg type} for `seconds` of wall time; return header-stamp rates (sim Hz)."""
        def counter(topic):
            def update(message):
                t = stamp(message.header)
                entry = self.rates.setdefault(topic, [0, t, t])
                entry[0] += 1
                entry[2] = t
            return update
        self.rates.clear()
        subs = [self.node.create_subscription(kind, topic, counter(topic), qos_profile_sensor_data)
                for topic, kind in topics.items()]
        time.sleep(seconds)
        for sub in subs:
            self.node.destroy_subscription(sub)
        out = {}
        for topic in topics:
            count, first, last = self.rates.get(topic, (0, 0.0, 0.0))
            out[topic] = {"messages": count, "sim_s": round(last - first, 3),
                          "hz": round((count - 1) / (last - first), 2) if count > 1 and last > first else None}
        return out
