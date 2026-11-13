"""--viz evaluation node: wheel-odometry and EKF error against ground truth, greyscale camera views.

Errors are estimate minus truth, one std_msgs/Float64MultiArray per odometry message on
ERROR_TOPICS, `data` in FIELDS order (also written into `layout.dim[0].label`). Poses are planar
in the greenhouse frame (m, rad), velocities in the body frame (vx m/s, wz rad/s).
Odometry frames start at the spawn pose: each source is aligned to the greenhouse frame once, by
the truth pose at its first sample (initial-pose offset), so its error is the drift since then.
Truth is /greenhouse/ground_truth (gz Pose_V, poses only), interpolated to each odometry stamp;
truth velocity is the central difference of that interpolated pose over +-HALF_WINDOW_S.
Each gimbal camera is also republished as mono8 on `<image>_mono`, subscribed to the colour
stream only while the mono topic has a subscriber.
"""
import array
import bisect
import json
import math
import os
from collections import deque
from pathlib import Path

import numpy as np

TRUTH_TOPIC = "/greenhouse/ground_truth"
ERROR_TOPICS = {"ekf": "/greenhouse/eval/ekf_error", "wheel_odom": "/greenhouse/eval/wheel_odom_error"}
MONO_SUFFIX = "_mono"
FIELDS = ("error_x_m", "error_y_m", "error_planar_m", "error_yaw_rad", "error_vx_m_s", "error_wz_rad_s",
          "estimate_x_m", "estimate_y_m", "estimate_yaw_rad", "estimate_vx_m_s", "estimate_wz_rad_s",
          "truth_x_m", "truth_y_m", "truth_yaw_rad", "truth_vx_m_s", "truth_wz_rad_s")
HALF_WINDOW_S = 0.05  # truth differentiation half-window; also the publish delay behind truth
BUFFER_S = 2.0        # truth history kept for late odometry
# ITU-R BT.601 luma, x256 (same ratios as cv2 RGB2GRAY); sums to 256 so 255 stays 255.
LUMA = (77, 150, 29)
CHANNELS = {"rgb8": (0, 1, 2, 3), "bgr8": (2, 1, 0, 3), "rgba8": (0, 1, 2, 4), "bgra8": (2, 1, 0, 4)}

Pose2 = tuple[float, float, float]  # x m, y m, yaw rad
Sample = tuple[float, float, float, float]  # sim time s, x, y, yaw


def topics(namespace: str) -> dict[str, str]:
    """Input topics for a Clearpath namespace ('' = root), as launch/greenhouse.launch.py names them."""
    ns = f"/{namespace}" if namespace else ""
    return {"ekf": ns + "/platform/odom/filtered", "wheel_odom": ns + "/platform/odom",
            "camera1": ns + "/greenhouse/camera1/image", "camera2": ns + "/greenhouse/camera2/image"}


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def compose(a: Pose2, b: Pose2) -> Pose2:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], wrap(a[2] + b[2]))


def inverse(a: Pose2) -> Pose2:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0] - s*a[1], s*a[0] - c*a[1], wrap(-a[2]))


def interpolate(samples: list[Sample], t: float) -> Pose2 | None:
    """Pose at t, linear in x/y and along the shorter arc in yaw; None outside the samples."""
    if not samples or not samples[0][0] <= t <= samples[-1][0]:
        return None
    i = bisect.bisect_left(samples, (t,))
    if samples[i][0] == t:
        return samples[i][1:]
    (t0, x0, y0, a0), (t1, x1, y1, a1) = samples[i - 1], samples[i]
    f = (t - t0) / (t1 - t0)
    return (x0 + f*(x1 - x0), y0 + f*(y1 - y0), wrap(a0 + f*wrap(a1 - a0)))


def truth_state(samples: list[Sample], t: float, half_window: float = HALF_WINDOW_S) -> tuple[float, ...] | None:
    """(x, y, yaw, vx, wz) at t; None until truth reaches t + half_window. vx is along the heading."""
    pose = interpolate(samples, t)
    if pose is None or samples[-1][0] < t + half_window:
        return None
    t0, t1 = max(samples[0][0], t - half_window), t + half_window
    a, b = interpolate(samples, t0), interpolate(samples, t1)
    vx = ((b[0] - a[0])*math.cos(pose[2]) + (b[1] - a[1])*math.sin(pose[2])) / (t1 - t0)
    return (*pose, vx, wrap(b[2] - a[2]) / (t1 - t0))


def errors(estimate: Pose2, vx: float, wz: float, truth: tuple[float, ...]) -> list[float]:
    """One FIELDS row: estimate minus truth, then the estimate, then the truth."""
    ex, ey = estimate[0] - truth[0], estimate[1] - truth[1]
    return [ex, ey, math.hypot(ex, ey), wrap(estimate[2] - truth[2]), vx - truth[3], wz - truth[4],
            *estimate, vx, wz, *truth]


class Evaluator:
    """Queues one odometry source until truth covers its stamps, aligns it once, yields FIELDS rows."""

    def __init__(self):
        self.offset: Pose2 | None = None
        self.pending: deque = deque(maxlen=500)

    def push(self, t: float, pose: Pose2, vx: float, wz: float):
        self.pending.append((t, pose, vx, wz))

    def drain(self, samples: list[Sample]) -> list[list[float]]:
        rows = []
        while self.pending:
            t, pose, vx, wz = self.pending[0]
            if samples and t < samples[0][0]:  # older than the truth history: unscorable
                self.pending.popleft()
                continue
            truth = truth_state(samples, t)
            if truth is None:
                break
            self.pending.popleft()
            if self.offset is None:  # world <- odom, so the first sample has zero pose error
                self.offset = compose(truth[:3], inverse(pose))
            rows.append(errors(compose(self.offset, pose), vx, wz, truth))
        return rows

    def reset(self):
        self.offset = None
        self.pending.clear()


def mono8(data, width: int, height: int, step: int, encoding: str) -> bytes:
    """Greyscale bytes (step = width) of a packed 8-bit colour or mono image; ValueError otherwise."""
    rows = np.frombuffer(data, np.uint8)[:height*step].reshape(height, step)
    if encoding == "mono8":
        return rows[:, :width].tobytes()
    if encoding not in CHANNELS:
        raise ValueError(f"unsupported image encoding: {encoding}")
    r, g, b, size = CHANNELS[encoding]
    pixels = rows[:, :width*size].reshape(height, width, size).astype(np.uint16)
    luma = LUMA[0]*pixels[..., r] + LUMA[1]*pixels[..., g] + LUMA[2]*pixels[..., b] + 128
    return (luma >> 8).astype(np.uint8).tobytes()


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray, MultiArrayDimension
    from tf2_msgs.msg import TFMessage

    state = json.loads((Path(os.environ["GREENHOUSE_STATE"]) / "runtime.json").read_text())
    names = topics(state["namespace"])
    rclpy.init(args=[])
    # Wall clock on purpose: every stamp comes from the messages (sim time), and a sim-time node
    # would process /clock at the physics step rate (measured: half a core).
    node = Node("greenhouse_eval")
    stamp = lambda s: s.sec + s.nanosec*1e-9
    samples: list[Sample] = []
    evaluators = {key: Evaluator() for key in ERROR_TOPICS}
    publishers = {key: node.create_publisher(Float64MultiArray, topic, 10) for key, topic in ERROR_TOPICS.items()}
    layout = MultiArrayDimension(label=",".join(FIELDS), size=len(FIELDS), stride=len(FIELDS))

    def truth(message):
        for transform in message.transforms:
            if transform.child_frame_id != state["robot_name"] or transform.header.frame_id != "greenhouse":
                continue
            t, p, r = stamp(transform.header.stamp), transform.transform.translation, transform.transform.rotation
            if samples and t <= samples[-1][0]:
                if t < samples[-1][0]:  # sim time went backwards (reset): start over
                    samples.clear()
                    for evaluator in evaluators.values():
                        evaluator.reset()
                continue
            samples.append((t, p.x, p.y, yaw_of(r.x, r.y, r.z, r.w)))
            del samples[:bisect.bisect_left(samples, (t - BUFFER_S,))]
        for key, evaluator in evaluators.items():
            for row in evaluator.drain(samples):
                out = Float64MultiArray(data=row)
                out.layout.dim = [layout]
                publishers[key].publish(out)

    def odometry(key):
        def update(message):
            p, r, twist = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
            evaluators[key].push(stamp(message.header.stamp), (p.x, p.y, yaw_of(r.x, r.y, r.z, r.w)),
                                 twist.linear.x, twist.angular.z)
        return update

    node.create_subscription(TFMessage, TRUTH_TOPIC, truth, qos_profile_sensor_data)
    for key in ERROR_TOPICS:
        node.create_subscription(Odometry, names[key], odometry(key), qos_profile_sensor_data)

    cameras = {camera: [names[camera], node.create_publisher(Image, names[camera] + MONO_SUFFIX, 10), None]
               for camera in ("camera1", "camera2")}
    warned = set()

    def convert(publisher):
        def update(message):
            if publisher.get_subscription_count() == 0:
                return
            try:
                data = mono8(message.data, message.width, message.height, message.step, message.encoding)
            except ValueError as error:
                if message.encoding not in warned:
                    warned.add(message.encoding)
                    node.get_logger().warning(str(error))
                return
            publisher.publish(Image(header=message.header, height=message.height, width=message.width,
                                    encoding="mono8", is_bigendian=0, step=message.width,
                                    data=array.array("B", data)))
        return update

    def watch():
        # Subscribing to the colour stream only while someone views the mono one keeps --viz free
        # of image transport and conversion when no viewer is open.
        for entry in cameras.values():
            source, publisher, subscription = entry
            wanted = publisher.get_subscription_count() > 0
            if wanted and subscription is None:
                # Reliable like the gz bridge: best effort lost most fragmented 0.9 MB frames under load.
                entry[2] = node.create_subscription(Image, source, convert(publisher), 2)
            elif not wanted and subscription is not None:
                node.destroy_subscription(subscription)
                entry[2] = None

    node.create_timer(0.5, watch, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
