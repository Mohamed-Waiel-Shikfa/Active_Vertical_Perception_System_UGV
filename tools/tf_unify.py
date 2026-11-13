"""Join Clearpath's namespaced TF tree and the navigation tree into one tree on /tf.

greenhouse -> map (static identity) -> odom -> base_link -> sensor frames, next to the
unchanged navigation branch greenhouse -> <ns>/nav_odom -> <ns>/nav_base.
map -> odom here is a truth localiser: T(map<-base_link, simulator truth) * T(odom<-base_link, EKF)^-1,
stamped with the EKF stamp, so the chain reproduces the true base_link pose. It is evaluation-only
scaffolding for the correction an estimator (M4) or SLAM (M6) will publish instead; nothing in
navigation reads it.
"""
import json
import math
import os
from collections import deque
from pathlib import Path

Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w
Pose = tuple[Vector, Quaternion]

PHYSICS_SOURCE = "Gazebo PostUpdate physical base_link"
TRUTH_SPAN_NS = 2_000_000_000
PENDING_MAX = 50
IDENTITY: Pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


def normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value*value for value in q))
    return tuple(value/norm for value in q)


def rotate(q: Quaternion, v: Vector) -> Vector:
    x, y, z, _ = quaternion_multiply(quaternion_multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def compose(a: Pose, b: Pose) -> Pose:
    """T(a) * T(b): b expressed in a's child frame, returned in a's parent frame."""
    offset = rotate(a[1], b[0])
    return (tuple(p + o for p, o in zip(a[0], offset)), normalize(quaternion_multiply(a[1], b[1])))


def inverse(a: Pose) -> Pose:
    conjugate = (-a[1][0], -a[1][1], -a[1][2], a[1][3])
    return (tuple(-value for value in rotate(conjugate, a[0])), conjugate)


def from_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Pose:
    """Fixed-axis roll, pitch, yaw (R = Rz Ry Rx), the convention of gz::math and tf2 setRPY."""
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return ((x, y, z), (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy))


def interpolate(a: Pose, b: Pose, fraction: float) -> Pose:
    """Linear in position, shortest-arc slerp in rotation (yaw across +-pi stays continuous)."""
    position = tuple(p + fraction*(q - p) for p, q in zip(a[0], b[0]))
    qa, qb = a[1], b[1]
    dot = sum(p*q for p, q in zip(qa, qb))
    if dot < 0:
        qb, dot = tuple(-value for value in qb), -dot
    if dot > 0.9995:
        return (position, normalize(tuple(p + fraction*(q - p) for p, q in zip(qa, qb))))
    angle = math.acos(dot)
    wa, wb = math.sin((1 - fraction)*angle), math.sin(fraction*angle)
    return (position, normalize(tuple(wa*p + wb*q for p, q in zip(qa, qb))))


def truth_at(samples, stamp: int) -> Pose | None:
    """Truth interpolated at `stamp` (ns) from (ns, Pose) samples in increasing order; None outside them."""
    if not samples or not samples[0][0] <= stamp <= samples[-1][0]:
        return None
    for index in range(len(samples) - 1, 0, -1):
        start, before = samples[index - 1]
        if start <= stamp:
            end, after = samples[index]
            return interpolate(before, after, (stamp - start)/(end - start))
    return samples[0][1]


def physics_truth(value: dict) -> tuple[int, Pose] | None:
    """(sim ns, base_link pose in greenhouse) from a /greenhouse/physics state, None if unusable."""
    keys = ("sim_time", "x", "y", "z", "roll", "pitch", "yaw")
    if value.get("source") != PHYSICS_SOURCE:
        return None
    if not all(isinstance(value.get(key), (int, float)) and math.isfinite(value[key]) for key in keys):
        return None
    return (round(value["sim_time"]*1e9), from_rpy(*(float(value[key]) for key in keys[1:])))


class Corrector:
    """Pairs each new EKF odom->base_link with truth at the same sim time; yields map->odom.

    An EKF sample newer than the newest truth waits (queued, so a steady truth lag never starves
    it); one older than the truth buffer is dropped; repeated EKF stamps are skipped because
    tf2 listeners reject redundant timestamps.
    """

    def __init__(self):
        self.truth = deque()
        self.pending = deque(maxlen=PENDING_MAX)
        self.last = -1

    def add_truth(self, stamp: int, pose: Pose) -> list:
        if self.truth and stamp <= self.truth[-1][0]:
            if stamp == self.truth[-1][0]:
                return []
            self.truth.clear(); self.pending.clear(); self.last = -1  # simulation clock restarted
        self.truth.append((stamp, pose))
        while self.truth[0][0] < stamp - TRUTH_SPAN_NS:
            self.truth.popleft()
        return self.resolve()

    def add_estimate(self, stamp: int, pose: Pose, tag=None) -> list:
        if stamp <= max(self.last, self.pending[-1][0] if self.pending else -1):
            return []
        self.pending.append((stamp, pose, tag))
        return self.resolve()

    def resolve(self) -> list:
        ready = []
        while self.pending and self.truth and self.pending[0][0] <= self.truth[-1][0]:
            stamp, estimate, tag = self.pending.popleft()
            truth = truth_at(self.truth, stamp)
            if truth is not None:
                self.last = stamp
                ready.append((stamp, tag, compose(truth, inverse(estimate))))
        return ready


def transform(stamp: int, parent: str, child: str, pose: Pose):
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    message = TransformStamped()
    message.header.stamp = Time(sec=stamp//1_000_000_000, nanosec=stamp % 1_000_000_000)
    message.header.frame_id, message.child_frame_id = parent, child
    t, q = message.transform.translation, message.transform.rotation
    t.x, t.y, t.z = (float(value) for value in pose[0])
    q.x, q.y, q.z, q.w = (float(value) for value in pose[1])
    return message


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    class TfUnify(Node):
        def __init__(self, namespace: str):
            # Wall clock on purpose: every stamp comes from the messages (sim time), and a sim-time
            # node would process /clock at the physics step rate (measured: half a core).
            super().__init__("greenhouse_tf_unify")
            self.corrector = Corrector()
            # tf2_ros broadcaster QoS: /tf reliable depth 100; /tf_static latched, one merged message.
            self.tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
            self.tf_static = self.create_publisher(
                TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.static = {"map": transform(0, "greenhouse", "map", IDENTITY)}
            prefix = "/" + namespace if namespace else ""
            if namespace:  # root namespace: Clearpath already publishes on /tf, a relay would loop
                # Raw bytes: no deserialisation for the high-rate joint transforms.
                self.create_subscription(TFMessage, prefix + "/tf", self.tf.publish, QoSProfile(depth=100), raw=True)
                self.create_subscription(TFMessage, prefix + "/tf_static", self.static_update, QoSProfile(
                    depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.publish_static()
            self.create_subscription(String, "/greenhouse/physics", self.physics, qos_profile_sensor_data)
            # The EKF's TF is built from this message (same stamp and pose), so the chain stays exact.
            self.create_subscription(Odometry, prefix + "/platform/odom/filtered", self.estimate, 10)

        def static_update(self, message):
            self.static.update((item.child_frame_id, item) for item in message.transforms)
            self.publish_static()

        def publish_static(self):
            self.tf_static.publish(TFMessage(transforms=list(self.static.values())))

        def physics(self, message):
            try:
                sample = physics_truth(json.loads(message.data))
            except ValueError:
                return
            if sample:
                self.send(self.corrector.add_truth(*sample))

        def estimate(self, message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            self.send(self.corrector.add_estimate(
                stamp, ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), message.header.frame_id))

        def send(self, corrections):
            if corrections:
                self.tf.publish(TFMessage(transforms=[
                    transform(stamp, "map", odom, pose) for stamp, odom, pose in corrections]))

    state = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
    namespace = json.loads((state/"runtime.json").read_text())["namespace"].strip("/")
    rclpy.init(args=[])
    node = TfUnify(namespace)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
