"""Live ROS observations shared by the health check and production smoke."""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def runtime_state():
    state = Path(os.environ.get("GREENHOUSE_STATE", os.path.join(os.environ.get("SETUP_PATH", "/etc/clearpath"), "greenhouse")))
    return json.loads((state / "runtime.json").read_text())


def euler(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def stamped_command(stamp, speed=0.0, yaw=0.0):
    command = TwistStamped()
    command.header.stamp = stamp
    command.twist.linear.x = float(speed)
    command.twist.angular.z = float(yaw)
    return command


class Observer(Node):
    def __init__(self, state, node_name="greenhouse_live_check"):
        super().__init__(node_name,
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.state = state
        self.seconds = 0
        self.first_clock = None
        self.last_clock_wall = 0
        self.last_pose_wall = 0
        self.last_odom_wall = 0
        self.last_imu_wall = 0
        self.last_controllers_wall = 0
        self.pose = None
        self.pose_names = set()
        self.pose_frames = set()
        self.odom = None
        self.imu_count = 0
        self.samples = []
        self.controllers = {}
        self.controller_future = None
        self.controller_sent_at = 0
        self.next_controller_poll = 0
        self.last_poll_wall = 0
        prefix = f"/{state['namespace']}" if state["namespace"] else ""
        self.create_subscription(Clock, "/clock", self.clock_update, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/greenhouse/ground_truth", self.pose_update, qos_profile_sensor_data)
        self.create_subscription(Odometry, prefix + "/platform/odom/filtered", self.odom_update, qos_profile_sensor_data)
        self.create_subscription(Imu, prefix + "/sensors/imu_0/data_raw", self.imu_update, qos_profile_sensor_data)
        self.client = self.create_client(ListControllers, state["controller_manager"] + "/list_controllers")
        self.command = self.create_publisher(TwistStamped, state["cmd_vel"], 10)

    def clock_update(self, message):
        self.seconds = message.clock.sec + message.clock.nanosec * 1e-9
        if self.first_clock is None:
            self.first_clock = self.seconds
        self.last_clock_wall = time.monotonic()

    def pose_update(self, message):
        for transform in message.transforms:
            self.pose_names.add(transform.child_frame_id)
            self.pose_frames.add(transform.header.frame_id)
            if transform.child_frame_id != self.state["robot_name"]:
                continue
            if transform.header.frame_id != "greenhouse":
                continue
            p = transform.transform.translation
            roll, pitch, yaw = euler(transform.transform.rotation)
            self.pose = (p.x, p.y, p.z, roll, pitch, yaw)
            self.last_pose_wall = time.monotonic()
            self.samples.append((self.seconds, *self.pose))
            if len(self.samples) > 20000:
                del self.samples[:10000]

    def odom_update(self, message):
        self.odom = message
        self.last_odom_wall = time.monotonic()

    def imu_update(self, message):
        self.imu_count += 1
        self.last_imu_wall = time.monotonic()

    def poll_controllers(self):
        self.last_poll_wall = time.monotonic()
        if self.controller_future is not None:
            if self.controller_future.done():
                response = self.controller_future.result()
                if response is not None:
                    self.controllers = {item.name: item.state for item in response.controller}
                    self.last_controllers_wall = time.monotonic()
                self.controller_future = None
                self.next_controller_poll = time.monotonic() + 1
            elif time.monotonic() - self.controller_sent_at > 5:
                self.client.remove_pending_request(self.controller_future)
                self.controller_future = None
                self.next_controller_poll = time.monotonic() + 1
        if (self.controller_future is None and self.client.service_is_ready()
                and time.monotonic() >= self.next_controller_poll):
            self.controller_future = self.client.call_async(ListControllers.Request())
            self.controller_sent_at = time.monotonic()

    def ready(self):
        return (
            self.first_clock is not None and self.seconds - self.first_clock > 0.1
            and time.monotonic() - self.last_clock_wall < 3
            and self.pose is not None and self.odom is not None and self.imu_count > 0
            and all(time.monotonic() - stamp < 3 for stamp in
                    (self.last_pose_wall, self.last_odom_wall, self.last_imu_wall))
            and time.monotonic() - self.last_controllers_wall < 5
            and all(self.controllers.get(name) == "active"
                    for name in ("joint_state_broadcaster", "platform_velocity_controller"))
        )

    def diagnostics(self):
        return {"clock_seconds": self.seconds, "ground_truth": self.pose,
                "odom_received": self.odom is not None, "imu_messages": self.imu_count,
                "controllers": self.controllers, "pose_names": sorted(self.pose_names),
                "pose_parent_frames": sorted(self.pose_frames)}

    def publish_speed(self, speed=0.0, yaw=0.0):
        self.command.publish(stamped_command(self.get_clock().now().to_msg(), speed, yaw))


def spin_until(node, predicate, deadline, launch_pid=None):
    while time.monotonic() < deadline:
        if launch_pid is not None:
            try:
                os.kill(launch_pid, 0)
            except ProcessLookupError as error:
                raise RuntimeError("production ROS launch exited before the check completed") from error
        rclpy.spin_once(node, timeout_sec=0.05)
        node.poll_controllers()
        if predicate():
            return
    raise TimeoutError("live simulation timed out: " + json.dumps(node.diagnostics()))
