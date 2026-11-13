"""Check the stock Gazebo Teleop -> ROS TwistStamped -> wheel-controller route."""
import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped

from sim_checks import Observer, runtime_state, spin_until


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    state = runtime_state()
    rclpy.init(args=[])
    node = Observer(state)
    observed = []
    node.create_subscription(TwistStamped, state["cmd_vel"],
                             lambda message: observed.append(message.twist.linear.x), 10)
    deadline = time.monotonic() + 60
    result = {"status": "FAIL", "gazebo_topic": state["cmd_vel"]}

    def publish(speed):
        subprocess.run(["gz", "topic", "-t", state["cmd_vel"], "-m", "gz.msgs.Twist",
                        "-p", f"linear {{ x: {speed} }} angular {{ z: 0.0 }}"],
                       check=True, timeout=8, capture_output=True, text=True)

    try:
        spin_until(node, node.ready, deadline)
        start = tuple(node.pose)
        for _ in range(4):
            publish(0.08)
            target = node.seconds + 0.35
            spin_until(node, lambda: node.seconds >= target, deadline)
        publish(0.0)
        node.publish_speed()
        target = node.seconds + 1
        spin_until(node, lambda: node.seconds >= target, deadline)
        end = tuple(node.pose)
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        received = sum(abs(speed - 0.08) < 1e-9 for speed in observed)
        if not received or not 0.005 < distance < 0.5:
            raise RuntimeError(f"Gazebo teleop routing/motion failed: received={received}, distance={distance}")
        if abs(node.odom.twist.twist.linear.x) > 0.025:
            raise RuntimeError("Gazebo teleop test did not stop the robot")
        result.update(status="PASS", ros_type="geometry_msgs/msg/TwistStamped",
                      received_bridge_commands=received, distance_m=distance,
                      start_pose=start, end_pose=end)
    except (OSError, RuntimeError, TimeoutError, subprocess.SubprocessError) as error:
        result["error"] = str(error)
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
