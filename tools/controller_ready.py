"""Preserve stock controllers; handle their short activation timeout during Ogre startup."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    directory = Path(os.environ["GREENHOUSE_STATE"])
    state = json.loads((directory / "runtime.json").read_text())
    rclpy.init(args=[])
    node = Node("greenhouse_controller_startup")
    clocks = []
    node.create_subscription(Clock, "/clock", lambda m: clocks.append(m.clock.sec + m.clock.nanosec*1e-9), qos_profile_sensor_data)
    client = node.create_client(ListControllers, state["controller_manager"] + "/list_controllers")
    deadline = time.monotonic() + 120
    names = ("joint_state_broadcaster", "platform_velocity_controller")
    recovered = False
    previous = None
    unchanged_since = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(clocks) < 2 or clocks[-1] - clocks[0] < 1.0 or not client.service_is_ready():
                continue
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            response = future.result() if future.done() else None
            if response is None:
                continue
            controllers = {item.name: item.state for item in response.controller}
            if all(controllers.get(name) == "active" for name in names):
                (directory / "controller-ready.json").write_text(json.dumps(controllers) + "\n")
                print("Stock wheel controllers active after real simulation clock startup.", flush=True)
                return 0
            if controllers != previous:
                previous = controllers
                unchanged_since = time.monotonic()
                continue
            if time.monotonic() - unchanged_since < 2:
                continue
            if not recovered:
                print("Renderer delayed stock controller activation; retrying stock spawner with a bounded switch timeout.", flush=True)
                inactive = [name for name in names if controllers.get(name) != "active"]
                result = subprocess.run(["ros2", "run", "controller_manager", "spawner",
                                         "-c", state["controller_manager"], "--controller-manager-timeout", "45",
                                         "--switch-timeout", "45", *inactive], timeout=70)
                if result.returncode:
                    final = client.call_async(ListControllers.Request())
                    rclpy.spin_until_future_complete(node, final, timeout_sec=3)
                    status = {item.name: item.state for item in final.result().controller} if final.done() and final.result() else {}
                    if not all(status.get(name) == "active" for name in names):
                        raise RuntimeError("stock controller spawner recovery failed")
                    print("Stock startup completed concurrently; active controllers verified.", flush=True)
                recovered = True
            else:
                time.sleep(0.2)
        raise TimeoutError("wheel controllers did not activate within the startup deadline")
    except (OSError, RuntimeError, TimeoutError, subprocess.SubprocessError) as error:
        print(f"Controller startup failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
