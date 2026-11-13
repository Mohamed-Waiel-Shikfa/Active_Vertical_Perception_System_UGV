"""Small deadman keyboard client for the stock Clearpath TwistStamped topic."""
import argparse
import select
import sys
import termios
import time
import tty
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, Float64, String
import json

from sim_checks import runtime_state, stamped_command
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from greenhouse.keyboard import KeyDecoder, velocities


def main():
    parser = argparse.ArgumentParser(description="Arrows drive; A/Q lift; C cycles 3D/camera 1/camera 2; space stops; Esc/Ctrl-C exits.")
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--turn", type=float, default=0.5)
    args = parser.parse_args()
    if not (0 < args.speed <= 0.3 and 0 < args.turn <= 0.6):
        parser.error("speed must be in (0, 0.3] m/s and turn in (0, 0.6] rad/s")
    if not sys.stdin.isatty():
        parser.error("keyboard teleop needs docker exec -it")
    state = runtime_state()
    rclpy.init(args=[])
    node = Node("greenhouse_keyboard", parameter_overrides=[Parameter("use_sim_time", value=True)])
    publisher = node.create_publisher(TwistStamped, state["cmd_vel"], 10)
    prefix = ("/" + state["namespace"] if state["namespace"] else "") + "/greenhouse"
    lift = node.create_publisher(Float64, prefix + "/lift/velocity", 10)
    camera = node.create_publisher(Empty, prefix + "/camera/toggle", 10)
    override = node.create_publisher(Empty, prefix + "/manual_override", 10)
    lift_state = {"blocked": False}
    def receive_lift(message):
        lift_state.update(json.loads(message.data))
    node.create_subscription(String, prefix + "/lift/state", receive_lift, 10)
    settings = termios.tcgetattr(sys.stdin)
    decoder, pressed = KeyDecoder(), {}
    last_camera = -1
    previous = (0.0, 0.0, 0.0)
    print(f"{state['cmd_vel']} (TwistStamped, simulation timestamps)\nArrows drive; A/Q lift; C: 3D -> cam 1 -> cam 2; Space stops; Esc/Ctrl-C quits. 0.35 s repeat deadman.")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            text = os.read(sys.stdin.fileno(), 64).decode("utf-8", errors="replace") if select.select([sys.stdin], [], [], 0.04)[0] else ""
            now = time.monotonic()
            keys = decoder.feed(text, now)
            if "quit" in keys:
                break
            for key in keys:
                if key == "stop":
                    pressed.clear()
                    override.publish(Empty())
                    publisher.publish(stamped_command(node.get_clock().now().to_msg()))
                elif key == "c":
                    if now - last_camera > 0.7:
                        if state.get("headless"):
                            print("\nC unavailable: this simulation was started headless.", file=sys.stderr)
                        else:
                            camera.publish(Empty())
                    last_camera = now
                else:
                    pressed[key] = now
                    if key in ("up","down","left","right"):
                        override.publish(Empty())
            pressed = {key: stamp for key, stamp in pressed.items() if now - stamp <= 0.35}
            if lift_state.get("blocked"):
                pressed.clear()
            if node.get_clock().now().nanoseconds == 0:
                continue
            forward, yaw, vertical = velocities(pressed, args.speed, args.turn)
            if forward != 0 or yaw != 0 or previous[0] != 0 or previous[1] != 0:
                publisher.publish(stamped_command(node.get_clock().now().to_msg(), forward, yaw))
            if vertical != 0 or previous[2] != 0:
                lift.publish(Float64(data=vertical))
            previous = (forward, yaw, vertical)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        override.publish(Empty())
        for _ in range(5):
            publisher.publish(stamped_command(node.get_clock().now().to_msg()))
            lift.publish(Float64(data=0.0))
            time.sleep(0.03)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
