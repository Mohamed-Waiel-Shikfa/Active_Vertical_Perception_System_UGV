"""Bounded actual-camera and finite-force endpoint check on a production server."""
import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, "/opt/greenhouse")
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from greenhouse.transport import settings, world_status


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=260)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    directory = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
    config = json.loads((directory / "actuator.json").read_text())
    prefix, stroke = config["prefix"], config["config"]["stroke_m"]
    rclpy.init(args=[])
    node = Node("fpb30_endpoint_verification")
    status, frames, transforms = {}, [], {}
    samples = []
    def state_update(message):
        status.update(json.loads(message.data))
        samples.append(dict(status))
    def camera_update(message):
        frames.append(message)
        del frames[:-2]
    def pose_update(message):
        for transform in message.transforms:
            if transform.child_frame_id.endswith("/fpb30_carriage"):
                transforms["carriage"] = transform
    node.create_subscription(String, prefix + "/lift/state", state_update, 10)
    # Camera 1 (left gimbal) rides on the carriage; its image must change with carriage height.
    node.create_subscription(Image, prefix + "/camera1/image", camera_update, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/greenhouse/ground_truth", pose_update, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    def wait(predicate, message):
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise TimeoutError(message)
    def capture(label):
        old = frames[-1].header.stamp if frames else None
        wait(lambda: frames and frames[-1].header.stamp != old, "camera stopped publishing")
        message = frames[-1]
        if message.encoding != "rgb8":
            raise ValueError(f"unexpected camera format: {message.encoding}")
        pixels = np.asarray(message.data, dtype=np.uint8).reshape(message.height, message.step)
        rgb = pixels[:, :message.width*3].reshape(message.height, message.width, 3)
        if not cv2.imwrite(str(args.output / (label + ".png")), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise OSError("could not save camera evidence")
        return {"frame_id": message.header.frame_id, "width": message.width, "height": message.height,
                "sha256": hashlib.sha256(rgb.tobytes()).hexdigest(),
                "position": status["position"],
                "carriage_z": transforms["carriage"].transform.translation.z}
    result = {"status": "FAIL", "stroke_m": stroke}
    try:
        wait(lambda: status.get("ready") and len(frames) > 0 and "carriage" in transforms,
             "live actuator state/camera/carriage transforms missing")
        wait(lambda: abs(status["velocity"]) < 0.002 and abs(status["position"]) < 0.002,
             "actuator did not settle at lower hard stop")
        result["lower"] = capture("camera-lower")
        before = world_status()
        first_sample = len(samples)
        settings(prefix, {"target": stroke})
        wait(lambda: abs(status["position"] - stroke) < 0.001 and abs(status["velocity"]) < 0.004,
             "actuator did not reach the upper hard stop")
        result["upper"] = capture("camera-upper")
        travel = result["upper"]["carriage_z"] - result["lower"]["carriage_z"]
        if abs(travel - stroke) > 0.002:
            raise RuntimeError("actual carriage transform does not match configured stroke")
        if result["lower"]["sha256"] == result["upper"]["sha256"]:
            raise RuntimeError("camera image did not change with physical carriage motion")
        try:
            settings(prefix, {"target": stroke + 0.1})
        except ValueError:
            result["outside_hard_stroke_rejected"] = True
        else:
            raise RuntimeError("target above hard stroke was accepted")
        settings(prefix, {"target": 0.0})
        wait(lambda: abs(status["position"]) < 0.001 and abs(status["velocity"]) < 0.004,
             "actuator did not return to the lower hard stop")
        measured = samples[first_sample:]
        max_force = max(abs(sample["force"]) for sample in measured)
        max_speed = max(abs(sample["velocity"]) for sample in measured)
        if max_force > config["config"]["force_limit_n"] + 1e-6:
            raise RuntimeError("finite force cap was exceeded")
        if max_speed > config["max_vertical_speed_m_s"] + 0.002:
            raise RuntimeError("native physical velocity limit was exceeded")
        if min(sample["position"] for sample in measured) < -0.002 or max(sample["position"] for sample in measured) > stroke+0.002:
            raise RuntimeError("native physical hard stop was exceeded")
        result.update(status="PASS", measured_travel_m=travel, samples=len(measured),
                      max_effort_n=max_force, max_speed_m_s=max_speed,
                      final_position=status["position"], native_robot_id_before=before["robot"]["id"],
                      native_robot_id_after=world_status()["robot"]["id"])
    except (ValueError, RuntimeError, TimeoutError, OSError) as error:
        result["error"] = str(error)
        result["last_state"] = status
    finally:
        try:
            if status:
                settings(prefix, {"target": max(0.0, min(stroke, status["position"]))})
        except (ValueError, RuntimeError, TimeoutError, OSError) as error:
            result["stop_error"] = str(error)
        node.destroy_node()
        rclpy.shutdown()
        (args.output / "result.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
