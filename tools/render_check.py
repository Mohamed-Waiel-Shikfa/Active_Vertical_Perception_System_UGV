"""Render the actual running greenhouse with a temporary native Gazebo camera."""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = "/greenhouse/render_check"
MODEL = "greenhouse_validation_camera"


def service(name, request_type, request):
    result = subprocess.run([
        "gz", "service", "-s", f"/world/greenhouse/{name}",
        "--reqtype", request_type, "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000", "--req", request,
    ], text=True, capture_output=True, timeout=10)
    if result.returncode or "data: true" not in result.stdout:
        raise RuntimeError(f"Gazebo {name} failed: {result.stdout} {result.stderr}")


def camera_sdf():
    root = ET.Element("sdf", version="1.9")
    model = ET.SubElement(root, "model", name=MODEL)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "-12 -12 9 0 0.45 0.785398"
    link = ET.SubElement(model, "link", name="camera_link")
    sensor = ET.SubElement(link, "sensor", name="camera", type="camera")
    for key, value in (("always_on", "true"), ("update_rate", "2"), ("topic", TOPIC)):
        ET.SubElement(sensor, key).text = value
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = "1.2"
    image = ET.SubElement(camera, "image")
    for key, value in (("width", "1280"), ("height", "720"), ("format", "R8G8B8")):
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.05"
    ET.SubElement(clip, "far").text = "60"
    return ET.tostring(root, encoding="unicode")


def pose_request(x, y, z, pitch, yaw):
    cp, sp, cy, sy = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2)
    return (f'name: "{MODEL}" position {{ x: {x} y: {y} z: {z} }} '
            f'orientation {{ x: {-sp * sy} y: {sp * cy} z: {cp * sy} w: {cp * cy} }}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = Node("greenhouse_render_check")
    frames = []

    def receive(message):
        frames.append(message)
        del frames[:-3]

    node.create_subscription(Image, TOPIC, receive, qos_profile_sensor_data)
    bridge_log = (args.output_dir / "camera-bridge.log").open("w")
    bridge = subprocess.Popen([
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        TOPIC + "@sensor_msgs/msg/Image[gz.msgs.Image",
    ], stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True)
    created = False
    reports = {}
    try:
        service("create", "gz.msgs.EntityFactory", "sdf: " + json.dumps(camera_sdf()))
        created = True
        views = [
            ("overview", (-12, -12, 9, 0.45, math.pi / 4)),
            ("aisle", (-8.4, 0, 1.7, 0.10, 0)),
            ("tomato", (-5.4, -0.4, 1.1, 0.24, 1.0)),
        ]
        for label, pose in views:
            service("set_pose", "gz.msgs.Pose", pose_request(*pose))
            frames.clear()
            deadline = time.monotonic() + 60
            while len(frames) < 3:
                if bridge.poll() is not None:
                    raise RuntimeError("camera bridge exited; inspect camera-bridge.log")
                if time.monotonic() > deadline:
                    raise TimeoutError(f"no rendered {label} frames; inspect the Gazebo log")
                rclpy.spin_once(node, timeout_sec=0.1)
            message = frames[-1]
            if message.encoding not in ("rgb8", "bgr8"):
                raise ValueError(f"unexpected camera encoding: {message.encoding}")
            pixels = np.asarray(message.data, dtype=np.uint8).reshape(message.height, message.step)
            pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
            bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else pixels.copy()
            blue, green, red = [bgr[:, :, index].astype(float) for index in (0, 1, 2)]
            red_count = int(np.sum((red > 60) & (red > green * 1.5) & (red > blue * 1.5)))
            green_count = int(np.sum((green > 35) & (green > red * 1.12) & (green > blue * 1.2)))
            if bgr.std() < 15 or (label == "tomato" and (red_count < 50 or green_count < 50)):
                raise RuntimeError(f"{label} render lacks expected geometry/foliage/fruit color")
            if not cv2.imwrite(str(args.output_dir / f"{label}.png"), bgr):
                raise OSError(f"failed to save {label}.png")
            reports[label] = {"width": message.width, "height": message.height,
                              "red_pixels": red_count, "green_pixels": green_count,
                              "pixel_standard_deviation": float(bgr.std())}
        (args.output_dir / "render-result.json").write_text(json.dumps(reports, indent=2) + "\n")
        print(json.dumps(reports, indent=2))
    finally:
        try:
            if created:
                service("remove", "gz.msgs.Entity", f'name: "{MODEL}" type: MODEL')
        finally:
            try:
                os.killpg(bridge.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                bridge.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(bridge.pid, signal.SIGKILL)
                bridge.wait()
            bridge_log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
