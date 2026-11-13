# SPDX-License-Identifier: GPL-3.0-only
import argparse
import ctypes
import json
import os
import re
from dataclasses import asdict
from pathlib import Path
from typing import Any
from urllib.parse import unquote
from xml.etree import ElementTree as ET

from greenhouse.config import default_config, load_config

ROOT = Path(__file__).resolve().parents[1]


def state_file() -> Path:
    """Path of runtime.json under $GREENHOUSE_STATE; raises KeyError if that variable is unset."""
    return Path(os.environ["GREENHOUSE_STATE"]) / "runtime.json"


def read_state() -> dict[str, Any]:
    """Parsed runtime.json written by `prepare`; raises OSError or json.JSONDecodeError if unreadable."""
    return json.loads(state_file().read_text(encoding="utf-8"))


def prepare(headless: bool) -> None:
    """Validate the J100 robot.yaml (and X11 unless `headless`) and write runtime.json and terrain-state.json.

    Raises ValueError for an unsupported robot, namespace or display; KeyError if a GREENHOUSE_* variable is unset.
    """
    from clearpath_config.clearpath_config import ClearpathConfig

    setup = Path(os.environ.get("SETUP_PATH", "/etc/clearpath"))
    robot_yaml = setup / "robot.yaml"
    if not robot_yaml.is_file():
        raise ValueError(f"missing {robot_yaml}; provide your J100 robot.yaml or use run.sh's example")
    config = ClearpathConfig(str(robot_yaml))
    if str(config.get_serial_number()).split("-")[0] != "j100":
        raise ValueError("this greenhouse launcher supports the Clearpath J100 (Jackal), not another platform")
    raw_namespace = str(config.system.namespace)
    namespace = raw_namespace.strip("/")
    if namespace and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", namespace):
        raise ValueError(f"unsupported ROS namespace in robot.yaml: {namespace!r}")
    world_config = load_config(os.environ["GREENHOUSE_CONFIG"])
    default = default_config()
    assets = ROOT / "generated" if world_config == default else state_file().parent / "generated"
    if not headless:
        if not os.environ.get("DISPLAY"):
            raise ValueError("DISPLAY is missing; use --headless or start an X11/Xwayland session")
        x11 = ctypes.CDLL("libX11.so.6")
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x11.XOpenDisplay(None)
        if not display:
            raise ValueError("cannot open DISPLAY; check the X11 socket and XAUTHORITY (never use xhost +)")
        x11.XCloseDisplay(display)
    state = {
        "namespace": namespace,
        "robot_name": f"{raw_namespace}/robot" if raw_namespace not in ("", "/") else "robot",
        "cmd_vel": f"/{namespace}/cmd_vel" if namespace else "/cmd_vel",
        "controller_manager": f"/{namespace}/controller_manager" if namespace else "/controller_manager",
        "assets_dir": str(assets),
        "world_config": asdict(world_config),
        "headless": headless,
        "navigation_enabled": True,
    }
    state_file().write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    (state_file().parent / "controller-ready.json").unlink(missing_ok=True)
    terrain_state = state_file().parent / "terrain-state.json"
    terrain_state.write_text(json.dumps({"name": "greenhouse_ground", "assets_dir": str(assets)}, indent=2) + "\n")
    print(f"Greenhouse: namespace=/{namespace}; ROS teleop={state['cmd_vel']} (geometry_msgs/msg/TwistStamped)")
    print(f"Clearpath setup: {setup}; local assets: {assets}")


def check_generated() -> None:
    """Check the Clearpath and actuator outputs, write robot.expanded.urdf, and resolve every mesh to a local file.

    Raises ValueError naming the first missing, remote or unresolvable resource.
    """
    from greenhouse.description import expand_description
    from ament_index_python.packages import get_package_share_directory

    setup = Path(os.environ.get("SETUP_PATH", "/etc/clearpath"))
    for relative in ("platform/launch/platform-service.launch.py",
                     "sensors/launch/sensors-service.launch.py"):
        if not (setup / relative).is_file():
            raise ValueError(f"Clearpath generator did not produce {relative}")
    description = setup / "greenhouse" / "robot-with-actuator.urdf.xacro"
    if not description.is_file():
        raise ValueError("Actuator wrapper was not generated")
    state = read_state()
    text = expand_description(setup, state["namespace"], os.environ["GREENHOUSE_CONFIG"])
    (state_file().parent / "robot.expanded.urdf").write_text(text, encoding="utf-8")
    robot = ET.fromstring(text)
    meshes = robot.findall(".//mesh")
    for mesh in meshes:
        uri = mesh.get("filename") or mesh.findtext("uri")
        if not uri:
            raise ValueError("generated robot mesh has no resource URI")
        if uri.startswith(("package://", "model://")):
            package, separator, relative = uri.split("://", 1)[1].partition("/")
            if not separator:
                raise ValueError(f"incomplete robot mesh URI: {uri}")
            path = Path(get_package_share_directory(package)) / unquote(relative)
        elif uri.startswith("file://"):
            path = Path(unquote(uri[7:]))
        elif "://" in uri:
            raise ValueError(f"remote robot mesh URI: {uri}; vendor custom assets locally")
        else:
            path = Path(uri)
            if not path.is_absolute():
                path = setup / path
        if not path.is_file():
            raise ValueError(f"robot mesh resource is missing: {uri} -> {path}")
    print(f"Checked the stock simulation Xacro expansion: {len(meshes)} local robot mesh references.")


def main() -> None:
    """CLI for prepare / assets / check-generated; exits with status 2 on OSError or ValueError."""
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("prepare", "assets", "check-generated"))
    parser.add_argument("--headless", choices=("true", "false"), default="false")
    args = parser.parse_args()
    try:
        if args.action == "prepare":
            prepare(args.headless == "true")
        elif args.action == "assets":
            print(read_state()["assets_dir"])
        else:
            check_generated()
    except (OSError, ValueError) as error:
        parser.exit(2, f"greenhouse preflight failed: {error}\n")


if __name__ == "__main__":
    main()
