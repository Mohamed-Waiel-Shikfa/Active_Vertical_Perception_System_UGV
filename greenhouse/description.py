# SPDX-License-Identifier: GPL-3.0-only
"""Expand the stock+actuator wrapper, changing only explicit soil contact inputs."""
import argparse
import os
from pathlib import Path
from xml.etree import ElementTree as ET

import xacro
from greenhouse.config import load_config


def expand_description(setup: Path | str, namespace: str, terrain_path: Path | str) -> str:
    """Expand the robot xacro under ``setup`` and return URDF text with the config's wheel slip.

    Slip compliances (unitless, per Gazebo WheelSlip) are copied unchanged from ``terrain_path``'s config.
    Raises ValueError unless exactly four distinct WheelSlip wheels are found.
    """
    setup = Path(setup)
    config = load_config(terrain_path)
    document = xacro.process_file(str(setup / "greenhouse" / "robot-with-actuator.urdf.xacro"), mappings={
        "is_sim": "true", "namespace": namespace,
        "gazebo_controllers": str(setup / "platform" / "config" / "control.yaml"),
        "use_fake_hardware": "false", "use_manipulation_controllers": "true",
        "use_platform_controllers": "true",
    })
    root = ET.fromstring(document.toxml())
    wheels = []
    for plugin in root.findall(".//plugin[@name='gz::sim::systems::WheelSlip']"):
        for wheel in plugin.findall("wheel"):
            wheel.find("slip_compliance_longitudinal").text = str(config.slip_longitudinal)
            wheel.find("slip_compliance_lateral").text = str(config.slip_lateral)
            wheels.append(wheel.get("link_name"))
    if len(wheels) != 4 or len(set(wheels)) != 4:
        raise ValueError("Expected the four stock J100 WheelSlip plugins; contact configuration not applied")
    return ET.tostring(root, encoding="unicode")


def main() -> None:
    """CLI entry point: print the expanded URDF to stdout."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", required=True)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--terrain", default=os.environ.get("GREENHOUSE_CONFIG"))
    args = parser.parse_args()
    print(expand_description(args.setup, args.namespace, args.terrain))


if __name__ == "__main__":
    main()
