# SPDX-License-Identifier: GPL-3.0-only
"""Compose stock Clearpath robot spawning with a local custom Gazebo world."""
import json
import os
import shlex
from pathlib import Path
from xml.etree import ElementTree as ET
from xml.sax.saxutils import escape

from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
VIZ_PORT = 8767  # Foxglove WebSocket; run.sh publishes it on host 127.0.0.1 only (8765 is the gateway)


def launch_setup(context):
    setup = LaunchConfiguration("setup_path").perform(context)
    assets = Path(LaunchConfiguration("assets_dir").perform(context))
    headless = LaunchConfiguration("headless").perform(context) == "true"
    manifest = json.loads((assets / "manifest.json").read_text(encoding="utf-8"))
    if manifest["world"] != "greenhouse":
        raise ValueError("the generated world must be named greenhouse")
    raw_namespace = str(ClearpathConfig(str(Path(setup) / "robot.yaml")).system.namespace)
    namespace = raw_namespace.strip("/")
    topic = f"/{namespace}/cmd_vel" if namespace else "/cmd_vel"
    robot_name = f"{raw_namespace}/robot" if raw_namespace not in ("", "/") else "robot"
    pose_topic = f"/model/{robot_name}/pose"
    prefix = f"/{namespace}/greenhouse" if namespace else "/greenhouse"
    gui = (ROOT / "launch" / "gui.config.in").read_text(encoding="utf-8")
    gui = gui.replace("@CMD_VEL@", escape(topic)).replace("@PREFIX@", escape(prefix))
    gui = gui.replace("@GROUND_RMS@", str(round(manifest["config"]["ground_bump_rms"] * 1000)))
    test_mode = LaunchConfiguration("gui_test").perform(context)
    test_dir = str(Path(setup) / "greenhouse") if test_mode != "false" else ""
    gui = gui.replace("@TEST_OUTPUT@", escape(test_dir))
    gui = gui.replace("@TEST_PHASE@", "remaining" if test_mode == "remaining" else "full")
    gui_path = Path(setup) / "greenhouse" / "gui.config"
    gui_path.write_text(gui, encoding="utf-8")
    resource_dirs = [str(assets / "models"), str(assets / "worlds")]
    resource_dirs += [str(Path(prefix) / "share") for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if prefix]
    actions = [AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_dirs))]
    actions.append(AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", str(ROOT / "native" / "lib")))
    actions.append(AppendEnvironmentVariable("GZ_GUI_PLUGIN_PATH", str(ROOT / "native" / "lib")))
    world = ET.parse(assets / "worlds" / "greenhouse.sdf")
    ET.SubElement(world.find("world"), "plugin", filename="libGreenhouseSystems.so", name="greenhouse::WorldState")
    world_path = Path(setup) / "greenhouse" / "runtime-world.sdf"
    world.write(world_path, encoding="utf-8", xml_declaration=True)
    gz_args = f'-r -v 3 "{world_path}"'
    gz_args += " -s --headless-rendering" if headless else f' --gui-config "{gui_path}"'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()))
    actions.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="greenhouse_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen"))
    actions.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="greenhouse_ground_truth_bridge",
        arguments=[pose_topic + "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[(pose_topic, "/greenhouse/ground_truth")],
        parameters=[{"use_sim_time": True}], output="screen"))
    spawn_args = {
        "setup_path": setup, "world": "greenhouse", "use_sim_time": "true",
        "generate": "false", "rviz": "false",
        **{key: str(value) for key, value in manifest["spawn"].items()},
        "robot_description_command": " ".join(shlex.quote(part) for part in (
            "python3", "-m", "greenhouse.description", "--setup", setup, "--namespace", namespace)),
    }
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("clearpath_gz")) / "launch" / "robot_spawn.launch.py")),
        launch_arguments=spawn_args.items()))
    actions.append(ExecuteProcess(cmd=["python3", str(ROOT / "tools" / "controller_ready.py")],
                                  output="screen"))
    actions.append(ExecuteProcess(cmd=["python3", str(ROOT / "tools" / "navigation_gateway.py")],
                                  output="screen"))
    actions.append(ExecuteProcess(cmd=["python3", str(ROOT / "tools" / "tf_unify.py")],
                                  output="screen"))
    nav_namespace=(namespace+"/" if namespace else "")+"navigation"
    nav_params=str(Path(setup)/"greenhouse"/"navigation.yaml")
    for package, executable in (
        ("nav2_map_server","map_server"), ("nav2_planner","planner_server"),
        ("nav2_controller","controller_server"), ("nav2_bt_navigator","bt_navigator")):
        actions.append(Node(package=package,executable=executable,name=executable,
                            namespace=nav_namespace,parameters=[nav_params],output="screen"))
    actions.append(Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",namespace=nav_namespace,
                        parameters=[{"use_sim_time":True,"autostart":True,"bond_timeout":10.0,
                                     "attempt_respawn_reconnection":False,
                                     "node_names":["map_server","planner_server","controller_server","bt_navigator"]}],
                        output="screen"))
    # Written by greenhouse.actuator in docker/start.sh; null when the lidar is disabled.
    lidar = json.loads((Path(setup) / "greenhouse" / "actuator.json").read_text(encoding="utf-8")).get("lidar")
    actions.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="greenhouse_sensor_bridge",
        arguments=[
            prefix + "/camera1/image@sensor_msgs/msg/Image[gz.msgs.Image",
            prefix + "/camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            prefix + "/camera2/image@sensor_msgs/msg/Image[gz.msgs.Image",
            prefix + "/camera2/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            prefix + "/gimbal/state@std_msgs/msg/String[gz.msgs.StringMsg",
            prefix + "/gimbal/command@std_msgs/msg/String]gz.msgs.StringMsg",
            prefix + "/lift/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            prefix + "/lift/state@std_msgs/msg/String[gz.msgs.StringMsg",
            prefix + "/lift/velocity@std_msgs/msg/Float64]gz.msgs.Double",
            prefix + "/camera/toggle@std_msgs/msg/Empty]gz.msgs.Empty",
            "/greenhouse/physics@std_msgs/msg/String[gz.msgs.StringMsg",
            prefix + "/manual_override@std_msgs/msg/Empty[gz.msgs.Empty",
        ] + ([lidar["topic"] + "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"] if lidar else []),
        remappings=[(prefix + "/lift/joint_state", f"/{namespace}/platform/joint_states" if namespace else "/platform/joint_states")],
        parameters=[{"use_sim_time": True}], output="screen"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory("clearpath_viz")) / "launch" / "view_robot.launch.py")),
        launch_arguments={"namespace": namespace, "use_sim_time": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("rviz"))))
    if LaunchConfiguration("viz").perform(context) == "true":
        # 0.0.0.0 inside the container so Docker's port publish reaches it; run.sh binds the host side.
        # Ground truth is a TFMessage holding <model> -> base_link: Foxglove would read it as a second
        # parent of base_link, so it stays inside ROS (the error topics carry the truth values).
        actions.append(Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
            parameters=[{"port": VIZ_PORT, "address": "0.0.0.0", "use_sim_time": True,
                         "topic_whitelist": ["^(?!/greenhouse/ground_truth$).*"]}], output="screen"))
        actions.append(ExecuteProcess(cmd=["python3", str(ROOT / "tools" / "pose_error.py")], output="screen"))
        actions.append(ExecuteProcess(cmd=["python3", str(ROOT / "tools" / "ros_graph.py")], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("setup_path", default_value=os.environ.get("SETUP_PATH", "/etc/clearpath")),
        DeclareLaunchArgument("assets_dir", default_value=str(ROOT / "generated")),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("viz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("gui_test", default_value="false", choices=["true", "false", "remaining"]),
        OpaqueFunction(function=launch_setup),
    ])
