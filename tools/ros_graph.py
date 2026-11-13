"""--viz ROS graph node: the live node/topic graph, grouped by robot subsystem, for the Lichtblick
"Subsystem graph" panel (viz/subsystem-graph).

Every PERIOD_S it publishes one std_msgs/String on GRAPH_TOPIC holding JSON:
  {"namespace": str, "subsystems": [{"id", "label"}],
   "topics": [{"name", "type", "publishers": [node], "subscribers": [node], "latched": bool, "subsystems": [id]}]}
`latched`: a publisher uses transient-local durability (sends once, e.g. tf_static, the map).
Node names are full ROS names. Plumbing is dropped (NOISE_TOPICS, NOISE_NODES) and each action's
`_action/feedback` + `_action/status` pair becomes one topic of type "action" from server to
clients. A topic outside every SUBSYSTEMS rule lands in "other", so nothing is hidden silently.
In SUBSYSTEMS and NOISE_TOPICS, `~` stands for the robot namespace (`/j100_0000`).
"""
import json
import os
import re
from pathlib import Path

GRAPH_TOPIC = "/greenhouse/viz/graph"
PERIOD_S = 2.0
NODE_NAME = "greenhouse_ros_graph"

# (id, label, topic regexes). Order is the panel's tab order; a topic may sit in several.
SUBSYSTEMS = (
    ("drive", "Drive & wheels", (r"~/cmd_vel$", r"~/greenhouse/nav_cmd$", r"~/joy_teleop/", r"~/rc_teleop/",
                                 r"~/twist_marker_server/cmd_vel$", r"~/platform/(cmd_vel|emergency_stop|safety_stop|odom)$",
                                 r"~/platform_velocity_controller/cmd_vel_out$", r"~/platform/joint_states$",
                                 r"~/greenhouse/manual_override$")),
    ("cameras", "Cameras & gimbal", (r"~/greenhouse/camera", r"~/greenhouse/gimbal/")),
    ("lift", "Lift", (r"~/greenhouse/lift/", r"~/platform/joint_states$")),
    ("sensors", "Sensors", (r"~/sensors/", r"^/greenhouse/physics$")),
    ("pose", "Pose & TF", (r"~/platform/odom(/filtered)?$", r"~/sensors/imu_0/data$", r"~/set_pose$",
                           r"(^|~)/tf(_static)?$", r"~/robot_description$", r"^/greenhouse/nav/odom$")),
    ("navigation", "Navigation", (r"~/navigation/(cmd_vel|plan|goal_pose|map|speed_limit)$",
                                  r"~/navigation/(navigate_to_pose|compute_path_to_pose|follow_path)$",
                                  r"~/navigation/(global|local)_costmap/costmap$", r"^/greenhouse/nav/odom$",
                                  r"~/greenhouse/nav_cmd$")),
    ("evaluation", "Evaluation", (r"^/greenhouse/ground_truth$", r"^/greenhouse/eval/", r"_mono$",
                                  r"~/platform/odom(/filtered)?$")),
)
OTHER = ("other", "Other")
NOISE_TOPICS = (r"^/rosout$", r"^/parameter_events$", r"^/clock$", r"/transition_event$", r"/bond$",
                r"diagnostics$", r"~/controller_manager/(statistics|introspection_data|activity)",
                r"_updates$", r"_raw$", r"/static_layer$", r"/(footprint|published_footprint)$",
                r"^/(clicked_point|initialpose|move_base_simple/goal)$", r"/behavior_tree_log$",
                r"/(curvature_lookahead_point|lookahead_collision_arc|unsmoothed_plan|is_rotating_to_heading)$",
                r"~/twist_marker_server/(feedback|update)$", r"/_action/(?!feedback$|status$)", r"^" + GRAPH_TOPIC + "$")
NOISE_NODES = (r"/foxglove_bridge$", r"/" + NODE_NAME + "$", r"/_ros2cli_", r"/transform_listener_impl_",
               r"/launch_ros_")
# Helper nodes shown as their owner: bt_navigator runs its action clients in `<name>_rclcpp_node`s.
NODE_ALIASES = ((re.compile(r"^(.*/bt_navigator)_\w+_rclcpp_node$"), r"\1"),)
ACTION = re.compile(r"^(.*)/_action/(feedback|status)$")


def compile_rules(namespace: str, patterns) -> list[re.Pattern]:
    ns = re.escape(f"/{namespace}" if namespace else "")
    return [re.compile(p.replace("~", ns)) for p in patterns]


def build(endpoints: dict[str, tuple], namespace: str) -> dict:
    """endpoints: topic -> (type, publisher nodes, subscriber nodes[, latched]), as the ROS graph reports them."""
    noise_topics = compile_rules(namespace, NOISE_TOPICS)
    noise_nodes = [re.compile(p) for p in NOISE_NODES]
    rules = [(sid, compile_rules(namespace, patterns)) for sid, _, patterns in SUBSYSTEMS]
    def keep(nodes):
        for pattern, owner in NODE_ALIASES:
            nodes = [pattern.sub(owner, n) for n in nodes]
        return {n for n in nodes if not any(r.search(n) for r in noise_nodes)}
    merged: dict[str, list] = {}
    for name, (kind, pubs, subs, *latched) in endpoints.items():
        if any(r.search(name) for r in noise_topics):
            continue
        action = ACTION.match(name)
        entry = merged.setdefault(action.group(1) if action else name, ["action" if action else kind, set(), set(), False])
        entry[3] = entry[3] or bool(latched and latched[0])
        entry[1] |= keep(pubs)
        entry[2] |= keep(subs) - (entry[1] if action else set())  # a server also listens to its own status
    topics = []
    for name, (kind, pubs, subs, latched) in sorted(merged.items()):
        if not pubs and not subs:
            continue
        groups = [sid for sid, patterns in rules if any(r.search(name) for r in patterns)] or [OTHER[0]]
        topics.append({"name": name, "type": kind, "publishers": sorted(pubs), "subscribers": sorted(subs),
                       "latched": latched, "subsystems": groups})
    used = {g for t in topics for g in t["subsystems"]}
    subsystems = [{"id": sid, "label": label} for sid, label, _ in SUBSYSTEMS]
    if OTHER[0] in used:
        subsystems.append({"id": OTHER[0], "label": OTHER[1]})
    return {"namespace": namespace, "subsystems": subsystems, "topics": topics}


def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from std_msgs.msg import String

    state = json.loads((Path(os.environ["GREENHOUSE_STATE"]) / "runtime.json").read_text())
    rclpy.init(args=[])
    node = Node(NODE_NAME)  # wall clock: a sim-time node would wake on every /clock message
    publisher = node.create_publisher(String, GRAPH_TOPIC, 1)
    full = lambda info: (info.node_namespace.rstrip("/") + "/" + info.node_name)

    def publish():
        endpoints = {}
        for name, types in node.get_topic_names_and_types():
            pubs = node.get_publishers_info_by_topic(name)
            endpoints[name] = (types[0], [full(i) for i in pubs], [full(i) for i in node.get_subscriptions_info_by_topic(name)],
                               any(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in pubs))
        publisher.publish(String(data=json.dumps(build(endpoints, state["namespace"]))))

    node.create_timer(PERIOD_S, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
