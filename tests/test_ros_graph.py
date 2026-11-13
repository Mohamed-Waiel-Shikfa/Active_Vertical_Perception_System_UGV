"""tools/ros_graph.py: noise filtering, action folding and subsystem grouping of the ROS graph."""
import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("ros_graph", ROOT / "tools" / "ros_graph.py")
graph = importlib.util.module_from_spec(spec)
spec.loader.exec_module(graph)

NS = "/j100_0000"
ENDPOINTS = {
    "/rosout": ("rcl_interfaces/msg/Log", ["/a"], []),
    NS + "/platform/cmd_vel": ("geometry_msgs/msg/TwistStamped", [NS + "/twist_mux"],
                               [NS + "/platform_velocity_controller", "/foxglove_bridge"]),
    NS + "/platform/joint_states": ("sensor_msgs/msg/JointState", ["/greenhouse_sensor_bridge"],
                                    [NS + "/robot_state_publisher"]),
    NS + "/navigation/navigate_to_pose/_action/feedback": ("x", [NS + "/navigation/bt_navigator"],
                                                           ["/greenhouse_navigation_gateway", NS + "/navigation/bt_navigator"]),
    NS + "/navigation/navigate_to_pose/_action/status": ("y", [NS + "/navigation/bt_navigator"],
                                                         ["/greenhouse_navigation_gateway"]),
    NS + "/navigation/compute_path_to_pose/_action/status": (
        "y", [NS + "/navigation/planner_server"], [NS + "/navigation/bt_navigator_navigate_to_pose_rclcpp_node"]),
    NS + "/navigation/map_server/transition_event": ("lifecycle_msgs/msg/TransitionEvent", [NS + "/navigation/map_server"], []),
    "/unclassified": ("std_msgs/msg/String", ["/b"], []),
    "/tf_static": ("tf2_msgs/msg/TFMessage", ["/rsp"], ["/c"], True),
    "/only_viewer": ("std_msgs/msg/String", [], ["/foxglove_bridge"]),
}


class RosGraph(unittest.TestCase):
    def setUp(self):
        self.out = graph.build(ENDPOINTS, "j100_0000")
        self.topics = {t["name"]: t for t in self.out["topics"]}

    def test_noise_and_viewer_only_topics_are_dropped(self):
        self.assertNotIn("/rosout", self.topics)
        self.assertNotIn(NS + "/navigation/map_server/transition_event", self.topics)
        self.assertNotIn("/only_viewer", self.topics)  # its only endpoint is the bridge
        self.assertEqual(self.topics[NS + "/platform/cmd_vel"]["subscribers"], [NS + "/platform_velocity_controller"])

    def test_action_topics_fold_into_one_edge(self):
        action = self.topics[NS + "/navigation/navigate_to_pose"]
        self.assertEqual(action["type"], "action")
        self.assertEqual(action["publishers"], [NS + "/navigation/bt_navigator"])
        self.assertEqual(action["subscribers"], ["/greenhouse_navigation_gateway"])
        planner = self.topics[NS + "/navigation/compute_path_to_pose"]
        self.assertEqual(planner["subscribers"], [NS + "/navigation/bt_navigator"])
        self.assertFalse(any("_action" in name for name in self.topics))

    def test_subsystems(self):
        self.assertEqual(self.topics[NS + "/platform/joint_states"]["subsystems"], ["drive", "lift"])
        self.assertEqual(self.topics[NS + "/navigation/navigate_to_pose"]["subsystems"], ["navigation"])
        self.assertEqual(self.topics["/unclassified"]["subsystems"], ["other"])
        ids = [s["id"] for s in self.out["subsystems"]]
        self.assertEqual(ids, [s[0] for s in graph.SUBSYSTEMS] + ["other"])

    def test_latched(self):
        self.assertTrue(self.topics["/tf_static"]["latched"])
        self.assertFalse(self.topics["/unclassified"]["latched"])

    def test_root_namespace(self):
        out = graph.build({"/platform/cmd_vel": ("t", ["/twist_mux"], ["/c"])}, "")
        self.assertEqual(out["topics"][0]["subsystems"], ["drive"])
        self.assertNotIn("other", [s["id"] for s in out["subsystems"]])


if __name__ == "__main__":
    unittest.main()
