"""config/foxglove-layout.json parses and names the topics the code publishes."""
import importlib.util
import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("pose_error", ROOT / "tools/pose_error.py")
pose_error = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose_error)


def leaves(node):
    if isinstance(node, str):
        return [node]
    return leaves(node["first"]) + leaves(node["second"])


class FoxgloveLayout(unittest.TestCase):
    def setUp(self):
        self.layout = json.loads((ROOT / "config/foxglove-layout.json").read_text(encoding="utf-8"))
        self.config = self.layout["configById"]
        example = (ROOT / "config/robot.example.yaml").read_text(encoding="utf-8")
        self.namespace = re.search(r"^\s+namespace:\s*(\S+)", example, re.M).group(1)

    def test_every_panel_is_placed_and_configured(self):
        placed = leaves(self.layout["layout"])
        for tab in self.config.get("Tab!hub", {}).get("tabs", []):
            placed += leaves(tab["layout"])
        self.assertEqual(sorted(placed), sorted(self.config))
        for panel in self.config:
            self.assertIn(panel.split("!")[0], {"3D", "Image", "Plot", "Greenhouse.Subsystem graph", "Tab"})

    def test_panels_are_titled(self):
        # Lichtblick reads lichtblickPanelTitle; foxglovePanelTitle is ignored there.
        for panel, value in self.config.items():
            if not panel.startswith(("3D!", "Image!", "Plot!")):
                continue
            self.assertTrue(value.get("lichtblickPanelTitle"), panel)
            self.assertNotIn("foxglovePanelTitle", value)

    def test_topics_match_the_published_ones(self):
        published = pose_error.topics(self.namespace)
        plots = [path["value"] for key, value in self.config.items() if key.startswith("Plot!") for path in value["paths"]]
        images = {value["imageMode"]["imageTopic"] for key, value in self.config.items() if key.startswith("Image!")}
        self.assertEqual(images, {published[c] + suffix for c in ("camera1", "camera2") for suffix in ("", pose_error.MONO_SUFFIX)})
        plot_topics = {value.split(".", 1)[0] for value in plots}
        self.assertTrue(set(pose_error.ERROR_TOPICS.values()) <= plot_topics)
        for value in plots:
            topic, field = value.split(".", 1)
            if topic in pose_error.ERROR_TOPICS.values():
                self.assertLess(int(re.fullmatch(r"data\[(\d+)\]", field).group(1)), len(pose_error.FIELDS))
            else:
                self.assertTrue(topic.startswith(f"/{self.namespace}/sensors/imu_0/"), topic)
        scene = self.config["3D!robot"]
        ns = f"/{self.namespace}"
        self.assertTrue({ns + "/tf", ns + "/tf_static", ns + "/sensors/lidar2d_0/scan"} <= set(scene["topics"]))
        self.assertEqual(scene["layers"]["robot"]["topic"], ns + "/robot_description")


if __name__ == "__main__":
    unittest.main()
