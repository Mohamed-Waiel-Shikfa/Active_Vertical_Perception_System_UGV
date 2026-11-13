import importlib.util
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("prepare_platform", ROOT / "tools" / "prepare_platform.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
# Clearpath's generated J100 control.yaml (clearpath_generator_common, Jazzy), unmodified.
GENERATED = (ROOT / "tests" / "fixtures" / "clearpath-control.yaml").read_text()


def controller(text):
    return yaml.safe_load(text)["j100_0000"]["platform_velocity_controller"]["ros__parameters"]


class PreparePlatformTests(unittest.TestCase):
    def test_bundled_overrides_apply_and_touch_only_their_lines(self):
        overrides = yaml.safe_load(module.OVERRIDES.read_text())
        text, applied = module.apply_overrides(GENERATED, overrides, {})
        wanted = module.flatten(overrides["platform_velocity_controller"])
        self.assertEqual(len(applied), len(wanted))
        for name, value in wanted.items():
            self.assertEqual(controller(text)[name], value)
        changed = [(a, b) for a, b in zip(GENERATED.splitlines(), text.splitlines()) if a != b]
        self.assertEqual(len(changed), sum(controller(GENERATED)[n] != v for n, v in wanted.items()))
        self.assertIn("linear.x.max_jerk: nan", text)  # a YAML round trip would quote it

    def test_robot_yaml_extras_keep_the_users_value(self):
        robot = {"platform": {"extras": {"ros_parameters": {"platform_velocity_controller": {
            "linear": {"x": {"max_acceleration": 3.0}}}}}}}
        extras = module.user_extras(robot)
        text, applied = module.apply_overrides(
            GENERATED, {"platform_velocity_controller": {"linear.x.max_acceleration": 10.0}}, extras)
        self.assertEqual(applied, [])
        self.assertEqual(text, GENERATED)

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaisesRegex(KeyError, "linear.x.max_acceleration_typo"):
            module.apply_overrides(GENERATED, {"platform_velocity_controller": {"linear.x.max_acceleration_typo": 1.0}}, {})


if __name__ == "__main__":
    unittest.main()
