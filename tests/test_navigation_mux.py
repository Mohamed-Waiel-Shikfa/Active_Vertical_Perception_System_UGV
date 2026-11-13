import importlib.util
import unittest
from pathlib import Path

spec=importlib.util.spec_from_file_location("prepare_navigation",Path(__file__).resolve().parents[1]/"tools/prepare_navigation.py")
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class NavigationMuxTests(unittest.TestCase):
    def test_navigation_is_selectable_but_below_manual_and_locks(self):
        parameters={"topics.external.topic":"cmd_vel","topics.external.priority":1,
                    "topics.joy.priority":10,"topics.rc.priority":12,
                    "topics.interactive_marker.priority":8,"locks.e_stop.priority":255,
                    "locks.safety_stop.priority":254,"locks.bt_quality.priority":253}
        before=dict(parameters)
        module.configure_mux(parameters)
        self.assertEqual(parameters["topics.navigation.priority"],1)
        self.assertEqual(parameters["topics.external.priority"],2)
        for key in before:
            if key!="topics.external.priority":
                self.assertEqual(parameters[key],before[key])

    def test_custom_higher_manual_priority_is_preserved(self):
        parameters={"topics.external.topic":"cmd_vel","topics.external.priority":5}
        module.configure_mux(parameters)
        self.assertEqual(parameters["topics.external.priority"],5)
        with self.assertRaises(RuntimeError):
            module.configure_mux({"topics.external.topic":"cmd_vel","topics.external.priority":0})
