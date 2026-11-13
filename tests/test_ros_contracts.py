import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "ROS contract checks run inside the image")
class RosContractTests(unittest.TestCase):
    def test_clearpath_root_namespace_uses_slash_not_empty_string(self):
        from clearpath_config.clearpath_config import ClearpathConfig
        from clearpath_config.common.types.namespace import Namespace

        path = Path(__file__).parent / "fixtures" / "robot-root-namespace.yaml"
        self.assertEqual(str(ClearpathConfig(str(path)).system.namespace), "/")
        with self.assertRaises(ValueError):
            Namespace("")

    def test_stop_forward_and_turn_messages_are_serializable_floats(self):
        from builtin_interfaces.msg import Time
        from rclpy.serialization import deserialize_message, serialize_message
        from geometry_msgs.msg import TwistStamped
        from sim_checks import stamped_command

        for speed, yaw in ((0, 0), (0.18, 0), (0, 0.5), (-0.2, 0)):
            with self.subTest(speed=speed, yaw=yaw):
                message = stamped_command(Time(sec=2), speed, yaw)
                self.assertIs(type(message.twist.linear.x), float)
                self.assertIs(type(message.twist.angular.z), float)
                copied = deserialize_message(serialize_message(message), TwistStamped)
                self.assertEqual(copied.twist.linear.x, float(speed))
                self.assertEqual(copied.twist.angular.z, float(yaw))

    def test_only_named_robot_world_pose_is_accepted(self):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from sim_checks import Observer

        subject = SimpleNamespace(state={"robot_name": "j100_0000/robot"}, pose=None,
                                  pose_names=set(), pose_frames=set(), samples=[], seconds=2.0)
        for parent, name in (("", ""), ("j100_0000/robot", "base_link"),
                             ("wrong_world", "j100_0000/robot")):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, name
            Observer.pose_update(subject, TFMessage(transforms=[transform]))
            self.assertIsNone(subject.pose)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "greenhouse", "j100_0000/robot"
        transform.transform.translation.x = -7.5
        transform.transform.rotation.w = 1.0
        Observer.pose_update(subject, TFMessage(transforms=[transform]))
        self.assertEqual(subject.pose[0], -7.5)
        self.assertEqual(len(subject.samples), 1)
