"""Pure math of tools/pose_error.py: no ROS graph needed."""
import importlib.util
import math
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("pose_error", Path(__file__).resolve().parents[1] / "tools/pose_error.py")
pose_error = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pose_error)


def track(x0, y0, yaw0, vx, wz, start=0.0, end=3.0, dt=0.01):
    """Truth samples of a unicycle moving at body speed vx and yaw rate wz."""
    samples, x, y, yaw, t = [], x0, y0, yaw0, start
    while t <= end + 1e-9:
        samples.append((round(t, 6), x, y, pose_error.wrap(yaw)))
        x += vx*math.cos(yaw)*dt
        y += vx*math.sin(yaw)*dt
        yaw += wz*dt
        t += dt
    return samples


class PoseErrorMath(unittest.TestCase):
    def test_wrap_stays_within_pi(self):
        for angle, expected in ((0.0, 0.0), (3*math.pi/2, -math.pi/2), (-3*math.pi/2, math.pi/2), (7.0, 7.0 - 2*math.pi)):
            self.assertAlmostEqual(pose_error.wrap(angle), expected)
        self.assertLessEqual(abs(pose_error.wrap(math.pi + 1e-9)), math.pi)

    def test_compose_and_inverse(self):
        a, b = (1.0, -2.0, 0.7), (0.3, 0.4, -2.9)
        identity = pose_error.compose(a, pose_error.inverse(a))
        for value in identity:
            self.assertAlmostEqual(value, 0.0)
        back = pose_error.compose(pose_error.inverse(a), pose_error.compose(a, b))
        for got, want in zip(back, b):
            self.assertAlmostEqual(got, want)
        self.assertAlmostEqual(pose_error.yaw_of(0, 0, math.sin(1.2/2), math.cos(1.2/2)), 1.2)

    def test_interpolate_takes_the_short_way_round_pi(self):
        samples = [(0.0, 0.0, 0.0, math.pi - 0.1), (1.0, 2.0, -4.0, -math.pi + 0.1)]
        x, y, yaw = pose_error.interpolate(samples, 0.5)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, -2.0)
        self.assertAlmostEqual(abs(yaw), math.pi)
        self.assertIsNone(pose_error.interpolate(samples, 1.01))
        self.assertIsNone(pose_error.interpolate([], 0.0))
        self.assertEqual(pose_error.interpolate(samples, 1.0), (2.0, -4.0, -math.pi + 0.1))

    def test_truth_velocity_is_body_frame_and_waits_for_the_window(self):
        samples = track(-7.5, 0.0, 2.5, vx=0.8, wz=0.3)
        self.assertIsNone(pose_error.truth_state(samples, samples[-1][0] - 0.01))
        x, y, yaw, vx, wz = pose_error.truth_state(samples, 1.5)
        self.assertAlmostEqual(vx, 0.8, places=3)
        self.assertAlmostEqual(wz, 0.3, places=6)
        reverse = pose_error.truth_state(track(0, 0, -3.0, vx=-0.5, wz=0.0), 1.0)
        self.assertAlmostEqual(reverse[3], -0.5, places=6)

    def test_alignment_removes_the_spawn_offset(self):
        # Odometry starts at (0, 0, 0) while truth starts at the greenhouse spawn pose.
        spawn = (-7.5, 0.0, 0.4)
        truth = track(*spawn, vx=0.6, wz=0.2)
        odom = track(0.0, 0.0, 0.0, vx=0.6, wz=0.2)
        evaluator = pose_error.Evaluator()
        for t, x, y, yaw in odom[:200]:
            evaluator.push(t, (x, y, yaw), 0.6, 0.2)
        rows = evaluator.drain(truth)
        self.assertEqual(len(rows), 200)
        for row in rows:
            values = dict(zip(pose_error.FIELDS, row))
            self.assertLess(values["error_planar_m"], 1e-9)
            self.assertLess(abs(values["error_yaw_rad"]), 1e-9)
            self.assertLess(abs(values["error_vx_m_s"]), 1e-3)
            self.assertLess(abs(values["error_wz_rad_s"]), 1e-6)
        last = dict(zip(pose_error.FIELDS, rows[-1]))
        self.assertAlmostEqual(last["estimate_x_m"], last["truth_x_m"])
        self.assertLess(last["truth_x_m"], -6.0)

    def test_drift_and_yaw_error_after_alignment(self):
        truth = track(2.0, 1.0, 0.0, vx=0.0, wz=0.0)
        evaluator = pose_error.Evaluator()
        evaluator.push(0.5, (0.0, 0.0, 0.0), 0.0, 0.0)          # aligns
        evaluator.push(1.0, (0.3, -0.4, math.pi - 0.05), 0.1, -0.2)  # drifted estimate
        first, drifted = (dict(zip(pose_error.FIELDS, row)) for row in evaluator.drain(truth))
        self.assertEqual(first["error_planar_m"], 0.0)
        self.assertAlmostEqual(drifted["error_x_m"], 0.3)
        self.assertAlmostEqual(drifted["error_y_m"], -0.4)
        self.assertAlmostEqual(drifted["error_planar_m"], 0.5)
        self.assertAlmostEqual(drifted["error_yaw_rad"], math.pi - 0.05)
        self.assertAlmostEqual(drifted["error_vx_m_s"], 0.1)
        self.assertAlmostEqual(drifted["error_wz_rad_s"], -0.2)

    def test_evaluator_waits_for_truth_and_drops_unscorable_samples(self):
        truth = track(0, 0, 0, vx=0.0, wz=0.0, start=1.0, end=2.0)
        evaluator = pose_error.Evaluator()
        evaluator.push(0.5, (0, 0, 0), 0, 0)   # before the truth history
        evaluator.push(1.99, (0, 0, 0), 0, 0)  # truth does not reach t + window yet
        self.assertEqual(evaluator.drain(truth), [])
        self.assertEqual(len(evaluator.pending), 1)
        self.assertEqual(len(evaluator.drain(track(0, 0, 0, 0.0, 0.0, start=1.0, end=2.1))), 1)
        self.assertEqual(evaluator.drain([]), [])
        evaluator.reset()
        self.assertIsNone(evaluator.offset)

    def test_mono8_luma_and_row_padding(self):
        # 2x2 rgb8 with 2 padding bytes per row: white, red / green, blue.
        rows = bytes([255, 255, 255, 255, 0, 0, 9, 9, 0, 255, 0, 0, 0, 255, 9, 9])
        self.assertEqual(pose_error.mono8(rows, 2, 2, 8, "rgb8"), bytes([255, 77, 149, 29]))
        bgr = bytes([255, 255, 255, 0, 0, 255, 9, 9, 0, 255, 0, 255, 0, 0, 9, 9])
        self.assertEqual(pose_error.mono8(bgr, 2, 2, 8, "bgr8"), bytes([255, 77, 149, 29]))
        self.assertEqual(pose_error.mono8(bytes([1, 2, 0, 3, 4, 0]), 2, 2, 3, "mono8"), bytes([1, 2, 3, 4]))
        with self.assertRaises(ValueError):
            pose_error.mono8(bytes(8), 2, 2, 4, "16UC1")

    def test_topics_follow_the_namespace(self):
        named, root = pose_error.topics("j100_0000"), pose_error.topics("")
        self.assertEqual(named["ekf"], "/j100_0000/platform/odom/filtered")
        self.assertEqual(named["wheel_odom"], "/j100_0000/platform/odom")
        self.assertEqual(named["camera1"], "/j100_0000/greenhouse/camera1/image")
        self.assertEqual(root["camera2"], "/greenhouse/camera2/image")
        self.assertEqual(len(pose_error.FIELDS), 16)


if __name__ == "__main__":
    unittest.main()
