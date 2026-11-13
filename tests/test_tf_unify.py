import importlib.util
import math
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("tf_unify", ROOT / "tools" / "tf_unify.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
SECOND = 1_000_000_000


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def matrix(q):
    return [module.rotate(q, axis) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]


class PoseMathTests(unittest.TestCase):
    def assertPose(self, actual, expected, places=9):
        for a, b in zip(actual[0], expected[0]):
            self.assertAlmostEqual(a, b, places=places)
        sign = 1 if sum(a*b for a, b in zip(actual[1], expected[1])) >= 0 else -1  # q and -q agree
        for a, b in zip(actual[1], expected[1]):
            self.assertAlmostEqual(a, sign*b, places=places)

    def test_rpy_matches_rz_ry_rx(self):
        roll, pitch, heading = 0.1, -0.2, 2.5
        c, s = math.cos, math.sin
        rx = [[1, 0, 0], [0, c(roll), -s(roll)], [0, s(roll), c(roll)]]
        ry = [[c(pitch), 0, s(pitch)], [0, 1, 0], [-s(pitch), 0, c(pitch)]]
        rz = [[c(heading), -s(heading), 0], [s(heading), c(heading), 0], [0, 0, 1]]
        product = lambda a, b: [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        expected = product(rz, product(ry, rx))
        columns = matrix(module.from_rpy(0, 0, 0, roll, pitch, heading)[1])
        for j in range(3):
            for i in range(3):
                self.assertAlmostEqual(columns[j][i], expected[i][j], places=12)

    def test_compose_translates_in_parent_frame_and_inverse_cancels(self):
        quarter = module.from_rpy(1, 2, 0, 0, 0, math.pi/2)
        moved = module.compose(quarter, ((1, 0, 0), (0, 0, 0, 1)))
        self.assertPose(moved, ((1, 3, 0), quarter[1]))
        pose = module.from_rpy(-7.5, 0.3, 0.12, 0.03, -0.02, 3.1)
        self.assertPose(module.compose(pose, module.inverse(pose)), module.IDENTITY)
        self.assertPose(module.compose(module.inverse(pose), pose), module.IDENTITY)

    def test_correction_chain_reproduces_truth_in_six_dof(self):
        truth = module.from_rpy(4.2, -1.1, 0.13, 0.04, -0.03, -2.9)
        ekf = module.from_rpy(4.0, -0.9, 0.0, 0.0, 0.0, -2.7)  # two_d_mode EKF: z, roll, pitch zero
        correction = module.compose(truth, module.inverse(ekf))
        self.assertPose(module.compose(correction, ekf), truth)

    def test_interpolation_takes_the_short_way_across_pi(self):
        a = module.from_rpy(0, 0, 0, 0, 0, math.pi - 0.1)
        b = module.from_rpy(2, 4, 0, 0, 0, -math.pi + 0.1)
        middle = module.interpolate(a, b, 0.5)
        self.assertPose(middle, module.from_rpy(1, 2, 0, 0, 0, math.pi))
        self.assertAlmostEqual(abs(yaw(module.interpolate(a, b, 0.25)[1])), math.pi - 0.05, places=9)
        self.assertPose(module.interpolate(a, b, 0.0), a)
        self.assertPose(module.interpolate(a, a, 0.7), a)

    def test_truth_at_interpolates_inside_and_refuses_outside(self):
        samples = [(10*SECOND, module.from_rpy(0, 0, 0, 0, 0, 0)), (11*SECOND, module.from_rpy(1, 0, 0, 0, 0, 0.2))]
        self.assertPose(module.truth_at(samples, 10*SECOND + SECOND//4), module.from_rpy(0.25, 0, 0, 0, 0, 0.05))
        self.assertPose(module.truth_at(samples, 10*SECOND), samples[0][1])
        self.assertPose(module.truth_at(samples, 11*SECOND), samples[1][1])
        self.assertIsNone(module.truth_at(samples, 10*SECOND - 1))
        self.assertIsNone(module.truth_at(samples, 11*SECOND + 1))
        self.assertIsNone(module.truth_at([], 0))
        self.assertPose(module.truth_at(samples[:1], 10*SECOND), samples[0][1])

    def test_physics_state_gives_base_link_pose_in_nanoseconds(self):
        state = {"source": module.PHYSICS_SOURCE, "sim_time": 12.34, "x": -7.5, "y": 0, "z": 0.1,
                 "roll": 0.01, "pitch": -0.02, "yaw": 1.0}
        stamp, pose = module.physics_truth(state)
        self.assertEqual(stamp, 12_340_000_000)
        self.assertPose(pose, module.from_rpy(-7.5, 0, 0.1, 0.01, -0.02, 1.0))
        self.assertIsNone(module.physics_truth({**state, "source": "other"}))
        self.assertIsNone(module.physics_truth({**state, "z": float("nan")}))
        self.assertIsNone(module.physics_truth({key: value for key, value in state.items() if key != "roll"}))


class CorrectorTests(unittest.TestCase):
    def truth(self, t):
        return module.from_rpy(t, 0.5*t, 0.1, 0.02, -0.01, 0.3*t)

    def test_estimate_waits_for_truth_and_is_stamped_with_its_own_time(self):
        corrector = module.Corrector()
        self.assertEqual(corrector.add_truth(0, self.truth(0)), [])
        ekf = module.from_rpy(0.9, 0.4, 0, 0, 0, 0.25)
        self.assertEqual(corrector.add_estimate(SECOND, ekf, "odom"), [])  # truth not there yet
        [(stamp, tag, correction)] = corrector.add_truth(2*SECOND, self.truth(2))
        self.assertEqual((stamp, tag), (SECOND, "odom"))
        expected = module.truth_at([(0, self.truth(0)), (2*SECOND, self.truth(2))], SECOND)
        chained = module.compose(correction, ekf)
        for a, b in zip(chained[0] + chained[1], expected[0] + expected[1]):
            self.assertAlmostEqual(a, b, places=9)

    def test_steady_truth_lag_does_not_starve_corrections(self):
        corrector = module.Corrector()
        period = SECOND//50
        published = []
        for k in range(1, 20):
            published += corrector.add_estimate(k*period, self.truth(k/50), "odom")
            if k >= 2:  # each truth sample arrives after the next EKF sample
                published += corrector.add_truth((k - 1)*period, self.truth((k - 1)/50))
        self.assertEqual([stamp for stamp, _, _ in published], [k*period for k in range(1, 19)])

    def test_repeated_old_and_out_of_buffer_estimates_are_skipped(self):
        corrector = module.Corrector()
        for t in range(0, 4):
            corrector.add_truth(t*SECOND, self.truth(t))
        self.assertEqual(len(corrector.add_estimate(3*SECOND, self.truth(3), "odom")), 1)
        self.assertEqual(corrector.add_estimate(3*SECOND, self.truth(3), "odom"), [])
        self.assertEqual(corrector.add_estimate(2*SECOND, self.truth(2), "odom"), [])
        corrector = module.Corrector()
        for t in range(0, 6):
            corrector.add_truth(t*SECOND, self.truth(t))
        self.assertEqual(corrector.add_estimate(SECOND//2, self.truth(0.5), "odom"), [])  # older than the 2 s span

    def test_clock_restart_clears_state(self):
        corrector = module.Corrector()
        corrector.add_truth(5*SECOND, self.truth(5))
        self.assertEqual(len(corrector.add_estimate(5*SECOND, self.truth(5), "odom")), 1)
        self.assertEqual(corrector.add_truth(5*SECOND, self.truth(5)), [])
        corrector.add_truth(SECOND, self.truth(1))
        self.assertEqual(len(corrector.truth), 1)
        self.assertEqual(len(corrector.add_estimate(SECOND, self.truth(1), "odom")), 1)


if __name__ == "__main__":
    unittest.main()
