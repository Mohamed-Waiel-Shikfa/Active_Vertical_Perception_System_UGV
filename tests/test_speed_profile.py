import math
import unittest

from greenhouse.speed_profile import speed_profile


def straight(length, step=.05):
    return [(i * step, 0.0) for i in range(int(round(length / step)) + 1)]


class SpeedProfileTests(unittest.TestCase):
    def test_straight_runs_fast_and_brakes_only_for_the_goal(self):
        caps = speed_profile(straight(10), .8, .7, 1.0)
        self.assertEqual(caps[-1], 0.0)
        self.assertGreater(caps[0], 4.0)
        self.assertAlmostEqual(caps[-21], math.sqrt(2 * 1.0 * 1.0), places=6)

    def test_hairpin_slows_inside_and_brakes_before_it(self):
        radius, step = .55, .05
        points = straight(6, step)
        arc = int(math.pi * radius / step)
        points += [(6 + radius * math.sin(k * step / radius), radius - radius * math.cos(k * step / radius))
                   for k in range(1, arc + 1)]
        points += [(6 - i * step, 2 * radius) for i in range(1, 121)]
        caps = speed_profile(points, .8, .7, 1.0)
        inside = min(caps[121:121 + arc])
        self.assertLess(inside, .7 * radius + .05)
        self.assertGreater(inside, .2)
        # Braking at 1 m/s^2 from 2 m/s needs about 2 m; the cap is already low 1 m before the curve.
        self.assertLess(caps[100], 1.6)
        self.assertGreater(caps[0], 2.0)

    def test_open_end_profile_only_limits_curves(self):
        caps = speed_profile(straight(10), .8, .7, 1.0, stop=False)
        self.assertTrue(all(math.isinf(c) for c in caps))

    def test_degenerate_paths(self):
        self.assertEqual(speed_profile([], .8, .7, 1.0), [])
        self.assertEqual(speed_profile([(0, 0)], .8, .7, 1.0), [0.0])


if __name__ == "__main__":
    unittest.main()
