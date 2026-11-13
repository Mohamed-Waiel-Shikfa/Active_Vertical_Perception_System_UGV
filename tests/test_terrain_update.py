import unittest
from dataclasses import asdict
from greenhouse.config import default_config
from greenhouse.terrain_update import proposed_config, support_points


class TerrainUpdateTests(unittest.TestCase):
    def test_all_fields_and_invalid_input(self):
        original = asdict(default_config())
        values = {"ground_resolution": 0.12, "ground_height_range": 0.02, "ground_headland": 3.5,
                  "ground_bump_rms": 0.006, "ground_bump_scale": 0.22, "ground_friction": 0.7,
                  "slip_longitudinal": 0.1, "slip_lateral": 1.0}
        result = proposed_config(original, values)
        self.assertTrue(all(result[k] == v for k, v in values.items()))
        self.assertEqual(original, asdict(default_config()))
        for values in ({"ground_resolution": 0}, {"ground_height_range": float("nan")},
                       {"row_length": 20}, {"slip_longitudinal": -1}, {"ground_headland": True}):
            with self.assertRaises(ValueError):
                proposed_config(original, values)

    def test_support_envelope_rotates(self):
        import math
        points = support_points({"x": 1.0, "y": 2.0, "yaw": math.pi/2})
        self.assertAlmostEqual(min(p[0] for p in points), 0.73)
        self.assertAlmostEqual(max(p[1] for p in points), 2.30)
