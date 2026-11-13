"""Pure-logic contract of self_driving_api; live behaviour is checked against a running sim."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "self_driving"), str(ROOT)]
import self_driving_api as api  # noqa: E402

LIMITS = {"roll": (-45.0, 45.0), "pitch": (-90.0, 30.0), "yaw": (-160.0, 160.0)}
START = {"x": -7.5, "y": 0.0, "rz": 0.0, "lift": 0.0, "lift_speed": 0.1, "drive_speed": None,
         "turn_speed": None, "cam1_rot_speed": None, "cam2_rot_speed": None, "stab1": True,
         "stab2": True, "p1": 0.0, "y1": 0.0, "r1": 0.0, "p2": 0.0, "y2": 0.0, "r2": 0.0}


class SelfDrivingApiTests(unittest.TestCase):
    def setUp(self):
        self.options = dict(api.OPTIONS)
        api.configure(verbose=False)
        self.addCleanup(api.OPTIONS.update, self.options)

    def merge(self, update, strict=None):
        return api.merge_pose(START, update, (0.0, 1500.0), 0.2, LIMITS, strict)

    def test_partial_pose_keeps_previous_values(self):
        merged = self.merge({"x": 5, "rz": -190, "lift": 500, "p1": 20})
        self.assertEqual((merged["x"], merged["y"], merged["lift"], merged["p1"]), (5.0, 0.0, 500.0, 20.0))
        self.assertAlmostEqual(merged["rz"], 170.0)
        self.assertEqual(START["x"], -7.5)  # input not mutated

    def test_oriented_is_checked_but_never_carried_over(self):
        merged = self.merge({"x": 1, "oriented": False})
        self.assertNotIn("oriented", merged)
        with self.assertRaises(ValueError):
            self.merge({"oriented": 0})

    def test_unknown_and_non_numeric_keys_always_raise(self):
        for update in ({"z": 1}, {"pitch1": 3}, {"x": "1"}, {"x": float("nan")}, {"lift": True},
                       {"stab1": 1}):
            with self.assertRaises(ValueError, msg=update):
                self.merge(update)

    def test_out_of_range_clamps_unless_strict(self):
        merged = self.merge({"lift": 1600, "p1": 50, "lift_speed": 0.5, "drive_speed": 3})
        self.assertEqual((merged["lift"], merged["p1"], merged["lift_speed"], merged["drive_speed"]),
                         (1500.0, 30.0, 0.2, 2.0))
        with self.assertRaises(ValueError):
            self.merge({"lift": 1600}, strict=True)
        api.configure(strict=True)
        with self.assertRaises(ValueError):
            self.merge({"p1": 50})

    def test_terrain_names_and_units_match_the_gui_panel(self):
        fields = api.terrain_fields({"sample_spacing": 0.2, "bump_band": 2, "bump_scale": 0.2,
                                     "slip_long": 0.08, "slip_lat": 1.1, "ground_friction": 0.9,
                                     "ground_padding": 3.5, "base_height_range": 0.04})
        self.assertEqual(fields, {"ground_resolution": 0.2, "ground_bump_rms": 0.002, "ground_bump_scale": 0.2,
                                  "slip_longitudinal": 0.08, "slip_lateral": 1.1, "ground_friction": 0.9,
                                  "ground_headland": 3.5, "ground_height_range": 0.04})
        self.assertEqual(api.terrain_fields({"base_height_range": 0.4, "sample_spacing": 0.3}),
                         {"ground_height_range": 0.08, "ground_resolution": 0.25})
        # 2.5 m padding is a generator failure for this house; the shared limit clamps it instead.
        self.assertEqual(api.terrain_fields({"ground_padding": 2.5}), {"ground_headland": 3.0})
        with self.assertRaises(ValueError):
            api.terrain_fields({"sample_spacing": 0.3}, strict=True)
        with self.assertRaises(ValueError):
            api.terrain_fields({"roughness": 1})
        self.assertEqual(api.terrain_from_config(dict(
            ground_resolution=0.15, ground_height_range=0.03, ground_headland=3.0, ground_bump_rms=0.003,
            ground_bump_scale=0.18, ground_friction=0.9, slip_longitudinal=0.1, slip_lateral=1.0))["bump_band"], 3.0)

    def test_gimbal_ramp_moves_all_axes_together_at_the_requested_rate(self):
        values, done = api.ramp([0, 0, 0], [10, -20, 0], 5.0, 2.0)
        self.assertEqual((values, done), ([5.0, -10.0, 0.0], False))
        self.assertEqual(api.ramp([0, 0, 0], [10, -20, 0], 5.0, 4.0), ([10, -20, 0], True))
        self.assertEqual(api.ramp([0], [10], None, 0.0), ([10], True))      # servo native rate
        self.assertEqual(api.ramp([0], [10], 200.0, 0.0), ([10], True))
        self.assertEqual(api.ramp([3], [3], 1.0, 0.0), ([3], True))

    def test_configure_rejects_unknown_options_and_wrong_types(self):
        with self.assertRaises(ValueError):
            api.configure(strickt=True)
        with self.assertRaises(ValueError):
            api.configure(strict="yes")
        self.assertEqual(api.configure(gimbal_tolerance_deg=2)["gimbal_tolerance_deg"], 2.0)


if __name__ == "__main__":
    unittest.main()
