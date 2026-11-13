import tempfile
import unittest
from collections import Counter
from dataclasses import replace
from pathlib import Path
from xml.etree import ElementTree as ET

from greenhouse.actuator import generate_wrapper, load_actuator
from greenhouse.mount import bracket_mesh

ROOT = Path(__file__).resolve().parents[1]


class ActuatorTests(unittest.TestCase):
    def test_verified_stroke_body_mass_and_envelope(self):
        config = load_actuator(ROOT / "config" / "actuator.json")
        self.assertAlmostEqual(config.body_length, 1.677)
        self.assertAlmostEqual(config.module_mass, 3.9)
        self.assertEqual(config.speed_m_s, 0.2)  # rated maximum for the gimbal payload
        self.assertEqual(config.ramp_time_s, 0.5)
        self.assertEqual(config.deck_hole_pitch_mm, 120)
        self.assertAlmostEqual(config.external_mass, 2 * (0.15 + 0.125) + 0.06)
        self.assertEqual(config.rated_speed, 0.2)
        heavy = replace(config, camera_mass_kg=0.2, gimbal_mass_kg=0.3, camera_bracket_mass_kg=0.5,
                        speed_m_s=0.1).validate()
        self.assertEqual(heavy.rated_speed, 0.1)  # > 1 kg external payload halves the B42 vertical speed
        with self.assertRaises(ValueError):
            replace(heavy, speed_m_s=0.2).validate()
        with self.assertRaises(ValueError):
            replace(config, force_limit_n=10).validate()

    def test_wrapper_preserves_user_source_and_has_real_camera_and_hard_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            setup = Path(folder)
            source = setup / "robot.urdf.xacro"
            source.write_text("user-owned generated description")
            config = load_actuator(ROOT / "tests" / "fixtures" / "actuator-short.json")
            report = generate_wrapper(config, setup, "custom_robot")
            self.assertEqual(source.read_text(), "user-owned generated description")
            root = ET.parse(report["wrapper"])
            limits = root.find(".//joint[@name='fpb30_slide_joint']/limit")
            self.assertEqual(float(limits.get("lower")), 0)
            self.assertEqual(float(limits.get("upper")), 0.3)
            self.assertEqual(float(limits.get("effort")), 30)
            topics = [element.text for element in root.findall(".//sensor[@type='camera']/topic")]
            self.assertEqual(topics, ["/custom_robot/greenhouse/camera1/image",
                                      "/custom_robot/greenhouse/camera2/image"])
            self.assertEqual(report["camera_topics"], topics)
            self.assertEqual(root.find(".//joint[@name='gimbal_crossbar_joint']/parent").get("link"), "fpb30_carriage")
            self.assertEqual(root.find(".//joint[@name='fpb30_deck_mount']/parent").get("link"), "base_link")
            self.assertIsNotNone(root.find(".//plugin[@name='greenhouse::Actuator']"))
            self.assertFalse(root.findall(".//use_velocity_commands"))

    def test_printable_mount_is_closed_manifold_and_has_two_rail_levels(self):
        vertices, triangles = bracket_mesh()
        edges = Counter(tuple(sorted(edge)) for a,b,c in triangles for edge in ((a,b),(b,c),(c,a)))
        self.assertTrue(all(count == 2 for count in edges.values()))
        self.assertEqual(min(v[2] for v in vertices), 0)
        self.assertEqual(max(v[2] for v in vertices), 198)
        self.assertGreater(len(triangles), 1000)
        self.assertLess(len(triangles), 200000)

    def test_gimbal_payload_ranges_and_legacy_keys(self):
        config = load_actuator(ROOT / "config" / "actuator.json")
        self.assertEqual(config.gimbal_mount_yaw_deg, (90.0, -90.0))
        with self.assertRaises(ValueError):
            replace(config, camera_mass_kg=0.21).validate()  # C-200T payload is 200 g
        with self.assertRaises(ValueError):
            replace(config, gimbal_initial_rpy_deg=((0, -91, 0), (0, 0, 0))).validate()
        with self.assertRaises(ValueError):
            replace(config, gimbal_initial_rpy_deg=((91, 0, 0), (0, 0, 0))).validate()
        with self.assertRaises(ValueError):
            replace(config, gimbal_initial_rpy_deg=((0, 0, 0), (0, 0, 91))).validate()
        replace(config, gimbal_initial_rpy_deg=((90, -90, 90), (-90, 90, -90))).validate()
        with self.assertRaises(ValueError):
            replace(config, gimbal_offset_m=0.06).validate()  # the two gimbals could touch
        with self.assertRaises(ValueError):
            replace(config, gimbal_stabilise=(True, 1)).validate()
        with tempfile.TemporaryDirectory() as folder:
            legacy = Path(folder) / "legacy.json"
            legacy.write_text('{"camera_xyz_m": [0.05, 0, 0.02]}')
            with self.assertRaisesRegex(ValueError, "old single fixed camera"):
                load_actuator(legacy)

    def test_two_three_axis_gimbals_left_and_right(self):
        import math
        with tempfile.TemporaryDirectory() as folder:
            setup = Path(folder)
            (setup / "robot.urdf.xacro").write_text("stub")
            report = generate_wrapper(load_actuator(ROOT / "config" / "actuator.json"), setup, "j100_0000")
            root = ET.parse(report["wrapper"])
            for index, yaw in ((1, 90.0), (2, -90.0)):
                mount = root.find(f".//joint[@name='gimbal{index}_mount_joint']")
                self.assertEqual(mount.find("parent").get("link"), "gimbal_crossbar")
                rpy = [float(v) for v in mount.find("origin").get("rpy").split()]
                self.assertAlmostEqual(math.degrees(rpy[2]), yaw)
                y = float(mount.find("origin").get("xyz").split()[1])
                self.assertEqual(math.copysign(1, y), math.copysign(1, yaw))
                chain = []
                link = f"gimbal{index}_base"
                for axis, name in (("0 0 1", "yaw"), ("1 0 0", "roll"), ("0 -1 0", "pitch")):
                    joint = root.find(f".//joint[@name='gimbal{index}_{name}_joint']")
                    self.assertEqual(joint.get("type"), "revolute")
                    self.assertEqual(joint.find("axis").get("xyz"), axis)
                    self.assertEqual(joint.find("parent").get("link"), link)
                    link = joint.find("child").get("link")
                    limit = joint.find("limit")
                    chain.append((round(math.degrees(float(limit.get("lower")))),
                                  round(math.degrees(float(limit.get("upper"))))))
                self.assertEqual(link, f"camera{index}_link")
                self.assertEqual(chain, [(-90, 90)] * 3)
                sensor = root.find(f".//gazebo[@reference='camera{index}_link']/sensor")
                self.assertEqual(sensor.get("type"), "camera")
                self.assertEqual(sensor.findtext("gz_frame_id"), f"camera{index}_optical")
                mass = float(root.find(f".//link[@name='camera{index}_link']/inertial/mass").get("value"))
                self.assertLessEqual(mass, 0.2)
            gimbal = root.find(".//plugin[@name='greenhouse::Gimbal']")
            self.assertEqual(gimbal.findtext("prefix"), "/j100_0000/greenhouse")
            self.assertEqual([g.findtext("id") for g in gimbal.findall("gimbal")], ["1", "2"])
            self.assertEqual([gimbal.findtext(f"{axis}_limits") for axis in ("roll", "pitch", "yaw")],
                             ["-90.0 90.0"] * 3)
            published = [e.text for e in root.findall(".//plugin[@name='gz::sim::systems::JointStatePublisher']/joint_name")]
            self.assertEqual(len(published), 7)
            self.assertIn("gimbal2_pitch_joint", published)
            gimbal_mass = sum(float(root.find(f".//link[@name='gimbal{i}_{part}']/inertial/mass").get("value"))
                              for i in (1, 2) for part in ("base", "yaw_link", "roll_link"))
            self.assertAlmostEqual(gimbal_mass, 2 * 0.125)

    @staticmethod
    def expanded(setup):
        """Default wrapper with a stand-in Jackal body box as base_link."""
        (setup / "robot.urdf.xacro").write_text("stub")
        report = generate_wrapper(load_actuator(ROOT / "config" / "actuator.json"), setup, "")
        robot = ET.parse(report["wrapper"]).getroot()
        for include in robot.findall("{http://www.ros.org/wiki/xacro}include"):
            robot.remove(include)
        base = ET.SubElement(robot, "link", name="base_link")
        ET.SubElement(ET.SubElement(ET.SubElement(base, "collision"), "geometry"), "box", size=".42 .31 .184")
        path = setup / "expanded.urdf"
        ET.ElementTree(robot).write(path)
        return path

    def test_hanging_gimbals_clear_each_other_the_carriage_and_the_deck(self):
        import numpy as np
        from greenhouse.navigation_map import swept_link_points
        with tempfile.TemporaryDirectory() as folder:
            links = swept_link_points(self.expanded(Path(folder)))
        swept = {i: np.concatenate([links[f"gimbal{i}_{part}"] for part in ("base", "yaw_link", "roll_link")]
                                   + [links[f"camera{i}_link"]]) for i in (1, 2)}
        # Over every sampled +/-90 deg angle and the whole stroke: left and right units never meet,
        self.assertGreater(swept[1][:, 1].min(), 0.002)
        self.assertLess(swept[2][:, 1].max(), -0.002)
        for points in swept.values():
            # stay in front of the carriage face and rail,
            self.assertGreater(points[:, 0].min(), links["fpb30_carriage"][:, 0].max() + 0.002)
            # hang below the plate, and clear the real J100 chassis top (z 0.184 m) at the lowest lift position.
            self.assertLess(points[:, 2].max(), links["gimbal_crossbar"][:, 2].max() - 0.006 + 1e-9)
            self.assertGreater(points[:, 2].min(), 0.184 + 0.02)

    def test_lidar_section_merges_defaults_and_validates_ranges(self):
        config = load_actuator(ROOT / "config" / "actuator.json")
        lidar = config.lidar
        # SICK TiM551-2050001 data sheet: 270 deg at 1 deg, 15 Hz, 0.05-10 m, 20 mm, 250 g, 60x60x86 mm.
        self.assertEqual((lidar.horizontal_samples, lidar.horizontal_fov_deg, lidar.update_rate_hz), (271, 270.0, 15.0))
        self.assertEqual((lidar.range_min_m, lidar.range_max_m, lidar.noise_stddev_m), (0.05, 10.0, 0.02))
        self.assertEqual((lidar.vertical_samples, lidar.vertical_fov_deg), (1, 0.0))
        partial = load_actuator(ROOT / "tests" / "fixtures" / "actuator-lidar-off.json")
        self.assertFalse(partial.lidar.enabled)
        self.assertEqual(partial.lidar.update_rate_hz, 10.0)
        self.assertEqual(replace(partial.lidar, enabled=True, update_rate_hz=15.0), lidar)  # rest from defaults
        self.assertEqual(partial.stroke_m, config.stroke_m)
        replace(lidar, vertical_samples=16, vertical_fov_deg=30.0).validate()  # a 3D lidar is a config change
        for bad in ({"update_rate_hz": 0.5}, {"horizontal_samples": 0}, {"horizontal_samples": 271.0},
                    {"vertical_samples": 16}, {"vertical_fov_deg": 10.0}, {"range_min_m": 0.0},
                    {"range_max_m": 0.04}, {"range_min_m": 0.9, "range_max_m": 0.8}, {"noise_stddev_m": -0.01},
                    {"mass_kg": 0.0}, {"size_m": (0.06, 0.06)}, {"size_m": (0.06, 0.06, 0.5)},
                    {"xyz_m": (0.24, 0.0, 0.184)}, {"xyz_m": (0.12, float("nan"), 0.184)},
                    {"rpy_deg": (0.0, 0.0, 181.0)}, {"focal_height_m": 0.09}, {"enabled": 1}, {"model": ""}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                replace(config, lidar=replace(lidar, **bad)).validate()
        with tempfile.TemporaryDirectory() as folder:
            for text in ('{"lidar": {"spin_hz": 5}}', '{"lidar": true}'):
                path = Path(folder) / "bad.json"
                path.write_text(text)
                with self.subTest(text=text), self.assertRaisesRegex(ValueError, "lidar"):
                    load_actuator(path)

    def test_lidar_sensor_frames_and_payload_in_description(self):
        import math
        with tempfile.TemporaryDirectory() as folder:
            setup = Path(folder)
            (setup / "robot.urdf.xacro").write_text("stub")
            report = generate_wrapper(load_actuator(ROOT / "config" / "actuator.json"), setup, "j100_0000")
            root = ET.parse(report["wrapper"])
            self.assertEqual(report["lidar"], {"name": "lidar2d_0", "model": "SICK TiM551-2050001",
                                               "topic": "/j100_0000/sensors/lidar2d_0/scan", "link": "lidar2d_0_link",
                                               "frame": "lidar2d_0_laser", "update_rate_hz": 15.0})
            mount = root.find(".//joint[@name='lidar2d_0_joint']")
            self.assertEqual(mount.find("parent").get("link"), "base_link")
            self.assertEqual([float(v) for v in mount.find("origin").get("xyz").split()], [0.12, 0.0, 0.184])
            laser = root.find(".//joint[@name='lidar2d_0_laser_joint']")
            self.assertEqual(laser.find("parent").get("link"), "lidar2d_0_link")
            self.assertEqual(laser.find("child").get("link"), "lidar2d_0_laser")
            self.assertAlmostEqual(float(laser.find("origin").get("xyz").split()[2]), 0.0625)
            housing = root.find(".//link[@name='lidar2d_0_link']")
            self.assertEqual(float(housing.find("inertial/mass").get("value")), 0.25)
            for tag in ("collision", "visual"):
                self.assertEqual([float(v) for v in housing.find(f"{tag}/geometry/box").get("size").split()],
                                 [0.06, 0.06, 0.086])
            sensor = root.find(".//gazebo[@reference='lidar2d_0_laser']/sensor")
            self.assertEqual((sensor.get("name"), sensor.get("type")), ("lidar2d_0", "gpu_lidar"))
            self.assertEqual(sensor.findtext("topic"), "/j100_0000/sensors/lidar2d_0/scan")
            self.assertEqual(sensor.findtext("gz_frame_id"), "lidar2d_0_laser")
            self.assertEqual(float(sensor.findtext("update_rate")), 15.0)
            horizontal = sensor.find("lidar/scan/horizontal")
            self.assertEqual(horizontal.findtext("samples"), "271")
            self.assertAlmostEqual(float(horizontal.findtext("min_angle")), -math.radians(135))
            self.assertAlmostEqual(float(horizontal.findtext("max_angle")), math.radians(135))
            vertical = sensor.find("lidar/scan/vertical")
            self.assertEqual((vertical.findtext("samples"), float(vertical.findtext("min_angle")),
                              float(vertical.findtext("max_angle"))), ("1", 0.0, 0.0))
            self.assertEqual((float(sensor.findtext("lidar/range/min")), float(sensor.findtext("lidar/range/max"))),
                             (0.05, 10.0))
            self.assertEqual(sensor.findtext("lidar/noise/type"), "gaussian")
            self.assertEqual(float(sensor.findtext("lidar/noise/stddev")), 0.02)
            off = generate_wrapper(load_actuator(ROOT / "tests" / "fixtures" / "actuator-lidar-off.json"), setup, "")
            self.assertIsNone(off["lidar"])
            self.assertIsNone(ET.parse(off["wrapper"]).find(".//link[@name='lidar2d_0_link']"))
            self.assertFalse(ET.parse(off["wrapper"]).findall(".//sensor[@type='gpu_lidar']"))

    def test_lidar_housing_clears_the_gimbal_sweep_and_the_footprint(self):
        from greenhouse.navigation_map import swept_link_points
        with tempfile.TemporaryDirectory() as folder:
            links = swept_link_points(self.expanded(Path(folder)))
        housing = links["lidar2d_0_link"]
        gimbals = [links[name] for name in links if name.startswith(("gimbal", "camera"))]
        # In front of every gimbal pose at every lift height, and inside the bare J100 footprint.
        self.assertGreater(housing[:, 0].min(), max(points[:, 0].max() for points in gimbals) + 0.01)
        self.assertLess(housing[:, 0].max(), 0.254)
        self.assertAlmostEqual(housing[:, 2].min(), 0.184)

    def test_gimbal_sweep_is_inside_the_navigation_footprint(self):
        from greenhouse.navigation_map import robot_envelope
        with tempfile.TemporaryDirectory() as folder:
            envelope = robot_envelope(self.expanded(Path(folder)))
            self.assertLess(max(abs(v) for v in envelope["bounds_m"][0][:2] + envelope["bounds_m"][1][:2]), 0.215)
            self.assertAlmostEqual(envelope["footprint"][2][1], 0.24)
            self.assertLess(envelope["bounds_m"][1][2], 1.9)


if __name__ == "__main__":
    unittest.main()
