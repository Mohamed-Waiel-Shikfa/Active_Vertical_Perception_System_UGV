import json
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from greenhouse.generate import generate
from greenhouse.config import default_config
from greenhouse.navigation_map import RADIUS, aisle_inflation_radius, create_map, robot_envelope


class NavigationMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        cls.root=Path(cls.temp.name)
        generate(default_config(),cls.root/"assets")
        cls.audit=create_map(cls.root/"assets",cls.root/"map",2.05)
        cls.image=cv2.imread(str(cls.root/"map/map.pgm"),cv2.IMREAD_GRAYSCALE)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def occupied(self,x,y):
        i=int(round((x-self.audit["map_origin"])/.05))
        j=self.image.shape[0]-1-int(round((y-self.audit["map_origin"])/.05))
        return self.image[j,i]==0

    def test_real_plants_walls_and_door_not_roof(self):
        self.assertTrue(self.occupied(-6,1))
        self.assertTrue(self.occupied(0,5))
        self.assertFalse(self.occupied(-7.5,0))
        self.assertFalse(self.occupied(-9,0))
        self.assertFalse(self.occupied(0,0))
        self.assertTrue(self.occupied(0,9.5))
        self.assertGreater(self.audit["colliders"],100)
        self.assertGreater(RADIUS,.36)
        self.assertLess(RADIUS,.38)

    def test_parameters_really_use_closed_loop_and_distinct_frames(self):
        import yaml
        root=Path(__file__).resolve().parents[1]
        config=yaml.safe_load((root/"config/navigation.yaml").read_text())
        follow=config["controller_server"]["ros__parameters"]["FollowPath"]
        local=config["local_costmap/local_costmap"]["ros__parameters"]
        self.assertIn("RegulatedPurePursuitController",follow["plugin"])
        self.assertTrue(follow["use_collision_detection"])
        self.assertFalse(follow["allow_reversing"])
        # Panel maxima: Drive 2.0 m/s; the fastest actual skid-steer turn is ~0.28 * 4.0 rad/s.
        self.assertEqual(follow["desired_linear_vel"],2.0)
        self.assertLessEqual(follow["rotate_to_heading_angular_vel"],1.2)
        # The local costmap must hold the collision projection and the farthest lookahead.
        self.assertGreater(local["width"]/2,follow["max_lookahead_dist"])
        curvature=2*math.sin(follow["rotate_to_heading_min_angle"])/follow["min_lookahead_dist"]
        half_step=local["resolution"]*(1+RADIUS*curvature)/2
        self.assertGreaterEqual(local["footprint_padding"],half_step)
        checker=config["controller_server"]["ros__parameters"]["goal_checker"]
        # A latched checker leaks "XY reached" into the next goal; the gateway finishes goals.
        self.assertFalse(checker["stateful"])
        self.assertLessEqual(checker["xy_goal_tolerance"],.075)
        self.assertEqual(config["controller_server"]["ros__parameters"]["odom_topic"],"/greenhouse/nav/odom")
        tree=(root/"config/navigate.xml").read_text()
        self.assertIn('RateController hz="1.0"',tree)
        # Static costmaps: replan only for an invalid path or a new goal, never periodically.
        self.assertIn("<IsPathValid",tree)
        self.assertIn("<GlobalUpdatedGoal/>",tree)
        self.assertNotIn("BackUp",tree)
        self.assertNotIn("Spin ",tree)

    def test_aisle_centrelines_stay_cost_free(self):
        # A costed inner-aisle centreline sent Smac 7.6 m round a wider, cost-free outer aisle.
        import yaml
        root=Path(__file__).resolve().parents[1]
        config=yaml.safe_load((root/"config/navigation.yaml").read_text())
        maximum=config["global_costmap/global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"]
        aisles=json.loads((self.root/"assets/manifest.json").read_text())["layout"]["clear_aisles_m"]
        radius=aisle_inflation_radius(aisles,maximum)
        free=(self.image!=0).astype(np.uint8)
        cells=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        j=lambda y:self.image.shape[0]-1-int(round((y-self.audit["map_origin"])/.05))
        i=int(round((0-self.audit["map_origin"])/.05))
        for y in (-2,0,2):  # inner aisle centrelines of the default four-row layout
            self.assertGreater(cells[j(y),i],math.ceil(radius/.05-1e-9))
        self.assertAlmostEqual(aisle_inflation_radius([1.2,1.3],1.0),.55)
        self.assertAlmostEqual(aisle_inflation_radius([2.0],.6),.6)

    def test_robot_swept_collision_envelope_is_measured(self):
        path=self.root/"robot.urdf"
        path.write_text("""<robot name="test">
        <link name="base_link"><collision><geometry><box size=".42 .31 .184"/></geometry></collision></link>
        <link name="camera"><collision><geometry><box size=".04 .04 .04"/></geometry></collision></link>
        <joint name="rail" type="prismatic"><parent link="base_link"/><child link="camera"/>
        <origin xyz="-.4 0 .3"/><axis xyz="0 0 1"/><limit lower="0" upper="1.5"/></joint></robot>""")
        measured=robot_envelope(path)
        self.assertAlmostEqual(measured["bounds_m"][0][0],-.42)
        self.assertAlmostEqual(measured["bounds_m"][1][2],1.82)
        self.assertAlmostEqual(measured["footprint"][1][0],.445)
        self.assertGreater(measured["radius_m"],RADIUS)
