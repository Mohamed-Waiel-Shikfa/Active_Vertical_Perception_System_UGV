import unittest
from types import SimpleNamespace

from greenhouse.health import snapshot


class HealthTests(unittest.TestCase):
    def setUp(self):
        self.observer=SimpleNamespace(ready=lambda:True,last_clock_wall=9.9,last_pose_wall=9.9,
            last_odom_wall=9.9,last_imu_wall=9.9,last_controllers_wall=9.9,last_poll_wall=9.9,
            controllers={"joint_state_broadcaster":"active","platform_velocity_controller":"active"})
        self.nav={"ready":True,"fresh":True,"feedback_age_wall_s":.1}
        self.native={"source":"Gazebo PostUpdate physical base_link","publication_sequence":5,"sim_time":2.0}

    def test_current_real_observer_is_healthy(self):
        self.assertTrue(snapshot(self.observer,self.nav,self.native,now=10)["ok"])

    def test_dead_observer_stale_source_unready_and_pause_fail_closed(self):
        self.observer.last_poll_wall=0
        self.assertFalse(snapshot(self.observer,self.nav,self.native,now=10)["ok"])
        self.observer.last_poll_wall=9.9
        self.assertFalse(snapshot(self.observer,dict(self.nav,fresh=False),self.native,now=10)["ok"])
        self.assertFalse(snapshot(self.observer,dict(self.nav,ready=False),self.native,now=10)["ok"])
        self.observer.ready=lambda:False
        self.assertFalse(snapshot(self.observer,self.nav,self.native,now=10)["ok"])
        self.observer.ready=lambda:True
        self.assertFalse(snapshot(self.observer,self.nav,self.native,True,now=10)["ok"])
        self.assertFalse(snapshot(self.observer,self.nav,dict(self.native,source="wheel odom"),now=10)["ok"])
