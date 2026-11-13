"""Health criteria reuse the existing live Observer, never wheel odometry as navigation truth."""
import time
from typing import Any


def snapshot(observer: Any, navigation: dict[str, Any], native: dict[str, Any], update_in_progress: bool = False,
             now: float | None = None) -> dict[str, Any]:
    """Health report from a live tools.sim_checks.Observer plus navigation and native-publisher status.

    `now` and the observer's last_*_wall stamps are time.monotonic() wall seconds (receipt_ages_s);
    native "sim_time" is simulation seconds. "ok" needs a controller poll under 3 s old.
    """
    now=time.monotonic() if now is None else now
    ages={name:now-getattr(observer,"last_"+name+"_wall")
          for name in ("clock","pose","odom","imu","controllers","poll")}
    source_ok=(native.get("source")=="Gazebo PostUpdate physical base_link"
               and native.get("publication_sequence",0)>0 and navigation.get("fresh") is True)
    healthy=(observer.ready() and ages["poll"]<3 and source_ok
             and navigation.get("ready") is True and not update_in_progress)
    if update_in_progress:
        reason="Terrain/contact update in progress; physics may be intentionally paused"
    elif not healthy:
        reason="Live observer, native publisher, controller query or navigation readiness is stale/unavailable"
    else:
        reason="Named pose, advancing clock, IMU, odometry, active controllers and native navigation are live"
    return {"ok":bool(healthy),"message":reason,"error":None if healthy else reason,
            "observer_alive":ages["poll"]<3,"receipt_ages_s":ages,
            "controllers":dict(observer.controllers),"navigation_ready":navigation.get("ready",False),
            "update_in_progress":update_in_progress,
            "native":{"source":native.get("source"),"sequence":native.get("publication_sequence"),
                      "iteration":native.get("iteration"),"sim_time":native.get("sim_time"),
                      "receipt_age_s":navigation.get("feedback_age_wall_s"),"fresh":navigation.get("fresh",False)}}
