# SPDX-License-Identifier: GPL-3.0-only
"""Standard scripting API for the Jackal greenhouse simulation (runs inside the sim container).

Minimum use (sample_use.py):

    from self_driving_api import pose, apply_terrain
    apply_terrain({"bump_band": 2})
    pose({"x": 0, "y": 0, "rz": 90, "lift": 500, "p1": 30})

Every call blocks until the simulation has physically reached the request.

pose() keys. All are optional: an omitted key keeps its last commanded value, so a list of poses
can name only what changes. Angles are degrees, rates are per SIMULATED second.
  x, y            m      goal position, greenhouse frame (origin = house centre, +X along the rows)
  rz              deg    goal heading, counter-clockwise from +X
  lift            mm     FPB30 carriage height above its lower hard stop (0..1500)
  p1, y1, r1      deg    camera 1 (left) gimbal pitch / yaw / roll (pitch + = up, 0 = horizon)
  p2, y2, r2      deg    camera 2 (right) gimbal pitch / yaw / roll
  stab1, stab2    bool   gimbal horizon lock (True) or angles relative to the robot (False)
  oriented        bool   finish at rz (True, default) or keep the heading it arrives with
                         (False). Applies to that pose only: it is never carried over.
  drive_speed     m/s    navigation speed limit                (0.05..2)
  turn_speed      rad/s  navigation turn-rate command          (0.05..4)
  lift_speed      m/s    carriage speed                        (0.005..0.2)
  cam1_rot_speed  deg/s  camera 1 slew rate, all three axes    (0.1..200, 200 = servo native)
  cam2_rot_speed  deg/s  camera 2 slew rate
A pose naming x, y or rz drives (Nav2 + the gateway's dock) to the merged (x, y, rz). Lift and
gimbals move at the same time; pose() returns once all of them have settled.

apply_terrain() keys (omitted keys keep the currently applied value; same as the GUI panel):
  sample_spacing m (0.04..0.25)   base_height_range m (0.005..0.08)   ground_padding m (3..8)
  bump_band mm RMS (0..20)        bump_scale m (0.16..0.5)            ground_friction (0.1..1.2)
  slip_long (0..2)                slip_lat (0..2)

Out-of-range values are clamped with a printed warning; configure(strict=True) raises instead.
Unknown keys always raise: a typo must never be silently ignored.

Perturbation objects (static, resting on the terrain; see docs/TECHNICAL.md "Scenarios"):
  add_object({"id": "bag1", "type": "fertiliser_block", "x": 2, "y": 1, "rz": 30})
  move_object({"id": "bag1", "x": 3})      omitted x, y, rz keep their current values
  remove_object("bag1")
  save_scenario("my_run.json")             seed, configs, robot pose and objects, as a scenario
  types: fertiliser_block, fallen_tomato, fallen_plant, robot, person; x, y in m, rz in deg (default 0).
"""
import json
import math
import os
import subprocess
import sys
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]  # /opt/greenhouse in the image
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
from greenhouse.config import LIMITS  # noqa: E402  (pure Python, no ROS)

__all__ = ["pose", "apply_terrain", "configure", "wait_ready", "get_state", "wait", "capture",
           "stop", "drive_to", "set_lift", "set_gimbal", "add_object", "move_object", "remove_object",
           "save_scenario"]

# API name -> (greenhouse config field, API unit -> config unit factor, unit label)
TERRAIN_FIELDS = {
    "sample_spacing": ("ground_resolution", 1.0, "m"),
    "base_height_range": ("ground_height_range", 1.0, "m"),
    "ground_padding": ("ground_headland", 1.0, "m"),
    "bump_band": ("ground_bump_rms", 0.001, "mm"),
    "bump_scale": ("ground_bump_scale", 1.0, "m"),
    "ground_friction": ("ground_friction", 1.0, ""),
    "slip_long": ("slip_longitudinal", 1.0, ""),
    "slip_lat": ("slip_lateral", 1.0, ""),
}
DRIVE_KEYS = ("x", "y", "rz")
GIMBAL_KEYS = {1: {"p1": "pitch", "y1": "yaw", "r1": "roll"},
               2: {"p2": "pitch", "y2": "yaw", "r2": "roll"}}
# Static ranges; lift and gimbal-angle ranges are read live from the simulated hardware.
RATE_RANGES = {"drive_speed": (0.05, 2.0, "m/s"), "turn_speed": (0.05, 4.0, "rad/s"),
               "lift_speed": (0.005, 0.2, "m/s"), "cam1_rot_speed": (0.1, 200.0, "deg/s"),
               "cam2_rot_speed": (0.1, 200.0, "deg/s")}
POSE_KEYS = set(DRIVE_KEYS) | {"lift", "stab1", "stab2", "oriented"} | set(RATE_RANGES) | {
    key for keys in GIMBAL_KEYS.values() for key in keys}
NATIVE_GIMBAL_RATE = 200.0  # deg/s, XF C-200T servo limit in native/Systems.cc

OPTIONS = {
    "strict": False,              # raise on out-of-range values instead of clamping
    "verbose": True,              # one progress line per call
    "gimbal_tolerance_deg": 1.0,  # camera attitude error accepted as "settled"
    "gimbal_settle_s": 5.0,       # sim seconds to wait for that before warning and moving on
    "ready_timeout_s": 900.0,     # wall seconds wait_ready() waits for the simulation
    "stall_timeout_s": 60.0,      # wall seconds without simulated time advancing -> error
}


def configure(**options: bool | float) -> dict[str, bool | float]:
    """Tune the API, e.g. configure(strict=True, gimbal_tolerance_deg=0.5). Returns all options.

    Units are in the option names: gimbal_tolerance_deg in degrees, gimbal_settle_s in simulated
    seconds, ready_timeout_s and stall_timeout_s in wall-clock seconds. strict and verbose are bool.
    Each value must have the option's type (an int is accepted for a float option)."""
    for key, value in options.items():
        if key not in OPTIONS:
            raise ValueError(f"Unknown option {key!r}; valid: {', '.join(OPTIONS)}")
        if type(value) is not type(OPTIONS[key]) and not (
                isinstance(OPTIONS[key], float) and isinstance(value, int) and not isinstance(value, bool)):
            raise ValueError(f"{key} must be {type(OPTIONS[key]).__name__}")
        OPTIONS[key] = type(OPTIONS[key])(value)
    return dict(OPTIONS)


def _log(message):
    if OPTIONS["verbose"]:
        print(f"[self_driving] {message}", flush=True)


def _number(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return float(value)


def check_range(name: str, value: float, low: float, high: float, unit: str = "",
                strict: bool | None = None) -> float:
    """Return value as float; out of [low, high] it is clamped with a warning, or raises if strict.

    value, low and high share one unit, which `unit` only labels in messages. strict=None follows
    configure(strict=...)."""
    value = _number(name, value)
    if low - 1e-9 <= value <= high + 1e-9:
        return value
    unit = f" {unit}" if unit else ""
    if OPTIONS["strict"] if strict is None else strict:
        raise ValueError(f"{name}={value:g}{unit} is outside {low:g}..{high:g}{unit}")
    clamped = min(max(value, low), high)
    print(f"[self_driving] WARNING: {name}={value:g}{unit} is outside {low:g}..{high:g}{unit}; "
          f"using {clamped:g}{unit}", flush=True)
    return clamped


def terrain_fields(values: dict[str, float], strict: bool | None = None) -> dict[str, float]:
    """Map API terrain keys (GUI names and units) to checked greenhouse config fields.

    Input uses apply_terrain() units (bump_band in mm, other lengths in m); the result is in config
    units, all lengths in metres. strict=None follows configure(strict=...)."""
    if not isinstance(values, dict):
        raise ValueError("apply_terrain() takes a dict")
    unknown = set(values) - set(TERRAIN_FIELDS)
    if unknown:
        raise ValueError(f"Unknown terrain key(s) {sorted(unknown)}; valid: {', '.join(TERRAIN_FIELDS)}")
    fields = {}
    for key, value in values.items():
        field, scale, unit = TERRAIN_FIELDS[key]
        low, high = LIMITS[field]
        fields[field] = check_range(key, value, low / scale, high / scale, unit, strict) * scale
    return fields


def terrain_from_config(config: dict[str, Any]) -> dict[str, float]:
    """Inverse of terrain_fields(): greenhouse config (metres) -> API terrain keys and units
    (bump_band in mm, other lengths in m), rounded to 6 decimals. Other config fields are ignored."""
    return {key: round(config[field] / scale, 6) for key, (field, scale, _) in TERRAIN_FIELDS.items()}


def wrap_degrees(angle: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    angle = math.fmod(angle + 180.0, 360.0)
    return (angle + 360.0 if angle < 0 else angle) - 180.0


def ramp(start: Sequence[float], target: Sequence[float], rate: float | None,
         elapsed: float) -> tuple[list[float], bool]:
    """Linear slew of all axes together: the largest change moves at `rate` units/s.

    For the gimbals: angles in deg, rate in deg/s, elapsed in simulated seconds since the start.
    rate None or >= NATIVE_GIMBAL_RATE jumps straight to target (the servo's own speed).
    Returns (values, finished)."""
    span = max((abs(t - s) for s, t in zip(start, target)), default=0.0)
    if span == 0 or rate is None or rate >= NATIVE_GIMBAL_RATE:
        return list(target), True
    fraction = min(1.0, max(0.0, elapsed) * rate / span)
    return [s + (t - s) * fraction for s, t in zip(start, target)], fraction >= 1.0


def merge_pose(commanded: dict[str, float | bool | None], update: dict[str, float | bool],
               lift_range: tuple[float, float], lift_max_speed: float,
               gimbal_limits: dict[str, Sequence[float]],
               strict: bool | None = None) -> dict[str, float | bool | None]:
    """Validate a pose update against live limits and return the merged commanded state.

    Keys and units are pose()'s (x, y in m, greenhouse world frame; rz in deg, wrapped to
    [-180, 180); lift in mm; lift_speed in m/s). lift_range is (lower, upper) in mm;
    lift_max_speed is in m/s; gimbal_limits maps roll/pitch/yaw to (low, high) deg.
    "oriented" is validated but never stored. Rate keys may be None (not yet commanded)."""
    if not isinstance(update, dict):
        raise ValueError("pose() takes a dict")
    unknown = set(update) - POSE_KEYS
    if unknown:
        raise ValueError(f"Unknown pose key(s) {sorted(unknown)}; valid: {', '.join(sorted(POSE_KEYS))}")
    merged = dict(commanded)
    for key, value in update.items():
        if key in ("x", "y"):
            merged[key] = _number(key, value)
        elif key == "rz":
            merged[key] = wrap_degrees(_number(key, value))
        elif key == "lift":
            merged[key] = check_range(key, value, *lift_range, "mm", strict)
        elif key in ("stab1", "stab2", "oriented"):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be True or False")
            if key != "oriented":  # per-pose, so a later rz is never silently ignored
                merged[key] = value
        elif key in RATE_RANGES:
            low, high, unit = RATE_RANGES[key]
            if key == "lift_speed":
                high = min(high, lift_max_speed)
            merged[key] = check_range(key, value, low, high, unit, strict)
        else:
            axis = next(axis for keys in GIMBAL_KEYS.values() for k, axis in keys.items() if k == key)
            merged[key] = check_range(key, value, *gimbal_limits[axis], "deg", strict)
    return merged


class _Simulation:
    """One ROS node plus the gateway/native-service clients; created on first use."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from greenhouse import navigation_api, transport

        self.rclpy, self.navigation, self.transport = rclpy, navigation_api, transport
        self.state_dir = Path(os.environ.get("GREENHOUSE_STATE", "/etc/clearpath/greenhouse"))
        self.prefix = json.loads((self.state_dir / "actuator.json").read_text())["prefix"]
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = Node(f"self_driving_api_{os.getpid()}")
        self.lift, self.gimbal, self.frames, self.cameras = {}, {}, {}, {}
        self.node.create_subscription(String, self.prefix + "/lift/state",
                                      lambda m: self.lift.update(json.loads(m.data)), 10)
        self.node.create_subscription(String, self.prefix + "/gimbal/state",
                                      lambda m: self.gimbal.update(json.loads(m.data)), 10)
        self.gimbal_command = self.node.create_publisher(String, self.prefix + "/gimbal/command", 10)
        self.String = String
        self.commanded = None
        self.ready = False

    # ---- low-level -------------------------------------------------------------------------
    def spin(self, seconds: float = 0.05) -> None:
        """Process at most one ROS callback, waiting up to `seconds` of wall-clock time."""
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    def sim_time(self) -> float:
        """Latest simulated time in seconds reported by the lift or gimbal state (0.0 before any)."""
        return max(self.lift.get("sim_time", 0.0), self.gimbal.get("sim_time", 0.0))

    def nav(self, action: str, values: dict[str, Any] | None = None) -> dict[str, Any]:
        """POST `action` to the navigation gateway; raises RuntimeError if unavailable or rejected.

        Its status pose is x, y in m and yaw in RADIANS, greenhouse world frame."""
        return self.navigation.request(action, values)

    def camera(self, number: int) -> dict[str, Any]:
        """Latest gimbal state of camera 1 or 2 (angles in deg)."""
        return next(c for c in self.gimbal["cameras"] if c["camera"] == number)

    def gimbal_settings(self, values: dict[str, Any]) -> dict[str, Any]:
        """Send checked gimbal settings (angles in deg); raises ValueError if the gimbal rejects them."""
        reply = self.transport.service(self.prefix + "/gimbal/settings", "gz.msgs.StringMsg",
                                       "gz.msgs.StringMsg", "data: " + json.dumps(json.dumps(values)))
        if reply.get("ok") is not True:
            raise ValueError(reply.get("error", "gimbal settings rejected"))
        return reply

    def publish_gimbal(self, values: dict[str, Any]) -> None:
        """Publish an unchecked intermediate gimbal setpoint (angles in deg) for a slew ramp."""
        self.gimbal_command.publish(self.String(data=json.dumps(values)))

    def lift_settled(self, target_m: float) -> bool:
        """True once the carriage is within 2 mm of target_m (METRES, not mm) and below 4 mm/s."""
        return abs(self.lift["position"] - target_m) < 0.002 and abs(self.lift["velocity"]) < 0.004

    def terrain(self) -> dict[str, float]:
        """Currently applied terrain in apply_terrain() keys and units (bump_band in mm)."""
        state = json.loads((self.state_dir / "terrain-state.json").read_text())
        config = state.get("config")
        if not config:
            config = json.loads((Path(state["assets_dir"]) / "manifest.json").read_text())["config"]
        return terrain_from_config(config)

    # ---- readiness ---------------------------------------------------------------------------
    def wait_ready(self, timeout: float | None = None) -> dict[str, Any]:
        """Block until navigation, lift, gimbals and drive controllers are ready; return nav status.

        timeout is wall-clock seconds (None = configure()'s ready_timeout_s). The first success
        seeds the commanded state from the robot pose and the actuators' current targets, in
        pose() units (rz in deg, lift in mm, lift_speed in m/s)."""
        timeout = OPTIONS["ready_timeout_s"] if timeout is None else timeout
        start = time.monotonic()
        progress = _Progress()
        while True:
            self.spin(0.1)
            reasons = []
            try:
                status = self.nav("status")
                if not status.get("ready"):
                    reasons.append(f"navigation {status.get('message', 'not ready')}")
            except RuntimeError as error:
                status = None
                reasons.append(str(error))
            if not self.lift.get("ready") or self.lift.get("blocked"):
                reasons.append("lift actuator")
            if not self.gimbal.get("ready"):
                reasons.append("gimbals")
            if not (self.state_dir / "controller-ready.json").exists():
                reasons.append("drive controllers")
            if not reasons:
                break
            if time.monotonic() - start > timeout:
                raise TimeoutError("Simulation not ready after %.0f s: %s" % (timeout, "; ".join(reasons)))
            progress.report(reasons)
        if self.commanded is None:
            p = status["pose"]
            self.commanded = {"x": p["x"], "y": p["y"], "rz": wrap_degrees(math.degrees(p["yaw"])),
                              "lift": self.lift["target"] * 1000, "lift_speed": self.lift["speed"],
                              "drive_speed": None, "turn_speed": None,
                              "cam1_rot_speed": None, "cam2_rot_speed": None}
            for number, keys in GIMBAL_KEYS.items():
                camera = self.camera(number)
                self.commanded["stab%d" % number] = camera["stabilise"]
                for key, axis in keys.items():
                    self.commanded[key] = camera["command"][axis]
        self.ready = True
        return status

    def wait_nav_ready(self, timeout: float = 120) -> dict[str, Any]:
        """Wait up to `timeout` wall-clock seconds for the gateway to accept a goal; return its status."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            status = self.nav("status")
            if status.get("ready"):
                return status
            self.spin(0.25)
        raise TimeoutError(f"Navigation not ready for a new goal: {status.get('message')}")

    # ---- pose ----------------------------------------------------------------------------------
    def pose(self, update: dict[str, float | bool]) -> dict[str, Any]:
        """Implementation of the module-level pose(); same keys and units. Returns state()."""
        self.wait_ready()
        merged = merge_pose(self.commanded, update, (self.lift["lower"] * 1000, self.lift["upper"] * 1000),
                            self.lift["max_speed"], self.gimbal["limits"])
        drive = any(key in update for key in DRIVE_KEYS)
        cameras = [n for n, keys in GIMBAL_KEYS.items()
                   if any(k in update for k in keys) or f"stab{n}" in update]
        lift_change = "lift" in update or "lift_speed" in update
        self.commanded = merged
        started_wall, started_sim = time.monotonic(), self.sim_time()

        if lift_change:
            values = {"speed": merged["lift_speed"]}
            if "lift" in update:
                values["target"] = merged["lift"] / 1000
            self.transport.settings(self.prefix, values)
        ramps = {}
        for number in cameras:
            keys = GIMBAL_KEYS[number]
            camera = self.camera(number)
            axes = list(keys.values())
            start = [camera["command"][axis] for axis in axes]
            target = [merged[key] for key in keys]
            self.gimbal_settings({"camera": number, "stabilise": merged[f"stab{number}"]})
            ramps[number] = {"axes": axes, "start": start, "target": target, "t0": self.sim_time(),
                             "rate": merged[f"cam{number}_rot_speed"], "done": False, "last": -1.0}
        if drive:
            self.wait_nav_ready()
            goal = {"x": merged["x"], "y": merged["y"], "yaw_deg": merged["rz"],
                    "oriented": update.get("oriented", True),
                    "max_speed": merged["drive_speed"], "max_turn": merged["turn_speed"]}
            self.nav("go", goal)
        _log("pose " + ", ".join(f"{k}={v:g}" if isinstance(v, float) else f"{k}={v}"
                                 for k, v in update.items()))

        nav_done, nav_polled = not drive, 0.0
        settle_since = None
        last_sim, last_advance = self.sim_time(), time.monotonic()
        while True:
            self.spin(0.02)
            now_sim, now_wall = self.sim_time(), time.monotonic()
            if now_sim > last_sim:
                last_sim, last_advance = now_sim, now_wall
            elif now_wall - last_advance > OPTIONS["stall_timeout_s"]:
                raise RuntimeError("Simulated time stopped advancing (paused or stalled simulation)")
            for number, r in ramps.items():
                if r["done"] or now_sim - r["last"] < 0.05:
                    continue
                values, finished = ramp(r["start"], r["target"], r["rate"], now_sim - r["t0"])
                command = dict(zip(r["axes"], values), camera=number)
                if finished:
                    self.gimbal_settings(command)  # checked final setpoint
                    r["done"] = True
                else:
                    self.publish_gimbal(command)
                r["last"] = now_sim
            if not nav_done and now_wall - nav_polled > 0.25:
                nav_polled = now_wall
                status = self.nav("status")
                if status["phase"] == "Succeeded":
                    nav_done = True
                    if not goal["oriented"]:  # remember the heading it actually arrived with
                        merged["rz"] = wrap_degrees(math.degrees(status["pose"]["yaw"]))
                elif status["phase"] in ("Error", "Canceled"):
                    raise RuntimeError(f"Navigation to ({merged['x']:g}, {merged['y']:g}, "
                                       f"{merged['rz']:g} deg) failed: {status['message']}")
            if not nav_done or not all(r["done"] for r in ramps.values()):
                continue
            lift_ok = self.lift_settled(merged["lift"] / 1000) if "lift" in update else True
            gimbals_ok = all(self.camera(n)["error_deg"] <= OPTIONS["gimbal_tolerance_deg"] for n in cameras)
            if lift_ok and gimbals_ok:
                break
            settle_since = now_sim if settle_since is None else settle_since
            if lift_ok and now_sim - settle_since > OPTIONS["gimbal_settle_s"]:
                errors = ", ".join(f"camera {n}: {self.camera(n)['error_deg']:.1f} deg" for n in cameras)
                print(f"[self_driving] WARNING: gimbal did not settle within "
                      f"{OPTIONS['gimbal_tolerance_deg']:g} deg ({errors}); continuing", flush=True)
                break
            if not lift_ok and now_sim - settle_since > 15 + 1.5 / merged["lift_speed"]:
                raise TimeoutError(f"Lift did not reach {merged['lift']:g} mm "
                                   f"(at {self.lift['position'] * 1000:.1f} mm)")
        _log(f"  reached in {now_sim - started_sim:.1f} s sim ({time.monotonic() - started_wall:.1f} s wall)")
        return self.state()

    # ---- other operations --------------------------------------------------------------------
    def apply_terrain(self, values: dict[str, float]) -> dict[str, float]:
        """Implementation of the module-level apply_terrain(); same keys and units."""
        fields = terrain_fields(values)
        self.wait_ready()
        _log("apply_terrain " + json.dumps(values))
        command = ["timeout", "--signal=INT", "--kill-after=5s", "180s", "python3",
                   str(ROOT / "tools" / "apply_terrain.py"), "--values", json.dumps(fields)]
        result = subprocess.run(command, text=True, capture_output=True)
        for line in result.stdout.splitlines():
            _log("  " + line)
        if result.returncode != 0:
            message = (result.stderr.strip() or result.stdout.strip())[-600:]
            for key, (field, _, _) in TERRAIN_FIELDS.items():  # speak the caller's names
                message = message.replace(field, key)
            raise RuntimeError("Terrain not applied (previous terrain kept): " + message)
        self.wait_ready()
        return self.terrain()

    def state(self) -> dict[str, Any]:
        """Implementation of get_state(); see there for keys and units."""
        self.spin(0.02)
        status = self.nav("status")
        p = status["pose"]
        measured = {"x": p["x"], "y": p["y"], "rz": wrap_degrees(math.degrees(p["yaw"])),
                    "lift": self.lift["position"] * 1000}
        for number, keys in GIMBAL_KEYS.items():
            camera = self.camera(number)
            for key, axis in keys.items():
                measured[key] = camera["measured"][axis]
        return {"sim_time": self.sim_time(), "pose": measured, "commanded": dict(self.commanded or {}),
                "navigation": {"phase": status["phase"], "message": status["message"]},
                "terrain": self.terrain()}

    def wait(self, seconds: float) -> None:
        """Block for `seconds` of simulated time; raises if simulated time stalls."""
        seconds = _number("seconds", seconds)
        end = self.sim_time() + seconds
        last, advanced = self.sim_time(), time.monotonic()
        while self.sim_time() < end:
            self.spin(0.05)
            if self.sim_time() > last:
                last, advanced = self.sim_time(), time.monotonic()
            elif time.monotonic() - advanced > OPTIONS["stall_timeout_s"]:
                raise RuntimeError("Simulated time stopped advancing (paused or stalled simulation)")

    def capture(self, number: int, path: str | os.PathLike[str] | None = None,
                timeout: float = 20) -> "numpy.ndarray":
        """Implementation of the module-level capture(); timeout is wall-clock seconds."""
        if number not in (1, 2):
            raise ValueError("camera must be 1 or 2")
        import numpy as np
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        if number not in self.cameras:
            self.cameras[number] = self.node.create_subscription(
                Image, f"{self.prefix}/camera{number}/image",
                lambda m, n=number: self.frames.__setitem__(n, m), qos_profile_sensor_data)
        requested = self.sim_time()
        end = time.monotonic() + timeout
        while True:
            self.spin(0.05)
            frame = self.frames.get(number)
            if frame is not None and frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9 >= requested:
                break
            if time.monotonic() > end:
                raise TimeoutError(f"No fresh image from camera {number}")
        if frame.encoding != "rgb8":
            raise ValueError(f"Unexpected camera encoding {frame.encoding}")
        rows = np.frombuffer(bytes(frame.data), dtype=np.uint8).reshape(frame.height, frame.step)
        image = rows[:, :frame.width * 3].reshape(frame.height, frame.width, 3).copy()
        if path is not None:
            import cv2
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
                raise OSError(f"Could not write {path}")
            _log(f"camera {number} -> {path}")
        return image

    def stop(self) -> None:
        """Cancel navigation and hold the lift at its measured position (commanded lift in mm)."""
        self.nav("cancel")
        self.spin(0.05)
        hold = min(max(self.lift["position"], self.lift["lower"]), self.lift["upper"])
        self.transport.settings(self.prefix, {"target": hold})
        if self.commanded is not None:
            self.commanded["lift"] = hold * 1000


class _Progress:
    """Log what is still awaited when it changes, and at most once a minute otherwise."""

    def __init__(self):
        self.last, self.when = None, time.monotonic()

    def report(self, reasons: list[str]) -> None:
        """Log the reasons the simulation is not ready yet, if they changed or a minute has passed."""
        text = "; ".join(reasons)
        if text != self.last or time.monotonic() - self.when > 60:
            self.last, self.when = text, time.monotonic()
            _log("waiting for the simulation: " + text)


def _wait_for_navigation(timeout=None):
    """Poll the gateway over HTTP only: no ROS participant joins while Nav2 is still bringing up its
    lifecycle nodes, a phase whose service calls already time out under load."""
    from greenhouse.navigation_api import request
    timeout = OPTIONS["ready_timeout_s"] if timeout is None else timeout
    start, progress = time.monotonic(), _Progress()
    while True:
        try:
            status = request("status", timeout=5)
            if status.get("ready"):
                return
            reason = f"navigation {status.get('message', 'not ready')}"
        except RuntimeError as error:
            reason = str(error)
        if time.monotonic() - start > timeout:
            raise TimeoutError(f"Simulation not ready after {timeout:.0f} s: {reason}")
        progress.report([reason])
        time.sleep(0.5)


_simulation = None


def _sim():
    global _simulation
    if _simulation is None:
        _wait_for_navigation()
        _simulation = _Simulation()
    return _simulation


# ---- public API --------------------------------------------------------------------------------
def pose(values: dict[str, float | bool]) -> dict[str, Any]:
    """Move to a (partial) pose and block until drive, lift and gimbals have settled.

    Keys and units are listed in the module docstring: x, y in m (greenhouse world frame),
    rz in deg, lift in mm, lift_speed and drive_speed in m/s, turn_speed in rad/s, gimbal
    angles in deg, camN_rot_speed in deg/s; stab1, stab2 and oriented are bool.
    Returns get_state() at arrival."""
    return _sim().pose(values)


def apply_terrain(values: dict[str, float]) -> dict[str, float]:
    """Apply terrain/contact settings (GUI names and units). Returns the applied terrain values.

    bump_band is RMS height in mm; sample_spacing, base_height_range, ground_padding and
    bump_scale are in m; ground_friction, slip_long and slip_lat are dimensionless."""
    return _sim().apply_terrain(values)


def wait_ready(timeout: float | None = None) -> None:
    """Block until navigation, lift, gimbals and drive controllers accept commands.

    timeout is wall-clock seconds; None uses configure()'s ready_timeout_s."""
    if _simulation is None:
        _wait_for_navigation(timeout)
    _sim().wait_ready(timeout)


def get_state() -> dict[str, Any]:
    """{'sim_time', 'pose' (measured, pose() keys and units), 'commanded', 'navigation', 'terrain'}.

    sim_time: simulated seconds. pose: measured x, y (m, greenhouse world frame), rz (deg),
    lift (mm), p1/y1/r1 and p2/y2/r2 (deg). commanded: last commanded pose() values in the same
    units (drive_speed, turn_speed and camN_rot_speed are None until first set). navigation:
    {'phase', 'message'}. terrain: apply_terrain() keys and units (bump_band in mm)."""
    return _sim().state()


def wait(seconds: float) -> None:
    """Sleep for `seconds` of simulated time."""
    _sim().wait(seconds)


def capture(camera: int, path: str | os.PathLike[str] | None = None,
            timeout: float = 20) -> "numpy.ndarray":
    """Return a fresh RGB frame (H x W x 3 uint8 numpy array) from camera 1 or 2; save PNG if path.

    Fresh means stamped at or after the call's simulated time. timeout is wall-clock seconds.
    A relative path resolves against the working directory (the script's folder under run.sh)."""
    return _sim().capture(camera, path, timeout)


def stop() -> None:
    """Cancel navigation and hold the lift where it is."""
    _sim().stop()


def drive_to(x: float, y: float, rz: float, speed: float | None = None,
             turn_speed: float | None = None) -> dict[str, Any]:
    """pose() shortcut for the base only.

    x, y in m (greenhouse world frame), rz in deg; speed is drive_speed in m/s and turn_speed is
    in rad/s (None keeps the last value). Returns get_state() at arrival."""
    values = {"x": x, "y": y, "rz": rz}
    if speed is not None:
        values["drive_speed"] = speed
    if turn_speed is not None:
        values["turn_speed"] = turn_speed
    return pose(values)


def set_lift(mm: float, speed: float | None = None) -> dict[str, Any]:
    """pose() shortcut for the lift only; speed in m/s.

    mm is carriage height above the lower hard stop; speed is lift_speed (None keeps the last
    value). Returns get_state() at arrival."""
    return pose({"lift": mm, **({} if speed is None else {"lift_speed": speed})})


def set_gimbal(camera: int, pitch: float | None = None, yaw: float | None = None,
               roll: float | None = None, speed: float | None = None,
               stabilise: bool | None = None) -> dict[str, Any]:
    """pose() shortcut for one gimbal; angles in deg, speed in deg/s.

    camera is 1 (left) or 2 (right); pitch + = up, 0 = horizon. speed is camN_rot_speed and
    stabilise is stabN (horizon lock). None leaves that value as last commanded.
    Returns get_state() at arrival."""
    if camera not in (1, 2):
        raise ValueError("camera must be 1 or 2")
    values = {f"{key}{camera}": value for key, value in (("p", pitch), ("y", yaw), ("r", roll))
              if value is not None}
    if speed is not None:
        values[f"cam{camera}_rot_speed"] = speed
    if stabilise is not None:
        values[f"stab{camera}"] = stabilise
    return pose(values)


def _scenario(*arguments: str) -> dict[str, Any]:
    """Run tools/scenario.py (the same code as the CLI and the scenario runner); return its JSON reply."""
    result = subprocess.run(["python3", str(ROOT / "tools" / "scenario.py"), *arguments],
                            text=True, capture_output=True, timeout=120)
    if result.returncode != 0:
        raise RuntimeError((result.stderr.strip() or result.stdout.strip())[-600:])
    return json.loads(result.stdout)


def _coordinates(values: dict[str, Any]) -> list[str]:
    return [part for key in ("x", "y", "rz") if key in values for part in (f"--{key}", repr(values[key]))]


def add_object(values: dict[str, Any]) -> dict[str, Any]:
    """Spawn a static perturbation object resting on the terrain.

    values: id (letters, digits, _ or -), type (fertiliser_block, fallen_tomato, fallen_plant, robot,
    person), x, y in m (greenhouse world frame), optional rz in deg (default 0). Returns its record
    {id, type, x, y, rz, model, z} with z the resting height in m. Raises ValueError for bad values,
    RuntimeError if the simulation refuses (duplicate id, outside the walls)."""
    from greenhouse.scenario import check_object
    item = check_object(values)
    _log(f"add_object {json.dumps(values)}")
    return _scenario("add", item["type"], "--id", item["id"], *_coordinates(item))


def move_object(values: dict[str, Any]) -> dict[str, Any]:
    """Move an object: values id plus any of x, y (m) and rz (deg); omitted ones are kept.

    The object is re-rested on the terrain at its new place. Returns its record as add_object()."""
    from greenhouse.scenario import check_move
    item = check_move(values)
    _log(f"move_object {json.dumps(values)}")
    return _scenario("move", item["id"], *_coordinates(item))


def remove_object(object_id: str) -> dict[str, Any]:
    """Delete object `object_id`; returns the record it had."""
    _log(f"remove_object {object_id}")
    return _scenario("remove", str(object_id))


def save_scenario(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Write the current run as a scenario JSON (seed, greenhouse/actuator configs in use, robot
    ground-truth pose as start, current objects; no timed events). A relative path resolves against
    the working directory. Load it with `run.sh --scenario FILE`. Returns {saved, start, objects}."""
    _log(f"save_scenario {path}")
    return _scenario("save", str(Path(path).resolve()))
