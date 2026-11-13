# SPDX-License-Identifier: GPL-3.0-only
"""FPB30/B42 dimensions from FUYU 2026 p27; two XF C-200T 3-axis gimbal cameras hang under the carriage.

Rigid component inertias are engineering estimates. C-200T figures (125 g with damper, 200 g
payload, 200 deg/s, camera-down mounting) are vendor listings; the +/-90 deg operating range on every
axis is the project's choice, inside the vendor's mechanical ranges.
"""
import argparse
import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Self
from xml.etree import ElementTree as ET

from greenhouse.world import child, write_xml

ROOT = Path(__file__).resolve().parents[1]
DEFAULTS_PATH = ROOT / "config" / "actuator.json"
GIMBAL_PAYLOAD_KG = 0.2
# Commandable range of every gimbal axis, the single source for the URDF joints, the native controller
# and the GUI: pitch positive tilts the camera up, roll positive lowers its right side, yaw positive
# turns it counterclockwise (seen from above) from its outward neutral.
GIMBAL_LIMITS_DEG = {"roll": (-90.0, 90.0), "pitch": (-90.0, 90.0), "yaw": (-90.0, 90.0)}
# Closest gimbal spacing at which the two units cannot touch anywhere in that range: the roll arm sweeps
# about 0.06 m from its yaw axis (checked in tests/test_actuator.py).
GIMBAL_MIN_OFFSET_M = 0.065
LEGACY_KEYS = {"camera_xyz_m", "camera_rpy_rad"}
# Clearpath sensor naming: link lidar2d_0_link (housing base), frame lidar2d_0_laser (scan origin),
# topic <ns>/sensors/lidar2d_0/scan.
LIDAR_NAME = "lidar2d_0"


@dataclass(frozen=True)
class LidarConfig:
    """Planar or multi-layer lidar on the Jackal body; ``model`` is a label only.

    The default is the SICK TiM551-2050001 (data sheet 1060445, 2026-08-04): 270 deg aperture,
    15 Hz, 1 deg resolution (271 beams), 0.05-10 m working range (8 m at 10% remission), statistical
    error < 20 mm, 250 g, 60 x 60 x 86 mm, light emission level 62.46 mm above the housing base.
    ``xyz_m``/``rpy_deg`` place the housing base centre in base_link; ``focal_height_m`` lifts the
    scan origin above it. A 3D lidar only needs vertical_samples > 1 and vertical_fov_deg > 0.
    """
    enabled: bool
    model: str
    xyz_m: tuple
    rpy_deg: tuple
    focal_height_m: float
    update_rate_hz: float
    horizontal_samples: int
    horizontal_fov_deg: float
    vertical_samples: int
    vertical_fov_deg: float
    range_min_m: float
    range_max_m: float
    noise_stddev_m: float
    mass_kg: float
    size_m: tuple

    def validate(self) -> Self:
        """Check every field against its range (unit in the key suffix) and return self; raises ValueError."""
        if not isinstance(self.enabled, bool):
            raise ValueError("lidar.enabled must be a boolean")
        if not isinstance(self.model, str) or not self.model.strip():
            raise ValueError("lidar.model must be a non-empty string")
        ranges = {
            "update_rate_hz": (1.0, 50.0), "horizontal_fov_deg": (1.0, 360.0),
            "vertical_fov_deg": (0.0, 180.0), "range_min_m": (0.01, 1.0), "range_max_m": (0.5, 200.0),
            "noise_stddev_m": (0.0, 0.5), "mass_kg": (0.05, 2.0), "focal_height_m": (0.0, 0.3),
        }
        for key, limits in ranges.items():
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"lidar.{key} must be finite")
            if not limits[0] <= value <= limits[1]:
                raise ValueError(f"lidar.{key} must be in {limits}")
        for key, high in (("horizontal_samples", 4096), ("vertical_samples", 128)):
            if type(getattr(self, key)) is not int or not 1 <= getattr(self, key) <= high:
                raise ValueError(f"lidar.{key} must be an integer in [1,{high}]")
        if (self.vertical_samples == 1) != (self.vertical_fov_deg == 0):
            raise ValueError("lidar.vertical_fov_deg must be 0 for one vertical sample (2D) and > 0 otherwise")
        if self.range_max_m <= self.range_min_m:
            raise ValueError("lidar.range_max_m must exceed range_min_m")
        for key, count in (("xyz_m", 3), ("rpy_deg", 3), ("size_m", 3)):
            value = getattr(self, key)
            if (not isinstance(value, (tuple, list)) or len(value) != count
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
                raise ValueError(f"lidar.{key} must contain {count} finite numbers")
        if any(abs(v) > 180 for v in self.rpy_deg):
            raise ValueError("lidar.rpy_deg must be within [-180,180]")
        if any(not 0.02 <= v <= 0.3 for v in self.size_m):
            raise ValueError("lidar.size_m must be in [0.02,0.3] m per side")
        if self.focal_height_m > self.size_m[2]:
            raise ValueError("lidar.focal_height_m must lie inside the housing height")
        # Housing inside the bare J100 footprint (0.254 x 0.215 m half-sizes, the navigation minimum).
        if (abs(self.xyz_m[0]) + self.size_m[0] / 2 > 0.254 or abs(self.xyz_m[1]) + self.size_m[1] / 2 > 0.215
                or not 0.0 <= self.xyz_m[2] <= 1.0):
            raise ValueError("lidar housing must stay on the J100 body: |x| <= 0.254, |y| <= 0.215, z in [0,1] m")
        return self


@dataclass(frozen=True)
class ActuatorConfig:
    stroke_m: float
    speed_m_s: float
    ramp_time_s: float
    force_limit_n: float
    camera_mass_kg: float
    camera_bracket_mass_kg: float
    camera_fov_rad: float
    camera_width: int
    camera_height: int
    camera_rate_hz: float
    gimbal_mass_kg: float
    gimbal_offset_m: float
    gimbal_mount_yaw_deg: tuple
    gimbal_initial_rpy_deg: tuple
    gimbal_stabilise: tuple
    gimbal_max_rate_deg_s: float
    gimbal_torque_nm: float
    deck_xyz_m: tuple
    deck_hole_pitch_mm: float
    print_clearance_mm: float
    mount_mass_kg: float
    lidar: LidarConfig

    @property
    def body_length(self) -> float:
        """FPB30 rail body length in metres: the stroke plus a fixed 0.177 m."""
        return self.stroke_m + 0.177

    @property
    def module_mass(self) -> float:
        """FPB30 module mass in kg: 0.9 kg plus 2 kg per metre of stroke."""
        return 0.9 + 2 * self.stroke_m

    @property
    def external_mass(self) -> float:
        """Payload on the carriage in kg: both cameras, both gimbals and the crossbar bracket."""
        return 2 * (self.camera_mass_kg + self.gimbal_mass_kg) + self.camera_bracket_mass_kg

    @property
    def moving_mass(self) -> float:
        """Total lifted mass in kg: the 0.32 kg carriage plus `external_mass`."""
        return 0.32 + self.external_mass

    @property
    def rated_speed(self) -> float:
        """B42 vertical speed ceiling in m/s: 0.2 for up to 1 kg external payload, else 0.1."""
        return 0.2 if self.external_mass <= 1.0 else 0.1

    def validate(self) -> Self:
        """Check every field against its hardware/project range and return self.

        Each range is in the unit of the field's suffix (_m, _m_s, _s, _n, _kg, _rad, _hz, _deg, _mm);
        deck_xyz_m is in the Jackal base_link frame. Raises ValueError naming the offending field.
        """
        ranges = {
            "stroke_m": (0.1, 1.8), "speed_m_s": (0.005, 0.2),
            "ramp_time_s": (0.5, 3.0), "force_limit_n": (10.0, 60.0),
            "camera_mass_kg": (0.02, GIMBAL_PAYLOAD_KG), "camera_bracket_mass_kg": (0.01, 0.5),
            "camera_fov_rad": (0.4, 1.8), "camera_rate_hz": (2.0, 20.0),
            "deck_hole_pitch_mm": (80, 120), "print_clearance_mm": (0.2, 0.8),
            "mount_mass_kg": (0.1, 1.0), "gimbal_mass_kg": (0.08, 0.3),
            "gimbal_offset_m": (GIMBAL_MIN_OFFSET_M, 0.16), "gimbal_max_rate_deg_s": (10.0, 200.0),
            "gimbal_torque_nm": (0.05, 1.0),
        }
        for key, limits in ranges.items():
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            if not limits[0] <= value <= limits[1]:
                raise ValueError(f"{key} must be in {limits}")
        def numbers(key, value, count):
            if not isinstance(value, (tuple, list)) or len(value) != count:
                raise ValueError(f"{key} must contain {count} finite numbers")
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
                raise ValueError(f"{key} must contain {count} finite numbers")
        numbers("deck_xyz_m", self.deck_xyz_m, 3)
        numbers("gimbal_mount_yaw_deg", self.gimbal_mount_yaw_deg, 2)
        if any(abs(v) > 180 for v in self.gimbal_mount_yaw_deg):
            raise ValueError("gimbal_mount_yaw_deg must be within [-180,180]")
        if not isinstance(self.gimbal_initial_rpy_deg, (tuple, list)) or len(self.gimbal_initial_rpy_deg) != 2:
            raise ValueError("gimbal_initial_rpy_deg must hold [roll, pitch, yaw] for both cameras")
        for index, angles in enumerate(self.gimbal_initial_rpy_deg):
            numbers(f"gimbal_initial_rpy_deg[{index}]", angles, 3)
            for axis, value in zip(("roll", "pitch", "yaw"), angles):
                low, high = GIMBAL_LIMITS_DEG[axis]
                if not low <= value <= high:
                    raise ValueError(f"camera {index + 1} initial {axis} must be in [{low},{high}] deg")
        if (not isinstance(self.gimbal_stabilise, (tuple, list)) or len(self.gimbal_stabilise) != 2
                or any(not isinstance(v, bool) for v in self.gimbal_stabilise)):
            raise ValueError("gimbal_stabilise must hold two booleans")
        for key in ("camera_width", "camera_height"):
            if type(getattr(self, key)) is not int or not 120 <= getattr(self, key) <= 1280:
                raise ValueError(f"{key} must be an integer in [120,1280]")
        if self.external_mass > 2:
            raise ValueError("FPB30/B42 supports at most 2 kg external vertical payload")
        if self.speed_m_s > self.rated_speed:
            raise ValueError("speed exceeds FUYU B42 vertical load/speed envelope")
        if self.force_limit_n < self.moving_mass * (9.81 + self.speed_m_s / self.ramp_time_s) * 1.15:
            raise ValueError("force cap is insufficient for gravity and the selected ramp")
        if self.deck_hole_pitch_mm not in (80, 120):
            raise ValueError("deck_hole_pitch_mm must be 120 (bare lid) or 80 (PACS plate)")
        if not (-0.08 <= self.deck_xyz_m[0] <= 0.0 and abs(self.deck_xyz_m[1]) <= 0.03):
            raise ValueError("rear-DECK adapter must remain inboard, leaving rear HMI/stop accessible")
        if not 0.12 <= self.deck_xyz_m[2] <= 0.3:
            raise ValueError("deck height must be in [0.12,0.3] m")
        if self.camera_mass_kg > GIMBAL_PAYLOAD_KG:
            raise ValueError("each camera must stay within the C-200T 200 g payload")
        if not isinstance(self.lidar, LidarConfig):
            raise ValueError("lidar must be a lidar section")
        self.lidar.validate()
        return self


def _read_actuator_settings(path: Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"actuator config {path} must be a JSON object")
    legacy = LEGACY_KEYS & set(data)
    if legacy:
        raise ValueError(f"{sorted(legacy)} described the old single fixed camera; "
                         "use gimbal_mount_yaw_deg / gimbal_initial_rpy_deg for the two gimbal cameras")
    unknown = set(data) - {f.name for f in fields(ActuatorConfig)}
    if unknown:
        raise ValueError(f"unknown actuator keys in {path}: {sorted(unknown)}")
    if "lidar" in data:
        if not isinstance(data["lidar"], dict):
            raise ValueError(f"lidar in {path} must be a JSON object")
        unknown = set(data["lidar"]) - {f.name for f in fields(LidarConfig)}
        if unknown:
            raise ValueError(f"unknown lidar keys in {path}: {sorted(unknown)}")
    return data


def load_actuator(path: Path | str) -> ActuatorConfig:
    """Load an FPB30 + gimbal JSON and validate it.

    Keys missing from ``path`` take their value from ``config/actuator.json``, the only place
    default values are written down; the ``lidar`` section merges key by key the same way.
    Units are in each key's suffix (_m, _kg, _deg, _mm, _hz).
    """
    defaults = _read_actuator_settings(DEFAULTS_PATH)
    missing = {f.name for f in fields(ActuatorConfig)} - set(defaults)
    missing |= {f"lidar.{f.name}" for f in fields(LidarConfig)} - {f"lidar.{key}" for key in defaults.get("lidar", {})}
    if missing:
        raise ValueError(f"{DEFAULTS_PATH} must define every key; missing: {sorted(missing)}")
    user = _read_actuator_settings(path)
    data = {**defaults, **user, "lidar": {**defaults["lidar"], **user.get("lidar", {})}}
    lidar = {key: tuple(value) if isinstance(value, list) else value for key, value in data["lidar"].items()}
    data["lidar"] = LidarConfig(**lidar)
    for key in ("gimbal_mount_yaw_deg", "gimbal_stabilise"):
        if isinstance(data.get(key), list):
            data[key] = tuple(data[key])
    if isinstance(data.get("gimbal_initial_rpy_deg"), list):
        data["gimbal_initial_rpy_deg"] = tuple(tuple(v) if isinstance(v, list) else v
                                               for v in data["gimbal_initial_rpy_deg"])
    return ActuatorConfig(**data).validate()


def vector(values: Iterable[float]) -> str:
    """Format numbers as a space-separated URDF attribute with 9 decimals; units pass through unchanged."""
    return " ".join(f"{v:.9f}" for v in values)


def solid_link(robot: ET.Element, name: str, size: Sequence[float], mass: float,
               xyz: Sequence[float] = (0, 0, 0),
               color: Sequence[float] = (0.55, 0.58, 0.60, 1)) -> ET.Element:
    """Append a uniform-density box link to `robot` and return it.

    `size` is the full box extent (x, y, z) in metres, `mass` in kg, `xyz` the box centre in metres
    in the link's own frame, `color` RGBA in [0, 1]. Collision and visual share the same box.
    """
    link = child(robot, "link", name=name)
    inertial = child(link, "inertial")
    child(inertial, "origin", xyz=vector(xyz), rpy="0 0 0")
    child(inertial, "mass", value=str(mass))
    x, y, z = size
    child(inertial, "inertia", ixx=str(mass * (y*y + z*z) / 12),
          iyy=str(mass * (x*x + z*z) / 12), izz=str(mass * (x*x + y*y) / 12),
          ixy="0", ixz="0", iyz="0")
    for tag in ("collision", "visual"):
        part = child(link, tag)
        child(part, "origin", xyz=vector(xyz), rpy="0 0 0")
        child(child(part, "geometry"), "box", size=vector(size))
        if tag == "visual":
            child(child(part, "material", name=f"{name}_color"), "color", rgba=vector(color))
    return link


def fixed_joint(robot: ET.Element, name: str, parent: str, child_name: str, xyz: Sequence[float],
                rpy: Sequence[float] = (0, 0, 0)) -> None:
    """Append a fixed joint placing `child_name` at `xyz` (metres) and `rpy` (radians) in the parent link frame."""
    joint = child(robot, "joint", name=name, type="fixed")
    child(joint, "parent", link=parent)
    child(joint, "child", link=child_name)
    child(joint, "origin", xyz=vector(xyz), rpy=vector(rpy))


GIMBAL_PLATE_DEPTH = 0.10
# Plate top flush with the carriage top, so the hanging gimbals clear the Jackal deck (z 0.184 m) by
# about 3 cm at the lowest lift position, whatever their angles.
GIMBAL_PLATE_Z = 0.039
# Yaw axes 65 mm in front of the carriage face: the swept roll arm then clears the carriage and rail.
GIMBAL_AXIS_X = 0.065


def revolute_joint(robot: ET.Element, name: str, parent: str, child_name: str, xyz: Sequence[float],
                   axis: str, limits: Sequence[float], effort: float, velocity: float) -> None:
    """Append a revolute joint at `xyz` (metres, parent link frame) about `axis` (URDF "x y z" string).

    `limits` is (lower, upper) in degrees and is written to the URDF in radians; `effort` is in N m
    and `velocity` in rad/s.
    """
    joint = child(robot, "joint", name=name, type="revolute")
    child(joint, "parent", link=parent)
    child(joint, "child", link=child_name)
    child(joint, "origin", xyz=vector(xyz), rpy="0 0 0")
    child(joint, "axis", xyz=axis)
    child(joint, "limit", lower=f"{math.radians(limits[0]):.9f}", upper=f"{math.radians(limits[1]):.9f}",
          effort=str(effort), velocity=f"{velocity:.9f}")
    child(joint, "dynamics", damping="0.002", friction="0")


def add_gimbal(robot: ET.Element, config: ActuatorConfig, index: int, prefix: str) -> dict[str, Any]:
    """One C-200T hanging camera-down: base/damper -> yaw motor -> roll motor -> pitch motor -> camera.

    The mount frame (the plate's underside) has +X along the camera's neutral (outward) view
    direction and +Z up; the gimbal hangs below it, so the camera image stays upright.
    Component boxes approximate the vendor 80 x 65-81 x 92 mm envelope; the 125 g mass is split
    0.28 base/damper, 0.32 yaw stage, 0.40 roll arm with pitch motor (engineering estimate).
    `index` 1 is the left (+Y) unit, 2 the right; `prefix` is the ROS namespace ("" or "/ns").
    Returns the camera's manifest entry plus a "plugin" dict whose roll/pitch/yaw are in degrees.
    """
    side = "left" if index == 1 else "right"
    base, yaw, roll, camera = (f"gimbal{index}_base", f"gimbal{index}_yaw_link",
                               f"gimbal{index}_roll_link", f"camera{index}_link")
    mass = config.gimbal_mass_kg
    dark = (0.08, 0.08, 0.09, 1)
    solid_link(robot, base, (0.050, 0.050, 0.014), mass * 0.28, xyz=(0, 0, -0.007), color=dark)
    sign = 1 if index == 1 else -1
    fixed_joint(robot, f"gimbal{index}_mount_joint", "gimbal_crossbar", base,
                (GIMBAL_AXIS_X - GIMBAL_PLATE_DEPTH / 2, sign * config.gimbal_offset_m, -0.003),
                (0, 0, math.radians(config.gimbal_mount_yaw_deg[index - 1])))
    # The 200 deg/s vendor control speed is enforced by greenhouse::Gimbal. The URDF velocity limit is a
    # higher mechanical ceiling: a DART velocity limit equal to the servo command was measured to latch
    # the joint at that speed (it overshot into the hard stop), so the two must not coincide.
    rate = 2 * math.radians(config.gimbal_max_rate_deg_s)
    solid_link(robot, yaw, (0.034, 0.034, 0.018), mass * 0.32, xyz=(-0.012, 0, -0.012), color=dark)
    revolute_joint(robot, f"gimbal{index}_yaw_joint", base, yaw, (0, 0, -0.014), "0 0 1",
                   GIMBAL_LIMITS_DEG["yaw"], config.gimbal_torque_nm, rate)
    solid_link(robot, roll, (0.050, 0.012, 0.030), mass * 0.40, xyz=(0.020, -0.031, 0), color=dark)
    revolute_joint(robot, f"gimbal{index}_roll_joint", yaw, roll, (-0.040, 0, -0.045), "1 0 0",
                   GIMBAL_LIMITS_DEG["roll"], config.gimbal_torque_nm, rate)
    solid_link(robot, camera, (0.040, 0.045, 0.035), config.camera_mass_kg, color=(0.10, 0.11, 0.12, 1))
    # Axis -Y: a positive pitch angle tilts the camera up.
    revolute_joint(robot, f"gimbal{index}_pitch_joint", roll, camera, (0.040, 0, 0), "0 -1 0",
                   GIMBAL_LIMITS_DEG["pitch"], config.gimbal_torque_nm, rate)
    optical = f"camera{index}_optical"
    fixed_joint(robot, f"camera{index}_optical_joint", camera, optical, (0.020, 0, 0),
                (-math.pi / 2, 0, -math.pi / 2))
    child(robot, "link", name=optical)
    topic = f"{prefix}/greenhouse/camera{index}/image"
    sensor = child(child(robot, "gazebo", reference=camera), "sensor", name=f"camera{index}_rgb", type="camera")
    child(sensor, "pose", "0.020 0 0 0 0 0")
    child(sensor, "always_on", "true")
    child(sensor, "update_rate", str(config.camera_rate_hz))
    child(sensor, "topic", topic)
    child(sensor, "gz_frame_id", optical)
    lens = child(sensor, "camera")
    child(lens, "horizontal_fov", str(config.camera_fov_rad))
    child(lens, "camera_info_topic", f"{prefix}/greenhouse/camera{index}/camera_info")
    image = child(lens, "image")
    child(image, "width", str(config.camera_width))
    child(image, "height", str(config.camera_height))
    child(image, "format", "R8G8B8")
    clip = child(lens, "clip")
    child(clip, "near", "0.025")
    child(clip, "far", "30")
    roll0, pitch0, yaw0 = config.gimbal_initial_rpy_deg[index - 1]
    joints = [f"gimbal{index}_yaw_joint", f"gimbal{index}_roll_joint", f"gimbal{index}_pitch_joint"]
    return {
        "id": index, "side": side, "topic": topic, "link": camera, "optical_frame": optical, "joints": joints,
        "plugin": {"id": index, "yaw_joint": joints[0], "roll_joint": joints[1], "pitch_joint": joints[2],
                   "yaw_link": yaw, "camera_link": camera, "roll": roll0, "pitch": pitch0, "yaw": yaw0,
                   "stabilise": config.gimbal_stabilise[index - 1]},
    }


def add_lidar(robot: ET.Element, lidar: LidarConfig, prefix: str) -> dict[str, Any]:
    """Housing box on base_link plus a gz gpu_lidar at the scan origin; return its manifest entry.

    Lengths in metres, angles converted from degrees to radians; `prefix` is the ROS namespace
    ("" or "/ns"). The scan is symmetric about the housing's +X axis.
    """
    link, frame = f"{LIDAR_NAME}_link", f"{LIDAR_NAME}_laser"
    # RAL 7032 pebble grey, the data sheet housing colour.
    solid_link(robot, link, lidar.size_m, lidar.mass_kg, xyz=(0, 0, lidar.size_m[2] / 2),
               color=(0.72, 0.71, 0.66, 1))
    fixed_joint(robot, f"{LIDAR_NAME}_joint", "base_link", link, lidar.xyz_m, [math.radians(v) for v in lidar.rpy_deg])
    fixed_joint(robot, f"{LIDAR_NAME}_laser_joint", link, frame, (0, 0, lidar.focal_height_m))
    child(robot, "link", name=frame)
    topic = f"{prefix}/sensors/{LIDAR_NAME}/scan"
    sensor = child(child(robot, "gazebo", reference=frame), "sensor", name=LIDAR_NAME, type="gpu_lidar")
    child(sensor, "always_on", "true")
    child(sensor, "update_rate", str(lidar.update_rate_hz))
    child(sensor, "topic", topic)
    child(sensor, "gz_frame_id", frame)
    ray = child(sensor, "lidar")
    scan = child(ray, "scan")
    for axis, samples, fov in (("horizontal", lidar.horizontal_samples, lidar.horizontal_fov_deg),
                               ("vertical", lidar.vertical_samples, lidar.vertical_fov_deg)):
        element = child(scan, axis)
        child(element, "samples", str(samples))
        child(element, "resolution", "1")
        child(element, "min_angle", f"{-math.radians(fov) / 2:.9f}")
        child(element, "max_angle", f"{math.radians(fov) / 2:.9f}")
    span = child(ray, "range")
    child(span, "min", str(lidar.range_min_m))
    child(span, "max", str(lidar.range_max_m))
    noise = child(ray, "noise")
    child(noise, "type", "gaussian")
    child(noise, "mean", "0")
    child(noise, "stddev", str(lidar.noise_stddev_m))
    return {"name": LIDAR_NAME, "model": lidar.model, "topic": topic, "link": link, "frame": frame,
            "update_rate_hz": lidar.update_rate_hz}


def generate_wrapper(config: ActuatorConfig, setup: Path | str, namespace: str) -> dict[str, Any]:
    """Write `setup`/greenhouse/robot-with-actuator.urdf.xacro and actuator.json; return that manifest.

    URDF lengths are metres and angles radians (the bracket STL is in mm, scaled by 0.001); the
    rail is mounted at `deck_xyz_m` in base_link. Manifest keys carry their unit suffix.
    """
    setup = Path(setup)
    state = setup / "greenhouse"
    state.mkdir(parents=True, exist_ok=True)
    robot = ET.Element("robot", name="greenhouse_jackal", attrib={"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    child(robot, "xacro:include", filename=str(setup / "robot.urdf.xacro"))
    length = config.body_length
    rail_mass = config.module_mass - 0.32 - 0.35 - 0.15
    mount = solid_link(robot, "fpb30_mount", (0.15, 0.15, 0.008), config.mount_mass_kg * 0.55,
                       xyz=(0, 0, 0.004), color=(0.17, 0.18, 0.18, 1))
    visual = mount.find("visual")
    visual.find("origin").set("xyz", "0 0 0")
    geometry = visual.find("geometry")
    geometry.remove(geometry.find("box"))
    child(geometry, "mesh", filename=(state / "cad" / "rear_deck_bracket.stl").resolve().as_uri(),
          scale="0.001 0.001 0.001")
    fixed_joint(robot, "fpb30_deck_mount", "base_link", "fpb30_mount", config.deck_xyz_m)
    upright = solid_link(robot, "fpb30_mount_upright", (0.010, 0.044, 0.190), config.mount_mass_kg * 0.45,
                         xyz=(0, 0, 0.095), color=(0.17, 0.18, 0.18, 1))
    upright.remove(upright.find("visual"))
    fixed_joint(robot, "fpb30_mount_brace", "fpb30_mount", "fpb30_mount_upright", (-0.055, 0, 0.008))
    solid_link(robot, "fpb30_rail", (0.030, 0.030, length), rail_mass,
               xyz=(0, 0, length / 2))
    fixed_joint(robot, "fpb30_fixed_rail", "fpb30_mount", "fpb30_rail", (-0.030, 0, 0.012))
    solid_link(robot, "fpb30_motor", (0.047, 0.042, 0.042), 0.35, color=(0.12, 0.13, 0.13, 1))
    fixed_joint(robot, "fpb30_motor_joint", "fpb30_rail", "fpb30_motor", (-0.026, 0, 0.030))
    solid_link(robot, "fpb30_top_cap", (0.036, 0.034, 0.028), 0.15, color=(0.12, 0.13, 0.13, 1))
    fixed_joint(robot, "fpb30_end_cap_joint", "fpb30_rail", "fpb30_top_cap", (0, 0, length - 0.014))
    solid_link(robot, "fpb30_carriage", (0.011, 0.065, 0.084), 0.32, color=(0.32, 0.34, 0.35, 1))
    joint = child(robot, "joint", name="fpb30_slide_joint", type="prismatic")
    child(joint, "parent", link="fpb30_rail")
    child(joint, "child", link="fpb30_carriage")
    child(joint, "origin", xyz="0.0205 0 0.086", rpy="0 0 0")
    child(joint, "axis", xyz="0 0 1")
    child(joint, "limit", lower="0", upper=str(config.stroke_m),
          effort=str(config.force_limit_n), velocity=str(config.rated_speed))
    child(joint, "dynamics", damping="0.4", friction="0")
    prefix = f"/{namespace.strip('/')}" if namespace.strip("/") else ""
    # Cantilevered crossbar plate on the carriage face; both gimbals hang under it ("camera down"
    # mounting), clear of the carriage, rail, deck and each other through their full range.
    plate_x = 0.0055 + GIMBAL_PLATE_DEPTH / 2
    plate_width = 2 * config.gimbal_offset_m + 0.09
    solid_link(robot, "gimbal_crossbar", (GIMBAL_PLATE_DEPTH, plate_width, 0.006),
               config.camera_bracket_mass_kg, color=(0.17, 0.18, 0.18, 1))
    fixed_joint(robot, "gimbal_crossbar_joint", "fpb30_carriage", "gimbal_crossbar",
                (plate_x, 0, GIMBAL_PLATE_Z))
    gimbals = []
    for index in (1, 2):
        gimbals.append(add_gimbal(robot, config, index, prefix))
    publisher = child(child(robot, "gazebo"), "plugin", filename="libGreenhouseSystems.so",
                      name="greenhouse::Gimbal")
    child(publisher, "prefix", prefix + "/greenhouse")
    child(publisher, "max_rate", str(math.radians(config.gimbal_max_rate_deg_s)))
    for axis, (low, high) in GIMBAL_LIMITS_DEG.items():
        child(publisher, f"{axis}_limits", f"{low} {high}")
    for gimbal in gimbals:
        element = child(publisher, "gimbal")
        for key, value in gimbal["plugin"].items():
            child(element, key, str(value).lower() if isinstance(value, bool) else str(value))
    plugin = child(child(robot, "gazebo"), "plugin",
                   filename="libGreenhouseSystems.so", name="greenhouse::Actuator")
    for key, value in {
        "joint_name": "fpb30_slide_joint", "prefix": prefix + "/greenhouse",
        "stroke": config.stroke_m, "speed": config.speed_m_s, "max_speed": config.rated_speed,
        "ramp_time": config.ramp_time_s, "force_limit": config.force_limit_n,
        "moving_mass": config.moving_mass,
    }.items():
        child(plugin, key, str(value))
    joint_publisher = child(child(robot, "gazebo"), "plugin",
                            filename="gz-sim-joint-state-publisher-system",
                            name="gz::sim::systems::JointStatePublisher")
    for name in ["fpb30_slide_joint"] + [joint for gimbal in gimbals for joint in gimbal["joints"]]:
        child(joint_publisher, "joint_name", name)
    child(joint_publisher, "topic", prefix + "/greenhouse/lift/joint_state")
    lidar = add_lidar(robot, config.lidar, prefix) if config.lidar.enabled else None
    path = state / "robot-with-actuator.urdf.xacro"
    write_xml(path, robot)
    manifest = {
        "config": asdict(config), "module_mass_kg": config.module_mass,
        "body_length_m": length, "moving_mass_kg": config.moving_mass,
        "max_vertical_speed_m_s": config.rated_speed, "force_limit_is_engineering_assumption": True,
        "namespace": namespace, "prefix": prefix + "/greenhouse",
        "wrapper": str(path),
        "camera_topics": [gimbal["topic"] for gimbal in gimbals],
        "cameras": [{key: gimbal[key] for key in ("id", "side", "topic", "link", "optical_frame", "joints")}
                    for gimbal in gimbals],
        "gimbal": {"model": "XF C-200T 3-axis FPV gimbal", "payload_kg": GIMBAL_PAYLOAD_KG,
                   "limits_deg": GIMBAL_LIMITS_DEG, "torque_is_engineering_assumption": True},
        "lidar": lidar,
    }
    (state / "actuator.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    """CLI: generate the actuator wrapper and print its manifest; exits with status 2 on a bad config."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "actuator.json")
    parser.add_argument("--setup", type=Path, required=True)
    parser.add_argument("--namespace", default="")
    args = parser.parse_args()
    try:
        result = generate_wrapper(load_actuator(args.config), args.setup, args.namespace)
    except (ValueError, OSError) as error:
        parser.exit(2, f"Actuator configuration failed: {error}\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
