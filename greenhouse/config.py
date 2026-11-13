# SPDX-License-Identifier: GPL-3.0-only
import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path

DEFAULTS_PATH = Path(__file__).resolve().parents[1] / "config" / "greenhouse.json"

# Valid ranges (metres unless dimensionless); runtime scripts clamp/check against the same table.
LIMITS = {
    "ground_resolution": (0.04, 0.25),
    "ground_height_range": (0.005, 0.08),
    "ground_headland": (3.0, 8.0),  # below ~2.75 m the soil mesh no longer covers the house
    "ground_bump_rms": (0.0, 0.02),
    "ground_bump_scale": (0.16, 0.5),
    "ground_friction": (0.1, 1.2),
    "slip_longitudinal": (0.0, 2.0),
    "slip_lateral": (0.0, 2.0),
    "row_spacing": (1.5, 4.0),
    "row_length": (3.0, 30.0),
    "plant_spacing": (0.5, 1.5),
    "plant_height": (1.0, 1.8),
    "plant_radius": (0.2, 0.5),
    "house_length": (8.0, 40.0),
    "house_width": (5.0, 30.0),
    "eaves_height": (2.0, 4.0),
    "ridge_height": (2.5, 5.0),
    "door_width": (1.5, 4.0),
    "spawn_clearance": (0.08, 0.3),
}


@dataclass(frozen=True)
class Config:
    seed: int
    ground_resolution: float
    ground_height_range: float
    ground_headland: float
    ground_bump_rms: float
    ground_bump_scale: float
    ground_friction: float
    slip_longitudinal: float
    slip_lateral: float
    row_count: int
    row_spacing: float
    row_length: float
    plant_spacing: float
    plant_height: float
    plant_radius: float
    house_length: float
    house_width: float
    eaves_height: float
    ridge_height: float
    door_width: float
    spawn_x: float
    spawn_y: float
    spawn_yaw: float
    spawn_clearance: float

    def validate(self) -> "Config":
        """Check every field against ``LIMITS`` and the layout/spawn geometry; return ``self``.

        Lengths are metres and ``spawn_yaw`` is radians, all in the greenhouse world frame.
        Raises ValueError naming the first violated constraint.
        """
        limits = LIMITS
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be a finite number")
            if not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
            if key in limits and not limits[key][0] <= value <= limits[key][1]:
                unit = "" if key in ("ground_friction", "slip_longitudinal", "slip_lateral") else " metres"
                raise ValueError(f"{key} must be in {limits[key]}{unit}")
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("seed must be an integer in [0, 2**32)")
        if type(self.row_count) is not int or not 2 <= self.row_count <= 8:
            raise ValueError("row_count must be an integer in [2, 8]")
        if self.row_spacing - 2 * self.plant_radius < 1.2 - 1e-9:
            raise ValueError("row_spacing must leave at least 1.2 m CLEAR after plant radius")
        if self.ridge_height <= self.eaves_height or self.eaves_height <= self.plant_height + 0.4:
            raise ValueError("roof must be above the eaves and at least 0.4 m above plants")
        if self.door_width >= self.house_width - 1:
            raise ValueError("door_width must leave space for both end walls")
        if self.end_clearance < 1.5:
            raise ValueError("house_length must leave at least 1.5 m end-turning headlands")
        if self.side_clearance < 1.2:
            raise ValueError("house_width must leave at least 1.2 m clear side aisles")
        if len(self.plant_positions()) > 240:
            raise ValueError("layout exceeds the 240-plant collision/render budget")
        # A conservative standard-J100 envelope, including its fenders.
        half_x = 0.4 * abs(math.cos(self.spawn_yaw)) + 0.32 * abs(math.sin(self.spawn_yaw))
        half_y = 0.4 * abs(math.sin(self.spawn_yaw)) + 0.32 * abs(math.cos(self.spawn_yaw))
        if abs(self.spawn_x) + half_x > self.house_length / 2 - 0.2:
            raise ValueError("spawn_x puts the Jackal envelope too close to an end wall")
        if abs(self.spawn_y) + half_y > self.house_width / 2 - 0.2:
            raise ValueError("spawn_y puts the Jackal envelope too close to a side wall")
        for x, y in self.plant_positions():
            dx = max(abs(x - self.spawn_x) - half_x, 0)
            dy = max(abs(y - self.spawn_y) - half_y, 0)
            if math.hypot(dx, dy) < self.plant_radius + 0.15:
                raise ValueError("spawn intersects a plant or its 0.15 m safety margin")
        return self

    @property
    def end_clearance(self) -> float:
        """Clear floor (m, along world x) from the end plants' radius, less 0.05 m, to each end wall."""
        return (self.house_length - self.row_length) / 2 - self.plant_radius - 0.05

    @property
    def side_clearance(self) -> float:
        """Clear floor (m, along world y) from the outer plants' radius, less 0.05 m, to each side wall."""
        return self.house_width / 2 - (self.row_count - 1) * self.row_spacing / 2 - self.plant_radius - 0.05

    def plant_positions(self) -> list[tuple[float, float]]:
        """Stem-base ``(x, y)`` of every plant in metres, world frame (house centre, +x along rows).

        Ordered row by row from -y to +y, and within a row from -x to +x.
        """
        count = math.floor(self.row_length / self.plant_spacing) + 1
        return [
            (-self.row_length / 2 + i * self.plant_spacing,
             (row - (self.row_count - 1) / 2) * self.row_spacing)
            for row in range(self.row_count) for i in range(count)
        ]


def _read_settings(path: Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"greenhouse configuration {path} must be a JSON object")
    unknown = set(data) - {field.name for field in fields(Config)}
    if unknown:
        raise ValueError(f"unknown greenhouse configuration keys in {path}: {', '.join(sorted(unknown))}")
    return data


def load_config(path: Path | str) -> Config:
    """Load a greenhouse JSON and validate it.

    Keys missing from ``path`` take their value from ``config/greenhouse.json``, the only
    place default values are written down. Lengths are metres, angles radians, world frame.
    """
    defaults = _read_settings(DEFAULTS_PATH)
    missing = {field.name for field in fields(Config)} - set(defaults)
    if missing:
        raise ValueError(f"{DEFAULTS_PATH} must define every key; missing: {', '.join(sorted(missing))}")
    return Config(**{**defaults, **_read_settings(path)}).validate()


def default_config() -> Config:
    """The bundled greenhouse, exactly as written in ``config/greenhouse.json``."""
    return load_config(DEFAULTS_PATH)
