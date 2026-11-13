# SPDX-License-Identifier: GPL-3.0-only
"""Speed limits along a planned path: slow before curves and the goal, full speed elsewhere."""
import math
from collections.abc import Sequence


def speed_profile(points: Sequence[Sequence[float]], lateral_accel: float, turn_rate: float, braking: float,
                  stride: int = 5, stop: bool = True) -> list[float]:
    """Return the highest safe speed (m/s) at each path point.

    Curvature over +/-stride points limits speed to sqrt(lateral_accel/curvature) and
    turn_rate/curvature; a backward pass then lets the robot brake at `braking` m/s^2
    into every slower point ahead, ending stopped at the last point unless `stop` is False.
    `points` are planar (x, y) in metres in any one frame; lateral_accel is m/s^2, turn_rate rad/s.
    Uncapped points are math.inf.
    """
    count = len(points)
    caps = [math.inf] * count
    for i in range(count):
        a, b, c = points[max(0, i - stride)], points[i], points[min(count - 1, i + stride)]
        before, after = math.dist(a, b), math.dist(b, c)
        if before < 1e-6 or after < 1e-6:
            continue
        turn = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0])
        curvature = abs(math.atan2(math.sin(turn), math.cos(turn))) / ((before + after) / 2)
        if curvature > 1e-6:
            caps[i] = min(math.sqrt(lateral_accel / curvature), turn_rate / curvature)
    if caps and stop:
        caps[-1] = 0.0
    for i in range(count - 2, -1, -1):
        caps[i] = min(caps[i], math.sqrt(caps[i + 1] ** 2 + 2 * braking * math.dist(points[i], points[i + 1])))
    return caps
