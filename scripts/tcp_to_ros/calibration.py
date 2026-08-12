"""Pure calibration and task-frame math for Group A."""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple


@dataclass(frozen=True)
class GlobalPoint:
    latitude: float
    longitude: float
    altitude: float


def median_global(points: Sequence[GlobalPoint]) -> GlobalPoint:
    if not points:
        raise ValueError("at least one global point is required")
    return GlobalPoint(
        statistics.median(item.latitude for item in points),
        statistics.median(item.longitude for item in points),
        statistics.median(item.altitude for item in points),
    )


def circular_mean(angles: Iterable[float]) -> float:
    values = tuple(float(value) for value in angles)
    if not values:
        raise ValueError("at least one heading is required")
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) < 1e-9:
        raise ValueError("headings have no unique circular mean")
    return math.atan2(sine, cosine)


def angular_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def headings_within(angles: Iterable[float], reference: float, limit_rad: float) -> bool:
    return all(angular_error(float(value), reference) <= float(limit_rad) for value in angles)


def task_vector_to_enu(x_right: float, y_forward: float, heading: float) -> Tuple[float, float]:
    """Rotate a right/forward vector into ROS ENU using aircraft heading."""
    return (
        x_right * math.sin(heading) + y_forward * math.cos(heading),
        -x_right * math.cos(heading) + y_forward * math.sin(heading),
    )


def task_point_to_local_enu(x: float, y: float, z: float, heading: float,
                            anchor=(10.0, 5.0, 0.0)) -> Tuple[float, float, float]:
    dx = float(x) - float(anchor[0])
    dy = float(y) - float(anchor[1])
    ex, ny = task_vector_to_enu(dx, dy, heading)
    return ex, ny, float(z) - float(anchor[2])
