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
                            anchor=(0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
    """Translate a task-frame (right/forward/up) point into ROS ENU.

    The task frame is right x 0-100m / forward y 0-150m relative to the
    mission origin.  The mission origin is aligned with the EKF origin
    (calibration freezes the A01-A06 global-position median into
    set_gp_origin), so mission_anchor stays [0,0,0] and the task point is
    rotated by the reference heading only.
    """
    dx = float(x) - float(anchor[0])
    dy = float(y) - float(anchor[1])
    ex, ny = task_vector_to_enu(dx, dy, heading)
    return ex, ny, float(z) - float(anchor[2])


def validate_calibration_config(calibration_uavs, all_uavs, identities=None):
    """Validate execution-layer (UAVn) calibration/confirmation sets.

    implementation_plan_26091101 §3.1/§3.2:
    - ``calibration_uavs`` is the required set used to sample global/local
      pose and freeze origin/heading; it must be non-empty and duplicate-free.
    - ``all_uavs`` is the known/deployed set used only for
      ``gp_origin_confirmed`` bookkeeping; it must be non-empty and
      duplicate-free and a superset of ``calibration_uavs``.
    - ``identities`` (optional sequence of identity entries exposing
      ``exec_target`` and ``enabled``) tightens validation: every configured
      UAVn must exist as a unique, enabled ``exec_target``.

    Returns ``(calibration_uavs, all_uavs)`` as tuples; raises ``ValueError``
    on any violation (fail-closed at startup).
    """
    calibration = tuple(str(item) for item in calibration_uavs)
    known = tuple(str(item) for item in all_uavs)
    if not calibration:
        raise ValueError("calibration_uavs must not be empty")
    if not known:
        raise ValueError("all_uavs must not be empty")
    if len(set(calibration)) != len(calibration):
        raise ValueError("calibration_uavs contains duplicates")
    if len(set(known)) != len(known):
        raise ValueError("all_uavs contains duplicates")
    if not set(calibration).issubset(set(known)):
        raise ValueError("calibration_uavs must be a subset of all_uavs")
    if identities is not None:
        enabled = {}
        for entry in identities:
            target = str(getattr(entry, "exec_target", ""))
            if not target:
                raise ValueError("identity map contains an empty exec_target")
            if target in enabled:
                raise ValueError("identity map repeats exec_target %s" % target)
            enabled[target] = bool(getattr(entry, "enabled", True))
        for uav in known:
            if uav not in enabled:
                raise ValueError("configured UAV %s missing from identity map" % uav)
            if not enabled[uav]:
                raise ValueError("configured UAV %s is disabled in identity map" % uav)
    return calibration, known

