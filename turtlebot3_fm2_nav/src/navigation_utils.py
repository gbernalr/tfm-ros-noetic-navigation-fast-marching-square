"""Shared, dependency-free helpers for navigation-node state and geometry."""

import math
from dataclasses import dataclass
from functools import wraps
from typing import Callable, Tuple


def synchronized(method: Callable[..., object]) -> Callable[..., object]:
    """Serialize a method using the instance ``_state_lock`` re-entrant lock."""

    @wraps(method)
    def wrapped(*args: object, **kwargs: object) -> object:
        self = args[0]
        with self._state_lock:
            return method(*args, **kwargs)

    return wrapped


def quaternion_yaw(quaternion: object) -> float:
    """Return the planar yaw represented by a quaternion-like object."""
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class GridGeometry:
    """Immutable geometry of an occupancy grid expressed in a map frame."""

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


def world_to_grid(x: float, y: float, geometry: GridGeometry) -> Tuple[int, int]:
    """Convert map-frame coordinates into occupancy-grid indices."""
    dx = x - geometry.origin_x
    dy = y - geometry.origin_y
    cos_yaw = math.cos(geometry.origin_yaw)
    sin_yaw = math.sin(geometry.origin_yaw)
    return (
        math.floor((cos_yaw * dx + sin_yaw * dy) / geometry.resolution),
        math.floor((-sin_yaw * dx + cos_yaw * dy) / geometry.resolution),
    )


def grid_to_world(ix: int, iy: int, geometry: GridGeometry) -> Tuple[float, float]:
    """Return the map-frame center coordinates of an occupancy-grid cell."""
    local_x = (ix + 0.5) * geometry.resolution
    local_y = (iy + 0.5) * geometry.resolution
    cos_yaw = math.cos(geometry.origin_yaw)
    sin_yaw = math.sin(geometry.origin_yaw)
    return (
        geometry.origin_x + cos_yaw * local_x - sin_yaw * local_y,
        geometry.origin_y + sin_yaw * local_x + cos_yaw * local_y,
    )


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
