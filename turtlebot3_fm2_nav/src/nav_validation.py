"""Small, dependency-free validation helpers shared by navigation nodes."""

import math
from numbers import Integral, Real
from typing import Iterable


def require_nonempty_string(name: str, value: object) -> str:
    """Return a non-empty string parameter or raise a descriptive error."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError("parameter {!r} must be a non-empty string".format(name))
    return value


def require_bool(name: str, value: object) -> bool:
    """Return a boolean parameter without accepting truthy strings."""
    if not isinstance(value, bool):
        raise ValueError("parameter {!r} must be a boolean".format(name))
    return value


def require_int(
    name: str, value: object, minimum: int = None, maximum: int = None
) -> int:
    """Return an integral parameter constrained to an optional closed range."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError("parameter {!r} must be an integer".format(name))
    value = int(value)
    if minimum is not None and value < minimum:
        raise ValueError("parameter {!r} must be >= {}".format(name, minimum))
    if maximum is not None and value > maximum:
        raise ValueError("parameter {!r} must be <= {}".format(name, maximum))
    return value


def require_float(
    name: str,
    value: object,
    minimum: float = None,
    maximum: float = None,
    *,
    minimum_inclusive: bool = True,
) -> float:
    """Return a finite real parameter constrained to an optional range."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("parameter {!r} must be a real number".format(name))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("parameter {!r} must be finite".format(name))
    if minimum is not None and (
        value < minimum or (not minimum_inclusive and value <= minimum)
    ):
        comparator = ">=" if minimum_inclusive else ">"
        raise ValueError(
            "parameter {!r} must be {} {}".format(name, comparator, minimum)
        )
    if maximum is not None and value > maximum:
        raise ValueError("parameter {!r} must be <= {}".format(name, maximum))
    return value


def require_positive_floats(name: str, values: object) -> tuple:
    """Validate and return a non-empty tuple of strictly positive floats."""
    if isinstance(values, (str, bytes)) or not isinstance(values, Iterable):
        raise ValueError(
            "parameter {!r} must be a non-empty list of numbers".format(name)
        )
    result = tuple(
        require_float("{}[{}]".format(name, index), value, 0.0, minimum_inclusive=False)
        for index, value in enumerate(values)
    )
    if not result:
        raise ValueError("parameter {!r} must not be empty".format(name))
    return result


def require_occupancy_grid(name: str, message: object) -> None:
    """Validate geometry and payload size of an OccupancyGrid-like message."""
    info = getattr(message, "info", None)
    if info is None:
        raise ValueError("{} has no map metadata".format(name))
    width = require_int("{}.info.width".format(name), info.width, 1)
    height = require_int("{}.info.height".format(name), info.height, 1)
    require_float(
        "{}.info.resolution".format(name), info.resolution, 0.0, minimum_inclusive=False
    )
    origin = info.origin.position
    require_float("{}.info.origin.position.x".format(name), origin.x)
    require_float("{}.info.origin.position.y".format(name), origin.y)
    quaternion = info.origin.orientation
    components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    for index, component in enumerate(components):
        require_float("{}.info.origin.orientation[{}]".format(name, index), component)
    if sum(component * component for component in components) <= 1e-12:
        raise ValueError(
            "{}.info.origin.orientation must not be the zero quaternion".format(name)
        )
    if len(message.data) != width * height:
        raise ValueError(
            "{} has {} cells but its geometry requires {}".format(
                name, len(message.data), width * height
            )
        )


def require_message_frame(name: str, message: object, expected_frame: str) -> None:
    """Require a message header frame to match the configured navigation frame."""
    header = getattr(message, "header", None)
    frame_id = getattr(header, "frame_id", None)
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("{} has no header frame_id".format(name))
    if frame_id != expected_frame:
        raise ValueError(
            "{}.header.frame_id must be {!r}, got {!r}".format(
                name, expected_frame, frame_id
            )
        )


def require_finite_xy(name: str, x: object, y: object) -> None:
    """Validate a finite planar coordinate without converting its representation."""
    require_float("{}.x".format(name), x)
    require_float("{}.y".format(name), y)
