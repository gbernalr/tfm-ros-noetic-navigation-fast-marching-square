#!/usr/bin/env python3
"""Unit tests for FM2 planner geometry and failed-plan behaviour."""

import math
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from fm2_planner_node import FM2Planner  # noqa: E402


class PlannerGeometryTest(unittest.TestCase):
    """Check rotated map conversions and an explicit no-route result."""

    def setUp(self) -> None:
        """Create a planner shell with a rotated grid geometry."""
        self.planner = FM2Planner.__new__(FM2Planner)
        self.planner.map_res = 0.5
        self.planner.map_ox = -2.0
        self.planner.map_oy = 3.0
        self.planner.map_origin_yaw = math.pi / 2.0

    def test_rotated_grid_round_trip(self) -> None:
        """Map cells remain stable after a rotated grid-to-world round trip."""
        x, y = self.planner._grid_to_world(4, 1)
        self.assertAlmostEqual(x, -2.75)
        self.assertAlmostEqual(y, 5.25)
        self.assertEqual(self.planner._world_to_grid(x, y), (4, 1))

    def test_negative_coordinate_uses_floor(self) -> None:
        """Coordinates just outside an origin map to the preceding cell."""
        self.planner.map_origin_yaw = 0.0
        self.assertEqual(self.planner._world_to_grid(-2.01, 3.01), (-1, 0))

    def test_no_fm2_route_returns_none(self) -> None:
        """An FM2 no-path result propagates as an explicit failure."""
        fake_fm2 = SimpleNamespace(
            set_map=lambda _map: None,
            get_path=lambda _start, _goal: SimpleNamespace(path=None),
        )
        with patch("fm2_planner_node.FM2", return_value=fake_fm2):
            self.assertIsNone(
                FM2Planner._solve_fm2_path(
                    np.ones((3, 3), dtype=np.uint8), (0, 0), (2, 2)
                )
            )


if __name__ == "__main__":
    unittest.main()
