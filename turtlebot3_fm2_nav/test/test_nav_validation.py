#!/usr/bin/env python3
"""Unit tests for navigation parameter and message validation."""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from nav_validation import (  # noqa: E402
    require_bool,
    require_float,
    require_int,
    require_message_frame,
    require_positive_floats,
)


class NavigationValidationTest(unittest.TestCase):
    """Exercise boundary values and unsafe parameter values."""

    def test_accepts_valid_values(self) -> None:
        """Accept values at valid boundaries and within the documented ranges."""
        self.assertEqual(require_int("~rate", 20, 1), 20)
        self.assertEqual(require_float("~lookahead", 0.2, 0.0), 0.2)
        self.assertEqual(require_positive_floats("~horizons", [0.5, 1.0]), (0.5, 1.0))
        self.assertTrue(require_bool("~enabled", True))
        message = SimpleNamespace(header=SimpleNamespace(frame_id="map"))
        self.assertIsNone(require_message_frame("costmap", message, "map"))

    def test_rejects_invalid_values(self) -> None:
        """Reject zero, non-finite, malformed, and incorrectly typed values."""
        with self.assertRaises(ValueError):
            require_int("~rate", 0, 1)
        with self.assertRaises(ValueError):
            require_float("~lookahead", float("nan"), 0.0)
        with self.assertRaises(ValueError):
            require_float("~timeout", 0.0, 0.0, minimum_inclusive=False)
        with self.assertRaises(ValueError):
            require_positive_floats("~horizons", [0.5, 0.0])
        with self.assertRaises(ValueError):
            require_bool("~enabled", "false")
        with self.assertRaises(ValueError):
            require_message_frame(
                "costmap",
                SimpleNamespace(header=SimpleNamespace(frame_id="odom")),
                "map",
            )


if __name__ == "__main__":
    unittest.main()
