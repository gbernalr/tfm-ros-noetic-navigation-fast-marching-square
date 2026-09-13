#!/usr/bin/env python3
"""Deprecated combined FM2 planner-and-controller for simple simulations.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math
from threading import RLock
from typing import List, Tuple

import cv2
import numpy as np
import rospy
import tf2_ros
from fm2 import FM2
from fm2.entities import FM2Map
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Path

from nav_validation import require_float, require_int, require_nonempty_string
from navigation_utils import (
    GridGeometry,
    grid_to_world,
    quaternion_yaw,
    synchronized,
    world_to_grid,
    wrap_to_pi,
)


class FM2TestPlanner:
    """Deprecated combined node retained only for backward-compatible demos."""

    def __init__(self) -> None:
        """Initialize the ROS node, configuration, state, and interfaces."""
        rospy.init_node("fm2_test_planner")
        rospy.logwarn(
            "simple_nav_node.py is deprecated; use fm2_planner_node.py and "
            "fm2_controller_node.py instead"
        )
        self._state_lock = RLock()

        self.frame_map = require_nonempty_string(
            "~frame_map", rospy.get_param("~frame_map", "map")
        )
        self.replan_period = require_float(
            "~replan_period", rospy.get_param("~replan_period", 0.2), 0.0
        )
        self.inflation = require_int("~inflate", rospy.get_param("~inflate", 2), 0)
        self.occupancy_threshold = require_int(
            "~occupancy_threshold", rospy.get_param("~occupancy_threshold", 50), 0, 100
        )
        self.tf_timeout = rospy.Duration(
            require_float(
                "~tf_timeout",
                rospy.get_param("~tf_timeout", 0.5),
                0.0,
                minimum_inclusive=False,
            )
        )

        self.lookahead_dist = require_float(
            "~lookahead",
            rospy.get_param("~lookahead", 0.35),
            0.0,
            minimum_inclusive=False,
        )
        self.v_lin = require_float("~v_lin", rospy.get_param("~v_lin", 0.22), 0.0)
        self.v_ang_max = require_float(
            "~v_ang_max",
            rospy.get_param("~v_ang_max", 1.5),
            0.0,
            minimum_inclusive=False,
        )
        self.goal_tolerance = require_float(
            "~goal_tolerance", rospy.get_param("~goal_tolerance", 0.08), 0.0
        )
        self.k_theta = require_float("~k_theta", rospy.get_param("~k_theta", 2.0), 0.0)
        self.replan_offpath = require_float(
            "~replan_offpath", rospy.get_param("~replan_offpath", 1.0), 0.0
        )
        self.offpath_window_points = require_int(
            "~offpath_window_points", rospy.get_param("~offpath_window_points", 30), 1
        )
        self.path_point_tolerance = require_float(
            "~path_point_tolerance", rospy.get_param("~path_point_tolerance", 0.25), 0.0
        )
        self.turn_slowdown_angle = require_float(
            "~turn_slowdown_angle",
            rospy.get_param("~turn_slowdown_angle", 1.2),
            0.0,
            minimum_inclusive=False,
        )
        self.max_turn_speed_reduction = require_float(
            "~max_turn_speed_reduction",
            rospy.get_param("~max_turn_speed_reduction", 0.8),
            0.0,
            1.0,
        )
        self.min_linear_speed_factor = require_float(
            "~min_linear_speed_factor",
            rospy.get_param("~min_linear_speed_factor", 0.2),
            0.0,
            1.0,
        )
        self.rate_hz = require_int("~rate", rospy.get_param("~rate", 20), 1)

        self.map_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.map_origin_yaw = None
        self.last_pose = None  # Latest (x, y, yaw) pose.
        self.goal_world = None
        self.path_world = None
        self.path_idx = 0

        self.fm2 = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_map = rospy.Subscriber(
            "fm2_costmap/costmap", OccupancyGrid, self.cb_map, queue_size=1
        )
        self.sub_goal = rospy.Subscriber(
            "move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1
        )
        self.sub_amcl = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1
        )

        self.pub_path = rospy.Publisher("fm2_path", Path, queue_size=1, latch=True)
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.last_replan_time = rospy.Time.now()

    @synchronized
    def cb_map(self, msg: OccupancyGrid) -> None:
        """Convert the latest occupancy grid into a binary FM2 map."""
        w = msg.info.width
        h = msg.info.height
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        self.map_origin_yaw = self._yaw_from_quat(msg.info.origin.orientation)
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ = data >= self.occupancy_threshold
        unk = data < 0
        obs = np.logical_or(occ, unk).astype(np.uint8)
        self.map_bin = (1 - obs).astype(np.uint8)

    @synchronized
    def cb_goal(self, msg: PoseStamped) -> None:
        """Transform, store, and immediately plan toward a new goal."""
        pose = msg
        if msg.header.frame_id != self.frame_map:
            try:
                pose = self.tf_buffer.transform(
                    msg, self.frame_map, timeout=self.tf_timeout
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn(
                    "Simple FM2 navigator could not transform the goal: %s", e
                )
                return
        self.goal_world = (pose.pose.position.x, pose.pose.position.y)
        rospy.loginfo("Simple FM2 navigator received goal %s", self.goal_world)
        self._plan(trigger="goal")

    @synchronized
    def cb_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        """Update the current robot pose from AMCL."""
        if msg.header.frame_id != self.frame_map:
            try:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = msg.pose.pose
                ps = self.tf_buffer.transform(
                    ps, self.frame_map, timeout=self.tf_timeout
                )
                x = ps.pose.position.x
                y = ps.pose.position.y
                yaw = self._yaw_from_quat(ps.pose.orientation)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn(
                    "Simple FM2 navigator could not transform the AMCL pose: %s", e
                )
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        self.last_pose = (x, y, yaw)

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        """Return the planar yaw represented by a quaternion-like object."""
        return quaternion_yaw(q)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Normalize an angle to the half-open interval [-pi, pi)."""
        return wrap_to_pi(angle)

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert map-frame coordinates into occupancy-grid indices."""
        return world_to_grid(
            x,
            y,
            GridGeometry(self.map_res, self.map_ox, self.map_oy, self.map_origin_yaw),
        )

    def _grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Return the world coordinates of the center of a grid cell."""
        return grid_to_world(
            ix,
            iy,
            GridGeometry(self.map_res, self.map_ox, self.map_oy, self.map_origin_yaw),
        )

    def _publish_path(self, pts_world: List[Tuple[float, float]]) -> None:
        """Publish world-frame points as a ROS path."""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_map
        for x, y in pts_world:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = self.frame_map
            ps.pose.position.x = x
            ps.pose.position.y = y
            path.poses.append(ps)
        self.pub_path.publish(path)

    @synchronized
    def _plan(self, trigger: str = "timer") -> None:
        """Plan and publish a path from the current robot pose to the goal."""
        if self.map_bin is None:
            rospy.logwarn_throttle(5, "Simple FM2 navigator is waiting for a map")
            return
        if self.last_pose is None:
            rospy.logwarn_throttle(5, "Simple FM2 navigator is waiting for AMCL")
            return
        if self.goal_world is None:
            return

        sx, sy, _ = self.last_pose
        gx, gy = self.goal_world

        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy = self._world_to_grid(gx, gy)

        binary = self.map_bin.copy().astype(np.uint8)

        if self.inflation > 0:
            k = 2 * self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        self.fm2 = FM2(mode="cpu")
        fm2_map = FM2Map.from_binary_map(binary, create_border=True)
        self.fm2.set_map(fm2_map)
        info = self.fm2.get_path(
            (int(start_iy), int(start_ix)), (int(goal_iy), int(goal_ix))
        )

        if info.path is None:
            rospy.logwarn("Simple FM2 navigator could not find a path")
            self.path_world = None
            return

        rows, cols = info.path
        # ROS Noetic uses Python 3.8, which does not support zip(strict=...).
        pts = [
            self._grid_to_world(int(col), int(row))
            for row, col in zip(rows, cols)  # noqa: B905
        ]
        self.path_world = pts
        self.path_idx = 0
        self._publish_path(pts)
        rospy.loginfo("Simple FM2 navigator planned a path; trigger=%s", trigger)

    def _stop(self) -> None:
        """Publish a zero-velocity command."""
        self.pub_cmd.publish(Twist())

    def _track_target(
        self, x: float, y: float, yaw: float, target: Tuple[float, float]
    ) -> None:
        """Publish a velocity command toward a path target."""
        tx, ty = target
        dx = tx - x
        dy = ty - y
        ang_ref = math.atan2(dy, dx)
        e_yaw = self._wrap_to_pi(ang_ref - yaw)

        fact = max(
            self.min_linear_speed_factor,
            1.0
            - min(
                abs(e_yaw) / self.turn_slowdown_angle,
                self.max_turn_speed_reduction,
            ),
        )
        v = self.v_lin * fact
        w = float(np.clip(self.k_theta * e_yaw, -self.v_ang_max, self.v_ang_max))

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)

    def _is_off_path(self, x: float, y: float) -> bool:
        """Return whether the robot has deviated from the active path."""
        window_end = min(
            self.path_idx + self.offpath_window_points, len(self.path_world)
        )
        window = self.path_world[self.path_idx : window_end]
        if not window:
            return False
        distance = min(np.hypot(px - x, py - y) for px, py in window)
        return distance > self.replan_offpath

    @synchronized
    def _control_step(self) -> None:
        """Execute one path-tracking iteration."""
        if self.path_world is None or self.last_pose is None or self.goal_world is None:
            return

        x, y, yaw = self.last_pose

        if self._is_off_path(x, y):
            self._plan(trigger="offpath")
            return

        target = None
        for i in range(self.path_idx, len(self.path_world)):
            tx, ty = self.path_world[i]
            if np.hypot(tx - x, ty - y) >= self.lookahead_dist:
                target = (tx, ty)
                self.path_idx = i
                break

        if self.path_idx < len(self.path_world):
            px, py = self.path_world[self.path_idx]
            if np.hypot(px - x, py - y) < self.path_point_tolerance:
                self.path_idx = min(self.path_idx + 1, len(self.path_world) - 1)

        if target is None:
            goal_x, goal_y = self.path_world[-1]
            if np.hypot(goal_x - x, goal_y - y) < self.goal_tolerance:
                self._stop()
                self.path_world = None
                self.goal_world = None
                return
            else:
                target = (goal_x, goal_y)

        self._track_target(x, y, yaw, target)

    def spin(self) -> None:
        """Run replanning and control until ROS shuts down."""
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (
                self.goal_world is not None
                and (now - self.last_replan_time).to_sec() > self.replan_period
            ):
                self.last_replan_time = now
                self._plan(trigger="timer")
            self._control_step()
            rate.sleep()


if __name__ == "__main__":
    node = FM2TestPlanner()
    rospy.loginfo("Simple FM2 navigator started; waiting for map data")
    node.spin()
