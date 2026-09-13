#!/usr/bin/env python3
"""Track FM2 paths and publish velocity commands for a differential-drive robot.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math
from threading import RLock
from typing import Tuple

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Path

from nav_validation import (
    require_bool,
    require_finite_xy,
    require_float,
    require_int,
    require_nonempty_string,
)
from navigation_utils import quaternion_yaw, synchronized, wrap_to_pi


class FM2Controller:
    """ROS node that follows a global path and aligns with the goal heading."""

    def __init__(self) -> None:
        """Read configuration and initialize ROS interfaces and controller state."""
        self._state_lock = RLock()
        # Coordinate frames
        self.frame_map = require_nonempty_string(
            "~frame_map", rospy.get_param("~frame_map", "map")
        )
        self.frame_base = require_nonempty_string(
            "~frame_base", rospy.get_param("~frame_base", "base_link")
        )

        # Path-tracking parameters
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
        # Stop translating when the target is too far to the side. Combining a
        # low linear velocity with maximum rotation creates very small circles.
        self.heading_align_threshold = require_float(
            "~heading_align_threshold",
            rospy.get_param("~heading_align_threshold", 0.35),
            0.0,
            math.pi,
        )
        self.align_v_ang_max = require_float(
            "~align_v_ang_max",
            rospy.get_param("~align_v_ang_max", 0.8),
            0.0,
            minimum_inclusive=False,
        )
        self.goal_tolerance = require_float(
            "~goal_tolerance", rospy.get_param("~goal_tolerance", 0.08), 0.0
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
        self.tf_timeout = rospy.Duration(
            require_float(
                "~tf_timeout",
                rospy.get_param("~tf_timeout", 0.5),
                0.0,
                minimum_inclusive=False,
            )
        )
        self.pose_timeout = require_float(
            "~pose_timeout", rospy.get_param("~pose_timeout", 0.5), 0.0
        )
        self.path_timeout = require_float(
            "~path_timeout", rospy.get_param("~path_timeout", 1.5), 0.0
        )

        # Final orientation
        self.k_theta = require_float("~k_theta", rospy.get_param("~k_theta", 2.0), 0.0)
        self.goal_yaw_tolerance = require_float(
            "~goal_yaw_tolerance",
            rospy.get_param("~goal_yaw_tolerance", 0.10),
            0.0,
            math.pi,
        )
        self.use_goal_yaw = require_bool(
            "~use_goal_yaw", rospy.get_param("~use_goal_yaw", True)
        )

        # Runtime state
        self.path_world = None  # List of (x, y) points in the map frame.
        self.path_idx = 0
        self.mode_align = False

        self.goal_yaw = None  # Desired goal yaw.

        self.last_pose = None  # Latest (x, y, yaw) pose.
        self.last_pose_time = None
        self.last_path_time = None

        # TF support for poses that are not expressed in frame_map.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS I/O
        self.sub_path = rospy.Subscriber("fm2_path", Path, self.cb_path, queue_size=1)
        self.sub_goal = rospy.Subscriber(
            "move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1
        )
        self.sub_amcl = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1
        )
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        rospy.on_shutdown(self._stop)

        rospy.loginfo("FM2 controller initialized")

    # ------------------------- Utility methods -------------------------

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        """Return the planar yaw represented by a quaternion-like object."""
        return quaternion_yaw(q)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Normalize an angle to the half-open interval [-pi, pi)."""
        return wrap_to_pi(angle)

    def _transform_pose(self, pose_stamped: PoseStamped, to_frame: str) -> PoseStamped:
        """Transform a stamped pose into the requested frame."""
        return tf2_geometry_msgs.do_transform_pose(
            pose_stamped,
            self.tf_buffer.lookup_transform(
                to_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),
                self.tf_timeout,
            ),
        )

    def _stop(self) -> None:
        """Publish a zero-velocity command."""
        self.pub_cmd.publish(Twist())

    def _update_pose_from_tf(self) -> bool:
        """Refresh the robot pose from the latest map-to-base TF transform.

        AMCL normally publishes this pose, but it may publish only an initial
        estimate in simulation.  TF is still updated by the localization and
        robot-state chains, so it is a safe fallback for the watchdog.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_map,
                self.frame_base,
                rospy.Time(0),
                self.tf_timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(
                2.0,
                "FM2 controller could not refresh pose from TF %s <- %s: %s",
                self.frame_map,
                self.frame_base,
                error,
            )
            return False

        translation = transform.transform.translation
        self.last_pose = (
            translation.x,
            translation.y,
            self._yaw_from_quat(transform.transform.rotation),
        )
        # Use receive time, rather than the source-transform timestamp: this
        # value is exclusively the watchdog's freshness indicator.
        self.last_pose_time = rospy.Time.now()
        return True

    # --------------------------- ROS callbacks ---------------------------

    @synchronized
    def cb_path(self, msg: Path) -> None:
        """Store a newly planned path and resume tracking near the robot."""
        if msg.poses and msg.header.frame_id != self.frame_map:
            rospy.logwarn(
                "FM2 controller rejected a path in frame %r; expected %r",
                msg.header.frame_id,
                self.frame_map,
            )
            self.path_world = None
            self.path_idx = 0
            self.last_path_time = None
            self._stop()
            return
        # Convert the ROS Path into a list of (x, y) points.
        pts = []
        for ps in msg.poses:
            x, y = ps.pose.position.x, ps.pose.position.y
            if not math.isfinite(x) or not math.isfinite(y):
                rospy.logwarn("FM2 controller rejected a path with non-finite points")
                self.path_world = None
                self.path_idx = 0
                self.last_path_time = None
                self._stop()
                return
            pts.append((x, y))

        if pts:
            self.path_world = pts
            self.last_path_time = rospy.Time.now()
            # The planner updates frequently. Starting at index zero after each
            # update would send the robot backwards; resume at the closest point.
            if self.last_pose is not None:
                x, y, _ = self.last_pose
                self.path_idx = int(
                    np.argmin([np.hypot(px - x, py - y) for px, py in pts])
                )
            else:
                self.path_idx = 0
            self.mode_align = False
            rospy.loginfo("FM2 controller received a path with %d points", len(pts))
        else:
            rospy.logwarn("FM2 controller received an empty path")
            self.path_world = None
            self.path_idx = 0
            self.last_path_time = None
            self._stop()

    @synchronized
    def cb_goal(self, msg: PoseStamped) -> None:
        """Store the requested final orientation from a navigation goal."""
        if not msg.header.frame_id:
            rospy.logwarn("FM2 controller rejected a goal without frame_id")
            return
        # The controller only uses the goal yaw during final alignment.
        if msg.header.frame_id != self.frame_map:
            try:
                msg = self._transform_pose(msg, self.frame_map)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn("FM2 controller could not transform the goal: %s", e)
                return

        try:
            require_finite_xy("goal", msg.pose.position.x, msg.pose.position.y)
        except ValueError as error:
            rospy.logwarn("FM2 controller rejected an invalid goal: %s", error)
            return

        if self.use_goal_yaw:
            self.goal_yaw = self._yaw_from_quat(msg.pose.orientation)
        else:
            self.goal_yaw = None

        self.mode_align = False
        self.path_world = None
        self.path_idx = 0
        self.last_path_time = None
        self._stop()

    @synchronized
    def cb_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        """Update the robot pose from AMCL, transforming it when required."""
        if not msg.header.frame_id:
            rospy.logwarn_throttle(
                2.0, "FM2 controller rejected an AMCL pose without frame_id"
            )
            return
        if msg.header.frame_id != self.frame_map:
            try:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = msg.pose.pose
                pose = self._transform_pose(pose, self.frame_map)
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self._yaw_from_quat(pose.pose.orientation)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn("FM2 controller could not transform the AMCL pose: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            rospy.logwarn_throttle(
                2.0, "FM2 controller rejected a non-finite AMCL pose"
            )
            return
        self.last_pose = (x, y, yaw)
        self.last_pose_time = rospy.Time.now()

    # --------------------------- Control logic ---------------------------

    def _track_target(
        self, x: float, y: float, yaw: float, target: Tuple[float, float]
    ) -> None:
        """Publish a velocity command that drives the robot toward a target."""
        tx, ty = target
        dx = tx - x
        dy = ty - y
        ang_ref = math.atan2(dy, dx)
        e_yaw = self._wrap_to_pi(ang_ref - yaw)

        if abs(e_yaw) > self.heading_align_threshold:
            # The target is too far to the side; orient the robot before moving.
            # The lower angular limit reduces heading overshoot.
            v = 0.0
            w = float(
                np.clip(
                    self.k_theta * e_yaw,
                    -self.align_v_ang_max,
                    self.align_v_ang_max,
                )
            )
        else:
            # Once aligned, reduce speed progressively through turns without
            # turning every tight curve into an in-place rotation.
            fact = max(
                self.min_linear_speed_factor,
                1.0
                - min(
                    abs(e_yaw) / self.turn_slowdown_angle,
                    self.max_turn_speed_reduction,
                ),
            )
            v = self.v_lin * fact
            w = float(
                np.clip(
                    self.k_theta * e_yaw,
                    -self.v_ang_max,
                    self.v_ang_max,
                )
            )

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)

    def _align_to_goal(self) -> None:
        """Rotate in place until the requested final heading is reached."""
        if self.last_pose is None:
            return

        _, _, yaw = self.last_pose
        if self.goal_yaw is None:
            self._stop()
            self.path_world = None
            self.mode_align = False
            return

        e_yaw = self._wrap_to_pi(self.goal_yaw - yaw)
        if abs(e_yaw) < self.goal_yaw_tolerance:
            self._stop()
            self.path_world = None
            self.goal_yaw = None
            self.mode_align = False
            return

        twist = Twist()
        twist.angular.z = float(
            np.clip(self.k_theta * e_yaw, -self.v_ang_max, self.v_ang_max)
        )
        self.pub_cmd.publish(twist)

    def _watchdog_allows_control(self, now: rospy.Time) -> bool:
        """Stop safely when the robot pose or active route is unavailable."""
        pose_missing = self.last_pose is None or self.last_pose_time is None
        pose_stale = (
            self.pose_timeout > 0.0
            and self.last_pose_time is not None
            and (now - self.last_pose_time).to_sec() > self.pose_timeout
        )
        if pose_missing or pose_stale:
            if self._update_pose_from_tf():
                # A current map-to-base transform is an equally valid pose
                # source. Continue controlling with this refreshed snapshot.
                pass
            else:
                reason = "unavailable" if pose_missing else "stale"
                rospy.logwarn_throttle(
                    2.0,
                    "FM2 controller stopped because the robot pose is %s",
                    reason,
                )
                self._stop()
                return False

        if self.last_pose is None:
            rospy.logwarn_throttle(
                2.0, "FM2 controller stopped because the robot pose is unavailable"
            )
            self._stop()
            return False
        if not self.path_world or self.last_path_time is None:
            self._stop()
            return False
        if (
            self.path_timeout > 0.0
            and (now - self.last_path_time).to_sec() > self.path_timeout
        ):
            rospy.logwarn_throttle(
                2.0, "FM2 controller stopped because the active path is stale"
            )
            self.path_world = None
            self.path_idx = 0
            self.last_path_time = None
            self._stop()
            return False
        return True

    @synchronized
    def _control_step(self) -> None:
        """Execute one iteration of the path-following state machine."""
        now = rospy.Time.now()
        if not self._watchdog_allows_control(now):
            return

        if self.mode_align:
            self._align_to_goal()
            return

        x, y, yaw = self.last_pose

        target = None
        n_pts = len(self.path_world)

        if 0 <= self.path_idx < n_pts:
            px, py = self.path_world[self.path_idx]
            if np.hypot(px - x, py - y) < self.path_point_tolerance:
                self.path_idx = min(self.path_idx + 1, n_pts - 1)

        for i in range(self.path_idx, n_pts):
            tx, ty = self.path_world[i]
            if np.hypot(tx - x, ty - y) >= self.lookahead_dist:
                target = (tx, ty)
                self.path_idx = i
                break

        if target is None:
            goal_x, goal_y = self.path_world[-1]
            dist_goal = np.hypot(goal_x - x, goal_y - y)

            if dist_goal < self.goal_tolerance:
                if self.use_goal_yaw and self.goal_yaw is not None:
                    self.mode_align = True
                    self._stop()
                else:
                    self._stop()
                    self.path_world = None
                return
            else:
                target = (goal_x, goal_y)

        self._track_target(x, y, yaw, target)

    def spin(self) -> None:
        """Run the controller loop until ROS shuts down."""
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("FM2 controller ready; tracking fm2_path")
        while not rospy.is_shutdown():
            self._control_step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fm2_controller")
    node = FM2Controller()
    node.spin()
