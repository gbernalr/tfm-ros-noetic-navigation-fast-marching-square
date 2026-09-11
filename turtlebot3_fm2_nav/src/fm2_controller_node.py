#!/usr/bin/env python3
"""Track FM2 paths and publish velocity commands for a differential-drive robot.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math
from functools import wraps
from threading import RLock
from typing import Callable, Tuple

import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Path


def _synchronized(method: Callable[..., object]) -> Callable[..., object]:
    """Serialize updates and reads of controller state across ROS threads."""

    @wraps(method)
    def wrapped(*args: object, **kwargs: object) -> object:
        self = args[0]
        with self._state_lock:
            return method(*args, **kwargs)

    return wrapped


class FM2Controller:
    """ROS node that follows a global path and aligns with the goal heading."""

    def __init__(self) -> None:
        """Read configuration and initialize ROS interfaces and controller state."""
        self._state_lock = RLock()
        # Coordinate frames
        self.frame_map = rospy.get_param("~frame_map", "map")

        # Path-tracking parameters
        self.lookahead_dist = float(rospy.get_param("~lookahead", 0.35))
        self.v_lin = float(rospy.get_param("~v_lin", 0.22))
        self.v_ang_max = float(rospy.get_param("~v_ang_max", 1.5))
        # Stop translating when the target is too far to the side. Combining a
        # low linear velocity with maximum rotation creates very small circles.
        self.heading_align_threshold = float(
            rospy.get_param("~heading_align_threshold", 0.35)
        )
        self.align_v_ang_max = float(rospy.get_param("~align_v_ang_max", 0.8))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.08))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        # Final orientation
        self.k_theta = float(rospy.get_param("~k_theta", 2.0))
        self.goal_yaw_tolerance = float(rospy.get_param("~goal_yaw_tolerance", 0.10))
        self.use_goal_yaw = bool(rospy.get_param("~use_goal_yaw", True))

        # Runtime state
        self.path_world = None  # List of (x, y) points in the map frame.
        self.path_idx = 0
        self.mode_align = False

        self.goal_yaw = None  # Desired goal yaw.

        self.last_pose = None  # Latest (x, y, yaw) pose.

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

        rospy.loginfo("FM2 controller initialized")

    # ------------------------- Utility methods -------------------------

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        """Return the planar yaw represented by a quaternion-like object."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        """Normalize an angle to the half-open interval [-pi, pi)."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _transform_pose(self, pose_stamped: PoseStamped, to_frame: str) -> PoseStamped:
        """Transform a stamped pose into the requested frame."""
        return tf2_geometry_msgs.do_transform_pose(
            pose_stamped,
            self.tf_buffer.lookup_transform(
                to_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.5),
            ),
        )

    def _stop(self) -> None:
        """Publish a zero-velocity command."""
        self.pub_cmd.publish(Twist())

    # --------------------------- ROS callbacks ---------------------------

    @_synchronized
    def cb_path(self, msg: Path) -> None:
        """Store a newly planned path and resume tracking near the robot."""
        # Convert the ROS Path into a list of (x, y) points.
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))

        if pts:
            self.path_world = pts
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

    @_synchronized
    def cb_goal(self, msg: PoseStamped) -> None:
        """Store the requested final orientation from a navigation goal."""
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

        if self.use_goal_yaw:
            self.goal_yaw = self._yaw_from_quat(msg.pose.orientation)
        else:
            self.goal_yaw = None

        self.mode_align = False

    @_synchronized
    def cb_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        """Update the robot pose from AMCL, transforming it when required."""
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

        self.last_pose = (x, y, yaw)

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
            fact = max(0.2, 1.0 - min(abs(e_yaw) / 1.2, 0.8))
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

    @_synchronized
    def _control_step(self) -> None:
        """Execute one iteration of the path-following state machine."""
        if self.mode_align:
            self._align_to_goal()
            return

        if not self.path_world or self.last_pose is None:
            return

        x, y, yaw = self.last_pose

        target = None
        n_pts = len(self.path_world)

        if 0 <= self.path_idx < n_pts:
            px, py = self.path_world[self.path_idx]
            if np.hypot(px - x, py - y) < 0.25:
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
