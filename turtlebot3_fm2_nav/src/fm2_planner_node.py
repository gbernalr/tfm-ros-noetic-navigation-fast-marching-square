#!/usr/bin/env python3
"""Plan collision-free FM2 paths over the combined occupancy grid.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math
from dataclasses import dataclass
from threading import RLock
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from fm2 import FM2
from fm2.entities import FM2Map
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path

from nav_validation import (
    require_finite_xy,
    require_float,
    require_int,
    require_message_frame,
    require_nonempty_string,
    require_occupancy_grid,
)
from navigation_utils import (
    GridGeometry,
    grid_to_world,
    quaternion_yaw,
    synchronized,
    world_to_grid,
)


@dataclass(frozen=True)
class PlanningSnapshot:
    """Immutable planner inputs captured from one consistent ROS state."""

    grid_bin: np.ndarray
    goal_world: Tuple[float, float]
    last_pose: Tuple[float, float, float]
    map_res: float
    map_ox: float
    map_oy: float
    map_origin_yaw: float


class FM2Planner:
    """ROS node responsible for global FM2 planning and periodic replanning."""

    def __init__(self) -> None:
        """Read configuration and initialize planner state and ROS interfaces."""
        self._state_lock = RLock()
        # Coordinate frames
        self.frame_map = require_nonempty_string(
            "~frame_map", rospy.get_param("~frame_map", "map")
        )
        self.frame_base = require_nonempty_string(
            "~frame_base", rospy.get_param("~frame_base", "base_link")
        )

        # Planning map
        self.inflation = require_int("~inflate", rospy.get_param("~inflate", 0), 0)
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

        # Replanning policy
        self.replan_offpath = require_float(
            "~replan_offpath", rospy.get_param("~replan_offpath", 0.6), 0.0
        )
        self.replan_period = require_float(
            "~replan_period", rospy.get_param("~replan_period", 1.0), 0.0
        )
        self.rate_hz = require_int("~rate", rospy.get_param("~rate", 20), 1)

        # Map state
        self.grid_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.map_origin_yaw = None

        # Planning state
        self.goal_world = None  # Goal (x, y) in the map frame.
        self.path_world = None  # Path points in the map frame.
        self.last_replan_time = rospy.Time.now()

        # Robot pose
        self.last_pose = None  # (x, y, yaw)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS I/O
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

        rospy.loginfo("FM2 planner initialized")

    # ---------------- Collision and coordinate utilities ----------------

    @synchronized
    def check_pts_collisions(
        self,
        pts_world: List[Tuple[float, float]],
        binary: Optional[np.ndarray] = None,
        radius_cells: int = 0,
    ) -> Dict[str, object]:
        """Check path points against a binary traversability grid."""
        if binary is None:
            if self.grid_bin is None:
                return {
                    "n": 0,
                    "collisions": [],
                    "n_collisions": 0,
                    "all_free": True,
                }
            binary = self.grid_bin

        h, w = binary.shape
        collisions = []

        for i, (x, y) in enumerate(pts_world):
            ix, iy = self._world_to_grid(x, y)

            if not (0 <= ix < w and 0 <= iy < h):
                collisions.append((i, (x, y), (ix, iy)))
                continue

            hit = False
            if radius_cells <= 0:
                if binary[iy, ix] == 0:
                    hit = True
            else:
                x0 = max(0, ix - radius_cells)
                x1 = min(w - 1, ix + radius_cells)
                y0 = max(0, iy - radius_cells)
                y1 = min(h - 1, iy + radius_cells)
                if (binary[y0 : y1 + 1, x0 : x1 + 1] == 0).any():
                    hit = True

            if hit:
                collisions.append((i, (x, y), (ix, iy)))

        result = {
            "n": len(pts_world),
            "collisions": collisions,
            "n_collisions": len(collisions),
            "all_free": (len(collisions) == 0),
        }

        return result

    def _clear_path_for_goal(self, goal_world: Tuple[float, float]) -> None:
        """Cancel a failed route without affecting a newer navigation goal."""
        with self._state_lock:
            if self.goal_world == goal_world:
                had_path = self.path_world is not None
                self.path_world = None
                if had_path:
                    self._publish_path([])

    @staticmethod
    def _validate_endpoint(
        label: str,
        grid_point: Tuple[int, int],
        world_point: Tuple[float, float],
        binary: np.ndarray,
    ) -> bool:
        """Validate that a planning endpoint is in bounds and traversable."""
        ix, iy = grid_point
        world_x, world_y = world_point
        height, width = binary.shape
        if not (0 <= ix < width and 0 <= iy < height):
            rospy.logwarn(
                "FM2 planner rejected %s outside the costmap: "
                "world=(%.3f, %.3f), grid=(%d, %d)",
                label,
                world_x,
                world_y,
                ix,
                iy,
            )
            return False
        if binary[iy, ix] == 0:
            rospy.logwarn(
                "FM2 planner rejected %s in an occupied or unknown cell: grid=(%d, %d)",
                label,
                ix,
                iy,
            )
            return False
        return True

    @staticmethod
    def _path_is_collision_free(
        pts_world: List[Tuple[float, float]],
        binary: np.ndarray,
        snapshot: PlanningSnapshot,
    ) -> bool:
        """Return whether all path points lie in free cells of a map snapshot."""
        height, width = binary.shape
        cos_yaw = math.cos(snapshot.map_origin_yaw)
        sin_yaw = math.sin(snapshot.map_origin_yaw)
        for x, y in pts_world:
            dx, dy = x - snapshot.map_ox, y - snapshot.map_oy
            ix = math.floor((cos_yaw * dx + sin_yaw * dy) / snapshot.map_res)
            iy = math.floor((-sin_yaw * dx + cos_yaw * dy) / snapshot.map_res)
            if not (0 <= ix < width and 0 <= iy < height) or binary[iy, ix] == 0:
                return False
        return True

    @staticmethod
    def _solve_fm2_path(
        binary: np.ndarray,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[np.ndarray]:
        """Configure FM2 and return its row-column path for valid endpoints."""
        try:
            fm2 = FM2(mode="cpu")
            fm2_map = FM2Map.from_binary_map(binary, create_border=True)
            fm2.set_map(fm2_map)
            info = fm2.get_path(start, goal)
        except IndexError as error:
            rospy.logwarn("FM2 planner received invalid path indices: %s", error)
            return None
        except (TypeError, ValueError, RuntimeError) as error:
            rospy.logwarn("FM2 planner failed while computing a path: %s", error)
            return None

        if info.path is None:
            rospy.logwarn("FM2 planner could not find a path")
            return None
        return info.path

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        """Return the planar yaw represented by a quaternion-like object."""
        return quaternion_yaw(q)

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

    # ---------------------------- ROS callbacks ----------------------------

    @synchronized
    def cb_map(self, msg: OccupancyGrid) -> None:
        """Convert the latest combined costmap into a binary FM2 grid."""
        try:
            require_occupancy_grid("costmap", msg)
            require_message_frame("costmap", msg, self.frame_map)
        except ValueError as error:
            rospy.logwarn("FM2 planner rejected an invalid costmap: %s", error)
            return
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

        grid_bin = (1 - obs).astype(np.uint8)
        grid_bin.setflags(write=False)
        self.grid_bin = grid_bin

    @synchronized
    def cb_goal(self, msg: PoseStamped) -> None:
        """Store a new navigation goal in the configured map frame."""
        if not msg.header.frame_id:
            rospy.logwarn("FM2 planner rejected a goal without frame_id")
            return
        if msg.header.frame_id != self.frame_map:
            try:
                msg = self._transform_pose(msg, self.frame_map)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn("FM2 planner could not transform the goal: %s", e)
                return

        try:
            require_finite_xy("goal", msg.pose.position.x, msg.pose.position.y)
        except ValueError as error:
            rospy.logwarn("FM2 planner rejected an invalid goal: %s", error)
            return
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(
            "FM2 planner received a new goal at (%.3f, %.3f)",
            self.goal_world[0],
            self.goal_world[1],
        )

        self.path_world = None
        self.last_replan_time = rospy.Time(0)
        self._publish_path([])

    @synchronized
    def cb_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        """Update the robot pose from AMCL, transforming it when required."""
        if not msg.header.frame_id:
            rospy.logwarn_throttle(
                2.0, "FM2 planner rejected an AMCL pose without frame_id"
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
                rospy.logwarn("FM2 planner could not transform the AMCL pose: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            rospy.logwarn_throttle(2.0, "FM2 planner rejected a non-finite AMCL pose")
            return
        self.last_pose = (x, y, yaw)

    # --------------------------- Planning logic ---------------------------

    def _update_pose_from_tf(self) -> bool:
        """Update the robot pose from TF when no AMCL pose is available."""
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
            rospy.logwarn(
                "FM2 planner could not obtain the robot pose from TF: %s", error
            )
            return False

        position = transform.transform.translation
        yaw = self._yaw_from_quat(transform.transform.rotation)
        with self._state_lock:
            self.last_pose = (position.x, position.y, yaw)
        return True

    def _planning_snapshot(self) -> Optional[PlanningSnapshot]:
        """Capture a valid, immutable set of inputs for one planning attempt."""
        with self._state_lock:
            grid_bin = self.grid_bin
            goal_world = self.goal_world
            last_pose = self.last_pose
            map_res = self.map_res
            map_ox = self.map_ox
            map_oy = self.map_oy
            map_origin_yaw = self.map_origin_yaw

        if grid_bin is None or goal_world is None:
            return None
        if map_res is None or map_res <= 0.0:
            rospy.logwarn("FM2 planner received an invalid costmap resolution")
            self._clear_path_for_goal(goal_world)
            return None
        if last_pose is None:
            if not self._update_pose_from_tf():
                return None
            with self._state_lock:
                last_pose = self.last_pose
            if last_pose is None:
                return None

        return PlanningSnapshot(
            grid_bin=grid_bin,
            goal_world=goal_world,
            last_pose=last_pose,
            map_res=map_res,
            map_ox=map_ox,
            map_oy=map_oy,
            map_origin_yaw=map_origin_yaw,
        )

    def _plan_from_current_pose(self) -> None:
        """Compute and publish a path from the current pose to the goal."""
        snapshot = self._planning_snapshot()
        if snapshot is None:
            return

        grid_bin = snapshot.grid_bin
        goal_world = snapshot.goal_world
        map_res = snapshot.map_res
        map_ox = snapshot.map_ox
        map_oy = snapshot.map_oy
        map_origin_yaw = snapshot.map_origin_yaw
        sx, sy, _ = snapshot.last_pose
        gx, gy = goal_world

        cos_yaw = math.cos(map_origin_yaw)
        sin_yaw = math.sin(map_origin_yaw)
        start_dx, start_dy = sx - map_ox, sy - map_oy
        goal_dx, goal_dy = gx - map_ox, gy - map_oy
        start_ix = math.floor((cos_yaw * start_dx + sin_yaw * start_dy) / map_res)
        start_iy = math.floor((-sin_yaw * start_dx + cos_yaw * start_dy) / map_res)
        goal_ix = math.floor((cos_yaw * goal_dx + sin_yaw * goal_dy) / map_res)
        goal_iy = math.floor((-sin_yaw * goal_dx + cos_yaw * goal_dy) / map_res)

        binary = grid_bin.copy()

        if binary.size == 0:
            return

        if self.inflation > 0:
            k = 2 * self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        if not self._validate_endpoint("start", (start_ix, start_iy), (sx, sy), binary):
            self._clear_path_for_goal(goal_world)
            return
        if not self._validate_endpoint("goal", (goal_ix, goal_iy), (gx, gy), binary):
            self._clear_path_for_goal(goal_world)
            return

        path = self._solve_fm2_path(binary, (start_iy, start_ix), (goal_iy, goal_ix))
        if path is None:
            self._clear_path_for_goal(goal_world)
            return

        rows, cols = path
        # ROS Noetic uses Python 3.8, which does not support zip(strict=...).
        pts = [
            (
                map_ox
                + cos_yaw * (int(col) + 0.5) * map_res
                - sin_yaw * (int(row) + 0.5) * map_res,
                map_oy
                + sin_yaw * (int(col) + 0.5) * map_res
                + cos_yaw * (int(row) + 0.5) * map_res,
            )
            for row, col in zip(rows, cols)  # noqa: B905
        ]

        # Validate the generated path against the planning grid.
        if not self._path_is_collision_free(pts, binary, snapshot):
            rospy.logwarn("FM2 planner rejected a colliding path")
            self._clear_path_for_goal(goal_world)
            return

        with self._state_lock:
            # Costmaps update at scan frequency and can change while FM2 is
            # solving. The next periodic cycle will re-evaluate the route, but
            # a newly received goal must never publish a path for its predecessor.
            if self.goal_world != goal_world:
                rospy.loginfo("FM2 planner discarded an obsolete planning result")
                return
            self.path_world = tuple(pts)
        self._publish_path(pts)
        rospy.loginfo("FM2 planner published a path with %d points", len(pts))

    def _planner_step(self) -> None:
        """Evaluate periodic and off-path replanning conditions."""
        with self._state_lock:
            grid_bin = self.grid_bin
            goal_world = self.goal_world
            last_pose = self.last_pose
            path_world = self.path_world
            last_replan_time = self.last_replan_time

        if grid_bin is None or goal_world is None:
            return

        if last_pose is None:
            if not self._update_pose_from_tf():
                return
            with self._state_lock:
                last_pose = self.last_pose
            if last_pose is None:
                return

        now = rospy.Time.now()
        need_replan = False

        if path_world is None:
            need_replan = True

        if (
            not need_replan
            and self.replan_period > 0.0
            and (now - last_replan_time).to_sec() >= self.replan_period
        ):
            need_replan = True

        if not need_replan and path_world and self.replan_offpath > 0.0:
            x, y, _ = last_pose
            dmin = min(np.hypot(px - x, py - y) for (px, py) in path_world)
            if dmin > self.replan_offpath:
                rospy.loginfo(
                    "FM2 planner detected an off-path robot (%.3f m); replanning",
                    dmin,
                )
                need_replan = True

        if need_replan:
            with self._state_lock:
                self.last_replan_time = now
            self._plan_from_current_pose()

    def spin(self) -> None:
        """Run the planner loop until ROS shuts down."""
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("FM2 planner ready; publishing fm2_path")
        while not rospy.is_shutdown():
            self._planner_step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fm2_planner")
    node = FM2Planner()
    node.spin()
