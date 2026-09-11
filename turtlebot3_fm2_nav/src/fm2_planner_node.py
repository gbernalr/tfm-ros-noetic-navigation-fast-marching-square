#!/usr/bin/env python3
"""Plan collision-free FM2 paths over the combined occupancy grid.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math
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


class FM2Planner:
    """ROS node responsible for global FM2 planning and periodic replanning."""

    def __init__(self) -> None:
        """Read configuration and initialize planner state and ROS interfaces."""
        # Coordinate frames
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")

        # Planning map
        self.inflation = int(rospy.get_param("~inflate", 0))

        # Replanning policy
        self.replan_offpath = float(rospy.get_param("~replan_offpath", 0.6))
        self.replan_period = float(rospy.get_param("~replan_period", 1.0))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        # Map state
        self.grid_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None

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

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        """Return the planar yaw represented by a quaternion-like object."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert map-frame coordinates into occupancy-grid indices."""
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        return ix, iy

    def _grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Return the world coordinates of the center of a grid cell."""
        x = self.map_ox + (ix + 0.5) * self.map_res
        y = self.map_oy + (iy + 0.5) * self.map_res
        return x, y

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

    def cb_map(self, msg: OccupancyGrid) -> None:
        """Convert the latest combined costmap into a binary FM2 grid."""
        w = msg.info.width
        h = msg.info.height
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ = data >= 50
        unk = data < 0
        obs = np.logical_or(occ, unk).astype(np.uint8)

        self.grid_bin = (1 - obs).astype(np.uint8)

    def cb_goal(self, msg: PoseStamped) -> None:
        """Store a new navigation goal in the configured map frame."""
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

        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(
            "FM2 planner received a new goal at (%.3f, %.3f)",
            self.goal_world[0],
            self.goal_world[1],
        )

        self.path_world = None
        self.last_replan_time = rospy.Time(0)

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
                rospy.logwarn("FM2 planner could not transform the AMCL pose: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)

        self.last_pose = (x, y, yaw)

    # --------------------------- Planning logic ---------------------------

    def _update_pose_from_tf(self) -> bool:
        """Update the robot pose from TF when no AMCL pose is available."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_map,
                self.frame_base,
                rospy.Time(0),
                rospy.Duration(0.5),
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
        self.last_pose = (position.x, position.y, yaw)
        return True

    def _plan_from_current_pose(self) -> None:
        """Compute and publish a path from the current pose to the goal."""
        if self.grid_bin is None or self.goal_world is None:
            return

        if self.last_pose is None and not self._update_pose_from_tf():
            return

        sx, sy, _ = self.last_pose
        gx, gy = self.goal_world

        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy = self._world_to_grid(gx, gy)

        binary = self.grid_bin.copy().astype(np.uint8)

        if binary.size == 0:
            return

        if self.inflation > 0:
            k = 2 * self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        try:
            self.fm2 = FM2(mode="cpu")
            fm2_map = FM2Map.from_binary_map(binary, create_border=True)
            self.fm2.set_map(fm2_map)
        except (TypeError, ValueError, RuntimeError) as e:
            rospy.logwarn("FM2 planner failed to configure the FM2 map: %s", e)
            self.path_world = None
            return

        try:
            info = self.fm2.get_path(
                (int(start_iy), int(start_ix)),
                (int(goal_iy), int(goal_ix)),
            )
        except IndexError as e:
            rospy.logwarn("FM2 planner received invalid path indices: %s", e)
            self.path_world = None
            return
        except (TypeError, ValueError, RuntimeError) as e:
            rospy.logwarn("FM2 planner failed while computing a path: %s", e)
            self.path_world = None
            return

        if info.path is None:
            rospy.logwarn("FM2 planner could not find a path")
            self.path_world = None
            return

        rows, cols = info.path
        # ROS Noetic uses Python 3.8, which does not support zip(strict=...).
        pts = [
            self._grid_to_world(int(col), int(row))
            for row, col in zip(rows, cols)  # noqa: B905
        ]

        # Validate the generated path against the planning grid.
        result = self.check_pts_collisions(pts, binary=binary, radius_cells=0)
        if not result["all_free"]:
            rospy.logwarn(
                "FM2 planner rejected a colliding path: %d colliding points",
                result["n_collisions"],
            )
            self.path_world = None
            return

        self.path_world = pts
        self._publish_path(pts)
        rospy.loginfo("FM2 planner published a path with %d points", len(pts))

    def _planner_step(self) -> None:
        """Evaluate periodic and off-path replanning conditions."""
        if self.grid_bin is None or self.goal_world is None or self.last_pose is None:
            return

        now = rospy.Time.now()
        need_replan = False

        if self.path_world is None:
            need_replan = True

        if (
            not need_replan
            and self.replan_period > 0.0
            and (now - self.last_replan_time).to_sec() >= self.replan_period
        ):
            need_replan = True

        if not need_replan and self.path_world and self.replan_offpath > 0.0:
            x, y, _ = self.last_pose
            dmin = min(np.hypot(px - x, py - y) for (px, py) in self.path_world)
            if dmin > self.replan_offpath:
                rospy.loginfo(
                    "FM2 planner detected an off-path robot (%.3f m); replanning",
                    dmin,
                )
                need_replan = True

        if need_replan:
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
