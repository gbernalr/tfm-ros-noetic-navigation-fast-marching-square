#!/usr/bin/env python3
"""Fuse static, laser, and person data into an FM2 occupancy grid.

ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import math
from functools import wraps
from threading import RLock
from typing import Callable, Optional, Tuple

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rgbd_person_tracker.msg import PersonPredictionArray, PersonTrack, PersonTrackArray
from sensor_msgs.msg import LaserScan


def _synchronized(method: Callable[..., object]) -> Callable[..., object]:
    """Serialize mutations and publications of the costmap layers."""

    @wraps(method)
    def wrapped(*args: object, **kwargs: object) -> object:
        self = args[0]
        with self._state_lock:
            return method(*args, **kwargs)

    return wrapped


class FM2CostmapNode:
    """ROS node that maintains and publishes the combined navigation costmap."""

    def __init__(self) -> None:
        """Read configuration and initialize costmap layers and ROS interfaces."""
        self._state_lock = RLock()
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.person_tracks_topic = rospy.get_param(
            "~person_tracks_topic", "/person_tracks"
        )

        self.obstacle_range = float(rospy.get_param("~obstacle_range", 2.5))
        self.min_range = float(rospy.get_param("~min_range", 0.05))
        self.occupancy_threshold = int(rospy.get_param("~occupancy_threshold", 50))
        self.tf_timeout = rospy.Duration(float(rospy.get_param("~tf_timeout", 0.1)))

        self.dynamic_inflate = int(rospy.get_param("~dynamic_inflate", 0))
        self.person_radius = float(rospy.get_param("~person_radius", 0.35))
        self.person_inflate = int(rospy.get_param("~person_inflate", 2))
        self.person_prediction_enabled = bool(
            rospy.get_param("~person_prediction_enabled", True)
        )
        self.person_predictions_topic = rospy.get_param(
            "~person_predictions_topic", "/person_predictions"
        )
        self.person_predictions_enabled = bool(
            rospy.get_param("~person_predictions_enabled", False)
        )
        self.prediction_sigma_multiplier = float(
            rospy.get_param("~prediction_sigma_multiplier", 1.0)
        )
        self.prediction_max_longitudinal_radius = float(
            rospy.get_param("~prediction_max_longitudinal_radius", 0.90)
        )
        self.prediction_max_lateral_radius = float(
            rospy.get_param("~prediction_max_lateral_radius", 0.55)
        )
        self.person_predictions_timeout = float(
            rospy.get_param("~person_predictions_timeout", 0.6)
        )
        self.person_use_confirmed_only = bool(
            rospy.get_param("~person_use_confirmed_only", True)
        )
        self.person_prediction_horizons = rospy.get_param(
            "~person_prediction_horizons", [0.5, 1.0, 1.5, 2.0]
        )
        self.person_tracks_timeout = float(
            rospy.get_param("~person_tracks_timeout", 0.6)
        )
        self.person_timeout_check_period = float(
            rospy.get_param("~person_timeout_check_period", 0.1)
        )
        self.person_max_speed_warn = float(
            rospy.get_param("~person_max_speed_warn", 1.5)
        )

        # Dynamic-obstacle lifetime measured in processed scans.
        self.dynamic_memory = int(rospy.get_param("~dynamic_memory", 15))

        # Static and dynamic grid state.
        self.static_grid = None  # np.array int8 (-1,0,100)
        self.map_res = None
        self.map_w = None
        self.map_h = None
        self.map_ox = None
        self.map_oy = None

        # dynamic_grid stores uint8 lifetime counters; values above zero denote
        # recently observed obstacles.
        self.dynamic_grid = None
        self.person_grid = None  # uint8 grid: 0 free, 100 occupied by a person.
        self.last_person_msg_time = None
        self.last_person_prediction_msg_time = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_map = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.cb_map, queue_size=1
        )
        self.sub_scan = rospy.Subscriber(
            self.scan_topic, LaserScan, self.cb_scan, queue_size=1
        )
        self.sub_persons = rospy.Subscriber(
            self.person_tracks_topic, PersonTrackArray, self.cb_persons, queue_size=1
        )
        self.sub_person_predictions = rospy.Subscriber(
            self.person_predictions_topic,
            PersonPredictionArray,
            self.cb_person_predictions,
            queue_size=1,
        )

        self.pub_costmap = rospy.Publisher(
            "fm2_costmap/costmap", OccupancyGrid, queue_size=1, latch=True
        )
        self.person_timeout_timer = rospy.Timer(
            rospy.Duration(self.person_timeout_check_period), self._person_timeout_cb
        )

        rospy.loginfo("FM2 costmap initialized; waiting for map, scan, and person data")

    # ---------------------------- ROS callbacks ----------------------------
    @_synchronized
    def cb_map(self, msg: OccupancyGrid) -> None:
        """Initialize the static layer from an occupancy-grid message."""
        # Store the static map as an int8 grid.
        self.map_res = msg.info.resolution
        self.map_w = msg.info.width
        self.map_h = msg.info.height
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(self.map_h, self.map_w)

        static_grid = np.full((self.map_h, self.map_w), -1, dtype=np.int8)

        # Values at or above the configured threshold are occupied, zero is
        # free, and all others are unknown.
        static_grid[data >= self.occupancy_threshold] = 100
        static_grid[data == 0] = 0

        self.static_grid = static_grid

        # Initialize the dynamic lifetime counters and the person layer.
        self.dynamic_grid = np.zeros_like(static_grid, dtype=np.uint8)
        self.person_grid = np.zeros_like(static_grid, dtype=np.uint8)

        free_ratio = float((self.static_grid == 0).sum()) / (self.map_w * self.map_h)
        occ_ratio = float((self.static_grid == 100).sum()) / (self.map_w * self.map_h)

        rospy.loginfo(
            "FM2 costmap loaded the static map: %.1f%% free, %.1f%% occupied",
            100.0 * free_ratio,
            100.0 * occ_ratio,
        )

        self.publish_costmap()

    @_synchronized
    def cb_persons(self, msg: PersonTrackArray) -> None:
        """Render tracks when no fresh external prediction is available."""
        if self.static_grid is None:
            return

        self.last_person_msg_time = rospy.Time.now()

        # Fresh predictions already contain current and future occupancy, so do
        # not overlay the constant-velocity fallback.
        if self.person_predictions_enabled and self._predictions_are_fresh():
            return

        if self.person_grid is None:
            self.person_grid = np.zeros((self.map_h, self.map_w), dtype=np.uint8)
        else:
            self.person_grid.fill(0)

        base_radius = max(1, int(math.ceil(self.person_radius / self.map_res)))
        total_radius = base_radius + max(self.person_inflate, 0)

        for track in msg.tracks:
            self._render_person_track(track, msg, total_radius)

        self.publish_costmap()

    def _render_person_track(
        self, track: PersonTrack, msg: PersonTrackArray, radius: int
    ) -> None:
        """Render one track and its optional constant-velocity projection."""
        if self.person_use_confirmed_only and not track.confirmed:
            return

        px, py, vx, vy = self._track_to_map(
            track, msg.header.frame_id, msg.header.stamp
        )
        if px is None:
            return

        speed = math.hypot(vx, vy)
        predicted_points = []
        self._paint_disc(px, py, radius)

        if self.person_prediction_enabled:
            for horizon in self.person_prediction_horizons:
                try:
                    prediction_time = float(horizon)
                except (TypeError, ValueError):
                    continue
                if prediction_time <= 0.0:
                    continue
                pred_x = px + vx * prediction_time
                pred_y = py + vy * prediction_time
                predicted_points.append((prediction_time, pred_x, pred_y))
                self._paint_disc(pred_x, pred_y, radius)

        rospy.loginfo_throttle(
            1.0,
            "Person track %d: position=(%.2f, %.2f), velocity="
            "(%.2f, %.2f), speed=%.2f m/s, predictions=%s",
            track.track_id,
            px,
            py,
            vx,
            vy,
            speed,
            [(round(t, 1), round(x, 2), round(y, 2)) for t, x, y in predicted_points],
        )

        if speed > self.person_max_speed_warn:
            rospy.logwarn(
                "Person track %d exceeds the configured speed threshold: "
                "speed=%.2f m/s, threshold=%.2f m/s, position=(%.2f, %.2f), "
                "farthest prediction=%s",
                track.track_id,
                speed,
                self.person_max_speed_warn,
                px,
                py,
                predicted_points[-1] if predicted_points else None,
            )

    @_synchronized
    def cb_person_predictions(self, msg: PersonPredictionArray) -> None:
        """Render current positions and uncertainty ellipses from predictions."""
        if self.static_grid is None or not self.person_predictions_enabled:
            return
        # The predictor emits an empty array before enough history is available;
        # retain the track-based fallback in that case.
        if not msg.predictions:
            return

        self.last_person_prediction_msg_time = rospy.Time.now()
        if self.person_grid is None:
            self.person_grid = np.zeros((self.map_h, self.map_w), dtype=np.uint8)
        else:
            self.person_grid.fill(0)

        current_radius = max(
            1, int(math.ceil(self.person_radius / self.map_res))
        ) + max(self.person_inflate, 0)
        rendered = 0
        for prediction in msg.predictions:
            n_points = min(
                len(prediction.positions),
                len(prediction.time_from_now),
                len(prediction.sigma_major),
                len(prediction.sigma_minor),
            )
            if n_points == 0:
                continue

            first_point = prediction.positions[0]
            px0, py0, vx, vy = self._xy_velocity_to_map(
                (float(first_point.x), float(first_point.y)),
                (float(prediction.velocity.x), float(prediction.velocity.y)),
                msg.header.frame_id,
                msg.header.stamp,
            )
            if px0 is None:
                continue

            t0 = max(0.0, float(prediction.time_from_now[0]))
            self._paint_disc(px0 - vx * t0, py0 - vy * t0, current_radius)
            heading = math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-3 else 0.0

            for index in range(n_points):
                point = prediction.positions[index]
                px, py, _, _ = self._xy_velocity_to_map(
                    (float(point.x), float(point.y)),
                    (0.0, 0.0),
                    msg.header.frame_id,
                    msg.header.stamp,
                )
                if px is None:
                    continue
                self._paint_prediction_ellipse(
                    px,
                    py,
                    heading,
                    max(0.0, float(prediction.sigma_major[index])),
                    max(0.0, float(prediction.sigma_minor[index])),
                )
            rendered += 1

        if rendered:
            rospy.loginfo_throttle(
                1.0,
                "FM2 costmap rendered prediction ellipses for %d tracks",
                rendered,
            )
        self.publish_costmap()

    def _predictions_are_fresh(self) -> bool:
        """Return whether the latest external prediction is still valid."""
        if self.last_person_prediction_msg_time is None:
            return False
        if self.person_predictions_timeout <= 0.0:
            return True
        return (
            rospy.Time.now() - self.last_person_prediction_msg_time
        ).to_sec() <= self.person_predictions_timeout

    @_synchronized
    def _person_timeout_cb(self, _event: object) -> None:
        """Clear stale person occupancy after the configured track timeout."""
        if self.static_grid is None or self.person_grid is None:
            return
        if self.last_person_msg_time is None:
            return
        if self.person_tracks_timeout <= 0.0:
            return

        age = (rospy.Time.now() - self.last_person_msg_time).to_sec()
        if age <= self.person_tracks_timeout:
            return

        if np.any(self.person_grid > 0):
            self.person_grid.fill(0)
            self.publish_costmap()
            rospy.loginfo_throttle(
                2.0,
                "FM2 costmap cleared stale person occupancy after %.2f s "
                "without person tracks",
                age,
            )

    @_synchronized
    def cb_scan(self, scan: LaserScan) -> None:
        """Update dynamic obstacle memory from a laser scan."""
        if self.static_grid is None:
            return

        # Transform laser-frame points into the map frame.
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.frame_map,
                scan.header.frame_id,
                rospy.Time(0),  # Use the latest available transform.
                self.tf_timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn_throttle(
                2.0,
                "FM2 costmap could not transform %s <- %s: %s",
                self.frame_map,
                scan.header.frame_id,
                e,
            )
            return

        # Extract planar yaw from the transform.
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        if self.dynamic_grid is None:
            self.dynamic_grid = np.zeros_like(self.static_grid, dtype=np.uint8)
        else:
            decay_mask = self.dynamic_grid > 0
            self.dynamic_grid[decay_mask] -= 1

        angle = scan.angle_min
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for r in scan.ranges:
            if not np.isfinite(r):
                angle += scan.angle_increment
                continue

            if r < self.min_range or r > self.obstacle_range:
                angle += scan.angle_increment
                continue

            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)

            x_m = tx + cos_yaw * x_l - sin_yaw * y_l
            y_m = ty + sin_yaw * x_l + cos_yaw * y_l

            ix, iy = self.world_to_grid(x_m, y_m)

            if 0 <= ix < self.map_w and 0 <= iy < self.map_h:
                self.dynamic_grid[iy, ix] = self.dynamic_memory

            angle += scan.angle_increment

        if self.dynamic_inflate > 0:
            try:
                import cv2

                k = 2 * self.dynamic_inflate + 1
                kernel = np.ones((k, k), np.uint8)
                dyn = (self.dynamic_grid > 0).astype(np.uint8)
                dyn = cv2.dilate(dyn, kernel, iterations=1)
                self.dynamic_grid[dyn == 1] = self.dynamic_memory
            except ImportError:
                rospy.logwarn_throttle(
                    10.0,
                    "OpenCV is unavailable; dynamic obstacle inflation is disabled",
                )

        dyn_count = int((self.dynamic_grid > 0).sum())
        rospy.loginfo_throttle(
            1.0,
            "FM2 costmap contains %d dynamic occupied cells",
            dyn_count,
        )

        self.publish_costmap()

    def _lookup_transform(
        self, source_frame: str, stamp: rospy.Time
    ) -> TransformStamped:
        """Look up a transform from a source frame into the map frame."""
        lookup_stamp = stamp if stamp != rospy.Time() else rospy.Time(0)
        return self.tf_buffer.lookup_transform(
            self.frame_map,
            source_frame,
            lookup_stamp,
            self.tf_timeout,
        )

    @staticmethod
    def _yaw_from_quat(q: Quaternion) -> float:
        """Return the planar yaw represented by a quaternion-like object."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _track_to_map(
        self, track: PersonTrack, source_frame: str, stamp: rospy.Time
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        """Transform a person's position and velocity into the map frame."""
        return self._xy_velocity_to_map(
            (float(track.position.x), float(track.position.y)),
            (float(track.velocity.x), float(track.velocity.y)),
            source_frame,
            stamp,
        )

    def _xy_velocity_to_map(
        self,
        position: Tuple[float, float],
        velocity: Tuple[float, float],
        source_frame: str,
        stamp: rospy.Time,
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        """Transform planar position and velocity vectors into the map frame."""
        px, py = position
        vx, vy = velocity
        if source_frame == self.frame_map:
            return px, py, vx, vy

        try:
            tf = self._lookup_transform(source_frame, stamp)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "FM2 costmap could not transform person data from %s to %s: %s",
                self.frame_map,
                source_frame,
                exc,
            )
            return None, None, None, None

        yaw = self._yaw_from_quat(tf.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        mx = tf.transform.translation.x + cos_yaw * px - sin_yaw * py
        my = tf.transform.translation.y + sin_yaw * px + cos_yaw * py
        mvx = cos_yaw * vx - sin_yaw * vy
        mvy = sin_yaw * vx + cos_yaw * vy
        return mx, my, mvx, mvy

    def _paint_disc(self, x: float, y: float, radius_cells: int) -> None:
        """Mark a circular footprint in the person layer."""
        ix, iy = self.world_to_grid(x, y)
        if radius_cells <= 0:
            radius_cells = 1

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                cx = ix + dx
                cy = iy + dy
                if 0 <= cx < self.map_w and 0 <= cy < self.map_h:
                    self.person_grid[cy, cx] = 100

    def _paint_prediction_ellipse(
        self,
        x: float,
        y: float,
        heading: float,
        sigma_major: float,
        sigma_minor: float,
    ) -> None:
        """Mark an uncertainty ellipse aligned with predicted motion."""
        base_radius = self.person_radius + max(self.person_inflate, 0) * self.map_res
        semi_major = min(
            self.prediction_max_longitudinal_radius,
            max(
                base_radius,
                base_radius + self.prediction_sigma_multiplier * sigma_major,
            ),
        )
        semi_minor = min(
            self.prediction_max_lateral_radius,
            max(
                base_radius,
                base_radius + self.prediction_sigma_multiplier * sigma_minor,
            ),
        )
        max_cells = int(math.ceil(max(semi_major, semi_minor) / self.map_res))
        ix, iy = self.world_to_grid(x, y)
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)

        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                dx_m, dy_m = dx * self.map_res, dy * self.map_res
                along = cos_heading * dx_m + sin_heading * dy_m
                lateral = -sin_heading * dx_m + cos_heading * dy_m
                if (along / semi_major) ** 2 + (lateral / semi_minor) ** 2 > 1.0:
                    continue
                cx, cy = ix + dx, iy + dy
                if 0 <= cx < self.map_w and 0 <= cy < self.map_h:
                    self.person_grid[cy, cx] = 100

    # ------------------------------- Helpers -------------------------------
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert map-frame coordinates into occupancy-grid indices."""
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        return ix, iy

    @_synchronized
    def publish_costmap(self) -> None:
        """Merge all layers and publish the resulting occupancy grid."""
        if self.static_grid is None:
            return

        combined = self.static_grid.copy()

        if self.dynamic_grid is not None:
            mask_dyn = self.dynamic_grid > 0
            combined[mask_dyn] = 100

        if self.person_grid is not None:
            mask_person = self.person_grid > 0
            combined[mask_person] = self.person_grid[mask_person].astype(combined.dtype)

        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_map

        msg.info.resolution = self.map_res
        msg.info.width = self.map_w
        msg.info.height = self.map_h
        msg.info.origin.position.x = self.map_ox
        msg.info.origin.position.y = self.map_oy
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = combined.reshape(-1).tolist()

        self.pub_costmap.publish(msg)


if __name__ == "__main__":
    rospy.init_node("fm2_costmap_node")
    node = FM2CostmapNode()
    rospy.spin()
