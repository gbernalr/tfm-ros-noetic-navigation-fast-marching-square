#!/usr/bin/env python3
"""Simulate ZED body tracking from Gazebo model states.

The node applies camera pose, field of view, range, sampling frequency, noise,
latency, missed detections, and short-term tracking before publishing
``rgbd_person_tracker/PersonTrackArray``. Its output interface matches the
planned physical ZED adapter. ROS parameters are documented in
``ROS_PARAMETERS.md``.
"""

import math
import random
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from rgbd_person_tracker.msg import PersonTrack, PersonTrackArray


@dataclass
class SimTrack:
    """Internal planar state for one simulated person track."""

    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    last_detection: rospy.Time


class ZedBodyTrackingSimNode:
    """ROS node that emulates a camera-limited body-tracking sensor."""

    def __init__(self) -> None:
        """Read sensor configuration and initialize tracking and ROS interfaces."""
        self.output_topic = rospy.get_param("~person_tracks_topic", "/person_tracks")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.camera_frame = rospy.get_param("~camera_frame", "zed_sim_camera_frame")
        self.model_prefixes = tuple(
            rospy.get_param("~person_model_prefixes", ["person_target", "person_"])
        )

        self.sample_rate = float(rospy.get_param("~sample_rate", 15.0))
        self.min_range = float(rospy.get_param("~min_range", 0.5))
        self.max_range = float(rospy.get_param("~max_range", 8.0))
        self.horizontal_fov = math.radians(
            float(rospy.get_param("~horizontal_fov_deg", 110.0))
        )
        self.simulate_sensor_noise = bool(
            rospy.get_param("~simulate_sensor_noise", False)
        )
        self.detection_probability = float(
            rospy.get_param("~detection_probability", 0.96)
        )
        self.distance_probability_drop = float(
            rospy.get_param("~distance_probability_drop", 0.35)
        )
        self.occlusion_probability = float(
            rospy.get_param("~occlusion_probability", 0.05)
        )
        self.position_noise_std = float(rospy.get_param("~position_noise_std", 0.06))
        self.velocity_noise_std = float(rospy.get_param("~velocity_noise_std", 0.08))
        self.latency = float(rospy.get_param("~latency", 0.10))
        self.tracking_timeout = float(rospy.get_param("~tracking_timeout", 0.50))
        self.velocity_alpha = float(rospy.get_param("~velocity_smoothing", 0.5))
        self.random = random.Random(int(rospy.get_param("~random_seed", 7)))

        # Deterministic mode preserves FOV, range, and frequency while disabling
        # noise, latency, and missed detections for reproducible tests.
        if not self.simulate_sensor_noise:
            self.detection_probability = 1.0
            self.distance_probability_drop = 0.0
            self.occlusion_probability = 0.0
            self.position_noise_std = 0.0
            self.velocity_noise_std = 0.0
            self.latency = 0.0

        self.sample_period = 1.0 / max(self.sample_rate, 1e-3)
        self.next_sample_time = rospy.Time(0)
        self.next_track_id = 1
        self.tracks_by_model = {}
        self.model_motion = {}  # model -> (stamp, x, y, vx, vy)
        self.pending_outputs = deque()  # (release_time, stamp, list[SimTrack])

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher(self.output_topic, PersonTrackArray, queue_size=1)
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._cb_models, queue_size=1
        )
        self.publish_timer = rospy.Timer(rospy.Duration(0.02), self._publish_ready)

        rospy.loginfo(
            "ZED body-tracking simulator initialized: frame=%s, FOV=%.1f deg, "
            "range=[%.1f, %.1f] m, sensor_noise=%s, output=%s",
            self.camera_frame,
            math.degrees(self.horizontal_fov),
            self.min_range,
            self.max_range,
            "enabled" if self.simulate_sensor_noise else "disabled",
            self.output_topic,
        )

    def _cb_models(self, msg: ModelStates) -> None:
        """Sample visible person models and update their simulated tracks."""
        now = rospy.Time.now()
        if self.next_sample_time != rospy.Time() and now < self.next_sample_time:
            return
        self.next_sample_time = now + rospy.Duration(self.sample_period)

        camera = self._camera_pose(now)
        if camera is None:
            return
        cam_x, cam_y, cam_yaw = camera

        for index, model_name in enumerate(msg.name):
            if not self._is_person_model(model_name):
                continue

            pose = msg.pose[index]
            x, y = float(pose.position.x), float(pose.position.y)
            vx, vy = self._estimate_model_velocity(model_name, x, y, now)
            visible, range_m = self._is_visible(x, y, cam_x, cam_y, cam_yaw)
            if not visible or not self._detect(range_m):
                continue

            track = self.tracks_by_model.get(model_name)
            if track is None:
                track = SimTrack(
                    track_id=self.next_track_id,
                    x=x,
                    y=y,
                    vx=vx,
                    vy=vy,
                    confidence=1.0,
                    last_detection=now,
                )
                self.next_track_id += 1
                self.tracks_by_model[model_name] = track

            confidence = self._confidence_for_range(range_m)
            track.x = x + self.random.gauss(0.0, self.position_noise_std)
            track.y = y + self.random.gauss(0.0, self.position_noise_std)
            track.vx = vx + self.random.gauss(0.0, self.velocity_noise_std)
            track.vy = vy + self.random.gauss(0.0, self.velocity_noise_std)
            track.confidence = confidence
            track.last_detection = now

        output_tracks = self._tracked_or_predicted_tracks(now)
        self.pending_outputs.append(
            (now + rospy.Duration(max(0.0, self.latency)), now, output_tracks)
        )

        # Retire identifiers that have remained unobserved beyond the timeout.
        # A model detected again later receives a new identifier.
        for model_name in list(self.tracks_by_model):
            age = (now - self.tracks_by_model[model_name].last_detection).to_sec()
            if age > self.tracking_timeout:
                del self.tracks_by_model[model_name]

    def _camera_pose(self, stamp: rospy.Time) -> Optional[Tuple[float, float, float]]:
        """Return the camera's planar map pose at a given timestamp."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.camera_frame,
                stamp if stamp != rospy.Time() else rospy.Time(0),
                rospy.Duration(0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "ZED simulator could not transform %s <- %s: %s",
                self.output_frame,
                self.camera_frame,
                exc,
            )
            return None

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        t = tf.transform.translation
        return float(t.x), float(t.y), yaw

    def _is_person_model(self, model_name: str) -> bool:
        """Return whether a Gazebo model name matches a person prefix."""
        return any(
            model_name == prefix or model_name.startswith(prefix)
            for prefix in self.model_prefixes
        )

    def _estimate_model_velocity(
        self, model_name: str, x: float, y: float, stamp: rospy.Time
    ) -> Tuple[float, float]:
        """Estimate and smooth a model's planar velocity."""
        previous = self.model_motion.get(model_name)
        vx = vy = 0.0
        if previous is not None:
            last_stamp, last_x, last_y, last_vx, last_vy = previous
            dt = (stamp - last_stamp).to_sec()
            if dt > 1e-3:
                raw_vx, raw_vy = (x - last_x) / dt, (y - last_y) / dt
                alpha = min(1.0, max(0.0, self.velocity_alpha))
                vx = alpha * raw_vx + (1.0 - alpha) * last_vx
                vy = alpha * raw_vy + (1.0 - alpha) * last_vy
            else:
                vx, vy = last_vx, last_vy
        self.model_motion[model_name] = (stamp, x, y, vx, vy)
        return vx, vy

    def _is_visible(
        self,
        x: float,
        y: float,
        cam_x: float,
        cam_y: float,
        cam_yaw: float,
    ) -> Tuple[bool, float]:
        """Evaluate range and horizontal-field-of-view visibility."""
        dx, dy = x - cam_x, y - cam_y
        range_m = math.hypot(dx, dy)
        if range_m < self.min_range or range_m > self.max_range:
            return False, range_m
        bearing = math.atan2(dy, dx) - cam_yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return abs(bearing) <= self.horizontal_fov * 0.5, range_m

    def _detect(self, range_m: float) -> bool:
        """Sample a detection according to range and occlusion probability."""
        normalized_range = (range_m - self.min_range) / max(
            self.max_range - self.min_range, 1e-6
        )
        probability = (
            self.detection_probability
            - self.distance_probability_drop * normalized_range
        )
        probability *= 1.0 - self.occlusion_probability
        return self.random.random() < max(0.0, min(1.0, probability))

    def _confidence_for_range(self, range_m: float) -> float:
        """Calculate a bounded confidence score from detection range."""
        normalized_range = (range_m - self.min_range) / max(
            self.max_range - self.min_range, 1e-6
        )
        return max(0.1, min(1.0, 1.0 - 0.55 * normalized_range))

    def _tracked_or_predicted_tracks(self, now: rospy.Time) -> List[SimTrack]:
        """Return detected tracks and short-term predictions for missed tracks."""
        output = []
        for track in self.tracks_by_model.values():
            age = max(0.0, (now - track.last_detection).to_sec())
            if age > self.tracking_timeout:
                continue
            # Emulate a tracker's SEARCHING state by retaining its identifier and
            # predicting its position during a brief detection loss.
            confidence = track.confidence * max(
                0.0, 1.0 - age / max(self.tracking_timeout, 1e-3)
            )
            output.append(
                SimTrack(
                    track_id=track.track_id,
                    x=track.x + track.vx * age,
                    y=track.y + track.vy * age,
                    vx=track.vx,
                    vy=track.vy,
                    confidence=confidence,
                    last_detection=track.last_detection,
                )
            )
        return output

    def _publish_ready(self, _event: object) -> None:
        """Publish the newest simulated output whose latency has elapsed."""
        now = rospy.Time.now()
        latest = None
        while self.pending_outputs and self.pending_outputs[0][0] <= now:
            _, stamp, tracks = self.pending_outputs.popleft()
            latest = (stamp, tracks)
        if latest is None:
            return

        stamp, tracks = latest
        msg = PersonTrackArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame
        for track in tracks:
            out = PersonTrack()
            out.track_id = track.track_id
            out.position.x, out.position.y, out.position.z = track.x, track.y, 0.0
            out.velocity.x, out.velocity.y, out.velocity.z = track.vx, track.vy, 0.0
            out.confidence = track.confidence
            out.confirmed = True
            msg.tracks.append(out)
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("zed_body_tracking_sim")
    ZedBodyTrackingSimNode()
    rospy.spin()
