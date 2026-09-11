#!/usr/bin/env python3
"""Publish Gazebo ground-truth person position and velocity as tracked data.

This test-only source assumes that the Gazebo world and static map are aligned.
Adjust the output frame or provide a ``world`` to ``map`` transform otherwise.
ROS parameters are documented in ``ROS_PARAMETERS.md``.
"""

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from rgbd_person_tracker.msg import PersonTrack, PersonTrackArray


class PersonTrackPublisher:
    """ROS adapter from Gazebo model state to a confirmed person track."""

    def __init__(self) -> None:
        """Read configuration and initialize tracking state and ROS interfaces."""
        self.model_name = rospy.get_param("~person_model_name", "person_target")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.track_id = int(rospy.get_param("~track_id", 1))
        self.velocity_alpha = float(rospy.get_param("~velocity_smoothing", 0.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 15.0))
        self.max_speed_warn = float(rospy.get_param("~max_speed_warn", 1.5))
        self.min_dt = float(rospy.get_param("~min_dt", 0.01))

        self._last_pos = None
        self._last_time = None
        self._vel = np.zeros(2, dtype=np.float64)
        self._last_publish_time = rospy.Time(0)
        self._publish_period = 1.0 / max(self.publish_rate, 1e-3)

        self.pub = rospy.Publisher("/person_tracks", PersonTrackArray, queue_size=1)
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._cb, queue_size=1
        )

        rospy.loginfo(
            "Ground-truth person publisher initialized: model=%s, output_frame=%s",
            self.model_name,
            self.output_frame,
        )

    def _cb(self, msg: ModelStates) -> None:
        """Estimate velocity and publish the configured Gazebo person model."""
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        pos = np.array([pose.position.x, pose.position.y], dtype=np.float64)
        now = rospy.Time.now()

        if self._last_pos is not None and self._last_time is not None:
            dt = (now - self._last_time).to_sec()
            if dt < self.min_dt:
                rospy.logwarn_throttle(
                    1.0,
                    "Person-track sample interval is too short (%.4f s); "
                    "skipping velocity estimation",
                    dt,
                )
            elif dt > 1e-3:
                raw_vel = (pos - self._last_pos) / dt
                raw_speed = float(np.linalg.norm(raw_vel))
                a = self.velocity_alpha
                self._vel = a * raw_vel + (1.0 - a) * self._vel
                smoothed_speed = float(np.linalg.norm(self._vel))

                rospy.loginfo_throttle(
                    1.0,
                    "Person track: position=(%.2f, %.2f), dt=%.4f s, "
                    "raw_velocity=(%.2f, %.2f), raw_speed=%.2f m/s, "
                    "filtered_velocity=(%.2f, %.2f), filtered_speed=%.2f m/s",
                    pos[0],
                    pos[1],
                    dt,
                    raw_vel[0],
                    raw_vel[1],
                    raw_speed,
                    self._vel[0],
                    self._vel[1],
                    smoothed_speed,
                )

                if raw_speed > self.max_speed_warn:
                    rospy.logwarn(
                        "Person speed exceeds the configured threshold: "
                        "raw_speed=%.2f m/s, threshold=%.2f m/s, dt=%.4f s, "
                        "previous_position=(%.2f, %.2f), "
                        "current_position=(%.2f, %.2f), "
                        "filtered_speed=%.2f m/s",
                        raw_speed,
                        self.max_speed_warn,
                        dt,
                        self._last_pos[0],
                        self._last_pos[1],
                        pos[0],
                        pos[1],
                        smoothed_speed,
                    )

        self._last_pos = pos
        self._last_time = now

        if (now - self._last_publish_time).to_sec() < self._publish_period:
            return
        self._last_publish_time = now

        out = PersonTrackArray()
        out.header.stamp = now
        out.header.frame_id = self.output_frame

        track = PersonTrack()
        track.track_id = self.track_id
        track.position.x = float(pos[0])
        track.position.y = float(pos[1])
        track.position.z = 0.0
        track.velocity.x = float(self._vel[0])
        track.velocity.y = float(self._vel[1])
        track.velocity.z = 0.0
        track.confidence = 1.0
        track.confirmed = True
        out.tracks.append(track)

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("person_track_publisher")
    node = PersonTrackPublisher()
    rospy.spin()
