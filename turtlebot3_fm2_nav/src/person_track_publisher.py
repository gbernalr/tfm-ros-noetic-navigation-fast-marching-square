#!/usr/bin/env python3
"""
person_track_publisher.py — Publica la posición y velocidad ground-truth de
un modelo de Gazebo (persona) como rgbd_person_tracker/PersonTrackArray en
/person_tracks, para poder validar la navegación FM2 sin depender del
tracker RGB-D real.

NOTA: es una fuente de "percepción perfecta" para pruebas. Publica en el
frame 'map' asumiendo que el mundo de Gazebo y el mapa estático están
razonablemente alineados en el origen (caso habitual con los mundos y mapas
por defecto de turtlebot3). Si detectas offset visual en RViz, ajusta
output_frame o añade una transformación estática world->map.
"""
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates

from rgbd_person_tracker.msg import PersonTrack, PersonTrackArray


class PersonTrackPublisher:
    def __init__(self):
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
            "[person_track_publisher.py::__init__] modelo=%s frame_salida=%s",
            self.model_name, self.output_frame,
        )

    def _cb(self, msg: ModelStates):
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
                    "[person_track_publisher.py::_cb] dt sospechosamente pequeño (%.4fs), "
                    "se omite esta muestra para no inflar la velocidad",
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
                    "[person_track_publisher.py::_cb] pos=(%.2f,%.2f) dt=%.4f raw_vel=(%.2f,%.2f)|%.2fm/s "
                    "vel_suavizada=(%.2f,%.2f)|%.2fm/s",
                    pos[0], pos[1], dt, raw_vel[0], raw_vel[1], raw_speed,
                    self._vel[0], self._vel[1], smoothed_speed,
                )

                if raw_speed > self.max_speed_warn:
                    rospy.logwarn(
                        "[person_track_publisher.py::_cb] PICO DE VELOCIDAD detectado: "
                        "raw_speed=%.2fm/s (umbral=%.2f) dt=%.4fs pos_prev=(%.2f,%.2f) "
                        "pos_actual=(%.2f,%.2f) -> vel_suavizada resultante=%.2fm/s",
                        raw_speed, self.max_speed_warn, dt,
                        self._last_pos[0], self._last_pos[1], pos[0], pos[1],
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
