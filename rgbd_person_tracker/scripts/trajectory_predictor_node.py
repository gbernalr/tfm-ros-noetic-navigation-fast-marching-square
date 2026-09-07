#!/usr/bin/env python3
"""Publica predicciones de trayectoria a partir de /person_tracks.

El nodo mantiene un historial temporal por ``track_id`` y constituye el
baseline reproducible de velocidad constante para sustituirlo posteriormente
por un modelo aprendido. La interfaz de salida no cambia al sustituir el
predictor: cada horizonte lleva una elipse de incertidumbre (eje mayor/menor)
en metros.
"""

from collections import deque
import math

import numpy as np
import rospy
from geometry_msgs.msg import Point

from rgbd_person_tracker.msg import (
    PersonPrediction,
    PersonPredictionArray,
    PersonTrackArray,
)


class TrajectoryPredictorNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~person_tracks_topic", "/person_tracks")
        self.output_topic = rospy.get_param("~person_predictions_topic", "/person_predictions")

        self.history_duration = float(rospy.get_param("~history_duration", 3.0))
        self.min_history_samples = int(rospy.get_param("~min_history_samples", 4))
        self.velocity_window = float(rospy.get_param("~velocity_window", 1.0))
        self.horizons = self._positive_horizons(
            rospy.get_param("~prediction_horizons", [0.5, 1.0, 1.5, 2.0])
        )
        self.base_sigma = float(rospy.get_param("~base_sigma", 0.05))
        self.process_sigma_per_second = float(
            rospy.get_param("~process_sigma_per_second", 0.08)
        )
        self.directional_reaction_time = float(
            rospy.get_param("~directional_reaction_time", 0.5)
        )
        self.longitudinal_sigma_scale = float(
            rospy.get_param("~longitudinal_sigma_scale", 1.2)
        )
        self.lateral_sigma_scale = float(
            rospy.get_param("~lateral_sigma_scale", 0.6)
        )
        self.track_timeout = float(rospy.get_param("~track_timeout", 1.0))

        self.histories = {}
        self.pub = rospy.Publisher(self.output_topic, PersonPredictionArray, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PersonTrackArray, self._cb_tracks, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self._cleanup_stale_tracks)

        rospy.loginfo(
            "Trajectory predictor listo: %s -> %s (historial %.1fs, horizontes=%s)",
            self.input_topic, self.output_topic, self.history_duration, self.horizons,
        )

    @staticmethod
    def _positive_horizons(values):
        result = []
        for value in values:
            try:
                horizon = float(value)
            except (TypeError, ValueError):
                continue
            if horizon > 0.0:
                result.append(horizon)
        return sorted(set(result)) or [0.5, 1.0, 1.5, 2.0]

    def _cb_tracks(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        stamp_sec = stamp.to_sec()
        frame_id = msg.header.frame_id
        seen_ids = set()

        for track in msg.tracks:
            if not track.confirmed:
                continue
            track_id = int(track.track_id)
            seen_ids.add(track_id)
            history = self.histories.setdefault(track_id, deque())
            sample = (
                stamp_sec,
                float(track.position.x),
                float(track.position.y),
                float(track.velocity.x),
                float(track.velocity.y),
                float(track.confidence),
            )
            if history and stamp_sec <= history[-1][0]:
                continue
            history.append(sample)
            cutoff = stamp_sec - self.history_duration
            while history and history[0][0] < cutoff:
                history.popleft()

        out = PersonPredictionArray()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        for track_id in sorted(seen_ids):
            prediction = self._make_prediction(track_id, stamp_sec)
            if prediction is not None:
                out.predictions.append(prediction)
        self.pub.publish(out)

    def _make_prediction(self, track_id, now_sec):
        history = self.histories.get(track_id)
        if history is None or len(history) < self.min_history_samples:
            return None

        samples = [sample for sample in history if sample[0] >= now_sec - self.velocity_window]
        if len(samples) < 2:
            samples = list(history)
        if len(samples) < 2:
            return None

        times = np.asarray([sample[0] - samples[-1][0] for sample in samples], dtype=np.float64)
        xs = np.asarray([sample[1] for sample in samples], dtype=np.float64)
        ys = np.asarray([sample[2] for sample in samples], dtype=np.float64)

        # Ajuste lineal sobre historial temporal: más estable que una diferencia
        # de dos frames y totalmente reemplazable por un predictor aprendido.
        if np.ptp(times) > 1e-4:
            vx, x0 = np.polyfit(times, xs, 1)
            vy, y0 = np.polyfit(times, ys, 1)
        else:
            x0, y0 = xs[-1], ys[-1]
            vx, vy = samples[-1][3], samples[-1][4]

        confidence = min(1.0, max(0.0, samples[-1][5]))
        confidence_scale = 1.0 / math.sqrt(max(confidence, 0.1))

        prediction = PersonPrediction()
        prediction.track_id = track_id
        prediction.velocity.x = float(vx)
        prediction.velocity.y = float(vy)
        prediction.velocity.z = 0.0
        prediction.confidence = confidence
        speed = math.hypot(vx, vy)

        for horizon in self.horizons:
            point = Point(
                x=float(x0 + vx * horizon),
                y=float(y0 + vy * horizon),
                z=0.0,
            )
            prediction.positions.append(point)
            sigma = confidence_scale * math.sqrt(
                self.base_sigma ** 2 + (self.process_sigma_per_second * horizon) ** 2
            )
            prediction.time_from_now.append(horizon)
            # El eje de avance incluye el espacio recorrido durante el tiempo
            # de reacción del robot. Así no se infla por igual toda la escena:
            # se reserva más distancia solamente hacia donde camina la persona.
            prediction.sigma_major.append(
                self.longitudinal_sigma_scale * sigma
                + speed * max(0.0, self.directional_reaction_time)
            )
            prediction.sigma_minor.append(self.lateral_sigma_scale * sigma)

        return prediction

    def _cleanup_stale_tracks(self, _event):
        now = rospy.Time.now().to_sec()
        stale_ids = [
            track_id for track_id, history in self.histories.items()
            if not history or now - history[-1][0] > self.track_timeout
        ]
        for track_id in stale_ids:
            del self.histories[track_id]


if __name__ == "__main__":
    rospy.init_node("trajectory_predictor")
    TrajectoryPredictorNode()
    rospy.spin()
