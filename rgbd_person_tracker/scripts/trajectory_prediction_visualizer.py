#!/usr/bin/env python3
"""Marcadores RViz para la salida de ``trajectory_predictor_node``.

Es una utilidad de inspección: no participa en la navegación ni modifica
ningún topic de control. Visualiza posición reconstruida, velocidad,
trayectoria futura e incertidumbre de cada predicción.
"""

import math

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from rgbd_person_tracker.msg import PersonPredictionArray


class TrajectoryPredictionVisualizer:
    def __init__(self):
        self.input_topic = rospy.get_param("~person_predictions_topic", "/person_predictions")
        self.output_topic = rospy.get_param("~markers_topic", "/person_predictions/markers")
        self.base_radius = float(rospy.get_param("~base_radius", 0.35))
        self.pub = rospy.Publisher(self.output_topic, MarkerArray, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PersonPredictionArray, self.cb_predictions, queue_size=1)
        rospy.loginfo("Visualizador de predicciones: %s -> %s", self.input_topic, self.output_topic)

    @staticmethod
    def _marker(frame_id, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = frame_id or "map"
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def cb_predictions(self, msg):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for prediction in msg.predictions:
            n_points = min(
                len(prediction.positions), len(prediction.time_from_now),
                len(prediction.sigma_major), len(prediction.sigma_minor),
            )
            if n_points == 0:
                continue

            track_id = int(prediction.track_id)
            vx, vy = float(prediction.velocity.x), float(prediction.velocity.y)
            first = prediction.positions[0]
            t0 = max(0.0, float(prediction.time_from_now[0]))
            current = Point(x=float(first.x) - vx * t0, y=float(first.y) - vy * t0, z=0.08)

            person = self._marker(msg.header.frame_id, msg.header.stamp, "tracked_person", track_id, Marker.SPHERE)
            person.pose.position = current
            person.scale.x = person.scale.y = 2.0 * self.base_radius
            person.scale.z = 0.12
            person.color.r, person.color.g, person.color.b, person.color.a = 0.15, 0.95, 0.20, 0.85
            markers.markers.append(person)

            text = self._marker(msg.header.frame_id, msg.header.stamp, "tracked_person_id", track_id, Marker.TEXT_VIEW_FACING)
            text.pose.position = Point(x=current.x, y=current.y, z=0.35)
            text.scale.z = 0.18
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = "ID %d" % track_id
            markers.markers.append(text)

            speed = math.hypot(vx, vy)
            if speed > 1e-3:
                arrow = self._marker(msg.header.frame_id, msg.header.stamp, "tracked_velocity", track_id, Marker.ARROW)
                arrow.points = [current, Point(x=current.x + vx, y=current.y + vy, z=0.08)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.035, 0.07, 0.08
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.85, 0.05, 1.0
                markers.markers.append(arrow)

            path = self._marker(msg.header.frame_id, msg.header.stamp, "predicted_trajectory", track_id, Marker.LINE_STRIP)
            path.scale.x = 0.035
            path.color.r, path.color.g, path.color.b, path.color.a = 0.0, 0.85, 1.0, 1.0
            path.points = [current]
            for index in range(n_points):
                point = prediction.positions[index]
                path.points.append(Point(x=float(point.x), y=float(point.y), z=0.10))
            markers.markers.append(path)

            heading = math.atan2(vy, vx) if speed > 1e-3 else 0.0
            half_heading = 0.5 * heading
            for index in range(n_points):
                point = prediction.positions[index]
                ellipse = self._marker(
                    msg.header.frame_id, msg.header.stamp, "prediction_uncertainty",
                    track_id * 100 + index, Marker.CYLINDER,
                )
                ellipse.pose.position = Point(x=float(point.x), y=float(point.y), z=0.02)
                ellipse.pose.orientation.z = math.sin(half_heading)
                ellipse.pose.orientation.w = math.cos(half_heading)
                ellipse.scale.x = 2.0 * max(self.base_radius, float(prediction.sigma_major[index]))
                ellipse.scale.y = 2.0 * max(self.base_radius, float(prediction.sigma_minor[index]))
                ellipse.scale.z = 0.02
                ellipse.color.r, ellipse.color.g, ellipse.color.b, ellipse.color.a = 1.0, 0.25, 0.05, 0.18
                markers.markers.append(ellipse)

        self.pub.publish(markers)


if __name__ == "__main__":
    rospy.init_node("trajectory_prediction_visualizer")
    TrajectoryPredictionVisualizer()
    rospy.spin()
