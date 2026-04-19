#!/usr/bin/env python3
"""
ground_projector.py — Proyección de puntos 3D del frame de la cámara al suelo.

Utiliza TF2 para transformar un punto 3D expresado en el frame óptico
de la cámara (camera_depth_optical_frame) al frame mundo (world).
El resultado es la coordenada (x, y) en el plano del suelo (z≈0 en world).
"""

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  — registra conversiones PointStamped
from geometry_msgs.msg import PointStamped


class GroundProjector:
    """Proyecta puntos 3D de cámara al plano del suelo vía TF2."""

    def __init__(
        self,
        optical_frame: str = "camera_depth_optical_frame",
        world_frame: str = "world",
    ):
        self.optical_frame = optical_frame
        self.world_frame = world_frame

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def project_to_ground(self, point3d_cam: np.ndarray, stamp=None):
        """
        Transforma un punto 3D (X, Y, Z) del frame óptico al frame mundo
        y devuelve (x_world, y_world).

        Parámetros
        ----------
        point3d_cam : np.ndarray, shape (3,)
            Punto en metros en el frame óptico de la cámara.
        stamp : rospy.Time, opcional
            Timestamp para la consulta TF. Si None, usa rospy.Time(0) (última).

        Retorna
        -------
        (x, y) : tuple(float, float) o None
            Coordenadas en el plano del suelo (frame world). None si falla TF.
        """
        if stamp is None:
            stamp = rospy.Time(0)

        pt = PointStamped()
        pt.header.frame_id = self.optical_frame
        pt.header.stamp = stamp
        pt.point.x = float(point3d_cam[0])
        pt.point.y = float(point3d_cam[1])
        pt.point.z = float(point3d_cam[2])

        try:
            pt_world = self._tf_buffer.transform(
                pt, self.world_frame, timeout=rospy.Duration(0.1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn_throttle(
                2.0, "GroundProjector: TF error: %s" % str(e)
            )
            return None

        return (pt_world.point.x, pt_world.point.y)
