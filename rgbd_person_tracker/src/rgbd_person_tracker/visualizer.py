#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


_PALETTE_BGR = [
    (0, 255, 0),
    (255, 0, 0),
    (0, 0, 255),
    (255, 255, 0),
    (0, 255, 255),
    (255, 0, 255),
    (128, 255, 0),
    (255, 128, 0),
    (0, 128, 255),
    (255, 0, 128),
]


def _color_bgr(track_id: int):
    return _PALETTE_BGR[track_id % len(_PALETTE_BGR)]


def _color_rgba(track_id: int, alpha: float = 1.0) -> ColorRGBA:
    b, g, r = _color_bgr(track_id)
    return ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=alpha)


class Visualizer:
    def __init__(
        self,
        world_frame="world",
        top_view_width=500,
        top_view_height=300,
        top_view_meters_x=10.0,
        top_view_meters_y=6.0,
        top_view_origin_x=0.0,
        top_view_origin_y=-3.0,
        hfov_deg=70.0,
    ):
        self.world_frame = world_frame
        self.bridge = CvBridge()

        self.tv_w = top_view_width
        self.tv_h = top_view_height
        self.tv_mx = top_view_meters_x
        self.tv_my = top_view_meters_y
        self.tv_ox = top_view_origin_x
        self.tv_oy = top_view_origin_y
        self.hfov_rad = math.radians(hfov_deg)

        self.pub_img = rospy.Publisher("~image_annotated", Image, queue_size=1)
        self.pub_top = rospy.Publisher("~top_view", Image, queue_size=1)
        self.pub_markers = rospy.Publisher("~markers", MarkerArray, queue_size=1)

    # ---------- Imagen anotada ----------
    def publish_image(self, rgb_bgr, tracks, detections, stamp):
        vis = rgb_bgr.copy()

        # Dibujar detecciones: bbox + punto de referencia
        for det in detections:
            if hasattr(det, 'bbox') and det.bbox is not None:
                bb = det.bbox
                pt1 = (int(round(bb.x1)), int(round(bb.y1)))
                pt2 = (int(round(bb.x2)), int(round(bb.y2)))
                cv2.rectangle(vis, pt1, pt2, (0, 255, 0), 2)

            u, v = int(round(det.ref_2d[0])), int(round(det.ref_2d[1]))
            cv2.circle(vis, (u, v), 6, (0, 255, 255), -1)
            cv2.putText(vis, det.ref_type, (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # IDs de tracks confirmados
        for tr in tracks:
            if not tr.confirmed:
                continue
            color = _color_bgr(tr.id)
            label = f"ID {tr.id}"

            best_det = None
            best_d = 1e9
            for det in detections:
                d = np.linalg.norm(tr.position - det.xy_ground)
                if d < best_d:
                    best_d = d
                    best_det = det
            if best_det is not None and best_d < 1.5:
                if hasattr(best_det, 'bbox') and best_det.bbox is not None:
                    bb = best_det.bbox
                    pt1 = (int(round(bb.x1)), int(round(bb.y1)))
                    pt2 = (int(round(bb.x2)), int(round(bb.y2)))
                    cv2.rectangle(vis, pt1, pt2, color, 2)
                    cv2.putText(vis, label, (pt1[0], pt1[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                else:
                    u = int(round(best_det.ref_2d[0]))
                    v = int(round(best_det.ref_2d[1])) - 20
                    cv2.putText(vis, label, (u, v),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        msg.header.stamp = stamp
        self.pub_img.publish(msg)

    # ---------- Vista cenital ----------
    def _world_to_px(self, x, y):
        px = int((x - self.tv_ox) / self.tv_mx * self.tv_w)
        py = int((1.0 - (y - self.tv_oy) / self.tv_my) * self.tv_h)
        return px, py

    def publish_top_view(self, tracks, stamp):
        canvas = np.zeros((self.tv_h, self.tv_w, 3), dtype=np.uint8)
        canvas[:] = (30, 30, 30)

        # Grid cada 1m
        for xm in range(int(self.tv_ox), int(self.tv_ox + self.tv_mx) + 1):
            px, _ = self._world_to_px(xm, 0)
            cv2.line(canvas, (px, 0), (px, self.tv_h), (50, 50, 50), 1)
        for ym_i in range(int(self.tv_oy), int(self.tv_oy + self.tv_my) + 1):
            _, py = self._world_to_px(0, ym_i)
            cv2.line(canvas, (0, py), (self.tv_w, py), (50, 50, 50), 1)

        # Cono de FOV
        cam_px = self._world_to_px(0, 0)
        fov_range = self.tv_mx
        half_fov = self.hfov_rad / 2.0
        pt_left = self._world_to_px(
            fov_range * math.cos(half_fov), fov_range * math.sin(half_fov))
        pt_right = self._world_to_px(
            fov_range * math.cos(-half_fov), fov_range * math.sin(-half_fov))

        overlay = canvas.copy()
        cone_pts = np.array([cam_px, pt_left, pt_right], dtype=np.int32)
        cv2.fillPoly(overlay, [cone_pts], (60, 80, 60))
        cv2.addWeighted(overlay, 0.4, canvas, 0.6, 0, canvas)
        cv2.line(canvas, cam_px, pt_left, (100, 180, 100), 1)
        cv2.line(canvas, cam_px, pt_right, (100, 180, 100), 1)

        # Camara
        cv2.drawMarker(canvas, cam_px, (255, 255, 255),
                       cv2.MARKER_DIAMOND, 10, 2)
        cv2.putText(canvas, "CAM", (cam_px[0] + 5, cam_px[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Tracks
        for tr in tracks:
            if not tr.confirmed:
                continue

            color = _color_bgr(tr.id)

            if len(tr.history) >= 2:
                pts = [self._world_to_px(p[0], p[1]) for p in tr.history]
                for k in range(1, len(pts)):
                    cv2.line(canvas, pts[k - 1], pts[k], color, 1)

            cur = self._world_to_px(tr.position[0], tr.position[1])
            cv2.circle(canvas, cur, 5, color, -1)

            vel = tr.velocity
            end_world = tr.position + vel
            end_px = self._world_to_px(end_world[0], end_world[1])
            cv2.arrowedLine(canvas, cur, end_px, color, 2, tipLength=0.3)

            cv2.putText(canvas, f"ID {tr.id}", (cur[0] + 8, cur[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        msg = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        msg.header.stamp = stamp
        self.pub_top.publish(msg)

    # ---------- Markers RViz 3D ----------
    def publish_markers(self, tracks, stamp):
        ma = MarkerArray()
        header = Header(stamp=stamp, frame_id=self.world_frame)
        marker_id = 0

        for tr in tracks:
            if not tr.confirmed:
                continue

            color = _color_rgba(tr.id)

            # Esfera posicion actual
            m_sphere = Marker()
            m_sphere.header = header
            m_sphere.ns = "tracker_pos"
            m_sphere.id = marker_id
            marker_id += 1
            m_sphere.type = Marker.SPHERE
            m_sphere.action = Marker.ADD
            m_sphere.pose.position.x = tr.position[0]
            m_sphere.pose.position.y = tr.position[1]
            m_sphere.pose.position.z = 0.9
            m_sphere.pose.orientation.w = 1.0
            m_sphere.scale.x = m_sphere.scale.y = m_sphere.scale.z = 0.3
            m_sphere.color = color
            m_sphere.lifetime = rospy.Duration(0.3)
            ma.markers.append(m_sphere)

            # Texto ID
            m_text = Marker()
            m_text.header = header
            m_text.ns = "tracker_id"
            m_text.id = marker_id
            marker_id += 1
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = tr.position[0]
            m_text.pose.position.y = tr.position[1]
            m_text.pose.position.z = 2.0
            m_text.pose.orientation.w = 1.0
            m_text.scale.z = 0.3
            m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            m_text.text = f"ID {tr.id}"
            m_text.lifetime = rospy.Duration(0.3)
            ma.markers.append(m_text)

            # Historial (LINE_STRIP)
            if len(tr.history) >= 2:
                m_line = Marker()
                m_line.header = header
                m_line.ns = "tracker_hist"
                m_line.id = marker_id
                marker_id += 1
                m_line.type = Marker.LINE_STRIP
                m_line.action = Marker.ADD
                m_line.scale.x = 0.05
                m_line.color = color
                m_line.color.a = 0.6
                m_line.lifetime = rospy.Duration(0.3)
                for pt in tr.history:
                    m_line.points.append(Point(x=pt[0], y=pt[1], z=0.05))
                m_line.pose.orientation.w = 1.0
                ma.markers.append(m_line)

            # Flecha de velocidad
            vel = tr.velocity
            speed = np.linalg.norm(vel)
            if speed > 0.05:
                m_arrow = Marker()
                m_arrow.header = header
                m_arrow.ns = "tracker_vel"
                m_arrow.id = marker_id
                marker_id += 1
                m_arrow.type = Marker.ARROW
                m_arrow.action = Marker.ADD
                m_arrow.points.append(
                    Point(x=tr.position[0], y=tr.position[1], z=0.5))
                m_arrow.points.append(
                    Point(x=tr.position[0] + vel[0],
                          y=tr.position[1] + vel[1], z=0.5))
                m_arrow.scale.x = 0.06
                m_arrow.scale.y = 0.12
                m_arrow.scale.z = 0.0
                m_arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
                m_arrow.lifetime = rospy.Duration(0.3)
                ma.markers.append(m_arrow)

        self.pub_markers.publish(ma)

    # ---------- Publicar todo ----------
    def publish_all(self, rgb_bgr, tracks, detections, stamp):
        if self.pub_img.get_num_connections() > 0:
            self.publish_image(rgb_bgr, tracks, detections, stamp)
        if self.pub_top.get_num_connections() > 0:
            self.publish_top_view(tracks, stamp)
        if self.pub_markers.get_num_connections() > 0:
            self.publish_markers(tracks, stamp)
