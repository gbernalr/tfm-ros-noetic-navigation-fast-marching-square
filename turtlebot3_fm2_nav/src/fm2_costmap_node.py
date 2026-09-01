#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import math

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from rgbd_person_tracker.msg import PersonTrackArray


class FM2CostmapNode:
    def __init__(self):
        self.frame_map   = rospy.get_param("~frame_map", "map")
        self.map_topic   = rospy.get_param("~map_topic", "/map")
        self.scan_topic  = rospy.get_param("~scan_topic", "/scan")
        self.person_tracks_topic = rospy.get_param("~person_tracks_topic", "/person_tracks")

        self.obstacle_range = float(rospy.get_param("~obstacle_range", 2.5))
        self.min_range      = float(rospy.get_param("~min_range", 0.05))

        self.dynamic_inflate = int(rospy.get_param("~dynamic_inflate", 0))
        self.person_radius = float(rospy.get_param("~person_radius", 0.35))
        self.person_inflate = int(rospy.get_param("~person_inflate", 2))
        self.person_prediction_enabled = bool(rospy.get_param("~person_prediction_enabled", True))
        self.person_use_confirmed_only = bool(rospy.get_param("~person_use_confirmed_only", True))
        self.person_prediction_horizons = rospy.get_param(
            "~person_prediction_horizons", [0.5, 1.0, 1.5, 2.0]
        )
        self.person_tracks_timeout = float(rospy.get_param("~person_tracks_timeout", 0.6))
        self.person_max_speed_warn = float(rospy.get_param("~person_max_speed_warn", 1.5))

        # Memoria de obstáculos dinámicos (en número de scans)
        self.dynamic_memory = int(rospy.get_param("~dynamic_memory", 15))

        # Grid estático y dinámico
        self.static_grid = None       # np.array int8 (-1,0,100)
        self.map_res = None
        self.map_w = None
        self.map_h = None
        self.map_ox = None
        self.map_oy = None

        # dynamic_grid ahora es un "contador" de memoria (uint8)
        # >0 => hay obstáculo dinámico reciente
        self.dynamic_grid = None      # np.array uint8 (0 libre, >0 obstáculo reciente)
        self.person_grid = None       # np.array uint8 (0 libre, 100 persona)
        self.last_person_msg_time = None

        # Posición del robot (en celdas de grid)
        self.robot_ix = None
        self.robot_iy = None

        # Path (lista de celdas (ix, iy))
        self.path_cells = []

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_map  = rospy.Subscriber(self.map_topic, OccupancyGrid,
                                         self.cb_map, queue_size=1)
        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan,
                                         self.cb_scan, queue_size=1)
        self.sub_persons = rospy.Subscriber(
            self.person_tracks_topic, PersonTrackArray, self.cb_persons, queue_size=1
        )

        # Suscribirse al path de FM2
        self.sub_path = rospy.Subscriber("fm2_path", Path,
                                         self.cb_path, queue_size=1)

        self.pub_costmap = rospy.Publisher("fm2_costmap/costmap",
                                           OccupancyGrid, queue_size=1, latch=True)
        self.person_timeout_timer = rospy.Timer(
            rospy.Duration(0.1), self._person_timeout_cb
        )

        # Para no volcar costmap combinado a lo loco
        self._last_costmap_dump_time = rospy.Time(0)

        rospy.loginfo("[fm2_costmap_node.py::__init__] listo. Esperando /map, /scan y /fm2_path...")

    # ---------- Callbacks ----------
    def cb_map(self, msg: OccupancyGrid):
        # Guardamos el mapa estático como grid de int8
        self.map_res = msg.info.resolution
        self.map_w = msg.info.width
        self.map_h = msg.info.height
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(self.map_h, self.map_w)

        static_grid = np.full((self.map_h, self.map_w), -1, dtype=np.int8)

        # Consideramos ocupado >= 50, libre == 0, resto desconocido
        static_grid[data >= 50] = 100
        static_grid[data == 0]  = 0

        self.static_grid = static_grid

        # Inicializamos la parte dinámica como contador de memoria
        self.dynamic_grid = np.zeros_like(static_grid, dtype=np.uint8)
        self.person_grid = np.zeros_like(static_grid, dtype=np.uint8)

        free_ratio = float((self.static_grid == 0).sum()) / (self.map_w * self.map_h)
        occ_ratio  = float((self.static_grid == 100).sum()) / (self.map_w * self.map_h)

        rospy.loginfo("[fm2_costmap_node.py::cb_map] Mapa estático cargado: %.1f%% libre, %.1f%% obstáculo.",
                      100.0 * free_ratio, 100.0 * occ_ratio)

        self.publish_costmap()

    def cb_persons(self, msg: PersonTrackArray):
        if self.static_grid is None:
            return

        self.last_person_msg_time = rospy.Time.now()

        if self.person_grid is None:
            self.person_grid = np.zeros((self.map_h, self.map_w), dtype=np.uint8)
        else:
            self.person_grid.fill(0)

        base_radius = max(1, int(math.ceil(self.person_radius / self.map_res)))
        total_radius = base_radius + max(self.person_inflate, 0)

        for track in msg.tracks:
            if self.person_use_confirmed_only and not track.confirmed:
                continue

            px, py, vx, vy = self._track_to_map(track, msg.header.frame_id, msg.header.stamp)
            if px is None:
                continue

            speed = math.hypot(vx, vy)
            predicted_pts = []

            self._paint_disc(px, py, total_radius)

            if self.person_prediction_enabled:
                for horizon in self.person_prediction_horizons:
                    try:
                        t = float(horizon)
                    except (TypeError, ValueError):
                        continue
                    if t <= 0.0:
                        continue
                    pred_x, pred_y = px + vx * t, py + vy * t
                    predicted_pts.append((t, pred_x, pred_y))
                    self._paint_disc(pred_x, pred_y, total_radius)

            rospy.loginfo_throttle(
                1.0,
                "[fm2_costmap_node.py::cb_persons] track_id=%d pos=(%.2f,%.2f) vel=(%.2f,%.2f)|%.2fm/s predicciones=%s",
                track.track_id, px, py, vx, vy, speed,
                [(round(t, 1), round(x, 2), round(y, 2)) for t, x, y in predicted_pts],
            )

            if speed > self.person_max_speed_warn:
                rospy.logwarn(
                    "[fm2_costmap_node.py::cb_persons] PROYECCION DISPARADA para track_id=%d: "
                    "vel=%.2fm/s (umbral=%.2f) pos=(%.2f,%.2f) -> punto predicho mas lejano=%s",
                    track.track_id, speed, self.person_max_speed_warn, px, py,
                    predicted_pts[-1] if predicted_pts else None,
                )

        self.publish_costmap()

    def _person_timeout_cb(self, _event):
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
                "[fm2_costmap_node.py::_person_timeout_cb] Limpiando capa de personas por timeout (%.2fs sin /person_tracks)",
                age,
            )

    def cb_scan(self, scan: LaserScan):
        if self.static_grid is None:
            return

        # TF: map -> frame del láser
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.frame_map,
                scan.header.frame_id,
                rospy.Time(0),               # usar la última TF disponible
                rospy.Duration(0.1)
            )
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[fm2_costmap_node.py::cb_scan] TF %s -> %s no disponible: %s",
                                   self.frame_map, scan.header.frame_id, e)
            return

        # Extraer yaw del TF
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        self.robot_ix, self.robot_iy = self.world_to_grid(tx, ty)

        if self.dynamic_grid is None:
            self.dynamic_grid = np.zeros_like(self.static_grid, dtype=np.uint8)
        else:
            decay_mask = self.dynamic_grid > 0
            self.dynamic_grid[decay_mask] -= 1

        angle = scan.angle_min
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        used_points = 0

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
                used_points += 1

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
                rospy.logwarn_throttle(10.0, "[fm2_costmap_node.py::cb_scan] OpenCV no disponible, dynamic_inflate ignorado.")

        dyn_count = int((self.dynamic_grid > 0).sum())
        rospy.loginfo_throttle(1.0, "[fm2_costmap_node.py::cb_scan] Celdas dinámicas ocupadas (memoria >0): %d", dyn_count)

        self.publish_costmap()

    def cb_path(self, msg: Path):
        """
        Guarda el path en coordenadas de grid para poder dibujarlo en el dump
        y además loguearlo.
        """
        if self.map_w is None or self.map_h is None:
            return

        cells = []
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            ix, iy = self.world_to_grid(x, y)
            if 0 <= ix < self.map_w and 0 <= iy < self.map_h:
                cells.append((ix, iy))
        self.path_cells = cells

    def _lookup_transform(self, source_frame, stamp):
        lookup_stamp = stamp if stamp != rospy.Time() else rospy.Time(0)
        return self.tf_buffer.lookup_transform(
            self.frame_map,
            source_frame,
            lookup_stamp,
            rospy.Duration(0.1),
        )

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _track_to_map(self, track, source_frame, stamp):
        px = float(track.position.x)
        py = float(track.position.y)
        vx = float(track.velocity.x)
        vy = float(track.velocity.y)

        if source_frame == self.frame_map:
            return px, py, vx, vy

        try:
            tf = self._lookup_transform(source_frame, stamp)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[fm2_costmap_node.py::_track_to_map] TF %s -> %s no disponible para tracks: %s",
                                   self.frame_map, source_frame, exc)
            return None, None, None, None

        yaw = self._yaw_from_quat(tf.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        mx = tf.transform.translation.x + cos_yaw * px - sin_yaw * py
        my = tf.transform.translation.y + sin_yaw * px + cos_yaw * py
        mvx = cos_yaw * vx - sin_yaw * vy
        mvy = sin_yaw * vx + cos_yaw * vy
        return mx, my, mvx, mvy

    def _paint_disc(self, x, y, radius_cells):
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

    # ---------- Helpers ----------
    def world_to_grid(self, x, y):
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        return ix, iy

    def publish_costmap(self):
        if self.static_grid is None:
            return

        combined = self.static_grid.copy()

        if self.dynamic_grid is not None:
            mask_dyn = (self.dynamic_grid > 0)
            combined[mask_dyn] = 100

        if self.person_grid is not None:
            mask_person = (self.person_grid > 0)
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
