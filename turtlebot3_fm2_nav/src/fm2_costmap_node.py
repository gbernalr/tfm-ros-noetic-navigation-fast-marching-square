#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import math

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped


class FM2CostmapNode:
    def __init__(self):
        self.frame_map   = rospy.get_param("~frame_map", "map")
        self.map_topic   = rospy.get_param("~map_topic", "/map")
        self.scan_topic  = rospy.get_param("~scan_topic", "/scan")

        self.obstacle_range = float(rospy.get_param("~obstacle_range", 2.5))
        self.min_range      = float(rospy.get_param("~min_range", 0.05))

        self.dynamic_inflate = int(rospy.get_param("~dynamic_inflate", 0))

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

        # Suscribirse al path de FM2
        self.sub_path = rospy.Subscriber("fm2_path", Path,
                                         self.cb_path, queue_size=1)

        self.pub_costmap = rospy.Publisher("fm2_costmap/costmap",
                                           OccupancyGrid, queue_size=1, latch=True)

        # Para no volcar costmap combinado a lo loco
        self._last_costmap_dump_time = rospy.Time(0)

        rospy.loginfo("fm2_costmap_node listo. Esperando /map, /scan y /fm2_path...")

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

        free_ratio = float((self.static_grid == 0).sum()) / (self.map_w * self.map_h)
        occ_ratio  = float((self.static_grid == 100).sum()) / (self.map_w * self.map_h)

        rospy.loginfo("Mapa estático cargado: %.1f%% libre, %.1f%% obstáculo.",
                      100.0 * free_ratio, 100.0 * occ_ratio)

        self.publish_costmap()

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
            rospy.logwarn_throttle(2.0, "TF %s -> %s no disponible: %s",
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
                rospy.logwarn_throttle(10.0, "OpenCV no disponible, dynamic_inflate ignorado.")

        dyn_count = int((self.dynamic_grid > 0).sum())
        rospy.loginfo_throttle(1.0, "Celdas dinámicas ocupadas (memoria >0): %d", dyn_count)

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
