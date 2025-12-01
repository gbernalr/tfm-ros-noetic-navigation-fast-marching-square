#!/usr/bin/env python3
import rospy
import numpy as np
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros
import tf2_geometry_msgs
import cv2

import sys
from fm2 import FM2
from fm2.entities import FM2Map, FM2Info


class FM2TestPlanner:
    def __init__(self):
        rospy.init_node("fm2_test_planner")

        # --- Parámetros ---
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")
        self.replan_period = float(rospy.get_param("~replan_period", 1.0))
        self.inflation = int(rospy.get_param("~inflate", 2.0))

        # --- Estado ---
        self.map_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.last_pose = None
        self.goal_world = None
        self.path_world = None

        # --- FM2 ---
        self.fm2 = None

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Subscribers ---
        self.sub_map = rospy.Subscriber("fm2_costmap/costmap", OccupancyGrid, self.cb_map, queue_size=1)
        self.sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_amcl = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1)

        # --- Publisher ---
        self.pub_path = rospy.Publisher("fm2_path", Path, queue_size=1, latch=True)

        # --- Control ---
        self.last_replan_time = rospy.Time.now()

    # ----------------- Callbacks -----------------
    def cb_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # Ocupado: >= 50
        occ = (data >= 50)
        # Desconocido: < 0
        unk = (data < 0)
        # Obstáculo = ocupado o desconocido
        obs = np.logical_or(occ, unk).astype(np.uint8)
        # binario: 1 = libre, 0 = obstáculo
        self.map_bin = (1 - obs).astype(np.uint8)

    def cb_goal(self, msg: PoseStamped):
        pose = msg
        if msg.header.frame_id != self.frame_map:
            try:
                pose = self.tf_buffer.transform(msg, self.frame_map, timeout=rospy.Duration(0.5))
            except Exception as e:
                rospy.logwarn("No se pudo transformar goal: %s", e)
                return
        self.goal_world = (pose.pose.position.x, pose.pose.position.y)
        rospy.loginfo("Nuevo goal recibido: %s", self.goal_world)
        self._plan(trigger="goal")

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        pose = msg
        if msg.header.frame_id != self.frame_map:
            try:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = msg.pose.pose
                ps = self.tf_buffer.transform(ps, self.frame_map, timeout=rospy.Duration(0.5))
                x = ps.pose.position.x
                y = ps.pose.position.y
            except Exception as e:
                rospy.logwarn("No se pudo transformar amcl_pose: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
        self.last_pose = (x, y)

    # ----------------- Helpers -----------------
    def _world_to_grid(self, x, y):
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        return ix, iy

    def _grid_to_world(self, ix, iy):
        x = self.map_ox + (ix + 0.5) * self.map_res
        y = self.map_oy + (iy + 0.5) * self.map_res
        return x, y

    def _publish_path(self, pts_world):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_map
        for x, y in pts_world:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = self.frame_map
            ps.pose.position.x = x
            ps.pose.position.y = y
            path.poses.append(ps)
        self.pub_path.publish(path)

    def _dump_fm2_debug(self, binary, start_ix, start_iy, goal_ix, goal_iy, info):
        """
        Dibuja el mapa binario que ve FM2 + path en /tmp/fm2_planner.png
        """
        try:
            h, w = binary.shape
            img = np.zeros((h, w, 3), dtype=np.uint8)

            # 1 = libre (blanco), 0 = obstáculo (negro)
            img[binary == 1] = [255, 255, 255]
            img[binary == 0] = [0, 0, 0]

            # Dibujar path de FM2 en azul
            if info is not None and info.path is not None:
                xs, ys = info.path
                for ix, iy in zip(xs, ys):
                    iy = int(iy)
                    ix = int(ix)
                    if 0 <= iy < h and 0 <= ix < w:
                        img[iy, ix] = [255, 0, 0]  # azul

            # Start en verde
            if 0 <= start_iy < h and 0 <= start_ix < w:
                img[start_iy, start_ix] = [0, 255, 0]

            # Goal en rojo
            if 0 <= goal_iy < h and 0 <= goal_ix < w:
                img[goal_iy, goal_ix] = [0, 0, 255]

            cv2.imwrite("/tmp/fm2_planner.png", img)
        except Exception as e:
            rospy.logwarn("Error en _dump_fm2_debug: %s", e)

    # ----------------- Planificación -----------------
    def _plan(self, trigger="timer"):
        if self.map_bin is None:
            rospy.logwarn_throttle(5, "Falta MAPA (map_bin es None)")
            return
        if self.last_pose is None:
            rospy.logwarn_throttle(5, "Falta AMCL POSE")
            return
        if self.goal_world is None:
            return

        sx, sy = self.last_pose
        gx, gy = self.goal_world

        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy = self._world_to_grid(gx, gy)

        # 1. Copia del mapa binario base
        binary = self.map_bin.copy().astype(np.uint8)

        # 2. Inflado
        if self.inflation > 0:
            k = 2 * self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        # 3. Planificación con FM2
        self.fm2 = FM2(mode="cpu")
        fm2_map = FM2Map.from_binary_map(binary, create_border=True)
        self.fm2.set_map(fm2_map)
        info = self.fm2.get_path((start_ix, start_iy), (goal_ix, goal_iy))

        # Dump de debug del mapa que VE FM2 (no el costmap puro)
        self._dump_fm2_debug(binary, start_ix, start_iy, goal_ix, goal_iy, info)

        # 4. Publicar Path
        if info.path is None:
            rospy.logwarn("FM2 no encontró ruta")
            self.path_world = None
            return

        xs, ys = info.path
        pts = [self._grid_to_world(ix, iy) for ix, iy in zip(xs, ys)]
        self.path_world = pts
        self._publish_path(pts)
        rospy.loginfo("Path calculado por FM2 (trigger: %s).", trigger)

    # ----------------- Spin -----------------
    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.goal_world is not None:
                if (now - self.last_replan_time).to_sec() > self.replan_period:
                    self.last_replan_time = now
                    self._plan(trigger="timer")
            rate.sleep()


if __name__ == "__main__":
    node = FM2TestPlanner()
    rospy.loginfo("FM2 Test Planner INICIADO. Esperando mapa...")
    node.spin()
