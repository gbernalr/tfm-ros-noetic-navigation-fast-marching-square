#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from fm2 import FM2
from fm2.entities import FM2Map, FM2Info


from datetime import datetime


class FM2Navigator:
    def __init__(self):
        # ----------------- Parámetros -----------------
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")
        self.inflation = int(rospy.get_param("~inflate", 0))
        self.lookahead_dist = float(rospy.get_param("~lookahead", 0.35))
        self.v_lin = float(rospy.get_param("~v_lin", 0.22))
        self.v_ang_max = float(rospy.get_param("~v_ang_max", 1.5))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.08))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        # Nuevos parámetros de control/orientación
        self.k_theta = float(rospy.get_param("~k_theta", 2.0))
        self.goal_yaw_tolerance = float(rospy.get_param("~goal_yaw_tolerance", 0.10))  # ~6°
        self.replan_offpath = float(rospy.get_param("~replan_offpath", 0.6))
        self.use_goal_yaw = bool(rospy.get_param("~use_goal_yaw", True))

        # Replan periódico (reactivo)
        self.replan_period = float(rospy.get_param("~replan_period", 1.0))
        self.last_replan_time = rospy.Time.now()

        # ----------------- Estado -----------------
        self.map_msg = None
        self.grid_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.goal_world = None
        self.goal_yaw = None
        self.path_world = None   # [(x,y), ...]
        self.path_idx = 0
        self.mode_align = False   # False: seguir ruta; True: alinear yaw final

        # FM2
        self.fm2 = FM2(mode="cpu")

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Pub/Sub
        self.sub_map = rospy.Subscriber("fm2_costmap/costmap", OccupancyGrid, self.cb_map, queue_size=1)
        self.sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_amcl = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1)
        self.pub_path = rospy.Publisher("fm2_path", Path, queue_size=1, latch=True)
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.have_amcl = False
        self.last_pose = None  # (x,y,yaw) en frame map
        
        
    def check_pts_collisions(self, pts_world, binary=None, radius_cells=0):
        """
        Comprueba, punto a punto, si la ruta pisa obstáculo.
        - pts_world: lista de (x, y) en frame 'map'
        - binary: np.ndarray uint8 con 1=libre, 0=obstáculo. Si None, usa self.grid_bin.
        - radius_cells: radio entero en celdas para chequear vecindario (0 = solo la celda exacta).
        Devuelve: dict con:
        {
            "n": total_puntos,
            "collisions": [(idx, (x,y), (ix,iy)) ...],
            "n_collisions": int,
            "all_free": bool
        }
        """
        if binary is None:
            if self.grid_bin is None:
                rospy.logwarn("check_pts_collisions: no hay grid_bin disponible.")
                return {"n": 0, "collisions": [], "n_collisions": 0, "all_free": True}
            binary = self.grid_bin

        h, w = binary.shape
        collisions = []

        for i, (x, y) in enumerate(pts_world):
            ix, iy = self._world_to_grid(x, y)

            if not (0 <= ix < w and 0 <= iy < h):
                collisions.append((i, (x, y), (ix, iy)))
                continue

            hit = False
            if radius_cells <= 0:
                if binary[iy, ix] == 0:
                    hit = True
            else:
                x0 = max(0, ix - radius_cells)
                x1 = min(w - 1, ix + radius_cells)
                y0 = max(0, iy - radius_cells)
                y1 = min(h - 1, iy + radius_cells)
                if (binary[y0:y1+1, x0:x1+1] == 0).any():
                    hit = True

            if hit:
                collisions.append((i, (x, y), (ix, iy)))

        result = {
            "n": len(pts_world),
            "collisions": collisions,
            "n_collisions": len(collisions),
            "all_free": (len(collisions) == 0),
        }

        if result["all_free"]:
            rospy.loginfo_throttle(1.0, "[CHK] Ruta: todas las posiciones caen en celdas libres.")
        else:
            ej = collisions[:5]
            rospy.logwarn_throttle(1.0, f"[CHK] {len(collisions)} puntos pisan obstáculo. Ejemplos: {ej}")

        return result

        

    # ---------- Dump binary map for debugging ----------
    def _dump_binary_map(self, binary, pts_world=None, tag=""):
        """
        Guarda dos imágenes:
        - ..._grid.pgm : grid tal cual (fila=y hacia abajo), con ruta en coords de grid
        - ..._rviz.pgm : imagen flipada verticalmente para verse como RViz (Y hacia arriba),
                        con la ruta también flipada.
        """
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_grid = f"map_dump_{tag}_{ts}_grid.pgm"
        base_rviz = f"map_dump_{tag}_{ts}_rviz.pgm"

        h, w = binary.shape

        # --- 1) Imagen base en grid (0=negro obstáculo, 254=blanco libre) ---
        img_grid = np.zeros((h, w), dtype=np.uint8)
        img_grid[binary == 1] = 254

        # Dibuja ruta en grid (SIN flip). OJO: usa tu helper correcto world->grid.
        if pts_world is not None:
            last = None
            for (x, y) in pts_world:
                ix, iy = self._world_to_grid(x, y)   # <-- aquí la corrección clave
                if 0 <= ix < w and 0 <= iy < h:
                    if last is not None:
                        cv2.line(img_grid, last, (ix, iy), 128, 1)
                    else:
                        cv2.circle(img_grid, (ix, iy), 1, 128, -1)
                    last = (ix, iy)
                else:
                    last = None

        cv2.imwrite(base_grid, img_grid)
        rospy.loginfo(f"[FM2] Dump grid (Y abajo): {base_grid}")

        # --- 2) Versión “como RViz”: flip vertical de TODO (mapa y ruta) ---
        # En imagen, fila 0 es arriba; en RViz, y+ es arriba. Hacemos flipud.
        img_rviz = np.flipud(img_grid.copy())

        # Si quisieras dibujar la ruta AQUI directamente, tendrías que convertir iy -> (h-1-iy).
        # Como ya hemos pintado en img_grid y fliplamos la imagen completa, ya queda coherente.
        cv2.imwrite(base_rviz, img_rviz)
        rospy.loginfo(f"[FM2] Dump RViz-like (Y arriba): {base_rviz}")
        
        
    def _dump_fm2_map(self, fm2_map, pts_world=None, tag="fm2_used"):
        """
        Vuelca a PGM el mapa binario que FM2 usa (fm2_map.binary_map).
        Blanco=libre (254), negro=obstáculo (0). Dibuja la ruta (si se pasa) en gris 128.
        Guarda en el directorio actual, como tus otros dumps.
        """
        from datetime import datetime
        import cv2
        import numpy as np

        binary_used = fm2_map.binary_map  # 1=libre, 0=obstáculo
        h, w = binary_used.shape

        # Base: 0 negro obstáculo, 254 blanco libre
        img = np.zeros((h, w), dtype=np.uint8)
        img[binary_used == 1] = 254

        # Overlay de la ruta (si se pasa)
        if pts_world is not None and len(pts_world) > 0:
            last = None
            for (x, y) in pts_world:
                ix, iy = self._world_to_grid(x, y)  # usa tu helper correcto
                if 0 <= ix < w and 0 <= iy < h:
                    if last is not None:
                        cv2.line(img, last, (ix, iy), 128, 1)
                    else:
                        cv2.circle(img, (ix, iy), 1, 128, -1)
                    last = (ix, iy)
                else:
                    last = None

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"map_dump_{tag}_{ts}.pgm"
        cv2.imwrite(filename, img)
        rospy.loginfo(f"[FM2] Dump fm2_map guardado en {filename}")


    # ---------- Callbacks ----------
    def cb_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        w = msg.info.width
        h = msg.info.height
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ = (data >= 50)      # 1 = obstáculo
        unk = (data < 0)         # 1 = desconocido
        obs = np.logical_or(occ, unk).astype(np.uint8)

        # 1 = libre, 0 = obstáculo
        self.grid_bin = (1 - obs).astype(np.uint8)

    def cb_goal(self, msg: PoseStamped):
        # Transforma goal a frame map si viene en otro frame
        if msg.header.frame_id != self.frame_map:
            try:
                msg = self._transform_pose(msg, self.frame_map)
            except Exception as e:
                rospy.logwarn("No pude transformar goal a %s: %s", self.frame_map, e)
                return

        self.goal_world = (msg.pose.position.x, msg.pose.position.y)

        if self.use_goal_yaw:
            self.goal_yaw = self._yaw_from_quat(msg.pose.orientation)
        else:
            self.goal_yaw = None

        rospy.loginfo("Nuevo goal: (%.3f, %.3f)%s",
                      self.goal_world[0], self.goal_world[1],
                      f" con yaw={self.goal_yaw:.3f} rad" if self.goal_yaw is not None else " sin yaw")

        self.mode_align = False
        self._plan_from_current_pose()

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        # Guarda pose actual desde AMCL en frame map
        if msg.header.frame_id != self.frame_map:
            try:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = msg.pose.pose
                pose = self._transform_pose(pose, self.frame_map)
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self._yaw_from_quat(pose.pose.orientation)
            except Exception as e:
                rospy.logwarn("TF amcl_pose -> map falló: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        self.last_pose = (x, y, yaw)
        self.have_amcl = True

    # ---------- Helpers ----------
    def _transform_pose(self, pose_stamped, to_frame):
        return tf2_geometry_msgs.do_transform_pose(
            pose_stamped,
            self.tf_buffer.lookup_transform(
                to_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(0.5)
            )
        )

    @staticmethod
    def _yaw_from_quat(q):
        import math
        # q es geometry_msgs/Quaternion
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_to_pi(a):
        import math
        return (a + math.pi) % (2*math.pi) - math.pi

    def _world_to_grid(self, x, y):
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        return ix, iy

    def _grid_to_world(self, ix, iy):
        x = self.map_ox + (ix + 0.5)*self.map_res
        y = self.map_oy + (iy + 0.5)*self.map_res
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

    # ---------- Plan & Control ----------
    def _plan_from_current_pose(self):
        if self.grid_bin is None or self.goal_world is None:
            rospy.logwarn("No hay mapa o goal.")
            return

        if self.last_pose is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_map, self.frame_base, rospy.Time(0), rospy.Duration(0.5)
                )
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                yaw = self._yaw_from_quat(tf.transform.rotation)
                self.last_pose = (x, y, yaw)
            except Exception as e:
                rospy.logwarn("No hay pose para planificar: %s", e)
                return

        sx, sy, _ = self.last_pose
        gx, gy = self.goal_world

        # Grid coords (col=x, fila=y)
        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy   = self._world_to_grid(gx, gy)

        # --- construir mapa binario con inflado opcional ---
        binary = self.grid_bin.copy().astype(np.uint8)
        if self.inflation > 0:
            k = 2*self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            # “inflado” = dilatar obstáculos
            inv = 1 - binary           # obst=1, libre=0
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        fm2_map = FM2Map.from_binary_map(binary, create_border=True)
        
        self.fm2.set_map(fm2_map)
        
        self._dump_fm2_map(fm2_map, tag="fm2_used")  # dump fm2 used map for debugging

        info: FM2Info = self.fm2.get_path(
            (int(start_ix), int(start_iy)),
            (int(goal_ix),  int(goal_iy))
        )

        if info.path is None:
            rospy.logwarn("FM2 no encontró ruta :(")
            self.path_world = None
            self._dump_binary_map(binary, tag="inflated_no_path")  # dump map for debugging
            return
    

        xs, ys = info.path  # xs=col(ix), ys=fila(iy)
        pts = [self._grid_to_world(int(ix), int(iy)) for ix, iy in zip(xs, ys)]
        
        self._dump_binary_map(binary, pts_world=pts, tag="inflated_with_path")
        self.check_pts_collisions(pts, binary=binary, radius_cells=0)
        
        
        
        self.path_world = pts
        self.path_idx = 0
        self.mode_align = False
        self._publish_path(pts)
        rospy.loginfo("Ruta FM2 con %d puntos.", len(pts))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._control_step()
            rate.sleep()

    def _control_step(self):
        if self.mode_align:
            if self.last_pose is None:
                return

            x, y, yaw = self.last_pose

            if self.goal_yaw is None:
                # no hay yaw final -> completado
                self._stop()
                self.path_world = None
                self.mode_align = False
                rospy.loginfo_throttle(2.0, "Objetivo alcanzado (sin yaw final).")
                return

            e_yaw = self._wrap_to_pi(self.goal_yaw - yaw)
            if abs(e_yaw) < self.goal_yaw_tolerance:
                self._stop()
                self.path_world = None
                self.goal_world = None
                self.goal_yaw = None
                self.mode_align = False
                rospy.loginfo_throttle(2.0, "Objetivo alcanzado y yaw alineado.")
                return


            # Giro puro para alinear
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = float(np.clip(self.k_theta * e_yaw,
                                            -self.v_ang_max,
                                            self.v_ang_max))
            self.pub_cmd.publish(twist)
            return


        # Replan periódico
        if self.goal_world is not None and self.grid_bin is not None and self.last_pose is not None:
            now = rospy.Time.now()
            if self.replan_period <= 0.0 or (now - self.last_replan_time).to_sec() >= self.replan_period:
                self.last_replan_time = now
                self._plan_from_current_pose()

        if self.path_world is None or self.last_pose is None:
            return

        x, y, yaw = self.last_pose

        # Si estamos fuera de ruta, replanificamos
        if self.path_world:
            window = self.path_world[self.path_idx : min(self.path_idx+30, len(self.path_world))]
            if window:
                dmin = min(np.hypot(px - x, py - y) for (px, py) in window)
                if dmin > self.replan_offpath:
                    rospy.logwarn_throttle(2.0, "Fuera de ruta (%.2fm). Replanificando...", dmin)
                    self._plan_from_current_pose()
                    return

        target = None
        for i in range(self.path_idx, len(self.path_world)):
            tx, ty = self.path_world[i]
            if np.hypot(tx - x, ty - y) >= self.lookahead_dist:
                target = (tx, ty)
                self.path_idx = i
                break

        if self.path_idx < len(self.path_world):
            px, py = self.path_world[self.path_idx]
            if np.hypot(px - x, py - y) < 0.25:
                self.path_idx = min(self.path_idx + 1, len(self.path_world)-1)

        if target is None:
            goal_x, goal_y = self.path_world[-1]
            if np.hypot(goal_x - x, goal_y - y) < self.goal_tolerance:
                # Cambiar a fase de alineación final (si procede)
                if self.use_goal_yaw and self.goal_yaw is not None:
                    self.mode_align = True
                    self._stop()
                else:
                    self._stop()
                    self.path_world = None
                    rospy.loginfo_throttle(2.0, "Objetivo alcanzado.")
                return
            else:
                target = (goal_x, goal_y)

        # Seguir el punto target
        self._track_target(x, y, yaw, target)

    def _track_target(self, x, y, yaw, target):
        import math
        tx, ty = target
        dx = tx - x
        dy = ty - y
        ang_ref = math.atan2(dy, dx)
        e_yaw = self._wrap_to_pi(ang_ref - yaw)

        # Escala de velocidad lineal por error angular (más giro -> menos v)
        fact = max(0.2, 1.0 - min(abs(e_yaw) / 1.2, 0.8))
        v = self.v_lin * fact

        # Control angular proporcional saturado
        w = float(np.clip(self.k_theta * e_yaw, -self.v_ang_max, self.v_ang_max))

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)

    def _stop(self):
        self.pub_cmd.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("fm2_navigator")
    node = FM2Navigator()
    rospy.loginfo("FM2 Navigator listo. Pulsa 2D Nav Goal en RViz.")
    node.spin()
