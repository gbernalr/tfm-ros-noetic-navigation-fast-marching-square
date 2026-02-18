#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from fm2 import FM2
from fm2.entities import FM2Map, FM2Info


class FM2Planner:
    def __init__(self):
        # Frames
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")

        # Planificación / mapa
        self.inflation = int(rospy.get_param("~inflate", 0))

        # Replan
        self.replan_offpath = float(rospy.get_param("~replan_offpath", 0.6))
        self.replan_period = float(rospy.get_param("~replan_period", 1.0))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        # Estado del mapa
        self.map_msg = None
        self.grid_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None

        # Estado de planificación
        self.goal_world = None  # (x, y) en mapa
        self.path_world = None  # lista de (x, y)
        self.last_replan_time = rospy.Time.now()

        # Pose
        self.last_pose = None   # (x, y, yaw)
        self.have_amcl = False

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS I/O
        self.sub_map = rospy.Subscriber(
            "fm2_costmap/costmap", OccupancyGrid, self.cb_map, queue_size=1
        )
        self.sub_goal = rospy.Subscriber(
            "move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1
        )
        self.sub_amcl = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1
        )
        self.pub_path = rospy.Publisher("fm2_path", Path, queue_size=1, latch=True)

        rospy.loginfo("FM2 Planner inicializado.")

    # ----------------- Utilidades de colisión / conversiones -----------------

    def check_pts_collisions(self, pts_world, binary=None, radius_cells=0):
        if binary is None:
            if self.grid_bin is None:
                return {
                    "n": 0,
                    "collisions": [],
                    "n_collisions": 0,
                    "all_free": True,
                }
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

        return result

    @staticmethod
    def _yaw_from_quat(q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_to_pi(a):
        import math
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _world_to_grid(self, x, y):
        ix = int((x - self.map_ox) / self.map_res)
        iy = int((y - self.map_oy) / self.map_res)
        return ix, iy

    def _grid_to_world(self, ix, iy):
        x = self.map_ox + (ix + 0.5) * self.map_res
        y = self.map_oy + (iy + 0.5) * self.map_res
        return x, y

    def _transform_pose(self, pose_stamped, to_frame):
        return tf2_geometry_msgs.do_transform_pose(
            pose_stamped,
            self.tf_buffer.lookup_transform(
                to_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.5),
            ),
        )

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

    # ----------------------- Callbacks de ROS -----------------------

    def cb_map(self, msg):
        self.map_msg = msg
        w = msg.info.width
        h = msg.info.height
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ = (data >= 50)
        unk = (data < 0)
        obs = np.logical_or(occ, unk).astype(np.uint8)

        self.grid_bin = (1 - obs).astype(np.uint8)

    def cb_goal(self, msg):
        if msg.header.frame_id != self.frame_map:
            try:
                msg = self._transform_pose(msg, self.frame_map)
            except Exception as e:
                rospy.logwarn("FM2Planner cb_goal: No se pudo transformar goal: %s", e)
                return

        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(
            "FM2Planner: nuevo goal recibido en (%.3f, %.3f)",
            self.goal_world[0],
            self.goal_world[1],
        )

        self.path_world = None
        self.last_replan_time = rospy.Time(0)

    def cb_amcl(self, msg):
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
                rospy.logwarn("FM2Planner cb_amcl: No se pudo transformar pose: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)

        self.last_pose = (x, y, yaw)
        self.have_amcl = True

    # ----------------------- Lógica de planificación -----------------------

    def _plan_from_current_pose(self):
        if self.grid_bin is None or self.goal_world is None:
            return

        if self.last_pose is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame_map,
                    self.frame_base,
                    rospy.Time(0),
                    rospy.Duration(0.5),
                )
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                yaw = self._yaw_from_quat(tf.transform.rotation)
                self.last_pose = (x, y, yaw)
            except Exception as e:
                rospy.logwarn(
                    "FM2Planner _plan_from_current_pose: No se pudo obtener TF: %s", e
                )
                return

        sx, sy, _ = self.last_pose
        gx, gy = self.goal_world

        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy = self._world_to_grid(gx, gy)

        binary = self.grid_bin.copy().astype(np.uint8)

        if binary.size == 0:
            return
        
        if self.inflation > 0:
            k = 2 * self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        try:
            self.fm2 = FM2(mode="cpu")
            fm2_map = FM2Map.from_binary_map(binary, create_border=True)
            self.fm2.set_map(fm2_map)
        except Exception as e:
            rospy.logwarn("FM2Planner: Error en set_map: %s", e)
            self.path_world = None
            return

        try:
            info = self.fm2.get_path(
                (int(start_iy), int(start_ix)),
                (int(goal_iy), int(goal_ix)),
            )
        except IndexError as e:
            rospy.logwarn("FM2Planner: IndexError en get_path: %s", e)
            self.path_world = None
            return
        except Exception as e:
            rospy.logwarn("FM2Planner: Error en get_path: %s", e)
            self.path_world = None
            return

        if info.path is None:
            rospy.logwarn("FM2Planner: No se encontró ruta")
            self.path_world = None
            return

        rows, cols = info.path
        pts = [self._grid_to_world(int(col), int(row)) for row, col in zip(rows, cols)]

        # Comprobación de colisiones
        result = self.check_pts_collisions(pts, binary=binary, radius_cells=0)
        if not result["all_free"]:
            rospy.logwarn(
                "FM2Planner: path rechazado por colisiones: %d puntos en colisión",
                result["n_collisions"],
            )
            self.path_world = None
            return

        self.path_world = pts
        self._publish_path(pts)
        rospy.loginfo("FM2Planner: nueva ruta con %d puntos", len(pts))

    def _planner_step(self):
        if self.grid_bin is None or self.goal_world is None or self.last_pose is None:
            return

        now = rospy.Time.now()
        need_replan = False

        if self.path_world is None:
            need_replan = True

        if not need_replan and self.replan_period > 0.0:
            if (now - self.last_replan_time).to_sec() >= self.replan_period:
                need_replan = True

        if (not need_replan and self.path_world and self.replan_offpath > 0.0):
            x, y, _ = self.last_pose
            dmin = min(np.hypot(px - x, py - y) for (px, py) in self.path_world)
            if dmin > self.replan_offpath:
                rospy.loginfo("FM2Planner: robot fuera de ruta (%.3f m), replanificando", dmin)
                need_replan = True

        if need_replan:
            self.last_replan_time = now
            self._plan_from_current_pose()

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("FM2Planner listo. Publicando fm2_path.")
        while not rospy.is_shutdown():
            self._planner_step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fm2_planner")
    node = FM2Planner()
    node.spin()
