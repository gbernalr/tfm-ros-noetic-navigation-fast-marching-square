#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path


class FM2Controller:
    def __init__(self):
        # Frames
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")

        # Parámetros de seguimiento
        self.lookahead_dist = float(rospy.get_param("~lookahead", 0.35))
        self.v_lin = float(rospy.get_param("~v_lin", 0.22))
        self.v_ang_max = float(rospy.get_param("~v_ang_max", 1.5))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.08))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        # Orientación final
        self.k_theta = float(rospy.get_param("~k_theta", 2.0))
        self.goal_yaw_tolerance = float(
            rospy.get_param("~goal_yaw_tolerance", 0.10)
        )
        self.use_goal_yaw = bool(rospy.get_param("~use_goal_yaw", True))

        # Estado
        self.path_world = None      # lista de (x, y)
        self.path_idx = 0
        self.mode_align = False

        self.goal_yaw = None        # yaw deseado en el goal

        self.last_pose = None       # (x, y, yaw)
        self.have_amcl = False

        # TF (por si amcl_pose no está en frame_map)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS I/O
        self.sub_path = rospy.Subscriber(
            "fm2_path", Path, self.cb_path, queue_size=1
        )
        self.sub_goal = rospy.Subscriber(
            "move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1
        )
        self.sub_amcl = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1
        )
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.loginfo("FM2 Controller inicializado.")

    # ------------------------ Utilidades ------------------------

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

    def _stop(self):
        self.pub_cmd.publish(Twist())

    # ------------------------ Callbacks ------------------------

    def cb_path(self, msg):
        # Convertimos Path a lista de puntos (x, y)
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))

        if pts:
            self.path_world = pts
            self.path_idx = 0
            self.mode_align = False
            rospy.loginfo(
                "FM2 Controller: nueva ruta recibida con %d puntos", len(pts)
            )
        else:
            rospy.logwarn("FM2 Controller: fm2_path vacío recibido")
            self.path_world = None
            self.path_idx = 0

    def cb_goal(self, msg):
        # Solo usamos el yaw del goal para la fase de alineación final
        if msg.header.frame_id != self.frame_map:
            try:
                msg = self._transform_pose(msg, self.frame_map)
            except Exception as e:
                rospy.logwarn(
                    "FM2 Controller cb_goal: No se pudo transformar goal: %s", e
                )
                return

        if self.use_goal_yaw:
            self.goal_yaw = self._yaw_from_quat(msg.pose.orientation)
        else:
            self.goal_yaw = None

        # Nuevo goal → salimos de modo align y esperaremos nueva path
        self.mode_align = False

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
                rospy.logwarn(
                    "FM2 Controller cb_amcl: No se pudo transformar pose: %s", e
                )
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)

        self.last_pose = (x, y, yaw)
        self.have_amcl = True

    # ------------------------ Control ------------------------

    def _track_target(self, x, y, yaw, target):
        import math
        tx, ty = target
        dx = tx - x
        dy = ty - y
        ang_ref = math.atan2(dy, dx)
        e_yaw = self._wrap_to_pi(ang_ref - yaw)

        fact = max(0.2, 1.0 - min(abs(e_yaw) / 1.2, 0.8))
        v = self.v_lin * fact

        w = float(np.clip(self.k_theta * e_yaw, -self.v_ang_max, self.v_ang_max))

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)

    def _control_step(self):
        # Fase de alineación final con el yaw objetivo
        if self.mode_align:
            if self.last_pose is None:
                return

            x, y, yaw = self.last_pose  # x, y no se usan, pero lo dejamos claro

            if self.goal_yaw is None:
                # No hay yaw objetivo, detener y limpiar
                self._stop()
                self.path_world = None
                self.mode_align = False
                return

            e_yaw = self._wrap_to_pi(self.goal_yaw - yaw)
            if abs(e_yaw) < self.goal_yaw_tolerance:
                # Alineado → paramos y limpiamos estado
                self._stop()
                self.path_world = None
                self.goal_yaw = None
                self.mode_align = False
                return

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = float(
                np.clip(self.k_theta * e_yaw, -self.v_ang_max, self.v_ang_max)
            )
            self.pub_cmd.publish(twist)
            return

        # Seguimiento de ruta normal
        if self.path_world is None or not self.path_world:
            return
        if self.last_pose is None:
            return

        x, y, yaw = self.last_pose

        # Selección de punto objetivo con lookahead
        target = None
        n_pts = len(self.path_world)

        # Avanzamos path_idx cuando estemos cerca del punto actual
        if 0 <= self.path_idx < n_pts:
            px, py = self.path_world[self.path_idx]
            if np.hypot(px - x, py - y) < 0.25:
                self.path_idx = min(self.path_idx + 1, n_pts - 1)

        for i in range(self.path_idx, n_pts):
            tx, ty = self.path_world[i]
            if np.hypot(tx - x, ty - y) >= self.lookahead_dist:
                target = (tx, ty)
                self.path_idx = i
                break

        if target is None:
            # Último punto de la ruta
            goal_x, goal_y = self.path_world[-1]
            dist_goal = np.hypot(goal_x - x, goal_y - y)

            if dist_goal < self.goal_tolerance:
                # Hemos llegado en posición
                if self.use_goal_yaw and self.goal_yaw is not None:
                    # Pasamos a fase de alineación
                    self.mode_align = True
                    self._stop()
                else:
                    self._stop()
                    self.path_world = None
                return
            else:
                target = (goal_x, goal_y)

        # Seguimos el target seleccionado
        self._track_target(x, y, yaw, target)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo("FM2 Controller listo. Siguiendo fm2_path.")
        while not rospy.is_shutdown():
            self._control_step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fm2_controller")
    node = FM2Controller()
    node.spin()
