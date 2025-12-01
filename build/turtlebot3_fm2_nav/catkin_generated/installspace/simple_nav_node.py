#!/usr/bin/env python3
import rospy
import numpy as np
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros
import tf2_geometry_msgs
import cv2
import math

from fm2 import FM2
from fm2.entities import FM2Map, FM2Info


class FM2TestPlanner:
    def __init__(self):
        rospy.init_node("fm2_test_planner")

        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")
        self.replan_period = float(rospy.get_param("~replan_period", 0.2))
        self.inflation = int(rospy.get_param("~inflate", 2.0))

        self.lookahead_dist = float(rospy.get_param("~lookahead", 0.35))
        self.v_lin = float(rospy.get_param("~v_lin", 0.22))
        self.v_ang_max = float(rospy.get_param("~v_ang_max", 1.5))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.08))
        self.k_theta = float(rospy.get_param("~k_theta", 2.0))
        self.replan_offpath = float(rospy.get_param("~replan_offpath", 0.2))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        self.map_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.last_pose = None  # (x, y, yaw)
        self.goal_world = None
        self.path_world = None
        self.path_idx = 0

        self.fm2 = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_map = rospy.Subscriber("fm2_costmap/costmap", OccupancyGrid, self.cb_map, queue_size=1)
        self.sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_amcl = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1)

        self.pub_path = rospy.Publisher("fm2_path", Path, queue_size=1, latch=True)
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.last_replan_time = rospy.Time.now()

    def cb_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ = (data >= 50)
        unk = (data < 0)
        obs = np.logical_or(occ, unk).astype(np.uint8)
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
                yaw = self._yaw_from_quat(ps.pose.orientation)
            except Exception as e:
                rospy.logwarn("No se pudo transformar amcl_pose: %s", e)
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        self.last_pose = (x, y, yaw)

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_to_pi(a):
        return (a + math.pi) % (2*math.pi) - math.pi

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
        try:
            h, w = binary.shape
            img = np.zeros((h, w, 3), dtype=np.uint8)

            img[binary == 1] = [255, 255, 255]
            img[binary == 0] = [0, 0, 0]

            if info is not None and info.path is not None:
                xs, ys = info.path
                for ix, iy in zip(xs, ys):
                    iy = int(iy)
                    ix = int(ix)
                    if 0 <= iy < h and 0 <= ix < w:
                        img[iy, ix] = [255, 0, 0]

            if 0 <= start_iy < h and 0 <= start_ix < w:
                img[start_iy, start_ix] = [0, 255, 0]

            if 0 <= goal_iy < h and 0 <= goal_ix < w:
                img[goal_iy, goal_ix] = [0, 0, 255]

            cv2.imwrite("/tmp/fm2_planner.png", img)
        except Exception as e:
            rospy.logwarn("Error en _dump_fm2_debug: %s", e)

    def _plan(self, trigger="timer"):
        if self.map_bin is None:
            rospy.logwarn_throttle(5, "Falta MAPA (map_bin es None)")
            return
        if self.last_pose is None:
            rospy.logwarn_throttle(5, "Falta AMCL POSE")
            return
        if self.goal_world is None:
            return

        sx, sy, _ = self.last_pose
        gx, gy = self.goal_world

        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy = self._world_to_grid(gx, gy)

        binary = self.map_bin.copy().astype(np.uint8)

        if self.inflation > 0:
            k = 2 * self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        self.fm2 = FM2(mode="cpu")
        fm2_map = FM2Map.from_binary_map(binary, create_border=True)
        self.fm2.set_map(fm2_map)
        info = self.fm2.get_path((int(start_ix), int(start_iy)), (int(goal_ix), int(goal_iy)))

        self._dump_fm2_debug(binary, start_ix, start_iy, goal_ix, goal_iy, info)

        if info.path is None:
            rospy.logwarn("FM2 no encontró ruta")
            self.path_world = None
            return

        xs, ys = info.path
        pts = [self._grid_to_world(int(ix), int(iy)) for ix, iy in zip(xs, ys)]
        self.path_world = pts
        self.path_idx = 0
        self._publish_path(pts)
        rospy.loginfo("Path calculado por FM2 (trigger: %s).", trigger)

    def _stop(self):
        self.pub_cmd.publish(Twist())

    def _track_target(self, x, y, yaw, target):
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
        if self.path_world is None or self.last_pose is None or self.goal_world is None:
            return

        x, y, yaw = self.last_pose

        if self.path_world:
            window = self.path_world[self.path_idx: min(self.path_idx + 30, len(self.path_world))]
            if window:
                dmin = min(np.hypot(px - x, py - y) for (px, py) in window)
                if dmin > self.replan_offpath:
                    self._plan(trigger="offpath")
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
                self.path_idx = min(self.path_idx + 1, len(self.path_world) - 1)

        if target is None:
            goal_x, goal_y = self.path_world[-1]
            if np.hypot(goal_x - x, goal_y - y) < self.goal_tolerance:
                self._stop()
                self.path_world = None
                self.goal_world = None
                return
            else:
                target = (goal_x, goal_y)

        self._track_target(x, y, yaw, target)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.goal_world is not None:
                if (now - self.last_replan_time).to_sec() > self.replan_period:
                    self.last_replan_time = now
                    self._plan(trigger="timer")
            self._control_step()
            rate.sleep()


if __name__ == "__main__":
    node = FM2TestPlanner()
    rospy.loginfo("FM2 Test Planner INICIADO. Esperando mapa...")
    node.spin()
