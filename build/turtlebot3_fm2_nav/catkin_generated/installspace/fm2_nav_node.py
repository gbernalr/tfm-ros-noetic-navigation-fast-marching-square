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
        self.frame_map = rospy.get_param("~frame_map", "map")
        self.frame_base = rospy.get_param("~frame_base", "base_link")
        self.inflation = int(rospy.get_param("~inflate", 0))
        self.lookahead_dist = float(rospy.get_param("~lookahead", 0.35))
        self.v_lin = float(rospy.get_param("~v_lin", 0.22))
        self.v_ang_max = float(rospy.get_param("~v_ang_max", 1.5))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.08))
        self.rate_hz = int(rospy.get_param("~rate", 20))

        self.k_theta = float(rospy.get_param("~k_theta", 2.0))
        self.goal_yaw_tolerance = float(rospy.get_param("~goal_yaw_tolerance", 0.10))
        self.replan_offpath = float(rospy.get_param("~replan_offpath", 0.6))
        self.use_goal_yaw = bool(rospy.get_param("~use_goal_yaw", True))

        self.replan_period = float(rospy.get_param("~replan_period", 1.0))
        self.last_replan_time = rospy.Time.now()

        self.map_msg = None
        self.grid_bin = None
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.goal_world = None
        self.goal_yaw = None
        self.path_world = None
        self.path_idx = 0
        self.mode_align = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_map = rospy.Subscriber("fm2_costmap/costmap", OccupancyGrid, self.cb_map, queue_size=1)
        self.sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_amcl = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.cb_amcl, queue_size=1)
        self.pub_path = rospy.Publisher("fm2_path", Path, queue_size=1, latch=True)
        self.pub_cmd = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.have_amcl = False
        self.last_pose = None

    def check_pts_collisions(self, pts_world, binary=None, radius_cells=0):
        if binary is None:
            if self.grid_bin is None:
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

        return result

    def cb_map(self, msg: OccupancyGrid):
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

    def cb_goal(self, msg: PoseStamped):
        if msg.header.frame_id != self.frame_map:
            try:
                msg = self._transform_pose(msg, self.frame_map)
            except Exception:
                return

        self.goal_world = (msg.pose.position.x, msg.pose.position.y)

        if self.use_goal_yaw:
            self.goal_yaw = self._yaw_from_quat(msg.pose.orientation)
        else:
            self.goal_yaw = None

        self.mode_align = False
        self._plan_from_current_pose()

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id != self.frame_map:
            try:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = msg.pose.pose
                pose = self._transform_pose(pose, self.frame_map)
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self._yaw_from_quat(pose.pose.orientation)
            except Exception:
                return
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        self.last_pose = (x, y, yaw)
        self.have_amcl = True

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

    def _plan_from_current_pose(self):
        if self.grid_bin is None or self.goal_world is None:
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
            except Exception:
                return

        sx, sy, _ = self.last_pose
        gx, gy = self.goal_world

        start_ix, start_iy = self._world_to_grid(sx, sy)
        goal_ix, goal_iy = self._world_to_grid(gx, gy)

        binary = self.grid_bin.copy().astype(np.uint8)

        if binary.size == 0:
            return

        if self.inflation > 0:
            k = 2*self.inflation + 1
            kernel = np.ones((k, k), np.uint8)
            inv = 1 - binary
            inv = cv2.dilate(inv, kernel, iterations=1)
            binary = 1 - inv

        self.fm2 = FM2(mode="cpu")
        fm2_map = FM2Map.from_binary_map(binary, create_border=True)
        self.fm2.set_map(fm2_map)

        try:
            info: FM2Info = self.fm2.get_path(
                (int(start_ix), int(start_iy)),
                (int(goal_ix), int(goal_iy))
            )
        except IndexError:
            self.path_world = None
            return

        if info.path is None:
            self.path_world = None
            return

        xs, ys = info.path
        pts = [self._grid_to_world(int(ix), int(iy)) for ix, iy in zip(xs, ys)]

        self.check_pts_collisions(pts, binary=binary, radius_cells=0)

        self.path_world = pts
        self.path_idx = 0
        self.mode_align = False
        self._publish_path(pts)

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
                self._stop()
                self.path_world = None
                self.mode_align = False
                return

            e_yaw = self._wrap_to_pi(self.goal_yaw - yaw)
            if abs(e_yaw) < self.goal_yaw_tolerance:
                self._stop()
                self.path_world = None
                self.goal_world = None
                self.goal_yaw = None
                self.mode_align = False
                return

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = float(np.clip(self.k_theta * e_yaw,
                                            -self.v_ang_max,
                                            self.v_ang_max))
            self.pub_cmd.publish(twist)
            return

        if self.goal_world is not None and self.grid_bin is not None and self.last_pose is not None:
            now = rospy.Time.now()
            if self.replan_period <= 0.0 or (now - self.last_replan_time).to_sec() >= self.replan_period:
                self.last_replan_time = now
                self._plan_from_current_pose()

        if self.path_world is None or self.last_pose is None:
            return

        x, y, yaw = self.last_pose

        if self.path_world:
            window = self.path_world[self.path_idx: min(self.path_idx+30, len(self.path_world))]
            if window:
                dmin = min(np.hypot(px - x, py - y) for (px, py) in window)
                if dmin > self.replan_offpath:
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
                if self.use_goal_yaw and self.goal_yaw is not None:
                    self.mode_align = True
                    self._stop()
                else:
                    self._stop()
                    self.path_world = None
                return
            else:
                target = (goal_x, goal_y)

        self._track_target(x, y, yaw, target)

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

    def _stop(self):
        self.pub_cmd.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("fm2_navigator")
    node = FM2Navigator()
    rospy.loginfo("FM2 Navigator listo. Pulsa 2D Nav Goal en RViz.")
    node.spin()
