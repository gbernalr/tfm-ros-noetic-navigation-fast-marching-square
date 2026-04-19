#!/usr/bin/env python3
import math
from collections import defaultdict, deque

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray


class ConstantAccelerationKalman2D:
    def __init__(self, process_noise=1.0, measurement_noise=0.08):
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.x = np.zeros((6, 1), dtype=float)
        self.P = np.diag([0.3, 0.3, 1.0, 1.0, 2.0, 2.0]).astype(float)
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        self.R = np.eye(2, dtype=float) * (self.measurement_noise ** 2)

    @staticmethod
    def transition(dt):
        dt2 = dt * dt
        return np.array(
            [
                [1.0, 0.0, dt, 0.0, 0.5 * dt2, 0.0],
                [0.0, 1.0, 0.0, dt, 0.0, 0.5 * dt2],
                [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def process_covariance(self, dt):
        pos_noise = (0.5 * dt * dt * self.process_noise) ** 2
        vel_noise = (dt * self.process_noise) ** 2
        acc_noise = self.process_noise ** 2
        return np.diag(
            [pos_noise, pos_noise, vel_noise, vel_noise, acc_noise, acc_noise]
        ).astype(float)

    def initialize(self, x, y):
        self.x[:, 0] = [x, y, 0.0, 0.0, 0.0, 0.0]
        self.P = np.diag([0.2, 0.2, 1.0, 1.0, 2.0, 2.0]).astype(float)

    def predict(self, dt):
        dt = max(float(dt), 1e-3)
        f = self.transition(dt)
        q = self.process_covariance(dt)
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q

    def update(self, meas_x, meas_y):
        z = np.array([[meas_x], [meas_y]], dtype=float)
        y = z - self.H @ self.x
        s = self.H @ self.P @ self.H.T + self.R
        k = self.P @ self.H.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        i = np.eye(6, dtype=float)
        self.P = (i - k @ self.H) @ self.P

    def predict_positions(self, horizon, dt):
        f = self.transition(dt)
        state = self.x.copy()
        out = []
        steps = max(int(math.ceil(horizon / dt)), 1)
        for _ in range(steps):
            state = f @ state
            out.append((float(state[0, 0]), float(state[1, 0])))
        return out

    @property
    def position(self):
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self):
        return float(self.x[2, 0]), float(self.x[3, 0])

    @property
    def acceleration(self):
        return float(self.x[4, 0]), float(self.x[5, 0])


class PersonTrackerNode:
    def __init__(self):
        rospy.init_node("person_tracker")

        self.tracking_frame = rospy.get_param("~tracking_frame", "world")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/camera/depth/points")

        self.roi_x_min = float(rospy.get_param("~roi_x_min", 0.3))
        self.roi_x_max = float(rospy.get_param("~roi_x_max", 8.0))
        self.roi_y_min = float(rospy.get_param("~roi_y_min", -3.0))
        self.roi_y_max = float(rospy.get_param("~roi_y_max", 3.0))
        self.ground_z_max = float(rospy.get_param("~ground_z_max", 0.15))
        self.person_z_min = float(rospy.get_param("~person_z_min", 0.2))
        self.person_z_max = float(rospy.get_param("~person_z_max", 2.2))

        self.grid_resolution = float(rospy.get_param("~grid_resolution", 0.12))
        self.cloud_stride = max(int(rospy.get_param("~cloud_stride", 4)), 1)
        self.cluster_min_points = int(rospy.get_param("~cluster_min_points", 30))
        self.min_points_per_cell = int(rospy.get_param("~min_points_per_cell", 1))
        self.init_confirmations = int(rospy.get_param("~init_confirmations", 3))
        self.track_timeout = float(rospy.get_param("~track_timeout", 0.7))
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 2.0))
        self.prediction_dt = float(rospy.get_param("~prediction_dt", 0.2))

        self.process_noise = float(rospy.get_param("~process_noise", 1.0))
        self.measurement_noise = float(rospy.get_param("~measurement_noise", 0.08))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.15))

        self.kalman = ConstantAccelerationKalman2D(
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.track_active = False
        self.pending_detection = None
        self.pending_count = 0
        self.last_filter_stamp = None
        self.last_measurement_stamp = None
        self.last_detection = None
        self.last_cluster_size = 0

        self.pub_detection = rospy.Publisher(
            "/person_tracking/detection", PoseStamped, queue_size=1
        )
        self.pub_track = rospy.Publisher(
            "/person_tracking/track", PoseStamped, queue_size=1
        )
        self.pub_velocity = rospy.Publisher(
            "/person_tracking/velocity", TwistStamped, queue_size=1
        )
        self.pub_predicted_path = rospy.Publisher(
            "/person_tracking/predicted_path", Path, queue_size=1
        )
        self.pub_markers = rospy.Publisher(
            "/person_tracking/markers", MarkerArray, queue_size=1
        )

        self.sub_cloud = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1
        )

    def cloud_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        detection = self.detect_person(msg, stamp)

        if self.track_active:
            dt = self.compute_dt(stamp)
            self.kalman.predict(dt)
            self.last_filter_stamp = stamp

            if detection is not None:
                self.kalman.update(detection[0], detection[1])
                self.last_measurement_stamp = stamp
                self.last_detection = detection
                self.last_cluster_size = detection[2]
            elif self.last_measurement_stamp is not None:
                age = (stamp - self.last_measurement_stamp).to_sec()
                if age > self.track_timeout:
                    self.clear_track()
        else:
            if detection is not None:
                self.update_pending_track(detection, stamp)
            else:
                self.pending_detection = None
                self.pending_count = 0

        if detection is not None:
            self.publish_detection(detection, stamp)

        self.publish_outputs(stamp, detection)

    def compute_dt(self, stamp):
        if self.last_filter_stamp is None:
            return self.prediction_dt
        dt = (stamp - self.last_filter_stamp).to_sec()
        return min(max(dt, 1e-3), 0.5)

    def update_pending_track(self, detection, stamp):
        xy = (detection[0], detection[1])
        if self.pending_detection is None:
            self.pending_detection = xy
            self.pending_count = 1
        else:
            dx = xy[0] - self.pending_detection[0]
            dy = xy[1] - self.pending_detection[1]
            if math.hypot(dx, dy) < 0.6:
                self.pending_detection = xy
                self.pending_count += 1
            else:
                self.pending_detection = xy
                self.pending_count = 1

        if self.pending_count >= self.init_confirmations:
            self.kalman.initialize(xy[0], xy[1])
            self.track_active = True
            self.last_filter_stamp = stamp
            self.last_measurement_stamp = stamp
            self.last_detection = detection
            self.last_cluster_size = detection[2]
            self.pending_detection = None
            self.pending_count = 0
            rospy.loginfo(
                "Track inicializado en %s con deteccion (%.2f, %.2f)",
                self.tracking_frame,
                xy[0],
                xy[1],
            )

    def clear_track(self):
        self.track_active = False
        self.pending_detection = None
        self.pending_count = 0
        self.last_filter_stamp = None
        self.last_measurement_stamp = None
        self.last_detection = None
        self.last_cluster_size = 0
        rospy.loginfo_throttle(2.0, "Track perdido por timeout")

    def detect_person(self, cloud_msg, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tracking_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp if cloud_msg.header.stamp != rospy.Time() else rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
            cloud_in_tracking = do_transform_cloud(cloud_msg, transform)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "No se pudo transformar la nube %s -> %s: %s",
                cloud_msg.header.frame_id,
                self.tracking_frame,
                exc,
            )
            return None

        cell_stats = defaultdict(lambda: [0, 0.0, 0.0])

        for idx, point in enumerate(
            pc2.read_points(
                cloud_in_tracking,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if idx % self.cloud_stride != 0:
                continue

            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            if x < self.roi_x_min or x > self.roi_x_max:
                continue
            if y < self.roi_y_min or y > self.roi_y_max:
                continue
            if z <= self.ground_z_max:
                continue
            if z < self.person_z_min or z > self.person_z_max:
                continue

            cell_x = int(math.floor((x - self.roi_x_min) / self.grid_resolution))
            cell_y = int(math.floor((y - self.roi_y_min) / self.grid_resolution))
            stats = cell_stats[(cell_x, cell_y)]
            stats[0] += 1
            stats[1] += x
            stats[2] += y

        occupied = {
            cell for cell, stats in cell_stats.items() if stats[0] >= self.min_points_per_cell
        }
        if not occupied:
            return None

        best_cluster = None
        visited = set()
        for start in occupied:
            if start in visited:
                continue

            queue = deque([start])
            visited.add(start)
            total_points = 0
            sum_x = 0.0
            sum_y = 0.0

            while queue:
                cell = queue.popleft()
                count, cell_sum_x, cell_sum_y = cell_stats[cell]
                total_points += count
                sum_x += cell_sum_x
                sum_y += cell_sum_y

                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        neigh = (cell[0] + dx, cell[1] + dy)
                        if neigh in occupied and neigh not in visited:
                            visited.add(neigh)
                            queue.append(neigh)

            if total_points < self.cluster_min_points:
                continue

            if best_cluster is None or total_points > best_cluster[2]:
                best_cluster = (
                    sum_x / float(total_points),
                    sum_y / float(total_points),
                    total_points,
                )

        return best_cluster

    def publish_detection(self, detection, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.tracking_frame
        msg.pose.position.x = detection[0]
        msg.pose.position.y = detection[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub_detection.publish(msg)

    def publish_outputs(self, stamp, detection):
        if not self.track_active:
            self.publish_empty_outputs(stamp, detection)
            return

        track_x, track_y = self.kalman.position
        vel_x, vel_y = self.kalman.velocity
        predicted_points = [(track_x, track_y)]
        predicted_points.extend(
            self.kalman.predict_positions(self.prediction_horizon, self.prediction_dt)
        )

        track_msg = PoseStamped()
        track_msg.header.stamp = stamp
        track_msg.header.frame_id = self.tracking_frame
        track_msg.pose.position.x = track_x
        track_msg.pose.position.y = track_y
        track_msg.pose.position.z = 0.0
        track_msg.pose.orientation.w = 1.0
        self.pub_track.publish(track_msg)

        vel_msg = TwistStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = self.tracking_frame
        vel_msg.twist.linear.x = vel_x
        vel_msg.twist.linear.y = vel_y
        self.pub_velocity.publish(vel_msg)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.tracking_frame
        for x, y in predicted_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_predicted_path.publish(path)
        self.pub_markers.publish(self.make_markers(stamp, detection, predicted_points))

    def publish_empty_outputs(self, stamp, detection):
        empty_path = Path()
        empty_path.header.stamp = stamp
        empty_path.header.frame_id = self.tracking_frame
        self.pub_predicted_path.publish(empty_path)
        self.pub_markers.publish(self.make_markers(stamp, detection, []))

    def make_markers(self, stamp, detection, predicted_points):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        if detection is not None:
            markers.markers.append(
                self.make_sphere_marker(
                    stamp,
                    marker_id=0,
                    namespace="detection",
                    x=detection[0],
                    y=detection[1],
                    z=1.0,
                    color=(1.0, 0.8, 0.1, 0.95),
                    scale=Vector3(0.22, 0.22, 0.22),
                )
            )

        if self.track_active:
            track_x, track_y = self.kalman.position
            vel_x, vel_y = self.kalman.velocity
            speed = math.hypot(vel_x, vel_y)

            markers.markers.append(
                self.make_sphere_marker(
                    stamp,
                    marker_id=1,
                    namespace="track",
                    x=track_x,
                    y=track_y,
                    z=1.0,
                    color=(0.1, 0.9, 0.4, 0.95),
                    scale=Vector3(0.28, 0.28, 0.28),
                )
            )

            markers.markers.append(
                self.make_arrow_marker(
                    stamp,
                    marker_id=2,
                    namespace="velocity",
                    x=track_x,
                    y=track_y,
                    vel_x=vel_x,
                    vel_y=vel_y,
                    speed=max(speed, 0.05),
                )
            )

            if predicted_points:
                markers.markers.append(
                    self.make_line_strip_marker(
                        stamp,
                        marker_id=3,
                        namespace="prediction",
                        points=predicted_points,
                    )
                )

        return markers

    def make_sphere_marker(self, stamp, marker_id, namespace, x, y, z, color, scale):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.tracking_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(0.3)
        return marker

    def make_arrow_marker(self, stamp, marker_id, namespace, x, y, vel_x, vel_y, speed):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.tracking_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.scale.y = 0.12
        marker.scale.z = 0.15
        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.3)

        start = Point(x=x, y=y, z=1.0)
        end = Point(x=x + vel_x * max(self.prediction_dt, 0.2), y=y + vel_y * max(self.prediction_dt, 0.2), z=1.0)
        if math.hypot(end.x - start.x, end.y - start.y) < 0.05:
            end.x += 0.05 * speed
        marker.points = [start, end]
        return marker

    def make_line_strip_marker(self, stamp, marker_id, namespace, points):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.tracking_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.2
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.3)
        marker.points = [Point(x=x, y=y, z=0.05) for x, y in points]
        return marker

    def spin(self):
        rospy.loginfo("Person tracker listo. Escuchando %s", self.cloud_topic)
        rospy.spin()


if __name__ == "__main__":
    PersonTrackerNode().spin()
