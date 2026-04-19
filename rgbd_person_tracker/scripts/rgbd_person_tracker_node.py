#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import time
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from rgbd_person_tracker.pose_detector import PoseDetector
from rgbd_person_tracker.reference_selector import select_reference_point
from rgbd_person_tracker.depth_utils import robust_depth, backproject_to_3d
from rgbd_person_tracker.ground_projector import GroundProjector
from rgbd_person_tracker.kalman_tracker import Detection, TrackerManager, build_measurement_cov
from rgbd_person_tracker.visualizer import Visualizer


class RGBDPersonTrackerNode:
    def __init__(self):
        # Parametros de deteccion
        self.pose_conf_th = rospy.get_param("~pose_conf_threshold", 0.4)
        self.depth_min = rospy.get_param("~depth_min", 0.5)
        self.depth_max = rospy.get_param("~depth_max", 8.0)
        self.patch_size = rospy.get_param("~patch_size", 5)

        max_assoc_dist = rospy.get_param("~max_assoc_dist", 1.0)
        max_misses = rospy.get_param("~max_misses", 10)
        min_hits = rospy.get_param("~min_hits_to_confirm", 3)
        q_pos = rospy.get_param("~kalman/q_pos", 0.05)
        q_vel = rospy.get_param("~kalman/q_vel", 0.20)

        mp_det_conf = rospy.get_param("~mediapipe/min_detection_confidence", 0.5)
        mp_trk_conf = rospy.get_param("~mediapipe/min_tracking_confidence", 0.5)
        mp_complexity = rospy.get_param("~mediapipe/model_complexity", 1)

        yolo_model = rospy.get_param("~yolo/model", "yolov8n.pt")
        yolo_conf = rospy.get_param("~yolo/conf_threshold", 0.4)
        yolo_device = rospy.get_param("~yolo/device", "cpu")
        yolo_imgsz = rospy.get_param("~yolo/imgsz", 320)

        self.process_every_n = rospy.get_param("~process_every_n", 2)
        self._frame_count = 0

        optical_frame = rospy.get_param("~optical_frame", "camera_depth_optical_frame")
        world_frame = rospy.get_param("~world_frame", "world")

        tv_w = rospy.get_param("~visualization/top_view_width", 500)
        tv_h = rospy.get_param("~visualization/top_view_height", 300)
        tv_mx = rospy.get_param("~visualization/top_view_meters_x", 10.0)
        tv_my = rospy.get_param("~visualization/top_view_meters_y", 6.0)
        tv_ox = rospy.get_param("~visualization/top_view_origin_x", 0.0)
        tv_oy = rospy.get_param("~visualization/top_view_origin_y", -3.0)

        rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )

        # Modulos
        self.bridge = CvBridge()

        rospy.loginfo("Inicializando PoseDetector (YOLO + MediaPipe)...")
        self.pose_detector = PoseDetector(
            min_detection_confidence=mp_det_conf,
            min_tracking_confidence=mp_trk_conf,
            model_complexity=mp_complexity,
            yolo_model=yolo_model,
            yolo_conf=yolo_conf,
            yolo_device=yolo_device,
            yolo_imgsz=yolo_imgsz,
        )

        self.ground_projector = GroundProjector(
            optical_frame=optical_frame,
            world_frame=world_frame,
        )

        self.tracker = TrackerManager(
            dt=1.0 / 15.0,
            max_assoc_dist=max_assoc_dist,
            max_misses=max_misses,
            min_hits_to_confirm=min_hits,
            q_pos=q_pos,
            q_vel=q_vel,
        )

        self.visualizer = Visualizer(
            world_frame=world_frame,
            top_view_width=tv_w,
            top_view_height=tv_h,
            top_view_meters_x=tv_mx,
            top_view_meters_y=tv_my,
            top_view_origin_x=tv_ox,
            top_view_origin_y=tv_oy,
        )

        # Leer intrínsecos una vez
        self.K = None
        rospy.loginfo(f"Esperando CameraInfo en {camera_info_topic} ...")
        ci_msg = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=30.0)
        self._parse_camera_info(ci_msg)
        rospy.loginfo(
            "Intrínsecos: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            self.K["fx"], self.K["fy"], self.K["cx"], self.K["cy"],
        )

        # Calcular FOV horizontal y pasarlo al visualizer
        img_w = ci_msg.width if ci_msg.width > 0 else 2.0 * self.K["cx"]
        hfov_deg = 2.0 * np.degrees(np.arctan2(img_w / 2.0, self.K["fx"]))
        self.visualizer.hfov_rad = np.radians(hfov_deg)
        rospy.loginfo("HFOV camara: %.1f deg", hfov_deg)

        self._prev_stamp = None

        # Suscripciones sincronizadas RGB + Depth
        sub_rgb = message_filters.Subscriber(rgb_topic, Image)
        sub_depth = message_filters.Subscriber(depth_topic, Image)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth], queue_size=5, slop=0.1
        )
        self._sync.registerCallback(self._callback)

        rospy.loginfo("rgbd_person_tracker_node listo.")

    def _parse_camera_info(self, msg: CameraInfo):
        K = np.array(msg.K).reshape(3, 3)
        self.K = {
            "fx": K[0, 0],
            "fy": K[1, 1],
            "cx": K[0, 2],
            "cy": K[1, 2],
        }

    def _callback(self, rgb_msg: Image, depth_msg: Image):
        stamp = rgb_msg.header.stamp

        # dt dinamico
        if self._prev_stamp is not None:
            dt = (stamp - self._prev_stamp).to_sec()
            if dt > 0:
                self.tracker.dt = dt
                for tr in self.tracker.tracks:
                    tr.A[0, 2] = dt
                    tr.A[1, 3] = dt
        self._prev_stamp = stamp

        # Frame-skip
        self._frame_count += 1
        run_detection = (self._frame_count % self.process_every_n == 0)

        # Convertir imagenes
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"Error convirtiendo RGB: {e}")
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"Error convirtiendo Depth: {e}")
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)

        # Detecciones (o solo prediccion Kalman si frame saltado)
        t0 = time.time()
        detections = self._extract_detections(rgb, depth, stamp) if run_detection else []
        t_det = time.time() - t0

        # Ciclo tracker
        t1 = time.time()
        tracks = self.tracker.step(detections)
        t_trk = time.time() - t1

        # Visualizacion (solo si alguien escucha)
        t2 = time.time()
        self.visualizer.publish_all(bgr, tracks, detections, stamp)
        t_vis = time.time() - t2

        t_total = t_det + t_trk + t_vis
        rospy.loginfo_throttle(
            5.0,
            "[perf] total=%.0fms det=%.0fms trk=%.0fms vis=%.0fms "
            "(det=%s, #trk=%d, skip_n=%d)",
            t_total * 1000, t_det * 1000, t_trk * 1000, t_vis * 1000,
            "ON" if run_detection else "skip",
            len(tracks),
            self.process_every_n,
        )

    def _extract_detections(self, rgb, depth, stamp):
        detections = []
        poses = self.pose_detector.detect(rgb)

        for pose in poses:
            ref_2d, conf_pose, ref_type = select_reference_point(
                pose, self.pose_conf_th
            )
            if ref_2d is None:
                continue

            u, v = ref_2d

            z, conf_depth = robust_depth(
                depth, u, v,
                patch_size=self.patch_size,
                depth_min=self.depth_min,
                depth_max=self.depth_max,
            )
            if z is None:
                continue

            point3d_cam = backproject_to_3d(u, v, z, self.K)
            xy_ground = self.ground_projector.project_to_ground(
                point3d_cam, stamp
            )
            if xy_ground is None:
                continue

            R = build_measurement_cov(conf_pose, conf_depth)

            det = Detection(
                xy_ground=np.array(xy_ground),
                conf_pose=conf_pose,
                conf_depth=conf_depth,
                R=R,
                ref_2d=(u, v),
                ref_type=ref_type,
                bbox=getattr(pose, 'bbox', None),
            )
            detections.append(det)

        return detections

    def run(self):
        rospy.spin()
        self.pose_detector.close()


if __name__ == "__main__":
    rospy.init_node("rgbd_person_tracker_node")
    try:
        node = RGBDPersonTrackerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
