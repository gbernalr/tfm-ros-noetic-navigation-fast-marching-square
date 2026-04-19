#!/usr/bin/env python3
"""
pose_detector.py — Estimación de pose 2D multi-persona.

Flujo:
  1. PersonDetector (YOLOv8) detecta bounding boxes de personas.
  2. Para cada bbox se recorta la imagen y se pasa a MediaPipe Pose.
  3. Los keypoints se re-proyectan a coordenadas de la imagen completa.

Esto supera la limitación de MediaPipe Pose (single-person) permitiendo
trackear N personas simultáneamente.
"""

import numpy as np

try:
    import mediapipe as mp
except ImportError:
    raise ImportError(
        "mediapipe no está instalado. Ejecuta: pip install mediapipe"
    )

from rgbd_person_tracker.person_detector import PersonDetector, PersonBBox


# Índices de MediaPipe Pose landmarks que nos interesan
_LANDMARK_MAP = {
    "left_shoulder":  11,
    "right_shoulder": 12,
    "left_hip":       23,
    "right_hip":      24,
    "left_ankle":     27,
    "right_ankle":    28,
}

# Margen relativo para ampliar el crop alrededor del bbox
_CROP_MARGIN = 0.1


class Pose:
    """Resultado de una detección de pose."""

    def __init__(self, keypoints: dict, bbox=None):
        """
        keypoints: dict  {nombre: (u_px, v_px, confianza)}
            Coordenadas en la imagen completa (no en el crop).
        bbox: PersonBBox o None
            Bounding box de la persona asociada.
        """
        self.keypoints = keypoints
        self.bbox = bbox


class PoseDetector:
    """Detector de pose humana multi-persona: YOLO bbox + MediaPipe Pose."""

    def __init__(
        self,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        model_complexity: int = 0,
        yolo_model: str = "yolov8n.pt",
        yolo_conf: float = 0.4,
        yolo_device: str = "cpu",
        yolo_imgsz: int = 320,
    ):
        # Detector de personas (bounding boxes)
        self._person_detector = PersonDetector(
            model_name=yolo_model,
            conf_threshold=yolo_conf,
            device=yolo_device,
            imgsz=yolo_imgsz,
        )

        # MediaPipe Pose (se ejecuta por cada crop)
        self._mp_pose = mp.solutions.pose
        self._pose = self._mp_pose.Pose(
            static_image_mode=True,   # True porque cada crop es independiente
            model_complexity=model_complexity,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def detect(self, rgb: np.ndarray) -> list:
        """
        Detecta poses de todas las personas en una imagen RGB (H, W, 3) uint8.

        Retorna
        -------
        list[Pose]  — una Pose por persona detectada (0..N).
        """
        h, w = rgb.shape[:2]

        # 1) Detectar bounding boxes de personas
        bboxes = self._person_detector.detect(rgb)

        if len(bboxes) == 0:
            return []

        poses = []

        for bbox in bboxes:
            # 2) Recortar con margen
            margin_x = bbox.width * _CROP_MARGIN
            margin_y = bbox.height * _CROP_MARGIN

            cx1 = max(0, int(bbox.x1 - margin_x))
            cy1 = max(0, int(bbox.y1 - margin_y))
            cx2 = min(w, int(bbox.x2 + margin_x))
            cy2 = min(h, int(bbox.y2 + margin_y))

            crop = rgb[cy1:cy2, cx1:cx2]
            if crop.size == 0:
                continue

            crop_h, crop_w = crop.shape[:2]

            # 3) MediaPipe Pose sobre el crop
            results = self._pose.process(crop)
            if results.pose_landmarks is None:
                continue

            lms = results.pose_landmarks.landmark

            keypoints = {}
            for name, idx in _LANDMARK_MAP.items():
                lm = lms[idx]
                # Re-proyectar coordenadas normalizadas del crop a imagen completa
                u = lm.x * crop_w + cx1
                v = lm.y * crop_h + cy1
                conf = lm.visibility
                keypoints[name] = (u, v, conf)

            poses.append(Pose(keypoints, bbox=bbox))

        return poses

    def close(self):
        self._pose.close()
