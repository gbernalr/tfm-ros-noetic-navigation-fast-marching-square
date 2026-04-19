#!/usr/bin/env python3
"""
person_detector.py — Detección de personas (bounding boxes) con YOLOv8.

Utiliza YOLOv8n (nano) para detectar personas en la imagen completa.
Devuelve una lista de bounding boxes con su confianza.
"""

import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError(
        "ultralytics no está instalado. Ejecuta: pip install ultralytics"
    )


class PersonBBox:
    """Bounding box de una persona detectada."""

    __slots__ = ["x1", "y1", "x2", "y2", "conf"]

    def __init__(self, x1: float, y1: float, x2: float, y2: float, conf: float):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.conf = conf

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center(self):
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


class PersonDetector:
    """Detector de personas basado en YOLOv8."""

    # Clase COCO 0 = 'person'
    _PERSON_CLASS = 0

    def __init__(
        self,
        model_name: str = "yolov8n.pt",
        conf_threshold: float = 0.4,
        device: str = "cpu",
        imgsz: int = 320,
    ):
        """
        Parámetros
        ----------
        model_name : str
            Modelo YOLOv8 a cargar (se descarga automáticamente).
        conf_threshold : float
            Umbral de confianza mínimo para aceptar una detección.
        device : str
            "cpu" o "cuda:0", etc.
        imgsz : int
            Tamaño de entrada para YOLO (menor = más rápido). Default 320.
        """
        self._model = YOLO(model_name)
        self._conf_threshold = conf_threshold
        self._device = device
        self._imgsz = imgsz

    def detect(self, rgb: np.ndarray) -> list:
        """
        Detecta personas en una imagen RGB (H, W, 3) uint8.

        Retorna
        -------
        list[PersonBBox]
            Lista de bounding boxes de personas detectadas.
        """
        results = self._model.predict(
            source=rgb,
            classes=[self._PERSON_CLASS],
            conf=self._conf_threshold,
            device=self._device,
            imgsz=self._imgsz,
            verbose=False,
        )

        bboxes = []
        if len(results) > 0 and results[0].boxes is not None:
            for box in results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                bboxes.append(PersonBBox(
                    x1=float(xyxy[0]),
                    y1=float(xyxy[1]),
                    x2=float(xyxy[2]),
                    y2=float(xyxy[3]),
                    conf=conf,
                ))

        return bboxes
