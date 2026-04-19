#!/usr/bin/env python3
"""
reference_selector.py — Selección del punto anatómico de referencia.

Prioridad:
  1) Pelvis  = media de caderas (left_hip + right_hip)
  2) Torso   = media de ≥ 2 puntos entre caderas y hombros
  3) Ankles  = media de tobillos (left_ankle + right_ankle)

Devuelve (u, v), confianza, tipo  o  (None, 0.0, None) si ningún punto
cumple el umbral de confianza.
"""

import numpy as np


def select_reference_point(pose, conf_threshold: float = 0.4):
    """
    Selecciona el punto de referencia anatómico más fiable de una pose.

    Parámetros
    ----------
    pose : Pose
        Objeto con atributo keypoints: dict {nombre: (u, v, conf)}.
    conf_threshold : float
        Umbral mínimo de confianza para considerar un keypoint válido.

    Retorna
    -------
    ref_2d : tuple(float, float) o None
        Coordenadas (u, v) en píxeles del punto de referencia.
    conf : float
        Confianza combinada (mín de los puntos usados).
    ref_type : str o None
        "pelvis", "torso" o "ankles".
    """
    kps = pose.keypoints

    # ── 1) Pelvis: media de caderas ──────────────────────────────────
    lhip = kps.get("left_hip")
    rhip = kps.get("right_hip")

    if lhip is not None and rhip is not None:
        if lhip[2] > conf_threshold and rhip[2] > conf_threshold:
            u = (lhip[0] + rhip[0]) / 2.0
            v = (lhip[1] + rhip[1]) / 2.0
            conf = min(lhip[2], rhip[2])
            return (u, v), conf, "pelvis"

    # ── 2) Torso: ≥ 2 puntos de caderas + hombros ───────────────────
    torso_names = ["left_hip", "right_hip", "left_shoulder", "right_shoulder"]
    torso_pts = []
    torso_confs = []

    for name in torso_names:
        kp = kps.get(name)
        if kp is not None and kp[2] > conf_threshold:
            torso_pts.append((kp[0], kp[1]))
            torso_confs.append(kp[2])

    if len(torso_pts) >= 2:
        u = np.mean([p[0] for p in torso_pts])
        v = np.mean([p[1] for p in torso_pts])
        conf = min(torso_confs)
        return (u, v), conf, "torso"

    # ── 3) Ankles: media de tobillos ─────────────────────────────────
    lank = kps.get("left_ankle")
    rank = kps.get("right_ankle")

    if lank is not None and rank is not None:
        if lank[2] > conf_threshold and rank[2] > conf_threshold:
            u = (lank[0] + rank[0]) / 2.0
            v = (lank[1] + rank[1]) / 2.0
            conf = min(lank[2], rank[2])
            return (u, v), conf, "ankles"

    # ── Ningún punto válido ──────────────────────────────────────────
    return None, 0.0, None
