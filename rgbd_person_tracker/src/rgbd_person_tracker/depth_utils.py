#!/usr/bin/env python3
"""
depth_utils.py — Utilidades de profundidad: lectura robusta y backprojection.

- robust_depth(): extrae profundidad mediana de un parche, con filtrado
  de píxeles inválidos y cálculo de confianza basada en desviación estándar.
- backproject_to_3d(): des-proyecta un píxel (u, v, z) a coordenadas 3D
  en el frame de la cámara usando los intrínsecos K.
"""

import numpy as np


def robust_depth(
    depth_img: np.ndarray,
    u: float,
    v: float,
    patch_size: int = 5,
    depth_min: float = 0.5,
    depth_max: float = 8.0,
):
    """
    Obtiene una profundidad robusta (mediana) de un parche alrededor de (u, v).

    Parámetros
    ----------
    depth_img : np.ndarray (H, W), float32
        Mapa de profundidad en metros.
    u, v : float
        Centro del parche en coordenadas de píxel.
    patch_size : int
        Tamaño del parche cuadrado (e.g. 5 → parche 5×5).
    depth_min, depth_max : float
        Rango de profundidad válida en metros.

    Retorna
    -------
    z_median : float o None
        Profundidad mediana en metros. None si no hay suficientes datos.
    conf_depth : float
        Confianza ∈ [0, 1] basada en la dispersión. 0.0 si z_median es None.
    """
    h, w = depth_img.shape[:2]

    ui = int(round(u))
    vi = int(round(v))

    half = patch_size // 2
    y0 = max(0, vi - half)
    y1 = min(h, vi + half + 1)
    x0 = max(0, ui - half)
    x1 = min(w, ui + half + 1)

    patch = depth_img[y0:y1, x0:x1].flatten()

    # Filtrar NaN, Inf y fuera de rango
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid >= depth_min) & (valid <= depth_max)]

    min_valid = 0.4 * patch_size * patch_size
    if len(valid) < min_valid:
        return None, 0.0

    z_med = float(np.median(valid))
    z_std = float(np.std(valid))

    # Confianza: más dispersión → menos confianza
    # conf = max(0, 1 - clamp(std / 0.3, 0, 1))
    conf_depth = max(0.0, 1.0 - min(z_std / 0.3, 1.0))

    return z_med, conf_depth


def backproject_to_3d(u: float, v: float, z: float, K: dict) -> np.ndarray:
    """
    Des-proyecta un píxel (u, v) con profundidad z a un punto 3D
    en el frame de la cámara.

    Parámetros
    ----------
    u, v : float
        Coordenadas de píxel.
    z : float
        Profundidad en metros.
    K : dict
        Intrínsecos de la cámara: {'fx', 'fy', 'cx', 'cy'}.

    Retorna
    -------
    np.ndarray, shape (3,)
        Punto 3D [X, Y, Z] en el frame óptico de la cámara.
    """
    X = (u - K["cx"]) * z / K["fx"]
    Y = (v - K["cy"]) * z / K["fy"]
    Z = z
    return np.array([X, Y, Z], dtype=np.float64)
