#!/usr/bin/env python3
"""
kalman_tracker.py — Filtro de Kalman lineal 4 estados + tracker multi-objeto.

Estado:  [x, y, vx, vy]  (posición y velocidad en el plano del suelo)
Medida:  [x, y]

Asociación de detecciones a tracks vía algoritmo Húngaro
(scipy.optimize.linear_sum_assignment).
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

# ─────────────────────────────────────────────────────────────────────
# Detección
# ─────────────────────────────────────────────────────────────────────

class Detection:
    """Detección métrica en el plano del suelo."""

    __slots__ = [
        "xy_ground",   # np.ndarray (2,)  [x, y] metros
        "conf_pose",   # float
        "conf_depth",  # float
        "R",           # np.ndarray (2,2) covarianza de medida
        "ref_2d",      # tuple (u, v) píxeles
        "ref_type",    # str: "pelvis" | "torso" | "ankles"
        "bbox",        # PersonBBox o None (para visualización)
    ]

    def __init__(self, xy_ground, conf_pose, conf_depth, R, ref_2d, ref_type,
                 bbox=None):
        self.xy_ground = np.asarray(xy_ground, dtype=np.float64)
        self.conf_pose = conf_pose
        self.conf_depth = conf_depth
        self.R = np.asarray(R, dtype=np.float64)
        self.ref_2d = ref_2d
        self.ref_type = ref_type
        self.bbox = bbox


# ─────────────────────────────────────────────────────────────────────
# Track
# ─────────────────────────────────────────────────────────────────────

class Track:
    """Track individual con estado Kalman."""

    def __init__(self, track_id: int, xy_initial: np.ndarray, dt: float,
                 q_pos: float = 0.05, q_vel: float = 0.20):
        self.id = track_id
        self.age = 1
        self.hits = 1
        self.misses = 0
        self.confirmed = False
        self.history = [xy_initial.copy()]
        self.last_ref_type = None

        # ── Kalman ──
        # Estado: [x, y, vx, vy]
        self.x = np.array([xy_initial[0], xy_initial[1], 0.0, 0.0],
                          dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 1.0

        # Matrices del modelo
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0,  dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=np.float64)

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

        self.Q = np.array([
            [q_pos, 0,     0,     0],
            [0,     q_pos, 0,     0],
            [0,     0,     q_vel, 0],
            [0,     0,     0,     q_vel],
        ], dtype=np.float64)

    # ── Predicción ──
    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    # ── Actualización ──
    def update(self, z: np.ndarray, R: np.ndarray):
        """z: medida (2,), R: covarianza de medida (2,2)."""
        y = z - self.H @ self.x                       # innovación
        S = self.H @ self.P @ self.H.T + R            # covarianza innovación
        K = self.P @ self.H.T @ np.linalg.inv(S)      # ganancia Kalman
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

    @property
    def position(self) -> np.ndarray:
        """Posición filtrada (x, y)."""
        return self.x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Velocidad filtrada (vx, vy)."""
        return self.x[2:4].copy()


# ─────────────────────────────────────────────────────────────────────
# Covarianza de medida adaptativa
# ─────────────────────────────────────────────────────────────────────

def build_measurement_cov(conf_pose: float, conf_depth: float) -> np.ndarray:
    """
    Genera R (2×2) adaptativa según la confianza de pose y profundidad.
    Menos confianza → más varianza.
    """
    c = 0.5 * conf_pose + 0.5 * conf_depth
    # lerp entre 0.05 (alta confianza) y 0.40 (baja confianza)
    sigma = 0.05 + (0.40 - 0.05) * (1.0 - c)
    return np.array([
        [sigma * sigma, 0.0],
        [0.0, sigma * sigma],
    ], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────
# Tracker Manager
# ─────────────────────────────────────────────────────────────────────

class TrackerManager:
    """
    Gestiona múltiples tracks: predicción, asociación húngara,
    actualización, creación y eliminación.
    """

    def __init__(
        self,
        dt: float = 1.0 / 15.0,
        max_assoc_dist: float = 1.0,
        max_misses: int = 10,
        min_hits_to_confirm: int = 3,
        q_pos: float = 0.05,
        q_vel: float = 0.20,
    ):
        self.dt = dt
        self.max_assoc_dist = max_assoc_dist
        self.max_misses = max_misses
        self.min_hits_to_confirm = min_hits_to_confirm
        self.q_pos = q_pos
        self.q_vel = q_vel

        self.tracks = []          # list[Track]
        self._next_id = 0

    # ─────────────────────────────────────────────────────────────────
    def step(self, detections: list) -> list:
        """
        Ejecuta un ciclo completo del tracker.

        Parámetros
        ----------
        detections : list[Detection]

        Retorna
        -------
        list[Track]  — tracks activos (incluidos los no confirmados).
        """
        # 1) Predicción
        self._predict_all()

        # 2) Asociación
        matches, unmatched_trk, unmatched_det = self._associate(detections)

        # 3) Actualizar emparejados
        self._update_matched(matches, detections)

        # 4) Tracks no observados
        self._handle_unmatched_tracks(unmatched_trk)

        # 5) Nuevas tracks
        self._create_new_tracks(unmatched_det, detections)

        # 6) Eliminar tracks muertos
        self._prune()

        return self.tracks

    # ─────────────────────────────────────────────────────────────────
    def _predict_all(self):
        for tr in self.tracks:
            tr.predict()
            tr.age += 1
            tr.misses += 1

    # ─────────────────────────────────────────────────────────────────
    def _associate(self, detections):
        n_trk = len(self.tracks)
        n_det = len(detections)

        if n_trk == 0 or n_det == 0:
            return [], list(range(n_trk)), list(range(n_det))

        INF = 1e9
        cost = np.full((n_trk, n_det), INF, dtype=np.float64)

        for i, tr in enumerate(self.tracks):
            pred_xy = tr.position
            for j, det in enumerate(detections):
                d = np.linalg.norm(pred_xy - det.xy_ground)
                if d <= self.max_assoc_dist:
                    cost[i, j] = d

        row_idx, col_idx = linear_sum_assignment(cost)

        matches = []
        unmatched_trk = set(range(n_trk))
        unmatched_det = set(range(n_det))

        for i, j in zip(row_idx, col_idx):
            if cost[i, j] < INF:
                matches.append((i, j))
                unmatched_trk.discard(i)
                unmatched_det.discard(j)

        return matches, list(unmatched_trk), list(unmatched_det)

    # ─────────────────────────────────────────────────────────────────
    def _update_matched(self, matches, detections):
        for i, j in matches:
            tr = self.tracks[i]
            det = detections[j]

            tr.update(det.xy_ground, det.R)
            tr.hits += 1
            tr.misses = 0
            tr.last_ref_type = det.ref_type
            tr.history.append(tr.position)

            if tr.hits >= self.min_hits_to_confirm:
                tr.confirmed = True

    # ─────────────────────────────────────────────────────────────────
    def _handle_unmatched_tracks(self, indices):
        for i in indices:
            tr = self.tracks[i]
            # Solo predicción (ya hecha), guardar posición predicha
            tr.history.append(tr.position)

    # ─────────────────────────────────────────────────────────────────
    def _create_new_tracks(self, indices, detections):
        for j in indices:
            det = detections[j]
            tr = Track(
                track_id=self._next_id,
                xy_initial=det.xy_ground,
                dt=self.dt,
                q_pos=self.q_pos,
                q_vel=self.q_vel,
            )
            tr.last_ref_type = det.ref_type
            self.tracks.append(tr)
            self._next_id += 1

    # ─────────────────────────────────────────────────────────────────
    def _prune(self):
        self.tracks = [tr for tr in self.tracks if tr.misses <= self.max_misses]
