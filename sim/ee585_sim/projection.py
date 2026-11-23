"""Pinhole projection of landmark centers, with occlusion and pixel noise."""

from __future__ import annotations

import math
from typing import Any, NamedTuple

import numpy as np


class Observation(NamedTuple):
    landmark_id: int
    u: float
    v: float


class Projector:
    def __init__(self, calib: dict[str, Any], landmarks: np.ndarray, occluders: list[dict[str, Any]], seed: int) -> None:
        self._fx = float(calib["fx"])
        self._fy = float(calib["fy"])
        self._cx = float(calib["cx"])
        self._cy = float(calib["cy"])
        self._width = float(calib["width"])
        self._height = float(calib["height"])
        self._min_range = float(calib["min_range_m"])
        self._max_range = float(calib["max_range_m"])
        self._sigma = float(calib["pixel_noise_sigma"])
        self._p_BC = np.asarray(calib["p_BC"], dtype=np.float64)
        self._R_BC = rotation_matrix(np.asarray(calib["q_BC"], dtype=np.float64))
        self._landmarks = np.asarray(landmarks, dtype=np.float64)
        self._occluders = [
            {
                "center": np.asarray(box["center"], dtype=np.float64),
                "half": np.asarray(box["size"], dtype=np.float64) / 2.0,
                "yaw": float(box["yaw"]),
            }
            for box in occluders
        ]
        self._generator = np.random.Generator(np.random.PCG64(seed))

    def project(
        self,
        t_ns: int,
        p_WB: np.ndarray,
        q_WB: np.ndarray,
    ) -> tuple[list[Observation], list[Observation]]:
        del t_ns
        position = np.asarray(p_WB, dtype=np.float64)
        rotation = rotation_matrix(np.asarray(q_WB, dtype=np.float64))
        p_WC = rotation @ self._p_BC + position
        R_WC = rotation @ self._R_BC
        visible: list[tuple[int, float, float]] = []
        for index, landmark in enumerate(self._landmarks):
            point = R_WC.T @ (landmark - p_WC)
            if not self._in_frame(point):
                continue
            if self._occluded(p_WC, landmark):
                continue
            visible.append((index, self._fx * point[0] / point[2] + self._cx, self._fy * point[1] / point[2] + self._cy))
        noisy: list[Observation] = []
        truth: list[Observation] = []
        for index, u, v in visible:
            u_noise = float(self._generator.standard_normal()) * self._sigma
            v_noise = float(self._generator.standard_normal()) * self._sigma
            truth.append(Observation(index, u, v))
            noisy.append(Observation(index, u + u_noise, v + v_noise))
        return noisy, truth

    def _in_frame(self, point: np.ndarray) -> bool:
        if point[2] <= 0.0:
            return False
        if not (self._min_range <= float(np.linalg.norm(point)) <= self._max_range):
            return False
        u = self._fx * point[0] / point[2] + self._cx
        v = self._fy * point[1] / point[2] + self._cy
        return 0.0 <= u < self._width and 0.0 <= v < self._height

    def _occluded(self, origin: np.ndarray, target: np.ndarray) -> bool:
        return any(_segment_hits(origin, target, box["center"], box["half"], box["yaw"]) for box in self._occluders)


def rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Hamilton (w, x, y, z) to the roadmap rotation matrix."""
    w, x, y, z = (float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3]))
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _segment_hits(
    origin: np.ndarray,
    target: np.ndarray,
    center: np.ndarray,
    half: np.ndarray,
    yaw: float,
) -> bool:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)

    def into_box(point: np.ndarray) -> np.ndarray:
        delta = point - center
        return np.array(
            [cosine * delta[0] + sine * delta[1], -sine * delta[0] + cosine * delta[1], delta[2]],
            dtype=np.float64,
        )

    start = into_box(origin)
    direction = into_box(target) - start
    s_max = 1.0 - 1e-6
    t_min = 0.0
    t_max = s_max
    for axis in range(3):
        if abs(float(direction[axis])) <= 1e-12:
            if start[axis] < -half[axis] or start[axis] > half[axis]:
                return False
            continue
        t1 = (-half[axis] - start[axis]) / direction[axis]
        t2 = (half[axis] - start[axis]) / direction[axis]
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, float(t1))
        t_max = min(t_max, float(t2))
        if t_min > t_max:
            return False
    return t_max > 0.0 and t_min < s_max
