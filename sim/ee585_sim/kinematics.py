"""Closed-form planar unicycle and the provider clock."""

from __future__ import annotations

import math

import numpy as np


def propagate(
    x: float,
    y: float,
    yaw: float,
    v: float,
    omega: float,
    tau: float,
) -> tuple[float, float, float]:
    """Advance (x, y, yaw) by tau seconds at constant body twist (v, omega)."""
    if abs(omega) < 1e-12:
        return (
            x + v * math.cos(yaw) * tau,
            y + v * math.sin(yaw) * tau,
            yaw,
        )
    yaw_next = yaw + omega * tau
    return (
        x + (v / omega) * (math.sin(yaw_next) - math.sin(yaw)),
        y + (v / omega) * (-math.cos(yaw_next) + math.cos(yaw)),
        yaw_next,
    )


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Hamilton (w, x, y, z) for a yaw about world +Z. w >= 0."""
    half = 0.5 * yaw
    w = math.cos(half)
    z = math.sin(half)
    if w < 0.0:
        w = -w
        z = -z
    return (w, 0.0, 0.0, z)


def quat_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Hamilton quaternion for a rotation matrix. w >= 0."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (float(w), float(x), float(y), float(z))


def imu_stamp(index: int) -> int:
    return index * 5_000_000


def camera_stamp(index: int) -> int:
    return ((index * 1000 + 15) // 30) * 1_000_000
