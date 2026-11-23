"""Analytic specific force and gyro, then optional Gaussian noise."""

from __future__ import annotations

import math

import numpy as np

DT_S = 0.005
G_W = (0.0, 0.0, -9.81)


def rest_sample() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Stationary prior at t_ns = 0, before noise."""
    return (0.0, 0.0, 0.0), (0.0, 0.0, 9.81)


def true_sample(
    v: float,
    v_prev: float,
    omega: float,
    yaw: float,
    dt: float = DT_S,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Gyro and specific force at the end of an interval, before noise.

    yaw is the heading at the end of the interval. v is the forward speed on
    this interval and v_prev is the speed on the previous one.
    """
    accel_b = ((v - v_prev) / dt, omega * v, 0.0)
    gravity_b = _transpose_yaw(yaw, G_W)
    specific = (
        accel_b[0] - gravity_b[0],
        accel_b[1] - gravity_b[1],
        accel_b[2] - gravity_b[2],
    )
    return (0.0, 0.0, omega), specific


class ImuNoise:
    def __init__(self, seed: int, gyro_density: float, accel_density: float, rate_hz: float) -> None:
        self._generator = np.random.Generator(np.random.PCG64(seed))
        self._sigma_g = gyro_density * math.sqrt(rate_hz)
        self._sigma_a = accel_density * math.sqrt(rate_hz)

    def apply(
        self,
        gyro: tuple[float, float, float],
        accel: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        draw = self._generator.standard_normal(6)
        noisy_gyro = (
            gyro[0] + self._sigma_g * float(draw[0]),
            gyro[1] + self._sigma_g * float(draw[1]),
            gyro[2] + self._sigma_g * float(draw[2]),
        )
        noisy_accel = (
            accel[0] + self._sigma_a * float(draw[3]),
            accel[1] + self._sigma_a * float(draw[4]),
            accel[2] + self._sigma_a * float(draw[5]),
        )
        return noisy_gyro, noisy_accel


def _transpose_yaw(
    yaw: float,
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """R(yaw)^T applied to a world vector. R is a pure yaw about +Z."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x, y, z = vector
    return (cosine * x + sine * y, -sine * x + cosine * y, z)
