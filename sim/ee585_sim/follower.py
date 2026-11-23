"""Pure pursuit on a map polyline."""

from __future__ import annotations

import math
from typing import Any


class Follower:
    def __init__(self, path: dict[str, Any], initial_rest_s: float) -> None:
        self._waypoints = [(float(point[0]), float(point[1])) for point in path["waypoints"]]
        self._speed = float(path["speed_mps"])
        self._accel = float(path["accel_max_mps2"])
        self._lookahead = float(path["lookahead_m"])
        self._tail_ns = int(round(float(path["tail_s"]) * 1e9))
        self._rest_ns = int(round(float(initial_rest_s) * 1e9))
        self._v = 0.0
        self._along = 0.0
        self._tail_start: int | None = None
        self._done = False

    @property
    def done(self) -> bool:
        return self._done

    def command(self, t_ns: int, x: float, y: float, yaw: float) -> tuple[float, float]:
        if self._done:
            return (0.0, 0.0)
        if self._tail_start is not None:
            if t_ns - self._tail_start >= self._tail_ns:
                self._done = True
            return (0.0, 0.0)
        if t_ns < self._rest_ns:
            return (0.0, 0.0)
        total, along, _point = _closest(self._waypoints, x, y, self._along)
        self._along = along
        remaining = total - along
        target = _point_along(self._waypoints, min(along + self._lookahead, total))
        if math.hypot(target[0] - x, target[1] - y) < 1e-9:
            alpha = 0.0
        else:
            alpha = _wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        omega = _clamp(1.5 * alpha, -1.0, 1.0)
        if remaining < 1e-3:
            v_target = 0.0
        else:
            d_stop = self._speed * self._speed / (2.0 * self._accel)
            v_target = self._speed * min(1.0, remaining / d_stop)
        step = self._accel * 0.005
        self._v = self._v + _clamp(v_target - self._v, -step, step)
        if remaining < 1e-3 and self._v == 0.0:
            self._tail_start = t_ns
            return (0.0, 0.0)
        return (self._v, omega)


def _closest(
    waypoints: list[tuple[float, float]],
    x: float,
    y: float,
    previous: float,
) -> tuple[float, float, tuple[float, float]]:
    best: tuple[float, float, tuple[float, float]] | None = None
    base = 0.0
    for index in range(len(waypoints) - 1):
        start = waypoints[index]
        end = waypoints[index + 1]
        length, point, travelled = _segment_point(start, end, x, y)
        along = base + travelled
        distance = math.hypot(point[0] - x, point[1] - y)
        if best is None or distance < best[0] - 1e-9 or (
            abs(distance - best[0]) <= 1e-9 and abs(along - previous) < abs(best[1] - previous)
        ):
            best = (distance, along, point)
        base += length
    if best is None:
        return 0.0, 0.0, waypoints[0]
    return base, best[1], best[2]


def _segment_point(
    start: tuple[float, float],
    end: tuple[float, float],
    x: float,
    y: float,
) -> tuple[float, tuple[float, float], float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-15:
        return 0.0, start, 0.0
    scale = ((x - start[0]) * dx + (y - start[1]) * dy) / (length * length)
    scale = min(1.0, max(0.0, scale))
    return length, (start[0] + scale * dx, start[1] + scale * dy), scale * length


def _point_along(waypoints: list[tuple[float, float]], distance: float) -> tuple[float, float]:
    if distance <= 0.0:
        return waypoints[0]
    cursor = 0.0
    for index in range(len(waypoints) - 1):
        start = waypoints[index]
        end = waypoints[index + 1]
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if cursor + length >= distance:
            if length < 1e-15:
                return end
            scale = (distance - cursor) / length
            return (start[0] + scale * (end[0] - start[0]), start[1] + scale * (end[1] - start[1]))
        cursor += length
    return waypoints[-1]


def _wrap(angle: float) -> float:
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped <= -math.pi:
        return math.pi
    return wrapped


def _clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))
