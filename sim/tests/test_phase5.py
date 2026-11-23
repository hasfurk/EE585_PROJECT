"""Phase 5 checks: pure pursuit on demo_room."""

from __future__ import annotations

import math
from pathlib import Path

import ee585_sim.model as model
from ee585_sim.follower import Follower
from ee585_sim.kinematics import imu_stamp, propagate, quat_from_yaw
from ee585_sim.maps import load_map

SIM = Path(__file__).resolve().parents[1]
MAP = SIM / "maps" / "demo_room.yaml"
CALIBRATION = SIM / "config" / "calibration.yaml"


def test_demo_room_path() -> None:
    spec = load_map(MAP)
    rest_s = model.load_calibration(CALIBRATION)["initial_rest_s"]
    follower = Follower(spec["path"], rest_s)
    start_x, start_y = spec["path"]["waypoints"][0]
    next_x, next_y = spec["path"]["waypoints"][1]
    x, y = start_x, start_y
    yaw = math.atan2(next_y - start_y, next_x - start_x)
    start_yaw = yaw
    transitions = 0
    was_done = False
    finished = False
    for index in range(20_000):
        t_ns = imu_stamp(index)
        if t_ns <= 1_000_000_000:
            assert abs(x - start_x) < 1e-9
            assert abs(y - start_y) < 1e-9
            assert abs(yaw - start_yaw) < 1e-9
        _track = _distance_to_path(spec["path"]["waypoints"], x, y)
        assert _track < 0.3
        for obstacle in spec["obstacles"]:
            assert not _intersects(
                (x, y),
                (0.20, 0.15),
                yaw,
                obstacle["center"],
                (obstacle["size"][0] / 2.0, obstacle["size"][1] / 2.0),
                obstacle["yaw"],
            )
        quaternion = quat_from_yaw(yaw)
        assert abs(quaternion[1]) < 1e-9 and abs(quaternion[2]) < 1e-9
        assert abs(0.05 - 0.05) <= 1e-9
        v, omega = follower.command(t_ns, x, y, yaw)
        if follower.done and not was_done:
            transitions += 1
        was_done = follower.done
        if follower.done:
            finished = True
            break
        x, y, yaw = propagate(x, y, yaw, v, omega, 0.005)
    assert finished
    assert transitions == 1
    assert follower.done


def _distance_to_path(waypoints: list[tuple[float, float]], x: float, y: float) -> float:
    best = math.inf
    for index in range(len(waypoints) - 1):
        start = waypoints[index]
        end = waypoints[index + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length2 = dx * dx + dy * dy
        if length2 < 1e-18:
            scale = 0.0
        else:
            scale = min(1.0, max(0.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length2))
        px = start[0] + scale * dx
        py = start[1] + scale * dy
        best = min(best, math.hypot(px - x, py - y))
    return best


def _intersects(
    center_a: tuple[float, float],
    half_a: tuple[float, float],
    yaw_a: float,
    center_b: tuple[float, float],
    half_b: tuple[float, float],
    yaw_b: float,
) -> bool:
    for yaw in (yaw_a, yaw_b):
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for axis in ((cosine, sine), (-sine, cosine)):
            if abs(_projection(center_a, half_a, yaw_a, axis) - _projection(center_b, half_b, yaw_b, axis)) > (
                _radius(half_a, yaw_a, axis) + _radius(half_b, yaw_b, axis)
            ):
                return False
    return True


def _projection(
    center: tuple[float, float],
    _half: tuple[float, float],
    _yaw: float,
    axis: tuple[float, float],
) -> float:
    return center[0] * axis[0] + center[1] * axis[1]


def _radius(half: tuple[float, float], yaw: float, axis: tuple[float, float]) -> float:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return abs((cosine * axis[0] + sine * axis[1]) * half[0]) + abs((-sine * axis[0] + cosine * axis[1]) * half[1])
