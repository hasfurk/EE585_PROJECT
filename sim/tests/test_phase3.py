"""Phase 3 checks: kinematics, IMU, map, and one offscreen frame."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

import ee585_sim.model as model
import mujoco
from ee585_sim.imu import DT_S, G_W, ImuNoise, rest_sample, true_sample
from ee585_sim.kinematics import camera_stamp, imu_stamp, propagate, quat_from_matrix, quat_from_yaw
from ee585_sim.maps import generate_landmarks, load_map

SIM = Path(__file__).resolve().parents[1]
MAP = SIM / "maps" / "demo_room.yaml"
CALIBRATION = SIM / "config" / "calibration.yaml"


def test_calibration_matches_section_2() -> None:
    calibration = model.load_calibration(CALIBRATION)
    expected_f = 320.0 / math.tan(calibration["hfov_rad"] / 2.0)
    assert calibration["fx"] == pytest.approx(expected_f, abs=1e-12)
    assert calibration["fy"] == pytest.approx(expected_f, abs=1e-12)
    # Locked hfov 1.5707963 is shorter than pi/2, so fx misses 320 by about 8.6e-6.
    assert abs(expected_f - 320.0) < 1e-5
    assert calibration["cx"] == 320.0
    assert calibration["cy"] == 240.0
    assert abs(calibration["fovy_deg"] - 73.7397938) < 1e-6
    assert calibration["q_BC"] == pytest.approx((0.5, -0.5, 0.5, -0.5), abs=1e-12)
    matrix = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    assert quat_from_matrix(matrix) == pytest.approx(calibration["q_BC"], abs=1e-12)


def test_zero_noise_imu() -> None:
    for yaw in (0.0, 0.4, -1.2):
        gyro, accel = rest_sample()
        assert gyro == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert accel == pytest.approx((0.0, 0.0, 9.81), abs=1e-12)
        gyro, accel = true_sample(0.0, 0.0, 0.0, yaw)
        assert gyro == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert accel == pytest.approx((0.0, 0.0, 9.81), abs=1e-12)
    gyro, accel = true_sample(0.5, 0.5, 0.2, 0.7)
    assert gyro == pytest.approx((0.0, 0.0, 0.2), abs=1e-12)
    assert accel == pytest.approx((0.0, 0.1, 9.81), abs=1e-12)


def test_dead_reckon_three_seconds() -> None:
    x = y = yaw = 0.0
    v_prev = 0.0
    samples: list[tuple[tuple[float, float, float], float]] = []
    for index in range(1, 201):
        v = 0.5 * index * DT_S
        _gyro, accel = true_sample(v, v_prev, 0.0, yaw)
        x, y, yaw = propagate(x, y, yaw, v, 0.0, DT_S)
        samples.append((accel, yaw))
        v_prev = v
    for _ in range(400):
        v = 0.5
        _gyro, accel = true_sample(v, v_prev, 0.0, yaw)
        x, y, yaw = propagate(x, y, yaw, v, 0.0, DT_S)
        samples.append((accel, yaw))
        v_prev = v
    position = np.zeros(3)
    velocity = np.zeros(3)
    gravity = np.array(G_W)
    for accel, sample_yaw in samples:
        cosine = math.cos(sample_yaw)
        sine = math.sin(sample_yaw)
        rotation = np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])
        accel_w = rotation @ np.array(accel) + gravity
        velocity = velocity + accel_w * DT_S
        position = position + velocity * DT_S
    assert math.hypot(position[0] - x, position[1] - y) < 0.01


def test_noise_statistics() -> None:
    noise = ImuNoise(2, 1.6968e-4, 2.0e-3, 200.0)
    gyro = np.empty((2000, 3))
    accel = np.empty((2000, 3))
    for index in range(2000):
        gyro[index], accel[index] = noise.apply(*rest_sample())
    assert accel.mean(axis=0) == pytest.approx((0.0, 0.0, 9.81), abs=0.05)
    assert gyro.std(axis=0) == pytest.approx((2.400e-3, 2.400e-3, 2.400e-3), rel=0.1)
    assert accel.std(axis=0) == pytest.approx((2.828e-2, 2.828e-2, 2.828e-2), rel=0.1)
    assert np.linalg.norm(gyro.mean(axis=0)) < 5e-4


def test_open_loop_four_seconds() -> None:
    x = y = yaw = 0.0
    for _ in range(800):
        x, y, yaw = propagate(x, y, yaw, 0.5, 0.0, DT_S)
    assert (x, y) == pytest.approx((2.0, 0.0), abs=1e-6)
    quaternion = quat_from_yaw(yaw)
    assert quaternion[1] == 0.0
    assert quaternion[2] == 0.0


def test_stamps() -> None:
    for index in range(2001):
        assert imu_stamp(index) == index * 5_000_000
    for index in range(31):
        assert camera_stamp(index) == ((index * 1000 + 15) // 30) * 1_000_000
    assert camera_stamp(30) == 1_000_000_000


def test_textures(tmp_path: Path) -> None:
    from ee585_sim.maps import render_textures

    render_textures(tmp_path)
    checker = np.array(Image.open(tmp_path / "checker.png"))
    plain = np.array(Image.open(tmp_path / "plain.png"))
    noise = np.array(Image.open(tmp_path / "noise.png"))
    assert checker.shape == plain.shape == noise.shape == (512, 512)
    assert checker[0, 0] == 32 and checker[0, 64] == 220
    assert int(plain[0, 0]) == 180
    expected = np.random.Generator(np.random.PCG64(0)).integers(0, 256, size=(512, 512), dtype=np.uint8)
    assert np.array_equal(noise, expected)


def test_short_face_is_rejected(tmp_path: Path) -> None:
    text = MAP.read_text().replace("wall_height: 2.5", "wall_height: 0.4")
    path = tmp_path / "short.yaml"
    path.write_text(text)
    with pytest.raises(ValueError):
        load_map(path)


def test_demo_room_pose_sensors_and_frame(tmp_path: Path) -> None:
    from ee585_sim.maps import build

    out = tmp_path / "demo_room"
    build(MAP, out)
    spec = load_map(MAP)
    landmarks = generate_landmarks(spec)
    assert [item["id"] for item in landmarks] == list(range(len(landmarks)))
    assert len(landmarks) == 126
    half = 6.0
    for item in landmarks:
        x, y, z = item["p_W"]
        assert -half < x < half
        assert -half < y < half
        assert 0.3 <= z <= 2.3

    compiled, data = model.load_model(out / "world.xml")
    model.set_pose(compiled, data, 1.0, 2.0, math.pi / 2.0)
    model.read_sensors(compiled, data)
    body = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, "base")
    assert data.xpos[body] == pytest.approx((1.0, 2.0, 0.05), abs=1e-8)
    rotation = data.xmat[body].reshape(3, 3)
    assert rotation @ np.array([1.0, 0.0, 0.0]) == pytest.approx((0.0, 1.0, 0.0), abs=1e-8)
    quaternion = data.xquat[body]
    assert quaternion[0] >= 0.0
    assert quaternion == pytest.approx(
        (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0)),
        abs=1e-8,
    )

    gyro, accel = model.read_sensors(compiled, data)
    assert accel == pytest.approx((0.0, 0.0, 9.81), abs=1e-4)
    assert gyro == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    model.set_rate(compiled, data, 0.0, 0.0, 0.3)
    gyro, _accel = model.read_sensors(compiled, data)
    assert gyro == pytest.approx((0.0, 0.0, 0.3), abs=1e-6)

    start_x, start_y = spec["path"]["waypoints"][0]
    next_x, next_y = spec["path"]["waypoints"][1]
    yaw = math.atan2(next_y - start_y, next_x - start_x)
    calibration = model.load_calibration(CALIBRATION)
    model.set_pose(compiled, data, start_x, start_y, yaw)
    image = model.render_rgb(compiled, data, calibration["width"], calibration["height"])
    assert image.shape == (480, 640, 3)
    assert image.dtype == np.uint8
    assert float(image.mean()) > 0.0
