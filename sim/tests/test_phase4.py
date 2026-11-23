"""Phase 4 checks: projection, occlusion, noise, and rendered centroids."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

import ee585_sim.model as model
from ee585_sim.kinematics import quat_from_yaw
from ee585_sim.maps import build, generate_landmarks, load_map, occluders
from ee585_sim.projection import Observation, Projector, rotation_matrix

SIM = Path(__file__).resolve().parents[1]
MAP = SIM / "maps" / "demo_room.yaml"
CALIBRATION = SIM / "config" / "calibration.yaml"


def _calib(**overrides: object) -> dict[str, object]:
    calibration = model.load_calibration(CALIBRATION)
    calibration["p_BC"] = (0.0, 0.0, 0.0)
    calibration["q_BC"] = (1.0, 0.0, 0.0, 0.0)
    calibration.update(overrides)
    return calibration


def _project(
    landmarks: np.ndarray,
    *,
    p_WB: np.ndarray | None = None,
    q_WB: np.ndarray | None = None,
    boxes: list[dict[str, object]] | None = None,
    seed: int = 1,
    **overrides: object,
) -> tuple[list[Observation], list[Observation]]:
    projector = Projector(_calib(**overrides), landmarks, boxes or [], seed)
    return projector.project(
        0,
        np.zeros(3) if p_WB is None else p_WB,
        np.array([1.0, 0.0, 0.0, 0.0]) if q_WB is None else q_WB,
    )


def test_optical_axis_is_the_principal_point() -> None:
    _noisy, truth = _project(np.array([[0.0, 0.0, 4.0]]), pixel_noise_sigma=0.0)
    assert truth == [Observation(0, 320.0, 240.0)]


def test_offset_landmark_uses_intrinsics() -> None:
    calibration = _calib(pixel_noise_sigma=0.0)
    _noisy, truth = Projector(calibration, np.array([[1.0, 0.0, 4.0]]), [], 1).project(
        0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    )
    assert truth[0].u == pytest.approx(float(calibration["fx"]) * 0.25 + float(calibration["cx"]))
    assert truth[0].v == pytest.approx(float(calibration["cy"]))


def test_rejections() -> None:
    landmarks = np.array(
        [
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 30.0],
            [0.0, 0.0, 0.05],
            [100.0, 0.0, 4.0],
        ]
    )
    noisy, truth = _project(landmarks, pixel_noise_sigma=0.0)
    assert noisy == []
    assert truth == []


def test_occlusion() -> None:
    landmark = np.array([[0.0, 0.0, 4.0]])
    blocking = [{"center": (0.0, 0.0, 2.0), "size": (1.0, 1.0, 1.0), "yaw": 0.0}]
    aside = [{"center": (3.0, 0.0, 2.0), "size": (1.0, 1.0, 1.0), "yaw": 0.0}]
    assert _project(landmark, boxes=blocking, pixel_noise_sigma=0.0)[1] == []
    assert len(_project(landmark, boxes=aside, pixel_noise_sigma=0.0)[1]) == 1
    own_face = [{"center": (0.0, 0.0, 4.04), "size": (2.0, 2.0, 0.04), "yaw": 0.0}]
    assert len(_project(landmark, boxes=own_face, pixel_noise_sigma=0.0)[1]) == 1


def test_noise_statistics() -> None:
    calibration = _calib()
    landmarks = np.array([[0.0, 0.0, 4.0]])
    samples = np.empty((10_000, 2))
    projector = Projector(calibration, landmarks, [], 1)
    for index in range(10_000):
        noisy, truth = projector.project(0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        samples[index] = (noisy[0].u - truth[0].u, noisy[0].v - truth[0].v)
    sigma = float(calibration["pixel_noise_sigma"])
    assert samples.std(axis=0) == pytest.approx((sigma, sigma), rel=0.03)
    assert np.abs(samples.mean(axis=0)).max() < 0.05 * sigma


def test_seed_and_truth_are_deterministic() -> None:
    landmarks = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, 4.0], [0.2, 0.0, 4.0]])
    pose = (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    first = Projector(_calib(), landmarks, [], 7).project(0, *pose)
    second = Projector(_calib(), landmarks, [], 7).project(0, *pose)
    assert first == second
    silent = Projector(_calib(pixel_noise_sigma=0.0), landmarks[1:], [], 7).project(0, *pose)
    assert [item.landmark_id for item in first[1]] == [1, 2]
    assert (first[1][0].u, first[1][0].v) == (silent[1][0].u, silent[1][0].v)
    only_visible = Projector(_calib(), landmarks[1:], [], 7).project(0, *pose)
    assert (first[0][0].u, first[0][0].v) == (only_visible[0][0].u, only_visible[0][0].v)


def test_left_turn_centers_a_left_landmark() -> None:
    calibration = _calib(p_BC=(-1.0, 0.0, 0.0), q_BC=model.load_calibration(CALIBRATION)["q_BC"])
    landmark = np.array([[0.0, 0.9, 0.0]])
    before = Projector(calibration, landmark, [], 1).project(0, np.zeros(3), np.array(quat_from_yaw(0.0)))
    after = Projector(calibration, landmark, [], 1).project(0, np.zeros(3), np.array(quat_from_yaw(math.pi / 2.0)))
    assert before[1][0].u < float(calibration["cx"])
    assert after[1][0].u == pytest.approx(float(calibration["cx"]))
    assert after[1][0].v == pytest.approx(float(calibration["cy"]))


def test_render_orientation(tmp_path: Path) -> None:
    calibration = model.load_calibration(CALIBRATION)
    p_WC = np.array([0.0, 0.0, 0.05]) + np.array(calibration["p_BC"])
    rotation = rotation_matrix(np.array(calibration["q_BC"]))
    optical = (np.array([0.0, 0.0, 4.0]), np.array([1.0, 0.0, 4.0]), np.array([0.0, 1.0, 4.0]))
    spheres = [p_WC + rotation @ point for point in optical]
    xml = f"""
<mujoco>
  <visual>
    <headlight active="0" ambient="0 0 0" diffuse="0 0 0" specular="0 0 0"/>
    <global offwidth="640" offheight="480"/>
  </visual>
  <asset>
    <material name="mark" rgba="1 1 1 1" emission="1" specular="0"/>
  </asset>
  <worldbody>
    <light directional="true" ambient="1 1 1" diffuse="0 0 0" specular="0 0 0" castshadow="false" dir="0 0 -1"/>
    <body pos="0 0 0.05">
      <camera name="cam" pos="0.15 0 0.10" xyaxes="0 -1 0 0 0 1" fovy="{calibration["fovy_deg"]}" mode="fixed"/>
    </body>
    {"".join(_sphere(index, point) for index, point in enumerate(spheres))}
  </worldbody>
</mujoco>
"""
    path = tmp_path / "orientation.xml"
    path.write_text(xml)
    try:
        compiled, data = model.load_model(path)
        rgb = model.render_rgb(compiled, data, 640, 480)
    finally:
        path.unlink(missing_ok=True)
    mono = model.rgb_to_mono(rgb)
    expected = [
        (float(calibration["cx"]), float(calibration["cy"])),
        (float(calibration["fx"]) * 0.25 + float(calibration["cx"]), float(calibration["cy"])),
        (float(calibration["cx"]), float(calibration["fy"]) * 0.25 + float(calibration["cy"])),
    ]
    centers = [_blob_center(mono, u, v) for u, v in expected]
    assert centers[0] == pytest.approx(expected[0], abs=1.0)
    assert centers[1][0] > float(calibration["cx"]) + 0.2 * float(calibration["fx"])
    assert centers[2][1] > float(calibration["cy"]) + 0.2 * float(calibration["fy"])


def test_demo_room_centroids(tmp_path: Path) -> None:
    out = tmp_path / "demo"
    build(MAP, out)
    spec = load_map(MAP)
    calibration = model.load_calibration(CALIBRATION)
    calibration["pixel_noise_sigma"] = 0.0
    points = np.array([item["p_W"] for item in generate_landmarks(spec)], dtype=float)
    boxes = occluders(spec)
    compiled, data = model.load_model(out / "world.xml")
    chosen = None
    for x, y, yaw in ((4.0, 0.0, 0.0), (0.0, -4.0, math.pi / 2.0), (-4.0, 0.0, math.pi), (0.0, 4.0, -math.pi / 2.0)):
        truth = Projector(calibration, points, boxes, spec["seed"] + 1).project(
            0, np.array([x, y, 0.05]), np.array(quat_from_yaw(yaw))
        )[1]
        separated = _separated(truth, 40.0)
        if len(separated) >= 5:
            chosen = (x, y, yaw, separated[:5])
            break
    assert chosen is not None
    x, y, yaw, targets = chosen
    model.set_pose(compiled, data, x, y, yaw)
    mono = model.rgb_to_mono(model.render_rgb(compiled, data, 640, 480))
    residuals = []
    for target in targets:
        center = _blob_center(mono, target.u, target.v)
        residuals.append(math.hypot(center[0] - target.u, center[1] - target.v))
        assert residuals[-1] < 1.0


def _sphere(index: int, point: np.ndarray) -> str:
    x, y, z = (float(point[0]), float(point[1]), float(point[2]))
    return f'<geom name="s{index}" type="sphere" size="0.05" pos="{x} {y} {z}" material="mark"/>'


def _blob_center(mono: np.ndarray, u: float, v: float, window: int = 20) -> tuple[float, float]:
    row0 = max(0, int(math.floor(v)) - window)
    row1 = min(mono.shape[0], int(math.ceil(v)) + window + 1)
    col0 = max(0, int(math.floor(u)) - window)
    col1 = min(mono.shape[1], int(math.ceil(u)) + window + 1)
    patch = mono[row0:row1, col0:col1]
    rows, cols = np.nonzero(patch >= 200)
    assert len(rows) > 0
    return (float((cols + col0 + 0.5).mean()), float((rows + row0 + 0.5).mean()))


def _separated(observations: list[Observation], gap: float) -> list[Observation]:
    kept: list[Observation] = []
    for item in observations:
        if all(math.hypot(item.u - other.u, item.v - other.v) >= gap for other in kept):
            kept.append(item)
    return kept
