"""Compile an MJCF world, write the planar pose, and read the engine sensors."""

from __future__ import annotations

from pathlib import Path
import math
from typing import Any

import numpy as np
import yaml

import ee585_sim.glsetup  # noqa: F401  configures MUJOCO_GL before mujoco
import mujoco

from ee585_sim.kinematics import quat_from_matrix


def load_calibration(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(Path(path).read_text())
    camera = raw["camera"]
    width = int(camera["width"])
    height = int(camera["height"])
    hfov = float(camera["hfov_rad"])
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    fy = (width / 2.0) / math.tan(hfov / 2.0)
    cx = width / 2.0
    cy = height / 2.0
    if abs(fx - fy) > 1e-6:
        raise ValueError("fx and fy differ by more than 1e-6; fovy cannot represent that camera")
    if cx != width / 2.0 or cy != height / 2.0:
        raise ValueError("principal point must be the image center")
    rotation = np.asarray(camera["R_BC"], dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("camera.R_BC must be 3x3")
    imu = raw["imu"]
    return {
        "schema_version": int(raw["schema_version"]),
        "imu_rate_hz": float(imu["rate_hz"]),
        "gyro_noise_density": float(imu["gyro_noise_density"]),
        "accel_noise_density": float(imu["accel_noise_density"]),
        "gravity_W": tuple(float(component) for component in imu["gravity_W"]),
        "width": width,
        "height": height,
        "camera_rate_hz": float(camera["rate_hz"]),
        "hfov_rad": hfov,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "fovy_deg": math.degrees(2.0 * math.atan((height / 2.0) / fy)),
        "pixel_noise_sigma": float(camera["pixel_noise_sigma"]),
        "p_BC": tuple(float(component) for component in camera["p_BC"]),
        "q_BC": quat_from_matrix(rotation),
        "R_BC": rotation,
        "min_range_m": float(camera["min_range_m"]),
        "max_range_m": float(camera["max_range_m"]),
        "initial_rest_s": float(raw["initial_rest_s"]),
    }


def load_model(path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(path))
    return model, mujoco.MjData(model)


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, x: float, y: float, yaw: float) -> None:
    _write_joint(model, data.qpos, model.jnt_qposadr, "jx", x)
    _write_joint(model, data.qpos, model.jnt_qposadr, "jy", y)
    _write_joint(model, data.qpos, model.jnt_qposadr, "jyaw", yaw)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0


def set_rate(model: mujoco.MjModel, data: mujoco.MjData, vx: float, vy: float, yaw_rate: float) -> None:
    """Joint rates in the slide axes (world x, world y) and yaw rate."""
    data.qvel[:] = 0.0
    _write_joint(model, data.qvel, model.jnt_dofadr, "jx", vx)
    _write_joint(model, data.qvel, model.jnt_dofadr, "jy", vy)
    _write_joint(model, data.qvel, model.jnt_dofadr, "jyaw", yaw_rate)
    data.qacc[:] = 0.0


def read_sensors(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    """Engine gyro and accelerometer. mj_forward fills sensordata on this build."""
    mujoco.mj_forward(model, data)
    return _sensor(model, data, "gyro"), _sensor(model, data, "accel")


def rgb_to_mono(rgb: np.ndarray) -> np.ndarray:
    image = np.asarray(rgb, dtype=float)
    gray = 0.299 * image[..., 0] + 0.587 * image[..., 1] + 0.114 * image[..., 2]
    return np.clip(np.floor(gray + 0.5), 0, 255).astype(np.uint8)


def render_rgb(model: mujoco.MjModel, data: mujoco.MjData, width: int, height: int) -> np.ndarray:
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera="cam")
        return np.array(renderer.render(), copy=True)
    finally:
        renderer.close()


def _write_joint(
    model: mujoco.MjModel,
    target: np.ndarray,
    addresses: np.ndarray,
    name: str,
    value: float,
) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"joint {name} is missing")
    target[int(addresses[joint_id])] = value


def _sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sensor_id < 0:
        raise ValueError(f"sensor {name} is missing")
    address = int(model.sensor_adr[sensor_id])
    width = int(model.sensor_dim[sensor_id])
    return np.array(data.sensordata[address : address + width], dtype=float, copy=True)
