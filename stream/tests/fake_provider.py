"""Synthetic 20 s circle log. Stamps follow the provider clock."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
import time

from ee585_stream.stream_pb2 import (
    Calibration,
    EndOfStream,
    ImageFrame,
    ImuSample,
    Landmark,
    Observation,
    Pose,
    Projections,
    Quat,
    Record,
    Scene,
    TruthProjections,
    TruthRecord,
    Vec3,
)

DURATION_NS = 20_000_000_000
RADIUS_M = 2.0
YAW_RATE = 0.2
WIDTH = 64
HEIGHT = 48


def write_run(path: Path, duration_ns: int = DURATION_NS) -> None:
    path.mkdir(parents=True, exist_ok=True)
    calibration = _calibration()
    scene = _scene()
    _write_frames(path / "calibration.bin", [calibration.SerializeToString()])
    _write_frames(path / "scene.bin", [scene.SerializeToString()])
    _write_frames(path / "stream.bin", [_record_bytes(record) for record in _stream(duration_ns)])
    _write_frames(path / "truth.bin", [_truth_bytes(record) for record in _truth(duration_ns)])
    (path / "meta.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "map_name": calibration.map_name,
                "counts": {
                    "calibration": 1,
                    "scene": 1,
                    "stream": _frame_count(path / "stream.bin"),
                    "truth": _frame_count(path / "truth.bin"),
                },
                "terminal_reason": "COMPLETED",
                "created_wall_time": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
        )
        + "\n"
    )


def imu_stamp(index: int) -> int:
    return index * 5_000_000


def camera_stamp(index: int) -> int:
    return ((index * 1000 + 15) // 30) * 1_000_000


def _calibration() -> Calibration:
    return Calibration(
        schema_version=1,
        map_name="fake_circle",
        imu_rate_hz=200.0,
        gyro_noise_density=1.6968e-4,
        accel_noise_density=2.0e-3,
        gravity_W=Vec3(x=0.0, y=0.0, z=-9.81),
        width=WIDTH,
        height=HEIGHT,
        camera_rate_hz=30.0,
        fx=32.0,
        fy=32.0,
        cx=32.0,
        cy=24.0,
        pixel_noise_sigma=0.5,
        p_BC=Vec3(x=0.15, y=0.0, z=0.10),
        q_BC=Quat(w=0.5, x=-0.5, y=0.5, z=-0.5),
        initial_rest_s=1.0,
    )


def _scene() -> Scene:
    points = ((2.0, 1.0, 0.5), (0.0, 3.0, 0.8), (-1.0, -2.0, 1.0), (3.0, -1.0, 0.4), (1.0, 1.0, 1.2))
    return Scene(
        landmarks=[Landmark(id=index, p_W=Vec3(x=x, y=y, z=z)) for index, (x, y, z) in enumerate(points)]
    )


def _stream(duration_ns: int) -> list[Record]:
    items: list[tuple[int, int, Record]] = []
    for stamp in _stamps(imu_stamp, duration_ns):
        items.append((stamp, 0, Record(t_ns=stamp, imu=_imu(stamp))))
    for stamp in _stamps(camera_stamp, duration_ns):
        items.append((stamp, 1, Record(t_ns=stamp, image=_image(stamp))))
        items.append((stamp, 2, Record(t_ns=stamp, projections=_projections(stamp))))
    items.sort()
    records = []
    for seq, (stamp, _order, record) in enumerate(items):
        record.seq = seq
        record.t_ns = stamp
        records.append(record)
    end_t = records[-1].t_ns if records else 0
    records.append(Record(seq=len(records), t_ns=end_t, end=EndOfStream()))
    return records


def _truth(duration_ns: int) -> list[TruthRecord]:
    items: list[tuple[int, int, TruthRecord]] = []
    for stamp in _stamps(imu_stamp, duration_ns):
        items.append((stamp, 0, TruthRecord(t_ns=stamp, pose=_pose(stamp))))
    for stamp in _stamps(camera_stamp, duration_ns):
        items.append((stamp, 1, TruthRecord(t_ns=stamp, projections=_truth_projections(stamp))))
    items.sort()
    records = []
    for seq, (stamp, _order, record) in enumerate(items):
        record.seq = seq
        record.t_ns = stamp
        records.append(record)
    return records


def _stamps(fn: object, duration_ns: int) -> list[int]:
    stamps = []
    index = 0
    while True:
        stamp = fn(index)  # type: ignore[operator]
        if stamp > duration_ns:
            break
        stamps.append(stamp)
        index += 1
    return stamps


def _imu(t_ns: int) -> ImuSample:
    del t_ns
    centripetal = RADIUS_M * YAW_RATE * YAW_RATE
    return ImuSample(
        gyro_B=Vec3(x=0.0, y=0.0, z=YAW_RATE),
        accel_B=Vec3(x=-centripetal, y=0.0, z=9.81),
    )


def _pose(t_ns: int) -> Pose:
    yaw = YAW_RATE * (t_ns / 1e9)
    half = yaw / 2.0
    w = math.cos(half)
    z = math.sin(half)
    if w < 0.0:
        w = -w
        z = -z
    return Pose(
        p_WB=Vec3(x=RADIUS_M * math.cos(yaw), y=RADIUS_M * math.sin(yaw), z=0.05),
        q_WB=Quat(w=w, x=0.0, y=0.0, z=z),
    )


def _image(t_ns: int) -> ImageFrame:
    mark = (t_ns // 1_000_000) % 256
    return ImageFrame(width=WIDTH, height=HEIGHT, mono8=bytes((index + mark) % 256 for index in range(WIDTH * HEIGHT)))


def _observations(t_ns: int) -> list[Observation]:
    seconds = t_ns / 1e9
    return [
        Observation(landmark_id=index, u=4.0 + index * 8.0, v=3.0 + seconds)
        for index in range(5)
    ]


def _projections(t_ns: int) -> Projections:
    return Projections(obs=_observations(t_ns))


def _truth_projections(t_ns: int) -> TruthProjections:
    return TruthProjections(obs=_observations(t_ns))


def _record_bytes(record: Record) -> bytes:
    return record.SerializeToString()


def _truth_bytes(record: TruthRecord) -> bytes:
    return record.SerializeToString()


def _write_frames(path: Path, frames: list[bytes]) -> None:
    with path.open("wb") as handle:
        for frame in frames:
            handle.write(struct.pack("<I", len(frame)))
            handle.write(frame)


def _frame_count(path: Path) -> int:
    data = path.read_bytes()
    count = 0
    offset = 0
    while offset < len(data):
        (length,) = struct.unpack_from("<I", data, offset)
        offset += 4 + length
        count += 1
    return count
