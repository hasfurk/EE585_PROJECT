"""Encode/decode, pacer timing, and retained MQTT publishes."""

from __future__ import annotations

import threading
import time

import paho.mqtt.client as mqtt
from google.protobuf.message import Message

from ee585_stream import stream_pb2
from ee585_stream.link import TOPIC_CALIBRATION, TOPIC_STATUS, Link, Pacer, decode, encode


def _round_trip(topic: str, msg: Message) -> None:
    raw = encode(msg)
    got = decode(topic, raw)
    assert got == msg
    assert encode(got) == raw


def test_round_trip_every_message() -> None:
    vec = stream_pb2.Vec3(x=0.1, y=-2.5, z=-9.81)
    quat = stream_pb2.Quat(w=0.5, x=-0.5, y=0.5, z=-0.5)
    imu = stream_pb2.ImuSample(
        gyro_B=stream_pb2.Vec3(x=1.6968e-4, y=0.0, z=-2.3996e-3),
        accel_B=stream_pb2.Vec3(x=0.0, y=2.0e-3, z=9.81),
    )
    image = stream_pb2.ImageFrame(width=640, height=480, mono8=bytes(range(256)) * 1200)
    assert len(image.mono8) == 307200
    observation = stream_pb2.Observation(landmark_id=7, u=320.5, v=0.1)
    projections = stream_pb2.Projections(obs=[observation, stream_pb2.Observation(landmark_id=8, u=1.5, v=2.5)])
    end = stream_pb2.EndOfStream()
    calibration = stream_pb2.Calibration(
        schema_version=1,
        map_name="demo_room",
        imu_rate_hz=200.0,
        gyro_noise_density=1.6968e-4,
        accel_noise_density=2.0e-3,
        gravity_W=stream_pb2.Vec3(x=0.0, y=0.0, z=-9.81),
        width=640,
        height=480,
        camera_rate_hz=30.0,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        pixel_noise_sigma=0.5,
        p_BC=stream_pb2.Vec3(x=0.15, y=0.0, z=0.10),
        q_BC=quat,
        initial_rest_s=1.0,
    )
    landmark = stream_pb2.Landmark(id=3, p_W=vec)
    scene = stream_pb2.Scene(landmarks=[landmark])
    pose = stream_pb2.Pose(p_WB=stream_pb2.Vec3(x=1.0, y=2.0, z=0.05), q_WB=quat)
    truth_projections = stream_pb2.TruthProjections(obs=[observation])
    records = [
        stream_pb2.Record(seq=0, t_ns=0, imu=imu),
        stream_pb2.Record(seq=1, t_ns=33_000_000, image=image),
        stream_pb2.Record(seq=2, t_ns=33_000_000, projections=projections),
        stream_pb2.Record(seq=3, t_ns=33_000_000, end=end),
    ]
    truth = [
        stream_pb2.TruthRecord(seq=0, t_ns=0, pose=pose),
        stream_pb2.TruthRecord(seq=1, t_ns=33_000_000, projections=truth_projections),
    ]
    statuses = [
        stream_pb2.Status(state=stream_pb2.Status.ONLINE, last_seq=-1, last_t_ns=-1, detail=""),
        stream_pb2.Status(state=stream_pb2.Status.COMPLETED, last_seq=3, last_t_ns=33_000_000, detail=""),
        stream_pb2.Status(state=stream_pb2.Status.ABORTED, last_seq=-1, last_t_ns=-1, detail="last will"),
    ]

    for msg in (vec, quat, imu, image, observation, projections, end, pose, truth_projections, landmark):
        restored = type(msg)()
        restored.ParseFromString(encode(msg))
        assert restored == msg

    _round_trip(TOPIC_CALIBRATION, calibration)
    _round_trip("ee585/v1/scene", scene)
    for record in records:
        _round_trip("ee585/v1/stream", record)
    for item in truth:
        _round_trip("ee585/v1/truth", item)
    for status in statuses:
        _round_trip(TOPIC_STATUS, status)


def test_pacer_speed_10() -> None:
    pacer = Pacer(10, 0)
    started = time.perf_counter()
    pacer.wait_until(5_000_000_000)
    elapsed = time.perf_counter() - started
    assert abs(elapsed - 0.5) < 0.05


def test_link_publishes_retained_status_and_calibration(broker: object) -> None:
    del broker
    calibration = stream_pb2.Calibration(
        schema_version=1,
        map_name="demo_room",
        imu_rate_hz=200.0,
        gyro_noise_density=1.6968e-4,
        accel_noise_density=2.0e-3,
        gravity_W=stream_pb2.Vec3(z=-9.81),
        width=640,
        height=480,
        camera_rate_hz=30.0,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        pixel_noise_sigma=0.5,
        initial_rest_s=1.0,
    )
    link = Link()
    link.connect("phase1-provider")
    try:
        link.publish(TOPIC_CALIBRATION, encode(calibration), qos=1, retain=True)
        _assert_retained(calibration)
    finally:
        link.finish(0, 0)


def _assert_retained(calibration: stream_pb2.Calibration) -> None:
    received: dict[str, bytes] = {}
    got = threading.Event()

    def on_message(_client: mqtt.Client, _userdata: object, message: mqtt.MQTTMessage) -> None:
        received[message.topic] = message.payload
        if TOPIC_STATUS in received and TOPIC_CALIBRATION in received:
            got.set()

    late = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="phase1-late")
    late.on_message = on_message
    late.connect("127.0.0.1", 1883, keepalive=2)
    late.loop_start()
    late.subscribe(TOPIC_STATUS, qos=1)
    late.subscribe(TOPIC_CALIBRATION, qos=1)
    assert got.wait(5)
    late.loop_stop()
    late.disconnect()

    status = decode(TOPIC_STATUS, received[TOPIC_STATUS])
    assert isinstance(status, stream_pb2.Status)
    assert status.state == stream_pb2.Status.ONLINE
    assert status.last_seq == -1
    assert status.last_t_ns == -1
    assert decode(TOPIC_CALIBRATION, received[TOPIC_CALIBRATION]) == calibration
