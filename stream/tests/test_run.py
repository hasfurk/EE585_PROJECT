"""Replay, recorder, and consumer acceptance checks."""

from __future__ import annotations

import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

from ee585_stream.consumer import COMPLETED, DATA_ERROR, STALLED, ABORTED, Consumer, Record, Terminal
from ee585_stream.link import TOPIC_STREAM, TOPIC_TRUTH, Link, decode, encode
from ee585_stream.replay import _read_frames
from conftest import clear_retained
from ee585_stream.stream_pb2 import Calibration, ImuSample, Record as PbRecord, Vec3

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fake_provider  # noqa: E402


def test_round_trip(tmp_path: Path, broker: object) -> None:
    del broker
    source = tmp_path / "source"
    copy = tmp_path / "copy"
    fake_provider.write_run(source)
    _replay_into_recorder(source, copy, speed=0.0)
    for name in ("stream.bin", "truth.bin", "calibration.bin", "scene.bin"):
        assert (source / name).read_bytes() == (copy / name).read_bytes()


def test_speed_independence(tmp_path: Path, broker: object) -> None:
    del broker
    full = tmp_path / "full"
    excerpt = tmp_path / "excerpt"
    fake_provider.write_run(full)
    _excerpt(full, excerpt, 3_000_000_000)
    first = _record_identity(_consume(excerpt, speed=1.0, startup_timeout_s=10.0))
    second = _record_identity(_consume(excerpt, speed=10.0, startup_timeout_s=10.0))
    third = _record_identity(_consume(excerpt, speed=0.0, startup_timeout_s=10.0))
    assert first == second == third
    assert first


def test_duplicate_is_dropped(tmp_path: Path, broker: object) -> None:
    del broker
    clean = tmp_path / "clean"
    mutated = tmp_path / "dup"
    fake_provider.write_run(clean, duration_ns=1_000_000_000)
    _copy_run(clean, mutated)
    frames = _read_frames(mutated / "stream.bin")
    frames.insert(4, frames[3])
    fake_provider._write_frames(mutated / "stream.bin", frames)
    assert _record_identity(_consume(clean, speed=0.0)) == _record_identity(_consume(mutated, speed=0.0))


def test_gap_is_data_error(tmp_path: Path, broker: object) -> None:
    del broker
    run = tmp_path / "gap"
    fake_provider.write_run(run, duration_ns=1_000_000_000)
    frames = _read_frames(run / "stream.bin")
    del frames[5]
    fake_provider._write_frames(run / "stream.bin", frames)
    events = _consume(run, speed=0.0)
    assert isinstance(events[-1], Terminal)
    assert events[-1].reason == DATA_ERROR


def test_disorder_is_data_error(tmp_path: Path, broker: object) -> None:
    del broker
    run = tmp_path / "disorder"
    fake_provider.write_run(run, duration_ns=1_000_000_000)
    frames = _read_frames(run / "stream.bin")
    record = decode(TOPIC_STREAM, frames[6])
    assert isinstance(record, PbRecord)
    record.t_ns = 0
    frames[6] = encode(record)
    fake_provider._write_frames(run / "stream.bin", frames)
    events = _consume(run, speed=0.0)
    assert isinstance(events[-1], Terminal)
    assert events[-1].reason == DATA_ERROR


def test_abrupt_halt(tmp_path: Path, broker: object) -> None:
    del broker
    run = tmp_path / "halt"
    fake_provider.write_run(run)
    consumer = Consumer("127.0.0.1", 1883, stall_timeout_s=5.0, startup_timeout_s=10.0)
    events: list[object] = []

    def collect() -> None:
        events.extend(consumer.events())

    thread = threading.Thread(target=collect)
    thread.start()
    assert consumer.subscribed.wait(5)
    proc = subprocess.Popen(
        [sys.executable, "-m", "ee585_stream.replay", "--run", str(run), "--speed", "1"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and not any(isinstance(event, Record) for event in events):
        time.sleep(0.02)
    assert any(isinstance(event, Record) for event in events)
    proc.kill()
    thread.join(timeout=4)
    assert not thread.is_alive()
    assert isinstance(events[-1], Terminal)
    assert events[-1].reason == ABORTED
    records = [event for event in events if isinstance(event, Record)]
    assert events[-1].last_seq == records[-1].seq


def test_stall(tmp_path: Path, broker: object) -> None:
    del broker
    consumer = Consumer("127.0.0.1", 1883, stall_timeout_s=1.0, startup_timeout_s=5.0)
    events: list[object] = []

    def collect() -> None:
        events.extend(consumer.events())

    thread = threading.Thread(target=collect)
    thread.start()
    assert consumer.subscribed.wait(5)
    link = Link()
    link.connect("stall-provider")
    try:
        calibration = Calibration(schema_version=1, map_name="stall", imu_rate_hz=200.0, camera_rate_hz=30.0)
        link.publish("ee585/v1/calibration", encode(calibration), qos=1, retain=True)
        record = PbRecord(seq=0, t_ns=0, imu=ImuSample(gyro_B=Vec3(), accel_B=Vec3(z=9.81)))
        link.publish("ee585/v1/stream", encode(record), qos=2, retain=False)
        thread.join(timeout=4)
        assert not thread.is_alive()
        assert isinstance(events[-1], Terminal)
        assert events[-1].reason == STALLED
    finally:
        link.finish(0, 0)


def test_clean_end(tmp_path: Path, broker: object) -> None:
    del broker
    run = tmp_path / "clean"
    fake_provider.write_run(run, duration_ns=1_000_000_000)
    events = _consume(run, speed=0.0)
    records = [event for event in events if isinstance(event, Record)]
    assert records[-1].WhichOneof("payload") == "end"
    assert isinstance(events[-1], Terminal)
    assert events[-1].reason == COMPLETED


def test_consumer_starts_first(tmp_path: Path, broker: object) -> None:
    del broker
    run = tmp_path / "early"
    fake_provider.write_run(run, duration_ns=1_000_000_000)
    events = _consume(run, speed=0.0, startup_timeout_s=10.0)
    assert isinstance(events[0], Calibration)
    assert events[0].map_name == "fake_circle"
    assert isinstance(events[-1], Terminal)
    assert events[-1].reason == COMPLETED
    assert sum(isinstance(event, Record) for event in events) == len(_read_frames(run / "stream.bin"))


def _replay_into_recorder(source: Path, copy: Path, speed: float) -> None:
    proc = subprocess.Popen(
        [sys.executable, "-m", "ee585_stream.recorder", "--run", str(copy)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    ready = copy / ".subscribed"
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not ready.exists():
        if proc.poll() is not None:
            raise RuntimeError(proc.stdout.read() if proc.stdout else "recorder exited")
        time.sleep(0.02)
    if not ready.exists():
        proc.kill()
        raise RuntimeError("recorder did not subscribe")
    replay = subprocess.run(
        [sys.executable, "-m", "ee585_stream.replay", "--run", str(source), "--speed", str(speed)],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        proc.wait(timeout=60)
    except subprocess.TimeoutExpired:
        proc.kill()
        raise
    assert replay.returncode == 0, replay.stdout
    assert proc.returncode == 0, proc.stdout.read() if proc.stdout else ""


def _consume(run: Path, speed: float, startup_timeout_s: float = 10.0) -> list[object]:
    clear_retained()
    consumer = Consumer("127.0.0.1", 1883, stall_timeout_s=5.0, startup_timeout_s=startup_timeout_s)
    events: list[object] = []
    errors: list[BaseException] = []

    def collect() -> None:
        try:
            events.extend(consumer.events())
        except BaseException as exc:
            errors.append(exc)

    thread = threading.Thread(target=collect)
    thread.start()
    assert consumer.subscribed.wait(5)
    proc = subprocess.run(
        [sys.executable, "-m", "ee585_stream.replay", "--run", str(run), "--speed", str(speed)],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    thread.join(timeout=30)
    if thread.is_alive():
        raise TimeoutError("consumer did not finish")
    if errors:
        raise errors[0]
    assert proc.returncode == 0, proc.stdout
    return events


def _record_identity(events: list[object]) -> list[tuple[int, int, str]]:
    rows = []
    for event in events:
        if isinstance(event, Record):
            raw = encode(event)
            rows.append((int(event.seq), int(event.t_ns), hashlib.sha256(raw).hexdigest()))
    return rows


def _excerpt(source: Path, dest: Path, limit_ns: int) -> None:
    dest.mkdir()
    shutil.copy(source / "calibration.bin", dest / "calibration.bin")
    shutil.copy(source / "scene.bin", dest / "scene.bin")
    for name, topic in (("stream.bin", TOPIC_STREAM), ("truth.bin", TOPIC_TRUTH)):
        kept = []
        for frame in _read_frames(source / name):
            decoded = decode(topic, frame)
            if int(decoded.t_ns) <= limit_ns and decoded.WhichOneof("payload") != "end":
                kept.append(frame)
        fake_provider._write_frames(dest / name, kept)


def _copy_run(source: Path, dest: Path) -> None:
    shutil.copytree(source, dest)
