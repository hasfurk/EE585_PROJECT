"""Republish a recorded run on MQTT."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys
import time

from ee585_stream.link import TOPIC_CALIBRATION, TOPIC_SCENE, TOPIC_STREAM, TOPIC_TRUTH, Link, Pacer, decode
from ee585_stream.stream_pb2 import Record, TruthRecord


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    args = parser.parse_args(argv)
    replay_run(Path(args.run), args.speed, args.host, args.port)


def replay_run(run: Path, speed: float, host: str, port: int) -> None:
    calibration = _read_frames(run / "calibration.bin")
    scene = _read_frames(run / "scene.bin")
    stream = _read_frames(run / "stream.bin")
    truth = _read_frames(run / "truth.bin")
    merged: list[tuple[int, int, int, str, bytes]] = []
    last_seq = -1
    last_t_ns = -1
    for index, payload in enumerate(stream):
        record = decode(TOPIC_STREAM, payload)
        if not isinstance(record, Record):
            raise TypeError("stream.bin contains a non-record payload")
        merged.append((int(record.t_ns), 0, index, TOPIC_STREAM, payload))
        last_seq = int(record.seq)
        last_t_ns = int(record.t_ns)
    for index, payload in enumerate(truth):
        record = decode(TOPIC_TRUTH, payload)
        if not isinstance(record, TruthRecord):
            raise TypeError("truth.bin contains a non-truth payload")
        merged.append((int(record.t_ns), 1, index, TOPIC_TRUTH, payload))
    merged.sort()
    link = Link(host, port)
    link.connect(f"ee585-replay-{time.time_ns()}")
    for payload in calibration:
        link.publish(TOPIC_CALIBRATION, payload, qos=1, retain=True)
    for payload in scene:
        link.publish(TOPIC_SCENE, payload, qos=1, retain=True)
    t0_ns = merged[0][0] if merged else 0
    pacer = Pacer(speed, t0_ns)
    for t_ns, _kind, _index, topic, payload in merged:
        pacer.wait_until(t_ns)
        link.publish(topic, payload, qos=2, retain=False)
    link.finish(last_seq, last_t_ns)


def _read_frames(path: Path) -> list[bytes]:
    data = path.read_bytes()
    frames: list[bytes] = []
    offset = 0
    total = len(data)
    while offset < total:
        (length,) = struct.unpack_from("<I", data, offset)
        offset += 4
        frames.append(data[offset : offset + length])
        offset += length
    return frames


if __name__ == "__main__":
    main(sys.argv[1:])
