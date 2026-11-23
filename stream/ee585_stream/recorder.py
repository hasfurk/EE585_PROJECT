"""Record an MQTT run to runs/<name>/."""

from __future__ import annotations

import argparse
from pathlib import Path
import queue
import struct
import sys
import threading
import time

import paho.mqtt.client as mqtt
from google.protobuf.message import DecodeError

from ee585_stream.consumer import ABORTED, COMPLETED, DATA_ERROR, STALLED, StreamChecks
from ee585_stream.link import (
    TOPIC_CALIBRATION,
    TOPIC_SCENE,
    TOPIC_STATUS,
    TOPIC_STREAM,
    TOPIC_TRUTH,
    decode,
)
from ee585_stream.stream_pb2 import Calibration, Record, Status

_QUIET_S = 0.5


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--stall-timeout", type=float, default=5.0)
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    args = parser.parse_args(argv)
    reason = record_run(
        Path(args.run),
        args.host,
        args.port,
        stall_timeout_s=args.stall_timeout,
        startup_timeout_s=args.startup_timeout,
    )
    if reason != COMPLETED:
        raise SystemExit(1)


def record_run(
    run: Path,
    host: str,
    port: int,
    *,
    stall_timeout_s: float,
    startup_timeout_s: float,
) -> str:
    run.mkdir(parents=True, exist_ok=True)
    created = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    bins = {
        TOPIC_CALIBRATION: (run / "calibration.bin").open("wb"),
        TOPIC_SCENE: (run / "scene.bin").open("wb"),
        TOPIC_STREAM: (run / "stream.bin").open("wb"),
        TOPIC_TRUTH: (run / "truth.bin").open("wb"),
    }
    counts = {name: 0 for name in ("calibration", "scene", "stream", "truth")}
    checks = StreamChecks()
    messages: queue.Queue[tuple[str, bytes]] = queue.Queue()
    suback = threading.Event()
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"ee585-recorder-{time.time_ns()}",
        protocol=mqtt.MQTTv311,
        reconnect_on_failure=False,
    )

    def on_message(_client: mqtt.Client, _userdata: object, message: mqtt.MQTTMessage) -> None:
        messages.put((message.topic, bytes(message.payload)))

    def on_subscribe(*_args: object) -> None:
        suback.set()

    client.on_message = on_message
    client.on_subscribe = on_subscribe
    map_name = ""
    schema_version = 0
    reason = STALLED
    saw_record = False
    online = False
    completed: tuple[int, int] | None = None
    started = time.monotonic()
    last_record_at = started
    try:
        client.connect(host, port, keepalive=2)
        client.loop_start()
        client.subscribe(
            [
                (TOPIC_STATUS, 1),
                (TOPIC_CALIBRATION, 1),
                (TOPIC_SCENE, 1),
                (TOPIC_STREAM, 2),
                (TOPIC_TRUTH, 2),
            ]
        )
        if not suback.wait(10):
            raise ConnectionError("broker did not confirm the subscription")
        (run / ".subscribed").write_text("1")
        quiet_until: float | None = None
        while True:
            now = time.monotonic()
            if quiet_until is not None:
                timeout = max(0.0, quiet_until - now)
            elif not saw_record:
                timeout = max(0.0, startup_timeout_s - (now - started))
            else:
                timeout = max(0.0, stall_timeout_s - (now - last_record_at))
            try:
                topic, payload = messages.get(timeout=timeout)
            except queue.Empty:
                if quiet_until is not None:
                    reason = COMPLETED
                else:
                    reason = STALLED
                break
            if topic == TOPIC_CALIBRATION:
                _write(bins[topic], payload)
                counts["calibration"] += 1
                try:
                    decoded = decode(topic, payload)
                except DecodeError:
                    reason = DATA_ERROR
                    break
                if isinstance(decoded, Calibration):
                    map_name = decoded.map_name
                    schema_version = int(decoded.schema_version)
                continue
            if topic in (TOPIC_SCENE, TOPIC_TRUTH):
                _write(bins[topic], payload)
                counts["scene" if topic == TOPIC_SCENE else "truth"] += 1
                if completed is not None and saw_record and checks.last_seq >= completed[0]:
                    quiet_until = time.monotonic() + _QUIET_S
                continue
            if topic == TOPIC_STATUS:
                try:
                    status = decode(topic, payload)
                except DecodeError:
                    reason = DATA_ERROR
                    break
                if not isinstance(status, Status):
                    reason = DATA_ERROR
                    break
                if status.state == Status.ONLINE:
                    online = True
                elif status.state == Status.ABORTED:
                    reason = ABORTED
                    break
                elif status.state == Status.COMPLETED:
                    completed = (int(status.last_seq), int(status.last_t_ns))
                    if saw_record and checks.last_seq >= completed[0]:
                        quiet_until = time.monotonic() + _QUIET_S
                continue
            if topic != TOPIC_STREAM:
                continue
            try:
                record = decode(topic, payload)
            except DecodeError:
                reason = DATA_ERROR
                break
            if not isinstance(record, Record):
                reason = DATA_ERROR
                break
            decision = checks.observe(record)
            if decision == "drop":
                continue
            if decision == "error":
                reason = DATA_ERROR
                break
            _write(bins[topic], payload)
            counts["stream"] += 1
            saw_record = True
            last_record_at = time.monotonic()
            if completed is not None and checks.last_seq >= completed[0]:
                quiet_until = time.monotonic() + _QUIET_S
            if not online and now - started > startup_timeout_s:
                reason = STALLED
                break
    finally:
        for handle in bins.values():
            handle.close()
        (run / ".subscribed").unlink(missing_ok=True)
        client.disconnect()
        client.loop_stop()
        _write_meta(run, schema_version, map_name, counts, reason, created)
    return reason


def _write(handle: object, payload: bytes) -> None:
    handle.write(struct.pack("<I", len(payload)))  # type: ignore[attr-defined]
    handle.write(payload)  # type: ignore[attr-defined]


def _write_meta(
    run: Path,
    schema_version: int,
    map_name: str,
    counts: dict[str, int],
    reason: str,
    created: str,
) -> None:
    import json

    (run / "meta.json").write_text(
        json.dumps(
            {
                "schema_version": schema_version,
                "map_name": map_name,
                "counts": counts,
                "terminal_reason": reason,
                "created_wall_time": created,
            }
        )
        + "\n"
    )


if __name__ == "__main__":
    main(sys.argv[1:])
