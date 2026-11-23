"""Stream consumer: calibration, records, and one terminal event."""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
import queue
import threading
import time

import paho.mqtt.client as mqtt
from google.protobuf.message import DecodeError

from ee585_stream.link import (
    TOPIC_CALIBRATION,
    TOPIC_SCENE,
    TOPIC_STATUS,
    TOPIC_STREAM,
    TOPIC_TRUTH,
    decode,
)
from ee585_stream.stream_pb2 import Calibration, Record, Scene, Status, TruthRecord

COMPLETED = "COMPLETED"
ABORTED = "ABORTED"
STALLED = "STALLED"
DATA_ERROR = "DATA_ERROR"


class Terminal:
    def __init__(self, reason: str, last_seq: int, last_t_ns: int) -> None:
        self.reason = reason
        self.last_seq = last_seq
        self.last_t_ns = last_t_ns


class StreamChecks:
    def __init__(self) -> None:
        self.last_seq = -1
        self.last_t_ns = -1

    def observe(self, record: Record) -> str:
        expected = self.last_seq + 1
        if record.seq < expected:
            return "drop"
        if record.seq > expected:
            return "error"
        if self.last_seq >= 0 and record.t_ns < self.last_t_ns:
            return "error"
        self.last_seq = int(record.seq)
        self.last_t_ns = int(record.t_ns)
        return "yield"


def write_run_status(path: str, terminal: Terminal) -> None:
    dest = Path(path)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(
        json.dumps(
            {
                "reason": terminal.reason,
                "last_seq": terminal.last_seq,
                "last_t_ns": terminal.last_t_ns,
            }
        )
        + "\n"
    )


class Consumer:
    def __init__(
        self,
        host: str,
        port: int,
        stall_timeout_s: float = 5.0,
        startup_timeout_s: float = 60.0,
        *,
        evaluator: bool = False,
    ) -> None:
        self._host = host
        self._port = port
        self._stall_timeout_s = stall_timeout_s
        self._startup_timeout_s = startup_timeout_s
        self._evaluator = evaluator
        self.subscribed = threading.Event()

    def events(self) -> Iterator[Calibration | Record | Scene | TruthRecord | Terminal]:
        messages: queue.Queue[tuple[str, bytes]] = queue.Queue()
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"ee585-consumer-{time.time_ns()}",
            protocol=mqtt.MQTTv311,
            reconnect_on_failure=False,
        )
        suback = threading.Event()

        def on_message(_client: mqtt.Client, _userdata: object, message: mqtt.MQTTMessage) -> None:
            messages.put((message.topic, bytes(message.payload)))

        def on_subscribe(
            _client: mqtt.Client,
            _userdata: object,
            _mid: int,
            _reason_codes: list[mqtt.ReasonCode],
            _properties: mqtt.Properties,
        ) -> None:
            suback.set()

        client.on_message = on_message
        client.on_subscribe = on_subscribe
        client.connect(self._host, self._port, keepalive=2)
        client.loop_start()
        topics = [
            (TOPIC_STATUS, 1),
            (TOPIC_CALIBRATION, 1),
            (TOPIC_STREAM, 2),
        ]
        if self._evaluator:
            topics.extend(((TOPIC_SCENE, 1), (TOPIC_TRUTH, 2)))
        client.subscribe(topics)
        if not suback.wait(10):
            client.loop_stop()
            raise ConnectionError("broker did not confirm the subscription")
        self.subscribed.set()
        try:
            yield from _events(
                messages,
                stall_timeout_s=self._stall_timeout_s,
                startup_timeout_s=self._startup_timeout_s,
                evaluator=self._evaluator,
            )
        finally:
            client.disconnect()
            client.loop_stop()


def _events(
    messages: queue.Queue[tuple[str, bytes]],
    *,
    stall_timeout_s: float,
    startup_timeout_s: float,
    evaluator: bool,
) -> Iterator[Calibration | Record | Scene | TruthRecord | Terminal]:
    checks = StreamChecks()
    online = False
    completed: tuple[int, int] | None = None
    saw_record = False
    started = time.monotonic()
    last_record_at = started
    held: list[tuple[str, bytes]] = []

    def terminal(reason: str) -> Terminal:
        return Terminal(reason, checks.last_seq, checks.last_t_ns)

    def wait_item() -> tuple[str, bytes] | None:
        now = time.monotonic()
        if not saw_record:
            remaining = startup_timeout_s - (now - started)
        else:
            remaining = stall_timeout_s - (now - last_record_at)
        if remaining <= 0:
            return None
        try:
            return messages.get(timeout=remaining)
        except queue.Empty:
            return None

    while True:
        item = wait_item()
        if item is None:
            yield terminal(STALLED)
            return
        topic, payload = item
        if topic == TOPIC_CALIBRATION:
            try:
                decoded = decode(topic, payload)
            except DecodeError:
                yield terminal(DATA_ERROR)
                return
            if not isinstance(decoded, Calibration):
                yield terminal(DATA_ERROR)
                return
            yield decoded
            break
        if topic == TOPIC_STATUS:
            outcome = _handle(topic, payload, checks=checks, online=online, evaluator=evaluator)
            online = outcome.online
            if outcome.stop is not None:
                yield terminal(outcome.stop)
                return
            continue
        held.append(item)

    pending = held
    while True:
        if not pending:
            item = wait_item()
            if item is None:
                yield terminal(STALLED)
                return
            pending.append(item)
        topic, payload = pending.pop(0)
        outcome = _handle(topic, payload, checks=checks, online=online, evaluator=evaluator)
        online = outcome.online
        if outcome.completed is not None:
            completed = outcome.completed
        if outcome.record_yielded:
            saw_record = True
            last_record_at = time.monotonic()
        if outcome.event is not None:
            yield outcome.event
        if outcome.stop == DATA_ERROR:
            yield terminal(DATA_ERROR)
            return
        if outcome.stop == ABORTED:
            rest = pending
            while True:
                try:
                    rest.append(messages.get_nowait())
                except queue.Empty:
                    break
            for queued_topic, queued_payload in rest:
                queued = _handle(
                    queued_topic,
                    queued_payload,
                    checks=checks,
                    online=online,
                    evaluator=evaluator,
                )
                if queued.record_yielded:
                    saw_record = True
                if queued.event is not None:
                    yield queued.event
                if queued.stop == DATA_ERROR:
                    yield terminal(DATA_ERROR)
                    return
            yield terminal(ABORTED)
            return
        if completed is not None and saw_record and checks.last_seq >= completed[0]:
            yield terminal(COMPLETED)
            return


class _Outcome:
    def __init__(self) -> None:
        self.event: Record | Scene | TruthRecord | None = None
        self.stop: str | None = None
        self.online = False
        self.completed: tuple[int, int] | None = None
        self.record_yielded = False


def _handle(
    topic: str,
    payload: bytes,
    *,
    checks: StreamChecks,
    online: bool,
    evaluator: bool,
) -> _Outcome:
    outcome = _Outcome()
    outcome.online = online
    if topic == TOPIC_STATUS:
        try:
            status = decode(topic, payload)
        except DecodeError:
            outcome.stop = DATA_ERROR
            return outcome
        if not isinstance(status, Status):
            outcome.stop = DATA_ERROR
            return outcome
        if status.state == Status.ONLINE:
            outcome.online = True
        elif status.state == Status.ABORTED:
            outcome.stop = ABORTED
        elif status.state == Status.COMPLETED:
            outcome.completed = (int(status.last_seq), int(status.last_t_ns))
        return outcome
    if topic == TOPIC_STREAM:
        try:
            record = decode(topic, payload)
        except DecodeError:
            outcome.stop = DATA_ERROR
            return outcome
        if not isinstance(record, Record):
            outcome.stop = DATA_ERROR
            return outcome
        decision = checks.observe(record)
        if decision == "drop":
            return outcome
        if decision == "error":
            outcome.stop = DATA_ERROR
            return outcome
        outcome.event = record
        outcome.record_yielded = True
        return outcome
    if evaluator and topic in (TOPIC_SCENE, TOPIC_TRUTH):
        try:
            decoded = decode(topic, payload)
        except DecodeError:
            outcome.stop = DATA_ERROR
            return outcome
        if isinstance(decoded, (Scene, TruthRecord)):
            outcome.event = decoded
        return outcome
    return outcome
