"""Topics, encode/decode, and the bounded MQTT publisher."""

from __future__ import annotations

import threading
import time

import paho.mqtt.client as mqtt
from google.protobuf.message import Message

from ee585_stream import stream_pb2

PREFIX = "ee585/v1/"
TOPIC_STATUS = PREFIX + "status"
TOPIC_CALIBRATION = PREFIX + "calibration"
TOPIC_SCENE = PREFIX + "scene"
TOPIC_STREAM = PREFIX + "stream"
TOPIC_TRUTH = PREFIX + "truth"

_BACKLOG_LIMIT = 100
_BACKLOG_WAIT_S = 2.0
# Mosquitto 2.1 drops QoS 1/2 publishes once a client exceeds its receive
# maximum. The default max_inflight_messages is 20. Stay inside that window.
# The 100-message ceiling above is unchanged: this window is stricter.
_INFLIGHT_LIMIT = 20

_BY_TOPIC: dict[str, type[Message]] = {
    TOPIC_STATUS: stream_pb2.Status,
    TOPIC_CALIBRATION: stream_pb2.Calibration,
    TOPIC_SCENE: stream_pb2.Scene,
    TOPIC_STREAM: stream_pb2.Record,
    TOPIC_TRUTH: stream_pb2.TruthRecord,
}


def encode(msg: Message) -> bytes:
    return msg.SerializeToString()


def decode(topic: str, payload: bytes) -> Message:
    try:
        cls = _BY_TOPIC[topic]
    except KeyError as exc:
        raise ValueError(f"unknown topic {topic}") from exc
    msg = cls()
    msg.ParseFromString(payload)
    return msg


class Pacer:
    def __init__(self, speed: float, t0_ns: int) -> None:
        self._speed = speed
        self._t0_ns = t0_ns
        self._wall_start = time.monotonic()

    def wait_until(self, t_ns: int) -> None:
        if self._speed == 0:
            return
        target = self._wall_start + (t_ns - self._t0_ns) / self._speed / 1e9
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)


class Link:
    def __init__(self, host: str = "127.0.0.1", port: int = 1883) -> None:
        self._host = host
        self._port = port
        self._client: mqtt.Client | None = None
        self._cond = threading.Condition()
        self._pending: set[int] = set()
        self._early: set[int] = set()
        self._connected = threading.Event()
        self._disconnected = threading.Event()
        self._closing = False

    def connect(self, client_id: str, will: bool = True) -> None:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            protocol=mqtt.MQTTv311,
            reconnect_on_failure=False,
        )
        client.max_inflight_messages_set(_INFLIGHT_LIMIT)
        client.on_connect = self._on_connect
        client.on_publish = self._on_publish
        client.on_disconnect = self._on_disconnect
        if will:
            client.will_set(
                TOPIC_STATUS,
                encode(_status(stream_pb2.Status.ABORTED, -1, -1, "last will")),
                qos=1,
                retain=True,
            )
        self._client = client
        rc = client.connect(self._host, self._port, keepalive=2)
        if rc != mqtt.MQTT_ERR_SUCCESS:
            raise ConnectionError(mqtt.error_string(rc))
        client.loop_start()
        if not self._connected.wait(10):
            client.loop_stop()
            raise ConnectionError("broker did not accept the connection")
        self.publish(
            TOPIC_STATUS,
            encode(_status(stream_pb2.Status.ONLINE, -1, -1, "")),
            qos=1,
            retain=True,
        )
        self._drain()

    def publish(self, topic: str, payload: bytes, qos: int, retain: bool) -> None:
        client = self._require_client()
        self._acquire_slot()
        info = client.publish(topic, payload, qos=qos, retain=retain)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            raise ConnectionError(mqtt.error_string(info.rc))
        if qos == 0:
            return
        with self._cond:
            if info.is_published() or info.mid in self._early:
                self._early.discard(info.mid)
                self._cond.notify_all()
            else:
                self._pending.add(info.mid)

    def finish(self, last_seq: int, last_t_ns: int) -> None:
        self._drain()
        self.publish(
            TOPIC_STATUS,
            encode(_status(stream_pb2.Status.COMPLETED, last_seq, last_t_ns, "")),
            qos=1,
            retain=True,
        )
        self._drain()
        self._close()

    def abort(self, detail: str) -> None:
        client = self._client
        if client is not None and self._connected.is_set() and not self._closing:
            info = client.publish(
                TOPIC_STATUS,
                encode(_status(stream_pb2.Status.ABORTED, -1, -1, detail)),
                qos=1,
                retain=True,
            )
            try:
                info.wait_for_publish(timeout=_BACKLOG_WAIT_S)
            except (RuntimeError, ValueError):
                pass
            self._close()
        raise SystemExit(1)

    def _acquire_slot(self) -> None:
        deadline = time.monotonic() + _BACKLOG_WAIT_S
        with self._cond:
            while len(self._pending) >= _INFLIGHT_LIMIT or len(self._pending) > _BACKLOG_LIMIT:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            else:
                return
        self.abort("publisher backlog")

    def _drain(self) -> None:
        deadline = time.monotonic() + _BACKLOG_WAIT_S
        with self._cond:
            while self._pending:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            else:
                return
        self.abort("publisher backlog")

    def _close(self) -> None:
        client = self._client
        if client is None or self._closing:
            return
        self._closing = True
        client.disconnect()
        self._disconnected.wait(timeout=_BACKLOG_WAIT_S)
        client.loop_stop()

    def _require_client(self) -> mqtt.Client:
        if self._client is None:
            raise RuntimeError("connect() was not called")
        return self._client

    def _on_connect(
        self,
        _client: mqtt.Client,
        _userdata: object,
        _flags: mqtt.ConnectFlags,
        reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties,
    ) -> None:
        if not reason_code.is_failure:
            self._connected.set()

    def _on_publish(
        self,
        _client: mqtt.Client,
        _userdata: object,
        mid: int,
        _reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties,
    ) -> None:
        with self._cond:
            if mid in self._pending:
                self._pending.remove(mid)
                self._cond.notify_all()
            else:
                self._early.add(mid)

    def _on_disconnect(
        self,
        _client: mqtt.Client,
        _userdata: object,
        _flags: mqtt.DisconnectFlags,
        _reason_code: mqtt.ReasonCode,
        _properties: mqtt.Properties,
    ) -> None:
        self._disconnected.set()


def _status(state: int, last_seq: int, last_t_ns: int, detail: str) -> stream_pb2.Status:
    return stream_pb2.Status(state=state, last_seq=last_seq, last_t_ns=last_t_ns, detail=detail)
