"""Stream tests use the Mac's already-running Mosquitto on 127.0.0.1:1883.

The suite does not start or stop a broker. Clients are paho.
"""

from __future__ import annotations

import socket
import time

import paho.mqtt.client as mqtt
import pytest

from ee585_stream.link import (
    TOPIC_CALIBRATION,
    TOPIC_SCENE,
    TOPIC_STATUS,
    TOPIC_STREAM,
    TOPIC_TRUTH,
)


def clear_retained() -> None:
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=f"ee585-clear-{time.time_ns()}",
        protocol=mqtt.MQTTv311,
        reconnect_on_failure=False,
    )
    client.connect("127.0.0.1", 1883, keepalive=2)
    client.loop_start()
    infos = [
        client.publish(topic, b"", qos=1, retain=True)
        for topic in (TOPIC_STATUS, TOPIC_CALIBRATION, TOPIC_SCENE, TOPIC_STREAM, TOPIC_TRUTH)
    ]
    for info in infos:
        info.wait_for_publish(timeout=2)
    client.disconnect()
    client.loop_stop()


def _broker_accepts() -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(1.0)
        sock.connect(("127.0.0.1", 1883))
    except OSError:
        return False
    else:
        return True
    finally:
        sock.close()


@pytest.fixture(scope="session")
def broker() -> None:
    if not _broker_accepts():
        raise RuntimeError(
            "127.0.0.1:1883 is not accepting connections. Start the local Mosquitto broker; "
            "these tests do not launch one."
        )


@pytest.fixture(autouse=True)
def _clear_retained(broker: None) -> None:
    del broker
    clear_retained()
