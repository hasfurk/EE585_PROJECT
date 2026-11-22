"""Loopback aMQTT broker on 127.0.0.1:1883."""

from __future__ import annotations

import asyncio
import socket
import sys
from typing import Any

from amqtt.broker import Broker
from amqtt.errors import BrokerError

# aMQTT 0.12 loads `auth` only through its deprecated entry-point path, which
# also starts the file-auth and $SYS plugins. The plugins table is the
# supported way to allow anonymous clients on this version.
CONFIG: dict[str, Any] = {
    "listeners": {"default": {"type": "tcp", "bind": "127.0.0.1:1883"}},
    "plugins": {
        "amqtt.plugins.authentication.AnonymousAuthPlugin": {"allow_anonymous": True},
    },
}


def _port_is_free(host: str, port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((host, port))
    except OSError:
        return False
    else:
        return True
    finally:
        sock.close()


async def _serve() -> None:
    broker = Broker(CONFIG)
    await broker.start()
    print("listening on 127.0.0.1:1883", flush=True)
    try:
        await asyncio.Event().wait()
    finally:
        await broker.shutdown()


def main() -> None:
    if not _port_is_free("127.0.0.1", 1883):
        print(
            "127.0.0.1:1883 is already in use. Free the port; this broker does not choose another.",
            file=sys.stderr,
        )
        raise SystemExit(1)
    try:
        asyncio.run(_serve())
    except KeyboardInterrupt:
        return
    except BrokerError as exc:
        print(f"broker failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
