"""Generate ee585_stream/stream_pb2.py from stream.proto."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    proto_dir = root / "ee585_stream"
    proto = proto_dir / "stream.proto"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"-I{proto_dir}",
            f"--python_out={proto_dir}",
            str(proto),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
