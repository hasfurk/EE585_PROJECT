"""Set MUJOCO_GL before the first import of mujoco."""

from __future__ import annotations

import os
import sys


def configure() -> str | None:
    """Leave a user-set MUJOCO_GL unchanged. On Linux, default to egl."""
    if "MUJOCO_GL" in os.environ:
        return os.environ["MUJOCO_GL"]
    if sys.platform.startswith("linux"):
        os.environ["MUJOCO_GL"] = "egl"
        return "egl"
    return None


MUJOCO_GL: str | None = configure()
