"""Low-latency GStreamer Unix-FD video output backed by Linux memfd buffers."""

from __future__ import annotations

from typing import Any

from .errors import GStreamerDependencyError, PyBulletBackendError
from dvrk_simulator_base.video import (
    UnixFdVideoSink as _BaseUnixFdVideoSink,
    VideoFrame,
)


class UnixFdVideoSink(_BaseUnixFdVideoSink):
    """PyBullet specialization of the shared Unix-FD GStreamer video sink."""

    def __init__(self, options: Any) -> None:
        super().__init__(
            options,
            pipeline_name="dvrk-camera",
            memfd_name="dvrk-pybullet-camera",
            error_cls=PyBulletBackendError,
        )
