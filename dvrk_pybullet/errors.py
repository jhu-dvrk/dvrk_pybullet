"""Backend-specific exception hierarchy."""

from dvrk_simulator_base.video import VideoSinkError, GStreamerDependencyError as _BaseGstError


class PyBulletBackendError(VideoSinkError):
    """Base class for actionable PyBullet backend failures."""


class PyBulletDependencyError(PyBulletBackendError):
    """Raised when the PyBullet Python module is unavailable."""


class GStreamerDependencyError(PyBulletBackendError, _BaseGstError):
    """Raised when the Python GStreamer Unix-FD stack is unavailable."""
