"""ECM optical camera rendering in the dedicated PyBullet render process."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from dvrk_simulator_base.types import Pose


@dataclass(frozen=True)
class CameraOptions:
    enabled: bool = True
    mode: str = "mono"
    renderer: str = "egl"
    socket_path: str | Path = "@dvrk:pybullet:mono_source"
    width: int = 1920
    height: int = 1080
    rate_hz: float = 30.0
    horizontal_fov_degrees: float = 60.0
    near_m: float = 0.01
    far_m: float = 10.0
    baseline_m: float = 0.006
    light_enabled: bool = True
    light_distance_m: float = 1.0
    light_ambient: float = 0.45
    light_diffuse: float = 0.65
    light_specular: float = 0.15

    def __post_init__(self) -> None:
        renderer = str(self.renderer).lower()
        mode = str(self.mode).lower()
        socket_reference = str(self.socket_path)
        if renderer not in {"egl", "tiny"}:
            raise ValueError("renderer must be 'egl' or 'tiny'")
        if mode not in {"mono", "stereo"}:
            raise ValueError("camera mode must be 'mono' or 'stereo'")
        if socket_reference.startswith("@dvrk:"):
            if len(socket_reference.split(":")) != 3:
                raise ValueError(
                    "dVRK camera socket must use @dvrk:<package>:<stream> syntax"
                )
            path: str | Path = socket_reference
        else:
            path = Path(socket_reference).expanduser()
            if not path.is_absolute():
                raise ValueError(
                    "camera socket must be @dvrk:<package>:<stream> or an absolute path"
                )
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        if not np.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("camera rate must be finite and positive")
        if not 0.0 < self.horizontal_fov_degrees < 180.0:
            raise ValueError("camera horizontal FOV must be between 0 and 180 degrees")
        if self.near_m <= 0.0 or self.far_m <= self.near_m:
            raise ValueError("camera clipping planes must satisfy 0 < near < far")
        if not np.isfinite(self.baseline_m) or self.baseline_m <= 0.0:
            raise ValueError("camera baseline must be finite and positive")
        if self.light_distance_m <= 0.0:
            raise ValueError("camera light distance must be positive")
        for name in ("light_ambient", "light_diffuse", "light_specular"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"camera {name} must be between 0 and 1")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "renderer", renderer)
        object.__setattr__(self, "socket_path", path)

    @property
    def vertical_fov_degrees(self) -> float:
        horizontal = math.radians(self.horizontal_fov_degrees)
        vertical = 2.0 * math.atan(math.tan(horizontal / 2.0) / (self.width / self.height))
        return math.degrees(vertical)

    @property
    def transport_width(self) -> int:
        return self.width * (2 if self.mode == "stereo" else 1)

    @classmethod
    def from_scene(cls, camera) -> "CameraOptions":
        settings = camera.as_dict()
        if camera.mode not in {"off", "mono", "stereo"}:
            raise ValueError("PyBullet supports off, mono, or stereo scene cameras")
        encoding = str(settings.get("encoding", "rgba8")).lower()
        if encoding != "rgba8":
            raise ValueError("PyBullet Unix-FD camera encoding must be rgba8")
        transports = settings.get("transports", ["unixfd"])
        if not isinstance(transports, list):
            raise ValueError("scene.camera.transports must be a list")
        unixfd = settings.get("unixfd", {}) or {}
        if not isinstance(unixfd, dict):
            raise ValueError("scene.camera.unixfd must be a mapping")
        light = settings.get("light", {}) or {}
        if not isinstance(light, dict):
            raise ValueError("scene.camera.light must be a mapping")
        return cls(
            enabled=camera.mode != "off" and "unixfd" in transports,
            mode="mono" if camera.mode == "off" else camera.mode,
            renderer=str(settings.get("renderer", "egl")),
            socket_path=str(
                unixfd.get(
                    "socket_path",
                    "@dvrk:pybullet:"
                    f"{'mono' if camera.mode == 'off' else camera.mode}_source",
                )
            ),
            width=int(settings.get("width", 1920)),
            height=int(settings.get("height", 1080)),
            rate_hz=float(settings.get("publish_rate_hz", 30.0)),
            horizontal_fov_degrees=float(settings.get("horizontal_fov_deg", 60.0)),
            near_m=float(settings.get("near_clip_m", 0.005)),
            far_m=float(settings.get("far_clip_m", 10.0)),
            baseline_m=float(settings.get("baseline_m", 0.006)),
            light_enabled=bool(light.get("enabled", True)),
            light_distance_m=float(light.get("distance_m", 1.0)),
            light_ambient=float(light.get("ambient", 0.45)),
            light_diffuse=float(light.get("diffuse", 0.65)),
            light_specular=float(light.get("specular", 0.15)),
        )


@dataclass(frozen=True)
class VideoFrame:
    rgba: np.ndarray
    simulation_time: float
    sequence: int


def view_vectors(optical_pose: Pose) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return PyBullet eye, target, and up vectors for ECM optical +X/+Z axes."""
    eye = optical_pose.position
    target = eye + optical_pose.orientation[:, 0]
    up = optical_pose.orientation[:, 2]
    return eye, target, up


class PyBulletCamera:
    def __init__(self, pybullet, connection: int, options: CameraOptions, renderer: int):
        self.pybullet = pybullet
        self.connection = connection
        self.options = options
        self.renderer = renderer
        self.sequence = 0
        self._projection = pybullet.computeProjectionMatrixFOV(
            fov=options.vertical_fov_degrees,
            aspect=options.width / options.height,
            nearVal=options.near_m,
            farVal=options.far_m,
        )

    def _capture_eye(self, optical_pose: Pose) -> np.ndarray:
        eye, target, up = view_vectors(optical_pose)
        view = self.pybullet.computeViewMatrix(eye, target, up)
        image_options = dict(
            width=self.options.width,
            height=self.options.height,
            viewMatrix=view,
            projectionMatrix=self._projection,
            renderer=self.renderer,
            physicsClientId=self.connection,
        )
        if self.options.light_enabled:
            # PyBullet exposes a directional light rather than a point light
            # for getCameraImage.  Aim it along the endoscope optical axis.
            image_options.update(
                lightDirection=tuple(target - eye),
                lightColor=(1.0, 1.0, 1.0),
                lightDistance=self.options.light_distance_m,
                shadow=1,
                lightAmbientCoeff=self.options.light_ambient,
                lightDiffuseCoeff=self.options.light_diffuse,
                lightSpecularCoeff=self.options.light_specular,
            )
        result = self.pybullet.getCameraImage(**image_options)
        rgba = np.asarray(result[2], dtype=np.uint8).reshape(
            self.options.height, self.options.width, 4
        )
        return np.ascontiguousarray(rgba)

    def capture(self, optical_pose: Pose, simulation_time: float) -> VideoFrame:
        if self.options.mode == "mono":
            rgba = self._capture_eye(optical_pose)
        else:
            half_baseline = 0.5 * self.options.baseline_m
            left_offset = optical_pose.orientation[:, 1] * half_baseline
            left_pose = Pose(optical_pose.position + left_offset, optical_pose.orientation)
            right_pose = Pose(optical_pose.position - left_offset, optical_pose.orientation)
            rgba = np.ascontiguousarray(
                np.concatenate(
                    (self._capture_eye(left_pose), self._capture_eye(right_pose)),
                    axis=1,
                )
            )
        frame = VideoFrame(rgba, float(simulation_time), self.sequence)
        self.sequence += 1
        return frame
