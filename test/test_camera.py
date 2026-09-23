from pathlib import Path

import numpy as np
import PyKDL

from dvrk_simulator_base.scene import SceneCamera
from dvrk_simulator_base.types import Pose
from dvrk_pybullet.camera import CameraOptions, PyBulletCamera, view_vectors


def test_scene_camera_uses_shared_isaac_field_names():
    camera = SceneCamera(
        mode="mono",
        owner="ECM",
        settings={
            "mode": "mono",
            "owner": "ECM",
            "width": 640,
            "height": 480,
            "horizontal_fov_deg": 72.0,
            "near_clip_m": 0.01,
            "far_clip_m": 5.0,
            "publish_rate_hz": 25.0,
            "transports": ["unixfd"],
            "unixfd": {"socket_path": "/tmp/test-camera.sock"},
        },
    )
    options = CameraOptions.from_scene(camera)
    assert options.enabled
    assert options.socket_path == Path("/tmp/test-camera.sock")
    assert (options.width, options.height, options.rate_hz) == (640, 480, 25.0)
    assert options.horizontal_fov_degrees == 72.0


def test_optical_axes_map_to_pybullet_view_vectors():
    pose = PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(1.0, 2.0, 3.0))
    eye, target, up = view_vectors(pose)
    np.testing.assert_allclose(eye, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(target, [2.0, 2.0, 3.0])
    np.testing.assert_allclose(up, [0.0, 0.0, 1.0])


def test_camera_accepts_canonical_dvrk_abstract_socket():
    options = CameraOptions(socket_path="@dvrk:pybullet:test")
    assert options.socket_path == "@dvrk:pybullet:test"


class _FakePyBullet:
    def __init__(self):
        self.views = []

    def computeProjectionMatrixFOV(self, **kwargs):
        self.projection = kwargs
        return "projection"

    def computeViewMatrix(self, eye, target, up):
        self.view = (eye, target, up)
        self.views.append(self.view)
        return "view"

    def getCameraImage(self, **kwargs):
        self.image = kwargs
        rgba = np.arange(kwargs["width"] * kwargs["height"] * 4, dtype=np.uint8)
        return kwargs["width"], kwargs["height"], rgba, None, None


def test_camera_returns_contiguous_rgba():
    backend = _FakePyBullet()
    options = CameraOptions(width=4, height=3)
    camera = PyBulletCamera(backend, 7, options, renderer=99)
    frame = camera.capture(PyKDL.Frame(), 1.25)
    assert frame.rgba.shape == (3, 4, 4)
    assert frame.rgba.flags.c_contiguous
    assert frame.simulation_time == 1.25
    assert backend.image["physicsClientId"] == 7
    assert backend.image["lightDirection"] == (1.0, 0.0, 0.0)
    assert backend.image["shadow"] == 1


def test_stereo_camera_renders_left_then_right_side_by_side():
    backend = _FakePyBullet()
    options = CameraOptions(mode="stereo", width=4, height=3, baseline_m=0.006)
    camera = PyBulletCamera(backend, 7, options, renderer=99)
    frame = camera.capture(PyKDL.Frame(), 1.25)
    assert frame.rgba.shape == (3, 8, 4)
    assert frame.rgba.flags.c_contiguous
    assert options.transport_width == 8
    np.testing.assert_allclose(backend.views[0][0], [0.0, 0.003, 0.0])
    np.testing.assert_allclose(backend.views[1][0], [0.0, -0.003, 0.0])
