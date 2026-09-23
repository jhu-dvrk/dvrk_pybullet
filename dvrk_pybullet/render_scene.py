"""Render-only PyBullet world owned exclusively by the camera process."""

from __future__ import annotations

from dataclasses import dataclass
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from dvrk_simulator_base.types import Pose

from .backend import load_pybullet
from .camera import PyBulletCamera
from .errors import PyBulletBackendError
from .robot import load_robot
from .scene_objects import load_scene_objects
from .video import UnixFdVideoSink


@dataclass(frozen=True)
class RenderRobot:
    name: str
    urdf_path: Path
    joint_names: tuple[str, ...]
    base_position: tuple[float, ...]
    base_orientation: tuple[float, ...]
    joint_count: int


class RenderScene:
    def __init__(
        self, robots, camera_options, scene_objects=(), video_sink_factory=UnixFdVideoSink
    ):
        self.robots = robots
        self.camera_options = camera_options
        self.scene_object_specs = scene_objects
        self._video_sink_factory = video_sink_factory
        self.pybullet = load_pybullet()
        self.connection = -1
        self._egl_plugin = -1
        self.arms = {}
        self.scene_objects = {}
        self.grasp_markers = {}
        self.camera = None
        self.video_sink = None

    def initialize(self) -> None:
        self.connection = self.pybullet.connect(self.pybullet.DIRECT)
        if self.connection < 0:
            raise PyBulletBackendError("could not connect the camera PyBullet world")
        # PyBullet's EGL plugin must be registered before creating visual mesh
        # shapes.  This is independent of the removed MTL/color workarounds.
        self._initialize_camera()
        self.scene_objects = load_scene_objects(
            self.pybullet, self.scene_object_specs, connection=self.connection
        )
        for spec in self.robots:
            robot = load_robot(
                self.pybullet, spec.urdf_path, spec.joint_names,
                base_position=spec.base_position,
                base_orientation_xyzw=spec.base_orientation,
            )
            if self.pybullet.getNumJoints(robot.body_id) != spec.joint_count:
                raise PyBulletBackendError(f"render joint layout differs for {spec.name}")
            self.arms[spec.name] = SimpleNamespace(
                robot=robot, artifact=SimpleNamespace(urdf_path=spec.urdf_path)
            )

    def apply_state(self, state):
        expected_length = len(self.robots) + 4
        if len(state) != expected_length:
            raise PyBulletBackendError("camera state does not match the render scene")
        joint_states = state[: len(self.robots)]
        position, orientation, object_poses, marker_poses = state[-4:]
        for spec, positions in zip(self.robots, joint_states):
            body = self.arms[spec.name].robot.body_id
            if len(positions) != spec.joint_count:
                raise PyBulletBackendError(
                    f"camera state joint count differs for {spec.name}"
                )
            for index, value in enumerate(positions):
                self.pybullet.resetJointState(
                    body, index, float(value),
                    physicsClientId=self.connection,
                )
        if len(object_poses) != len(self.scene_objects):
            raise PyBulletBackendError("camera object state does not match the render scene")
        for item, pose in zip(self.scene_objects.values(), object_poses):
            object_position, object_orientation = pose
            self.pybullet.resetBasePositionAndOrientation(
                item.body_id,
                object_position,
                object_orientation,
                physicsClientId=self.connection,
            )
        self._sync_grasp_markers(marker_poses)
        return Pose(
            np.asarray(position, dtype=float),
            np.asarray(orientation, dtype=float).reshape(3, 3),
        )

    def _sync_grasp_markers(self, marker_poses) -> None:
        """Mirror control-world grasp markers into this render-only world."""
        active = set(marker_poses)
        for name in tuple(self.grasp_markers):
            if name not in active:
                self.pybullet.removeBody(
                    self.grasp_markers.pop(name), physicsClientId=self.connection
                )
        for name, (position, orientation) in marker_poses.items():
            if name not in self.grasp_markers:
                shape = self.pybullet.createVisualShape(
                    self.pybullet.GEOM_BOX,
                    halfExtents=(0.002, 0.002, 0.002),
                    rgbaColor=(1.0, 0.0, 0.0, 1.0),
                    physicsClientId=self.connection,
                )
                self.grasp_markers[name] = self.pybullet.createMultiBody(
                    baseMass=0.0,
                    baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=shape,
                    physicsClientId=self.connection,
                )
            self.pybullet.resetBasePositionAndOrientation(
                self.grasp_markers[name], position, orientation,
                physicsClientId=self.connection,
            )

    def render(self, state, timestamp):
        pose = self.apply_state(state)
        # No stepSimulation: all physical state comes from the control process.
        self.video_sink.push(self.camera.capture(pose, timestamp))

    def _initialize_camera(self) -> None:
        options = self.camera_options
        if options.renderer == "tiny":
            renderer = self.pybullet.ER_TINY_RENDERER
        else:
            spec = importlib.util.find_spec("eglRenderer")
            if spec is None or not spec.origin:
                raise PyBulletBackendError(
                    "headless EGL renderer is unavailable; install the PyBullet EGL "
                    "plugin or set renderer: tiny in pybullet.yaml"
                )
            self._egl_plugin = self.pybullet.loadPlugin(
                spec.origin, "_eglRendererPlugin", physicsClientId=self.connection
            )
            if self._egl_plugin < 0:
                raise PyBulletBackendError(
                    "PyBullet failed to load its headless EGL renderer; use "
                    "renderer: tiny in pybullet.yaml to diagnose without GPU rendering"
                )
            renderer = self.pybullet.ER_BULLET_HARDWARE_OPENGL
        self.camera = PyBulletCamera(
            self.pybullet, self.connection, options, renderer
        )
        self.video_sink = self._video_sink_factory(options)
        self.video_sink.start()
    def close(self) -> None:
        try:
            if self.video_sink is not None:
                self.video_sink.close()
        finally:
            if self.connection >= 0 and self.pybullet.isConnected(self.connection):
                if self._egl_plugin >= 0:
                    self.pybullet.unloadPlugin(
                        self._egl_plugin, physicsClientId=self.connection
                    )
                self.pybullet.disconnect(self.connection)
            self.video_sink = None
            self.camera = None
            self.connection = -1
