"""Latest-state camera worker isolated from the control PyBullet connection."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from multiprocessing import get_context
import os
from queue import Empty, Full
import signal
import sys
import time

from .errors import PyBulletBackendError
from .render_scene import RenderRobot, RenderScene


def _exit_when_parent_exits() -> None:
    """Ensure a spawned renderer cannot outlive a killed simulator process."""
    if not sys.platform.startswith("linux"):
        return
    parent_pid = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    # PR_SET_PDEATHSIG is Linux-specific.  Do not make camera rendering depend
    # on it being available, but use it whenever the platform provides it.
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        return
    # The parent could have exited in the small interval before prctl above.
    if os.getppid() != parent_pid:
        raise SystemExit(0)


def _worker_main(robots, camera_options, scene_objects, initial_state, states, status) -> None:
    scene = None
    try:
        _exit_when_parent_exits()
        scene = RenderScene(robots, camera_options, scene_objects)
        scene.initialize()
        latest = initial_state
        next_frame = time.monotonic()
        status.put(("ready", ""))
        period = 1.0 / camera_options.rate_hz
        while True:
            # A slow render can overrun the camera period.  Drain the state
            # queue before every render so it never repeats an old robot pose.
            try:
                while True:
                    candidate = states.get_nowait()
                    if candidate == "shutdown":
                        return
                    latest = candidate
            except Empty:
                pass

            now = time.monotonic()
            if now >= next_frame:
                scene.render(latest, now)
                next_frame = max(next_frame + period, time.monotonic())
                continue
            try:
                candidate = states.get(timeout=next_frame - now)
            except Empty:
                continue
            if candidate == "shutdown":
                return
            latest = candidate
    except BaseException as error:
        status.put(("error", f"{type(error).__name__}: {error}"))
    finally:
        if scene is not None:
            scene.close()


@dataclass
class CameraWorker:
    arms: dict
    camera_options: object
    scene_objects: tuple = ()

    def __post_init__(self) -> None:
        self._context = get_context("spawn")
        self._states = self._context.Queue(maxsize=1)
        self._status = self._context.Queue()
        self._process = None

    def _robots(self):
        return tuple(
            RenderRobot(
                name=name,
                urdf_path=arm.artifact.urdf_path,
                joint_names=arm.robot.controlled_joint_names,
                base_position=tuple(arm.config.base_position),
                base_orientation=tuple(arm.config.base_orientation_xyzw),
                joint_count=arm.pybullet.getNumJoints(arm.robot.body_id),
            )
            for name, arm in self.arms.items()
        )

    def start(self, initial_state) -> None:
        self._process = self._context.Process(
            target=_worker_main,
            args=(self._robots(), self.camera_options, self.scene_objects, initial_state,
                  self._states, self._status),
            daemon=True,
        )
        self._process.start()
        try:
            kind, detail = self._status.get(timeout=15.0)
        except Empty as error:
            self.close()
            raise PyBulletBackendError(
                "camera worker did not report readiness within 15 seconds"
            ) from error
        if kind != "ready":
            self.close()
            raise PyBulletBackendError(f"camera worker failed to start: {detail}")

    def submit(self, state) -> None:
        try:
            self._states.put_nowait(state)
        except Full:
            try:
                self._states.get_nowait()
            except Empty:
                return
            try:
                self._states.put_nowait(state)
            except Full:
                pass

    def check(self) -> None:
        try:
            kind, detail = self._status.get_nowait()
        except Empty:
            if self._process is not None and not self._process.is_alive():
                raise PyBulletBackendError("camera worker stopped unexpectedly")
            return
        if kind == "error":
            raise PyBulletBackendError(f"camera worker failed: {detail}")

    def close(self) -> None:
        if self._process is None:
            return
        try:
            self._states.put_nowait("shutdown")
        except Full:
            try:
                self._states.get_nowait()
            except Empty:
                pass
            try:
                self._states.put_nowait("shutdown")
            except Full:
                pass
        self._process.join(timeout=3.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._process = None
