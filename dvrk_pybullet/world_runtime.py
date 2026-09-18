"""One PyBullet connection shared by every arm in a configured scene."""

from __future__ import annotations

import time
from typing import Mapping

import numpy as np

from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.config import RobotConfig
from dvrk_simulator_base.snapshots import ArmSnapshot
from dvrk_simulator_base.scene import SceneObject

from .backend import load_pybullet
from .camera_worker import CameraWorker
from .collision_debug import CollisionShapeOverlay
from .errors import PyBulletBackendError
from .grasp import GraspManager
from .runtime import PyBulletRuntime, RuntimeOptions
from .scene_objects import load_scene_objects, resolve_asset_uri


class PyBulletWorldRuntime:
    def __init__(
        self,
        configs: tuple[RobotConfig, ...],
        options: RuntimeOptions,
        commands: Mapping[str, CommandMailboxes],
        camera_options=None,
        scene_objects: tuple[SceneObject, ...] = (),
        grasp_config=None,
    ) -> None:
        if not configs:
            raise ValueError("a PyBullet world requires at least one robot")
        self.options = options
        self.pybullet = load_pybullet()
        self.connection = -1
        self.camera_options = camera_options
        self.grasp_config = grasp_config
        self.scene_object_specs = scene_objects
        self.scene_objects = {}
        self.grasp_manager = None
        self.collision_debug = None
        self._reset_requested = False
        self._initial_object_poses = {}
        self.camera_worker = None
        self.arms = {
            config.name: PyBulletRuntime(
                config,
                options,
                commands=commands[config.name],
                pybullet_client=self.pybullet,
            )
            for config in configs
        }

    def initialize(self) -> dict[str, ArmSnapshot]:
        mode = self.pybullet.GUI if self.options.gui else self.pybullet.DIRECT
        self.connection = self.pybullet.connect(mode)
        if self.connection < 0:
            raise PyBulletBackendError("PyBullet could not create a world connection")
        try:
            self.pybullet.setGravity(0.0, 0.0, -9.81)
            self.scene_objects = load_scene_objects(
                self.pybullet, self.scene_object_specs, connection=self.connection
            )
            self._initial_object_poses = {
                name: self.pybullet.getBasePositionAndOrientation(
                    item.body_id, physicsClientId=self.connection
                ) for name, item in self.scene_objects.items()
            }
            snapshots = {
                name: arm.initialize(self.connection)
                for name, arm in self.arms.items()
            }
            self.grasp_manager = GraspManager(
                self.pybullet, self.connection, self.arms, self.scene_objects,
                show_markers=self.grasp_config.show_grasps if self.grasp_config else True,
                max_grasps_per_object=(self.grasp_config.max_grasps_per_object if self.grasp_config else 1),
                default_policy=self.grasp_config.policy if self.grasp_config else "pose_error",
                arm_policies=self.grasp_config.arm_policies if self.grasp_config else {},
                close_threshold=self.grasp_config.close_threshold_rad if self.grasp_config else 0.04,
                release_threshold=self.grasp_config.release_threshold_rad if self.grasp_config else 0.08,
                break_distance=self.grasp_config.break_distance_m if self.grasp_config else 0.005,
                break_orientation=(self.grasp_config.break_orientation_rad if self.grasp_config else 0.2617993877991494),
                break_tension_force=(self.grasp_config.break_tension_force_n if self.grasp_config else 10.0),
                break_shear_force=(self.grasp_config.break_shear_force_n if self.grasp_config else 10.0),
                break_torque=self.grasp_config.break_torque_nm if self.grasp_config else 0.25,
                break_load_duration=(self.grasp_config.break_load_duration_s if self.grasp_config else 0.05),
                max_force=self.grasp_config.max_force_n if self.grasp_config else 100.0,
                constraint_erp=self.grasp_config.constraint_erp if self.grasp_config else 0.8,
                contact_region_offset=(self.grasp_config.contact_region_offset_m if self.grasp_config else (0.0, 0.0, -0.003)),
                contact_region_radius=(self.grasp_config.contact_region_radius_m if self.grasp_config else 0.008),
            )
            self._start_camera_worker(snapshots)
            if self.options.gui:
                self.pybullet.resetDebugVisualizerCamera(
                    cameraDistance=0.8,
                    cameraYaw=45.0,
                    cameraPitch=-25.0,
                    cameraTargetPosition=(0.0, 0.0, 0.1),
                )
            return snapshots
        except BaseException:
            self.shutdown()
            raise

    def _start_camera_worker(self, snapshots: Mapping[str, ArmSnapshot]) -> None:
        if (
            self.camera_options is None
            or not self.camera_options.enabled
            or "ECM" not in self.arms
        ):
            return
        self.camera_worker = CameraWorker(
            self.arms, self.camera_options, self.scene_object_specs
        )
        self.camera_worker.start(self._camera_state(snapshots))

    def _camera_state(
        self, snapshots: Mapping[str, ArmSnapshot]
    ) -> tuple[np.ndarray, ...]:
        joint_positions = []
        for arm in self.arms.values():
            count = self.pybullet.getNumJoints(arm.robot.body_id)
            states = self.pybullet.getJointStates(
                arm.robot.body_id, tuple(range(count)),
                physicsClientId=self.connection,
            )
            joint_positions.append(
                np.asarray([state[0] for state in states], dtype=float)
            )
        camera_pose = snapshots["ECM"].measured_cp_world
        object_poses = tuple(
            self.pybullet.getBasePositionAndOrientation(
                item.body_id, physicsClientId=self.connection
            )
            for item in self.scene_objects.values()
        )
        marker_poses = (
            {} if self.grasp_manager is None else self.grasp_manager.marker_poses()
        )
        return (
            *joint_positions,
            np.asarray(camera_pose.position, dtype=float),
            np.asarray(camera_pose.orientation, dtype=float).reshape(9),
            object_poses,
            marker_poses,
        )

    def step(self) -> dict[str, ArmSnapshot]:
        if self._reset_requested:
            self._reset_scene()
            self._reset_requested = False
        now_ns = time.monotonic_ns()
        now = now_ns * 1e-9
        for arm in self.arms.values():
            arm.prepare_step(now_ns, now)
        self.pybullet.stepSimulation()
        snapshots = {name: arm.finish_step() for name, arm in self.arms.items()}
        if self.grasp_manager is not None:
            self.grasp_manager.step(snapshots)
        if self.collision_debug is not None:
            self.collision_debug.update()
        if self.camera_worker is not None:
            self.camera_worker.submit(self._camera_state(snapshots))
            self.camera_worker.check()
        return snapshots

    def is_connected(self) -> bool:
        return self.connection >= 0 and bool(self.pybullet.isConnected(self.connection))

    def request_reset(self) -> None:
        self._reset_requested = True

    def set_collision_debug(self, enabled: bool) -> None:
        if enabled and self.collision_debug is None:
            self.collision_debug = CollisionShapeOverlay(self.pybullet, self.connection)
            for arm in self.arms.values():
                self.collision_debug.add_urdf(arm.robot.body_id, arm.artifact.urdf_path)
            for item in self.scene_objects.values():
                self.collision_debug.add_urdf(
                    item.body_id, resolve_asset_uri(item.spec.asset)
                )
        elif not enabled and self.collision_debug is not None:
            self.collision_debug.clear()
            self.collision_debug = None

    def _reset_scene(self) -> None:
        if self.grasp_manager is not None:
            self.grasp_manager.release_all()
        for arm in self.arms.values():
            arm.reset_to_home()
        for name, item in self.scene_objects.items():
            position, orientation = self._initial_object_poses[name]
            self.pybullet.resetBasePositionAndOrientation(
                item.body_id, position, orientation, physicsClientId=self.connection
            )
            self.pybullet.resetBaseVelocity(
                item.body_id, (0, 0, 0), (0, 0, 0), physicsClientId=self.connection
            )

    def run(self, publish_snapshots, should_continue=None) -> None:
        period = 1.0 / self.options.simulation_rate_hz
        deadline = time.monotonic()
        while self.is_connected() and (
            should_continue is None or should_continue()
        ):
            publish_snapshots(self.step())
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                deadline = time.monotonic()

    def shutdown(self) -> None:
        if self.camera_worker is not None:
            self.camera_worker.close()
        self.camera_worker = None
        if self.collision_debug is not None:
            self.collision_debug.clear()
        self.collision_debug = None
        if self.grasp_manager is not None:
            self.grasp_manager.release_all()
        self.grasp_manager = None
        for arm in self.arms.values():
            arm.shutdown()
        if self.connection >= 0 and self.pybullet.isConnected(self.connection):
            self.pybullet.disconnect(self.connection)
        self.connection = -1
