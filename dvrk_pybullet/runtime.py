"""Per-arm kinematics and command execution within a shared PyBullet world."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from dvrk_arm_description import RobotConfig
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.cartesian_command import resolve_cartesian_command
from dvrk_simulator_base.arm_controller import ArmController
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.rotations import quaternion_matrix_xyzw
from dvrk_simulator_base.types import IKResult, JointState, Pose, Twist

from .errors import PyBulletBackendError
from .robot import (
    LoadedRobot,
    load_robot,
    reset_joint_positions,
    reset_joint_with_mimics,
)
from .urdf_materializer import MaterializedUrdf, materialize_virtual_robot


@dataclass(frozen=True)
class RuntimeOptions:
    headless: bool = False
    simulation_rate_hz: float = 120.0
    generated_root: Path | None = None


class PyBulletArm(ArmController):
    """An arm whose connection and step lifecycle are owned by its world."""

    def __init__(
        self,
        config: RobotConfig,
        options: RuntimeOptions,
        commands: CommandMailboxes,
        pybullet_client,
    ) -> None:
        if options.simulation_rate_hz <= 0.0:
            raise ValueError("simulation_rate_hz must be positive")
        self.config = config
        self.options = options
        self.commands = commands
        self.pybullet = pybullet_client
        self.connection = -1
        self.artifact: MaterializedUrdf | None = None
        self.robot: LoadedRobot | None = None
        self._tool_link_index: int | None = None
        self._jaw_joint_index: int | None = None
        super().__init__(config, commands)
        self._sequence = 0
        self._simulation_time = 0.0

    def initialize(self, connection: int) -> ArmSnapshot:
        self.connection = connection
        try:
            self.artifact = materialize_virtual_robot(
                self.config.name,
                instrument=self.config.instrument,
                endoscope=self.config.endoscope,
                generated_root=self.options.generated_root,
            )
            self.robot = load_robot(
                self.pybullet,
                self.artifact.urdf_path,
                (joint.name for joint in self.config.joints),
                base_position=self.config.base_position,
                base_orientation_xyzw=self.config.base_orientation_xyzw,
            )
            self._apply_setpoints()
            self._jaw_joint_index = self.robot.joint_indices.get("jaw")
            self._apply_jaw_setpoint()

            tool_names = (
                f"{self.config.name}_tool_tip_link",
                self.config.tool_frame,
                f"{self.config.tool_frame}_link",
                f"{self.config.name}_tip_link",
                f"{self.config.name}_endoscope_frame_link",
            )
            for name in tool_names:
                if name in self.robot.link_indices:
                    self._tool_link_index = self.robot.link_indices[name]
                    break
            if self._tool_link_index is None:
                raise PyBulletBackendError(
                    "materialized robot is missing a configured tool link; tried "
                    + ", ".join(tool_names)
                )

            return self.snapshot()
        except BaseException:
            self.shutdown()
            raise

    def prepare_step(self, now: float, ecm_pose=None, *, has_ecm=False) -> None:
        self._update_commands(now, ecm_pose, has_ecm=has_ecm)

    def finish_step(self) -> ArmSnapshot:
        """Apply this arm's kinematic state after one shared world step."""
        # Initial fidelity is kinematic: apply the target directly after the
        # physics step so measured joints and FK describe the exact same state.
        self._apply_setpoints()
        self._apply_jaw_setpoint()
        self._simulation_time += 1.0 / self.options.simulation_rate_hz
        self._sequence += 1
        result = self.snapshot()
        self.operating_state_event_pending = False
        return result

    def _update_commands(self, now: float, ecm_pose=None, *, has_ecm=False) -> None:
        self.advance_commands(
            now, self.compute_ik,
            lambda target: resolve_cartesian_command(target, self.config, ecm_pose, has_ecm=has_ecm),
        )

    def _fk_for_joint_position(self, position: np.ndarray) -> Pose:
        if self.robot is None or self._tool_link_index is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        reset_joint_positions(self.pybullet, self.robot, position)
        link_state = self.pybullet.getLinkState(
            self.robot.body_id,
            self._tool_link_index,
            computeForwardKinematics=True,
        )
        return Pose(np.asarray(link_state[4], dtype=float), quaternion_matrix_xyzw(link_state[5]))

    @staticmethod
    def _pose_error(
        current: Pose, target: Pose, use_orientation: bool = True
    ) -> tuple[np.ndarray, float, float]:
        position_error = np.array(
            target.position - current.position, dtype=float
        )
        c_rot = np.array(
            current.orientation, dtype=float
        )
        t_rot = np.array(
            target.orientation, dtype=float
        )
        orientation_error = 0.5 * (
            np.cross(c_rot[:, 0], t_rot[:, 0])
            + np.cross(c_rot[:, 1], t_rot[:, 1])
            + np.cross(c_rot[:, 2], t_rot[:, 2])
        )
        return (
            np.concatenate((position_error, orientation_error))
            if use_orientation
            else position_error,
            float(np.linalg.norm(position_error)),
            float(np.linalg.norm(orientation_error)),
        )

    def compute_ik(
        self,
        target: Pose,
        seed=None,
        max_iterations: int = 50,
    ) -> IKResult:
        """Solve six-axis PSM IK using PyBullet FK and constrained DLS."""
        if self.robot is None or self._tool_link_index is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        q = np.asarray(
            self.joint_setpoint if seed is None else seed, dtype=float
        ).copy()
        if not self.valid_joint_target(q):
            raise ValueError("IK seed is invalid or outside configured limits")
        lower = np.asarray([joint.lower for joint in self.config.joints])
        upper = np.asarray([joint.upper for joint in self.config.joints])
        difference_step = np.asarray(
            [1e-5 if joint.type == "prismatic" else 1e-4 for joint in self.config.joints]
        )
        maximum_step = np.asarray(
            [0.01 if joint.type == "prismatic" else 0.15 for joint in self.config.joints]
        )
        restore = self.joint_setpoint.copy()
        use_orientation = len(self.config.joints) >= 6
        position_error = float("inf")
        orientation_error = float("inf")
        try:
            for iteration in range(max_iterations + 1):
                pose = self._fk_for_joint_position(q)
                error, position_error, orientation_error = self._pose_error(
                    pose, target, use_orientation
                )
                if position_error <= 2e-4 and (
                    not use_orientation or orientation_error <= 2e-3
                ):
                    return IKResult(
                        q,
                        True,
                        iteration,
                        position_error,
                        orientation_error,
                        "pose converged",
                    )
                if iteration == max_iterations:
                    break

                jacobian = np.empty((len(error), len(q)), dtype=float)
                for index, epsilon in enumerate(difference_step):
                    perturbed = q.copy()
                    direction = 1.0 if q[index] + epsilon <= upper[index] else -1.0
                    perturbed[index] += direction * epsilon
                    perturbed_pose = self._fk_for_joint_position(perturbed)
                    perturbed_error, _, _ = self._pose_error(
                        perturbed_pose, target, use_orientation
                    )
                    jacobian[:, index] = (
                        perturbed_error - error
                    ) / (direction * epsilon)

                damping = 1e-6 * np.eye(jacobian.shape[0])
                delta = -jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + damping, error
                )
                delta = np.clip(delta, -maximum_step, maximum_step)
                candidate = np.clip(q + delta, lower, upper)
                if np.allclose(candidate, q, atol=1e-10, rtol=0.0):
                    break
                q = candidate
        finally:
            reset_joint_positions(self.pybullet, self.robot, restore)
        return IKResult(
            q,
            False,
            max_iterations,
            position_error,
            orientation_error,
            "pose IK did not converge",
        )

    def _apply_setpoints(self) -> None:
        if self.robot is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        reset_joint_positions(
            self.pybullet,
            self.robot,
            self.joint_setpoint,
            self.joint_velocity,
        )

    def _apply_jaw_setpoint(self) -> None:
        if self.robot is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        if "jaw" in self.robot.joint_indices:
            reset_joint_with_mimics(
                self.pybullet,
                self.robot,
                "jaw",
                self.jaw_setpoint,
                self.jaw_velocity,
            )

    def reset_to_home(self) -> None:
        """Restore this kinematic arm to its initial commanded state."""
        self.reset_motion()
        self._apply_setpoints()
        self._apply_jaw_setpoint()

    @property
    def tool_link_index(self) -> int:
        """PyBullet link used as the fixed frame for grasp attachments."""
        if self._tool_link_index is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        return self._tool_link_index

    def snapshot(self) -> ArmSnapshot:
        if self.robot is None or self._tool_link_index is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        states = self.pybullet.getJointStates(
            self.robot.body_id, self.robot.controlled_joint_indices
        )
        names = tuple(joint.name for joint in self.config.joints)
        positions = np.asarray([state[0] for state in states], dtype=float)
        velocities = np.asarray([state[1] for state in states], dtype=float)
        measured_js = JointState(names, positions, velocities)
        setpoint_js = JointState(
            names, self.joint_setpoint, self.joint_velocity
        )

        link_state = self.pybullet.getLinkState(
            self.robot.body_id,
            self._tool_link_index,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
        )
        pose = Pose(np.asarray(link_state[4], dtype=float), quaternion_matrix_xyzw(link_state[5]))
        twist = Twist(np.asarray(link_state[6], dtype=float), np.asarray(link_state[7], dtype=float))
        jaw = (
            float(self.pybullet.getJointState(self.robot.body_id, self._jaw_joint_index)[0])
            if self._jaw_joint_index is not None else None
        )
        state = OperatingStateSnapshot(
            self.operating_state.state,
            is_homed=self.operating_state.is_homed,
            is_busy=(
                self.joint_trajectory is not None
                or self.jaw_trajectory is not None
                or self.move_failure_pending
            ),
        )
        return ArmSnapshot(
            sequence=self._sequence,
            simulation_time=self._simulation_time,
            valid=True,
            measured_js=measured_js,
            setpoint_js=setpoint_js,
            measured_cp_world=pose,
            setpoint_cp_world=pose,
            measured_cv_world=twist,
            jaw_measured=jaw,
            jaw_setpoint=self.jaw_setpoint,
            operating_state=state,
            operating_state_event=self.operating_state_event_pending,
        )

    def shutdown(self) -> None:
        self.connection = -1
