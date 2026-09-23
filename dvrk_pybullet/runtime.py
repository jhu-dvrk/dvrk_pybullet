"""Single-thread-owned PyBullet runtime for one Virtual PSM."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np

from dvrk_arm_description import RobotConfig
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.operating_state import CRTKOperatingState
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.rotations import quaternion_matrix_xyzw
from dvrk_simulator_base.types import IKResult, JointState, Pose, Twist
from dvrk_simulator_base.trajectory import JointTrajectory

from .backend import load_pybullet
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
    gui: bool = False
    simulation_rate_hz: float = 120.0
    generated_root: Path | None = None


class PyBulletRuntime:
    """Own all calls into one PyBullet connection."""

    def __init__(
        self,
        config: RobotConfig,
        options: RuntimeOptions,
        commands: CommandMailboxes | None = None,
        pybullet_client=None,
    ) -> None:
        if options.simulation_rate_hz <= 0.0:
            raise ValueError("simulation_rate_hz must be positive")
        self.config = config
        self.options = options
        self.commands = commands or CommandMailboxes()
        self.pybullet = pybullet_client or load_pybullet()
        self.connection = -1
        self._owns_connection = False
        self.artifact: MaterializedUrdf | None = None
        self.robot: LoadedRobot | None = None
        self._tool_link_index: int | None = None
        self._jaw_joint_index: int | None = None
        self._joint_setpoint = np.array(config.home_position, dtype=float, copy=True)
        self._joint_velocity = np.zeros_like(self._joint_setpoint)
        self._jaw_setpoint = 0.0
        self._jaw_velocity = 0.0
        self._joint_trajectory: JointTrajectory | None = None
        self._jaw_trajectory: JointTrajectory | None = None
        self._move_failure_pending = False
        self._operating_state_event_pending = False
        self._operating_state = CRTKOperatingState(CRTKOperatingState.ENABLED)
        jaw = config.raw.get("robot", {}).get("jaw", {})
        self._jaw_lower = float(jaw.get("lower", -0.349066))
        self._jaw_upper = float(jaw.get("upper", 1.39626))
        self._jaw_speed = float(jaw.get("velocity", 0.4))
        if not self._jaw_lower <= self._jaw_upper:
            raise ValueError("jaw lower limit cannot exceed upper limit")
        if not np.isfinite(self._jaw_speed) or self._jaw_speed <= 0.0:
            raise ValueError("jaw velocity must be finite and positive")
        self.commands_applied = 0
        self.commands_rejected = 0
        self.commands_canceled = 0
        self._sequence = 0
        self._simulation_time = 0.0

    def initialize(self, connection: int | None = None) -> ArmSnapshot:
        if connection is None:
            mode = self.pybullet.GUI if self.options.gui else self.pybullet.DIRECT
            self.connection = self.pybullet.connect(mode)
            self._owns_connection = True
            if self.connection < 0:
                raise PyBulletBackendError("PyBullet could not create a connection")
            self.pybullet.setGravity(0.0, 0.0, -9.81)
        else:
            self.connection = int(connection)
            self._owns_connection = False
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

            if self.options.gui and self._owns_connection:
                self.pybullet.resetDebugVisualizerCamera(
                    cameraDistance=0.65,
                    cameraYaw=45.0,
                    cameraPitch=-25.0,
                    cameraTargetPosition=(0.0, 0.0, 0.12),
                )
            self.pybullet.stepSimulation()
            self._apply_setpoints()
            self._apply_jaw_setpoint()
            return self.snapshot()
        except BaseException:
            self.shutdown()
            raise

    def is_connected(self) -> bool:
        return self.connection >= 0 and bool(self.pybullet.isConnected(self.connection))

    def step(self) -> ArmSnapshot:
        if self.robot is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        now_ns = time.monotonic_ns()
        self.prepare_step(now_ns, now_ns * 1e-9)
        self.pybullet.stepSimulation()
        return self.finish_step()

    def prepare_step(self, now_ns: int, now: float) -> None:
        """Drain commands before the shared world advances."""
        self._update_commands(now_ns, now)

    def finish_step(self) -> ArmSnapshot:
        """Apply this arm's kinematic state after one shared world step."""
        # Initial fidelity is kinematic: apply the target directly after the
        # physics step so measured joints and FK describe the exact same state.
        self._apply_setpoints()
        self._apply_jaw_setpoint()
        self._simulation_time += 1.0 / self.options.simulation_rate_hz
        self._sequence += 1
        result = self.snapshot()
        self._operating_state_event_pending = False
        return result

    def _update_commands(self, now_ns: int, now: float) -> None:
        if self._move_failure_pending:
            self._move_failure_pending = False
        joint_move_started = False
        jaw_move_started = False
        for command in self.commands.drain():
            if command.channel == "state_command":
                success, _ = self._operating_state.command(command.payload)
                if not success:
                    self.commands_rejected += 1
                    continue
                self.commands_applied += 1
                self._operating_state_event_pending = True
                if not self._operating_state.accepts_motion:
                    self._cancel_motion()
                continue

            if not self._operating_state.accepts_motion:
                self.commands_rejected += 1
                if self._is_move_command(command.channel):
                    self._move_failure_pending = True
                continue

            if command.channel in {"servo_jp", "move_jp"}:
                target = np.asarray(command.payload, dtype=float)
                if not self._valid_joint_target(target):
                    self.commands_rejected += 1
                    if command.channel == "move_jp":
                        self._move_failure_pending = True
                    continue
                if command.channel == "servo_jp":
                    if self._joint_trajectory is not None:
                        self.commands_canceled += 1
                    self._joint_trajectory = None
                    self._joint_setpoint = target.copy()
                    self._joint_velocity = np.zeros_like(target)
                else:
                    if self._joint_trajectory is not None:
                        self.commands_canceled += 1
                    self._joint_trajectory = JointTrajectory(
                        self._joint_setpoint,
                        target,
                        [joint.velocity for joint in self.config.joints],
                        now,
                    )
                    joint_move_started = True
                self.commands_applied += 1
                continue

            if command.channel in {"servo_cp", "move_cp"}:
                result = self.compute_ik(command.payload, self._joint_setpoint)
                if not result.success or not self._valid_joint_target(result.position):
                    self.commands_rejected += 1
                    if command.channel == "move_cp":
                        self._move_failure_pending = True
                    continue
                if command.channel == "servo_cp":
                    if self._joint_trajectory is not None:
                        self.commands_canceled += 1
                    self._joint_trajectory = None
                    self._joint_setpoint = result.position.copy()
                    self._joint_velocity = np.zeros_like(self._joint_setpoint)
                else:
                    if self._joint_trajectory is not None:
                        self.commands_canceled += 1
                    self._joint_trajectory = JointTrajectory(
                        self._joint_setpoint,
                        result.position,
                        [joint.velocity for joint in self.config.joints],
                        now,
                    )
                    joint_move_started = True
                self.commands_applied += 1
                continue

            if command.channel in {"jaw/servo_jp", "jaw/move_jp"}:
                target = float(command.payload)
                if not np.isfinite(target) or not self._jaw_lower <= target <= self._jaw_upper:
                    self.commands_rejected += 1
                    if command.channel == "jaw/move_jp":
                        self._move_failure_pending = True
                    continue
                if command.channel == "jaw/servo_jp":
                    if self._jaw_trajectory is not None:
                        self.commands_canceled += 1
                    self._jaw_trajectory = None
                    self._jaw_setpoint = target
                    self._jaw_velocity = 0.0
                else:
                    if self._jaw_trajectory is not None:
                        self.commands_canceled += 1
                    self._jaw_trajectory = JointTrajectory(
                        [self._jaw_setpoint], [target], [self._jaw_speed], now
                    )
                    jaw_move_started = True
                self.commands_applied += 1
                continue

            self.commands_rejected += 1

        if self._joint_trajectory is not None and not joint_move_started:
            sample = self._joint_trajectory.sample(now)
            self._joint_setpoint = sample.position.copy()
            self._joint_velocity = sample.velocity.copy()
            if sample.complete:
                self._joint_trajectory = None
        if self._jaw_trajectory is not None and not jaw_move_started:
            sample = self._jaw_trajectory.sample(now)
            self._jaw_setpoint = float(sample.position[0])
            self._jaw_velocity = float(sample.velocity[0])
            if sample.complete:
                self._jaw_trajectory = None

    def _valid_joint_target(self, target: np.ndarray) -> bool:
        if target.shape != self._joint_setpoint.shape or not np.all(np.isfinite(target)):
            return False
        return all(
            joint.lower <= value <= joint.upper
            for value, joint in zip(target, self.config.joints)
        )

    @staticmethod
    def _is_move_command(channel: str) -> bool:
        return channel in {"move_jp", "move_cp", "jaw/move_jp"}

    def _cancel_motion(self) -> None:
        self.commands_canceled += int(self._joint_trajectory is not None)
        self.commands_canceled += int(self._jaw_trajectory is not None)
        self._joint_trajectory = None
        self._jaw_trajectory = None
        self._joint_velocity.fill(0.0)
        self._jaw_velocity = 0.0

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
            self._joint_setpoint if seed is None else seed, dtype=float
        ).copy()
        if not self._valid_joint_target(q):
            raise ValueError("IK seed is invalid or outside configured limits")
        lower = np.asarray([joint.lower for joint in self.config.joints])
        upper = np.asarray([joint.upper for joint in self.config.joints])
        difference_step = np.asarray(
            [1e-5 if joint.type == "prismatic" else 1e-4 for joint in self.config.joints]
        )
        maximum_step = np.asarray(
            [0.01 if joint.type == "prismatic" else 0.15 for joint in self.config.joints]
        )
        restore = self._joint_setpoint.copy()
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
            self._joint_setpoint,
            self._joint_velocity,
        )

    def _apply_jaw_setpoint(self) -> None:
        if self.robot is None:
            raise RuntimeError("PyBullet runtime is not initialized")
        if "jaw" in self.robot.joint_indices:
            reset_joint_with_mimics(
                self.pybullet,
                self.robot,
                "jaw",
                self._jaw_setpoint,
                self._jaw_velocity,
            )

    def reset_to_home(self) -> None:
        """Restore this kinematic arm to its initial commanded state."""
        self._joint_setpoint = np.array(self.config.home_position, dtype=float, copy=True)
        self._joint_velocity = np.zeros_like(self._joint_setpoint)
        self._jaw_setpoint = 0.0
        self._jaw_velocity = 0.0
        self._joint_trajectory = None
        self._jaw_trajectory = None
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
            names, self._joint_setpoint, self._joint_velocity
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
            self._operating_state.state,
            is_homed=self._operating_state.is_homed,
            is_busy=(
                self._joint_trajectory is not None
                or self._jaw_trajectory is not None
                or self._move_failure_pending
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
            jaw_setpoint=self._jaw_setpoint,
            operating_state=state,
            operating_state_event=self._operating_state_event_pending,
        )

    def run(self, publish_snapshot, should_continue=None) -> None:
        period = 1.0 / self.options.simulation_rate_hz
        deadline = time.monotonic()
        while self.is_connected() and (
            should_continue is None or should_continue()
        ):
            snapshot = self.step()
            publish_snapshot(snapshot)
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                deadline = time.monotonic()

    def shutdown(self) -> None:
        if (
            self._owns_connection
            and self.connection >= 0
            and self.pybullet.isConnected(self.connection)
        ):
            self.pybullet.disconnect(self.connection)
        self.connection = -1
