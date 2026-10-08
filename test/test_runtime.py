import numpy as np
import pytest

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from dvrk_arm_description import load_robot_config
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.cartesian_command import CartesianCommand
from dvrk_simulator_base.types import Pose

import dvrk_pybullet.runtime as runtime_module
from dvrk_pybullet.runtime import PyBulletArm, RuntimeOptions
from dvrk_pybullet.world_runtime import PyBulletWorldRuntime


def _psm1_config():
    return load_robot_config(Path(get_package_share_directory("dvrk_arm_description")) / "arms/PSM1.yaml", instrument="420006")


def test_runtime_produces_coherent_pybullet_snapshot(tmp_path):
    pytest.importorskip("pybullet")
    commands = CommandMailboxes()
    world = PyBulletWorldRuntime((_psm1_config(),), RuntimeOptions(gui=False, generated_root=tmp_path), {"PSM1": commands})
    runtime = world.arms["PSM1"]
    try:
        initial = world.initialize()["PSM1"]
        np.testing.assert_allclose(
            initial.measured_js.position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0]
        )
        assert isinstance(initial.measured_cp_world, Pose)
        assert initial.jaw_measured == pytest.approx(0.0)
        stepped = world.step()["PSM1"]
        assert stepped.sequence == initial.sequence + 1
        assert stepped.simulation_time > initial.simulation_time

        target = np.array([0.1, 0.1, 0.14, 0.2, -0.1, 0.1])
        runtime.commands.submit_servo("servo_jp", target)
        runtime.commands.submit_servo("jaw/servo_jp", 0.7)
        commanded = world.step()["PSM1"]
        np.testing.assert_allclose(commanded.measured_js.position, target)
        np.testing.assert_allclose(commanded.setpoint_js.position, target)
        assert commanded.jaw_measured == pytest.approx(0.7)
        assert commanded.jaw_setpoint == pytest.approx(0.7)

        for mimic in runtime.robot.mimic_joints:
            if mimic.source_joint_name != "jaw":
                continue
            measured = runtime.pybullet.getJointState(
                runtime.robot.body_id,
                runtime.robot.joint_indices[mimic.joint_name],
            )[0]
            assert measured == pytest.approx(0.7 * mimic.multiplier + mimic.offset)

        cartesian_target = Pose(commanded.measured_cp_world.position + [0.005, 0, 0], commanded.measured_cp_world.orientation)
        runtime.commands.submit_servo("servo_cp", CartesianCommand(cartesian_target, "world"))
        cartesian = world.step()["PSM1"]
        np.testing.assert_allclose(cartesian.measured_cp_world.position, cartesian_target.position, atol=2e-4)
        np.testing.assert_allclose(cartesian.measured_cp_world.orientation, cartesian_target.orientation, atol=2e-3)
    finally:
        world.shutdown()


def _runtime_without_connection(monkeypatch):
    return PyBulletArm(_psm1_config(), RuntimeOptions(), CommandMailboxes(), object())


def test_servo_and_move_commands_are_applied_kinematically(monkeypatch):
    runtime = _runtime_without_connection(monkeypatch)
    servo = runtime.commands.submit_servo(
        "servo_jp", np.array([0.1, 0.2, 0.12, 0.3, 0.4, 0.5])
    )
    runtime._update_commands(10.0)
    np.testing.assert_allclose(
        runtime._joint_setpoint, [0.1, 0.2, 0.12, 0.3, 0.4, 0.5]
    )
    assert runtime._joint_trajectory is None

    move = runtime.commands.submit_discrete(
        "move_jp", np.array([0.6, 0.2, 0.12, 0.3, 0.4, 0.5])
    )
    assert move is not None
    runtime._update_commands(20.0)
    assert runtime._joint_trajectory is not None
    runtime._update_commands(20.05)
    assert 0.1 < runtime._joint_setpoint[0] < 0.6
    runtime._update_commands(20.2)
    np.testing.assert_allclose(
        runtime._joint_setpoint, [0.6, 0.2, 0.12, 0.3, 0.4, 0.5]
    )
    assert runtime._joint_trajectory is None


def test_limits_and_operating_state_gate_motion(monkeypatch):
    runtime = _runtime_without_connection(monkeypatch)
    invalid = runtime.commands.submit_discrete(
        "move_jp", np.array([9.0, 0.0, 0.12, 0.0, 0.0, 0.0])
    )
    assert invalid is not None
    runtime._update_commands(1.0)
    assert runtime._move_failure_pending

    disable = runtime.commands.submit_discrete("state_command", "disable")
    assert disable is not None
    runtime._update_commands(2.0)
    assert not runtime._operating_state.accepts_motion

    servo = runtime.commands.submit_servo(
        "servo_jp", np.array([0.1, 0.0, 0.12, 0.0, 0.0, 0.0])
    )
    runtime._update_commands(3.0)
    np.testing.assert_allclose(runtime._joint_setpoint, _psm1_config().home_position)


def test_jaw_move_uses_configured_velocity(monkeypatch):
    runtime = _runtime_without_connection(monkeypatch)
    move = runtime.commands.submit_discrete("jaw/move_jp", 0.4)
    assert move is not None
    runtime._update_commands(5.0)
    assert runtime._jaw_trajectory is not None
    runtime._update_commands(5.5)
    assert runtime._jaw_setpoint == pytest.approx(0.2)
    runtime._update_commands(6.0)
    assert runtime._jaw_setpoint == pytest.approx(0.4)
    assert runtime._jaw_trajectory is None
