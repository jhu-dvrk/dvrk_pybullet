import numpy as np
import PyKDL
import pytest

from dvrk_arm_description import JointConfig, RobotConfig
from dvrk_simulator_base.types import Pose

import dvrk_pybullet.runtime as runtime_module
from dvrk_pybullet.runtime import PyBulletRuntime, RuntimeOptions


def _psm1_config():
    definitions = (
        ("yaw", "revolute", -1.5707963, 1.5707963, 5.0),
        ("pitch", "revolute", -0.7853982, 0.7853982, 5.0),
        ("insertion", "prismatic", 0.0, 0.240, 0.4),
        ("roll", "revolute", -4.53786, 4.53786, 50.0),
        ("wrist_pitch", "revolute", -1.2217, 1.2217, 50.0),
        ("wrist_yaw", "revolute", -1.39626, 1.39626, 50.0),
    )
    return RobotConfig(
        name="PSM1",
        type="PSM",
        model="Virtual/PSM1.urdf.xacro",
        instrument="420006",
        endoscope=None,
        parent_frame="world",
        base_frame="PSM1_base",
        rcm_frame="PSM1_RCM",
        tool_frame="PSM1_tool",
        adaptor_frame="PSM1_adaptor_link",
        base_position=np.zeros(3),
        base_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        joints=tuple(JointConfig(*definition) for definition in definitions),
        home_position=np.array([0.0, 0.0, 0.12, 0.0, 0.0, 0.0]),
        raw={"robot": {"cartesian": {"reference_frame": "world"}}},
    )


def test_runtime_produces_coherent_pybullet_snapshot(tmp_path):
    pytest.importorskip("pybullet")
    runtime = PyBulletRuntime(
        _psm1_config(), RuntimeOptions(gui=False, generated_root=tmp_path)
    )
    try:
        initial = runtime.initialize()
        np.testing.assert_allclose(
            initial.measured_js.position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0]
        )
        assert isinstance(initial.measured_cp_world, PyKDL.Frame)
        assert initial.jaw_measured == pytest.approx(0.0)
        stepped = runtime.step()
        assert stepped.sequence == initial.sequence + 1
        assert stepped.simulation_time > initial.simulation_time

        target = np.array([0.1, 0.1, 0.14, 0.2, -0.1, 0.1])
        runtime.commands.submit_servo("servo_jp", target)
        runtime.commands.submit_servo("jaw/servo_jp", 0.7)
        commanded = runtime.step()
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

        offset = PyKDL.Vector(0.005, 0.0, 0.0)
        cartesian_target = PyKDL.Frame(
            commanded.measured_cp_world.M,
            commanded.measured_cp_world.p + offset,
        )
        runtime.commands.submit_servo("servo_cp", cartesian_target)
        cartesian = runtime.step()
        np.testing.assert_allclose(
            [cartesian.measured_cp_world.p[i] for i in range(3)],
            [cartesian_target.p[i] for i in range(3)],
            atol=2e-4,
        )
        np.testing.assert_allclose(
            [[cartesian.measured_cp_world.M[i, j] for j in range(3)] for i in range(3)],
            [[cartesian_target.M[i, j] for j in range(3)] for i in range(3)],
            atol=2e-3,
        )
    finally:
        runtime.shutdown()


def _runtime_without_connection(monkeypatch):
    monkeypatch.setattr(runtime_module, "load_pybullet", lambda: object())
    return PyBulletRuntime(_psm1_config(), RuntimeOptions())


def test_servo_and_move_commands_are_applied_kinematically(monkeypatch):
    runtime = _runtime_without_connection(monkeypatch)
    servo = runtime.commands.submit_servo(
        "servo_jp", np.array([0.1, 0.2, 0.12, 0.3, 0.4, 0.5])
    )
    runtime._update_commands(servo.received_at_ns, 10.0)
    np.testing.assert_allclose(
        runtime._joint_setpoint, [0.1, 0.2, 0.12, 0.3, 0.4, 0.5]
    )
    assert runtime._joint_trajectory is None

    move = runtime.commands.submit_discrete(
        "move_jp", np.array([0.6, 0.2, 0.12, 0.3, 0.4, 0.5])
    )
    assert move is not None
    runtime._update_commands(move.received_at_ns, 20.0)
    assert runtime._joint_trajectory is not None
    runtime._update_commands(move.received_at_ns, 20.05)
    assert 0.1 < runtime._joint_setpoint[0] < 0.6
    runtime._update_commands(move.received_at_ns, 20.2)
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
    runtime._update_commands(invalid.received_at_ns, 1.0)
    assert runtime.commands_rejected == 1
    assert runtime._move_failure_pending

    disable = runtime.commands.submit_discrete("state_command", "disable")
    assert disable is not None
    runtime._update_commands(disable.received_at_ns, 2.0)
    assert not runtime._operating_state.accepts_motion

    servo = runtime.commands.submit_servo(
        "servo_jp", np.array([0.1, 0.0, 0.12, 0.0, 0.0, 0.0])
    )
    runtime._update_commands(servo.received_at_ns, 3.0)
    np.testing.assert_allclose(runtime._joint_setpoint, _psm1_config().home_position)
    assert runtime.commands_rejected == 2


def test_jaw_move_uses_configured_velocity(monkeypatch):
    runtime = _runtime_without_connection(monkeypatch)
    move = runtime.commands.submit_discrete("jaw/move_jp", 0.4)
    assert move is not None
    runtime._update_commands(move.received_at_ns, 5.0)
    assert runtime._jaw_trajectory is not None
    runtime._update_commands(move.received_at_ns, 5.5)
    assert runtime._jaw_setpoint == pytest.approx(0.2)
    runtime._update_commands(move.received_at_ns, 6.0)
    assert runtime._jaw_setpoint == pytest.approx(0.4)
    assert runtime._jaw_trajectory is None
