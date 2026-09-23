from pathlib import Path
import pytest

from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_arm_description import load_robot_config

from dvrk_pybullet.runtime import RuntimeOptions
from dvrk_pybullet.world_runtime import PyBulletWorldRuntime


def test_world_owns_one_connection_and_multiple_kinematic_arms(tmp_path):
    pytest.importorskip("pybullet")
    arm_root = Path(__file__).parents[2] / "dvrk_arm_description" / "arms"
    configs = (
        load_robot_config(
            arm_root / "PSM1.yaml", instrument="420006", base_position=[-0.1, 0, 0.17]
        ),
        load_robot_config(
            arm_root / "PSM2.yaml", instrument="420006", base_position=[0.1, 0, 0.17]
        ),
        load_robot_config(
            arm_root / "ECM.yaml", endoscope="Si_straight", base_position=[0, 0, 0.2]
        ),
    )
    commands = {config.name: CommandMailboxes() for config in configs}
    world = PyBulletWorldRuntime(
        configs,
        RuntimeOptions(generated_root=tmp_path),
        commands,
    )
    try:
        initial = world.initialize()
        assert set(initial) == {"PSM1", "PSM2", "ECM"}
        assert len({arm.robot.body_id for arm in world.arms.values()}) == 3
        assert all(arm.connection == world.connection for arm in world.arms.values())
        assert initial["ECM"].jaw_measured is None
        stepped = world.step()
        assert all(snapshot.sequence == 1 for snapshot in stepped.values())
    finally:
        world.shutdown()
