from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from dvrk_pybullet.robot import load_robot, reset_joint_positions
from dvrk_pybullet.urdf_materializer import materialize_virtual_robot
import pytest


def test_virtual_psm1_loads_in_direct_mode(tmp_path):
    pybullet = pytest.importorskip("pybullet")
    connection = pybullet.connect(pybullet.DIRECT)
    assert connection >= 0
    try:
        artifact = materialize_virtual_robot("PSM1", instrument="420006", generated_root=tmp_path)
        names = ("yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw")
        robot = load_robot(pybullet, artifact.urdf_path, names)
        reset_joint_positions(pybullet, robot, (0.0, 0.0, 0.12, 0.0, 0.0, 0.0))
        measured = tuple(
            pybullet.getJointState(robot.body_id, index)[0]
            for index in robot.controlled_joint_indices
        )
        assert measured == pytest.approx((0.0, 0.0, 0.12, 0.0, 0.0, 0.0))
    finally:
        pybullet.disconnect(connection)


def test_cuhk_primitive_board_and_csg_triangular_block_load_in_direct_mode():
    pybullet = pytest.importorskip("pybullet")
    assets = (
        Path(get_package_share_directory("dvrk_simulator_base"))
        / "share"
        / "assets"
    )
    connection = pybullet.connect(pybullet.DIRECT)
    assert connection >= 0
    try:
        board = pybullet.loadURDF(
            str(assets / "peg_board_CUHK" / "peg_board_CUHK.urdf"),
            useFixedBase=True,
            physicsClientId=connection,
        )
        visual_types = [
            shape[2]
            for shape in pybullet.getVisualShapeData(
                board, physicsClientId=connection
            )
        ]
        assert visual_types.count(pybullet.GEOM_BOX) == 1
        assert visual_types.count(pybullet.GEOM_CYLINDER) == 12
        assert pybullet.GEOM_MESH not in visual_types

        pybullet.resetSimulation(physicsClientId=connection)
        triangular_block = pybullet.loadURDF(
            str(assets / "triangular_block" / "triangular_block.urdf"),
            physicsClientId=connection,
        )
        center, solid = pybullet.rayTestBatch(
            [[0.0, 0.0, -0.02], [0.008, 0.0, -0.02]],
            [[0.0, 0.0, 0.02], [0.008, 0.0, 0.02]],
            physicsClientId=connection,
        )
        assert center[0] == -1
        assert solid[0] == triangular_block
    finally:
        pybullet.disconnect(connection)
