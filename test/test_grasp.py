from types import SimpleNamespace
from pathlib import Path

import pytest

from dvrk_pybullet.grasp import GraspManager


def test_default_grasp_region_matches_420006_jaws_without_moving_tool_frame(tmp_path):
    bullet = pytest.importorskip("pybullet")
    from dvrk_arm_description import load_robot_config
    from dvrk_pybullet.robot import load_robot, reset_joint_positions
    from dvrk_pybullet.urdf_materializer import materialize_virtual_robot

    connection = bullet.connect(bullet.DIRECT)
    try:
        config = load_robot_config(
            Path(__file__).parents[2] / "dvrk_arm_description/arms/PSM1.yaml",
            instrument="420006",
        )
        artifact = materialize_virtual_robot("PSM1", instrument="420006", generated_root=tmp_path)
        robot = load_robot(bullet, artifact.urdf_path, [joint.name for joint in config.joints])
        reset_joint_positions(bullet, robot, config.home_position)
        tool_index = robot.link_indices["PSM1_tool_tip_link"]
        arm = SimpleNamespace(config=config, robot=robot, tool_link_index=tool_index)
        tool_pose = bullet.getLinkState(robot.body_id, tool_index, computeForwardKinematics=True)[4:6]
        # A 3 mm cube held near the distal ends of the closed needle-driver jaws.
        object_pose = bullet.multiplyTransforms(*tool_pose, (0.0, 0.0, 0.007), (0, 0, 0, 1))
        shape = bullet.createCollisionShape(bullet.GEOM_BOX, halfExtents=(0.0015,) * 3)
        object_id = bullet.createMultiBody(
            baseMass=0.01, baseCollisionShapeIndex=shape,
            basePosition=object_pose[0], baseOrientation=object_pose[1],
        )
        objects = {"cube": SimpleNamespace(body_id=object_id, spec=SimpleNamespace(fixed=False))}
        bullet.performCollisionDetection()
        manager = GraspManager(bullet, connection, {"PSM1": arm}, objects, show_markers=False)
        jaws = manager._jaw_links(arm)
        assert all(bullet.getContactPoints(
            bodyA=robot.body_id, bodyB=object_id, linkIndexA=jaw
        ) for jaw in jaws)
        old = GraspManager(
            bullet, connection, {"PSM1": arm}, objects, show_markers=False,
            contact_region_offset=(0.0, 0.0, -0.003),
        )
        assert not old._contact_qualified(arm, object_id, jaws)
        manager.step({"PSM1": SimpleNamespace(jaw_measured=0.0)})
        assert manager.attachments["PSM1"].object_name == "cube"
        assert bullet.getLinkState(robot.body_id, tool_index, computeForwardKinematics=True)[4:6] == tool_pose
        manager.step({"PSM1": SimpleNamespace(jaw_measured=0.2)})
        assert not manager.attachments
    finally:
        bullet.disconnect(connection)


class FakePyBullet:
    JOINT_FIXED = 4
    GEOM_BOX = 3

    def __init__(self):
        self.enabled = []
        self.removed = []
        self.constraint_options = None
        self.marker_shapes = []
        self.marker_bodies = []
        self.marker_poses = []
        self.object_position = (0, 0, 0)

    def getContactPoints(self, **_kwargs):
        return (("contact",),)

    def getLinkState(self, *_args, **_kwargs):
        return (None, None, None, None, (0, 0, 0), (0, 0, 0, 1))

    def getBasePositionAndOrientation(self, *_args, **_kwargs):
        return self.object_position, (0, 0, 0, 1)

    def invertTransform(self, *_args):
        return (0, 0, 0), (0, 0, 0, 1)

    def multiplyTransforms(self, *_args):
        return (0, 0, 0), (0, 0, 0, 1)

    def createConstraint(self, **_kwargs):
        return 7

    def changeConstraint(self, *_args, **kwargs):
        self.constraint_options = kwargs

    def setCollisionFilterPair(self, *_args, **_kwargs):
        self.enabled.append(_args[4])

    def removeConstraint(self, constraint, **_kwargs):
        self.removed.append(constraint)

    def createVisualShape(self, shape_type, **kwargs):
        self.marker_shapes.append((shape_type, kwargs))
        return 8

    def createMultiBody(self, **kwargs):
        self.marker_bodies.append(kwargs)
        return 9

    def resetBasePositionAndOrientation(self, body, position, orientation, **_kwargs):
        self.marker_poses.append((body, position, orientation))

    def removeBody(self, body, **_kwargs):
        self.removed.append(body)


def test_two_jaw_contact_attaches_and_opening_releases_dynamic_object():
    bullet = FakePyBullet()
    arm = SimpleNamespace(
        config=SimpleNamespace(name="PSM1"),
        robot=SimpleNamespace(
            body_id=1,
            link_indices={"PSM1_jaw_1_link": 3, "PSM1_jaw_2_link": 4},
        ),
        tool_link_index=5,
    )
    objects = {"cube": SimpleNamespace(body_id=2, spec=SimpleNamespace(fixed=False))}
    manager = GraspManager(bullet, 0, {"PSM1": arm}, objects)
    manager.step({"PSM1": SimpleNamespace(jaw_measured=0.0)})
    assert manager.attachments["PSM1"].object_name == "cube"
    assert bullet.constraint_options == {"maxForce": 100.0, "erp": 0.8, "physicsClientId": 0}
    assert bullet.marker_shapes == [(3, {"halfExtents": (0.002, 0.002, 0.002), "rgbaColor": (1.0, 0.0, 0.0, 1.0), "physicsClientId": 0})]
    assert bullet.marker_bodies[0]["baseCollisionShapeIndex"] == -1
    assert bullet.marker_bodies[0]["baseVisualShapeIndex"] == 8
    assert bullet.enabled == [0, 0]
    manager.step({"PSM1": SimpleNamespace(jaw_measured=0.2)})
    assert manager.attachments == {}
    assert bullet.removed == [7, 9]
    assert bullet.enabled == [0, 0, 1, 1]


def test_unlimited_grasps_and_constraint_break_require_jaw_reopen():
    bullet = FakePyBullet()

    def arm(name):
        return SimpleNamespace(
            config=SimpleNamespace(name=name),
            robot=SimpleNamespace(
                body_id=1,
                link_indices={f"{name}_jaw_1_link": 3, f"{name}_jaw_2_link": 4},
            ),
            tool_link_index=5,
        )

    arms = {"PSM1": arm("PSM1"), "PSM2": arm("PSM2")}
    objects = {"cube": SimpleNamespace(body_id=2, spec=SimpleNamespace(fixed=False))}
    manager = GraspManager(
        bullet, 0, arms, objects, max_grasps_per_object=0, break_distance=0.005
    )
    manager.step({
        "PSM1": SimpleNamespace(jaw_measured=0.0),
        "PSM2": SimpleNamespace(jaw_measured=0.0),
    })
    assert set(manager.attachments) == {"PSM1", "PSM2"}

    # A geometric break latches the arm out while the jaw remains closed.
    bullet.object_position = (0.006, 0, 0)
    manager.step({
        "PSM1": SimpleNamespace(jaw_measured=0.0),
        "PSM2": SimpleNamespace(jaw_measured=0.2),
    })
    assert "PSM1" not in manager.attachments
    assert "PSM1" in manager.regrasp_blocked_arms
    manager.step({"PSM1": SimpleNamespace(jaw_measured=0.0)})
    assert "PSM1" not in manager.attachments
    manager.step({"PSM1": SimpleNamespace(jaw_measured=0.2)})
    assert "PSM1" not in manager.regrasp_blocked_arms
