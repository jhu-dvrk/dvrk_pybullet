from types import SimpleNamespace

from dvrk_pybullet.grasp import GraspManager


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
