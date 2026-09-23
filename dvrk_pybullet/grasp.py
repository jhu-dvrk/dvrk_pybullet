"""Contact-qualified, constraint-backed grasping for kinematic PSMs."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Mapping


@dataclass(frozen=True)
class GraspAttachment:
    arm_name: str
    object_name: str
    constraint_id: int
    jaw_link_indices: tuple[int, int]
    marker_body_id: int | None
    parent_frame_position: tuple[float, float, float]
    parent_frame_orientation: tuple[float, float, float, float]


class GraspManager:
    """Attach a dynamic object only when both PSM jaws contact while closed."""

    def __init__(
        self,
        pybullet,
        connection: int,
        arms: Mapping,
        scene_objects: Mapping,
        *,
        show_markers: bool = True,
        max_grasps_per_object: int = 1,
        default_policy: str = "pose_error",
        arm_policies: Mapping[str, str] | None = None,
        close_threshold: float = 0.04,
        release_threshold: float = 0.08,
        break_distance: float = 0.005,
        break_orientation: float = math.radians(15.0),
        break_tension_force: float = 10.0,
        break_shear_force: float = 10.0,
        break_torque: float = 0.25,
        break_load_duration: float = 0.05,
        max_force: float = 100.0,
        constraint_erp: float = 0.8,
        contact_region_offset: tuple[float, float, float] = (0.0, 0.0, -0.003),
        contact_region_radius: float = 0.008,
    ) -> None:
        self.pybullet = pybullet
        self.connection = connection
        self.arms = arms
        self.objects = {
            name: item for name, item in scene_objects.items() if not item.spec.fixed
        }
        self.close_threshold = close_threshold
        self.show_markers = show_markers
        self.max_grasps_per_object = max_grasps_per_object
        self.default_policy = default_policy
        self.arm_policies = {} if arm_policies is None else dict(arm_policies)
        self.release_threshold = release_threshold
        self.break_distance = break_distance
        self.break_orientation_radians = break_orientation
        self.break_tension_force = break_tension_force
        self.break_shear_force = break_shear_force
        self.break_torque = break_torque
        self.break_load_duration = break_load_duration
        self.max_force = max_force
        self.constraint_erp = constraint_erp
        self.contact_region_offset = contact_region_offset
        self.contact_region_radius = contact_region_radius
        self.attachments: dict[str, GraspAttachment] = {}
        self.contact_marker_body_ids: dict[str, int] = {}
        self.regrasp_blocked_arms: set[str] = set()
        self.overload_started_at: dict[str, float] = {}

    def step(self, snapshots) -> None:
        """Release opened grippers, then seek one two-jaw contact per PSM."""
        for arm_name, snapshot in snapshots.items():
            if (
                snapshot.jaw_measured is not None
                and snapshot.jaw_measured >= self.release_threshold
            ):
                self.regrasp_blocked_arms.discard(arm_name)
        for arm_name, attachment in tuple(self.attachments.items()):
            jaw = snapshots[arm_name].jaw_measured
            if jaw is None or jaw >= self.release_threshold:
                self.overload_started_at.pop(arm_name, None)
                self._release(attachment)
            else:
                reason = self._break_reason(self.arms[arm_name], attachment)
                if reason is None:
                    self.overload_started_at.pop(arm_name, None)
                    continue
                if self._policy(attachment.arm_name) == "force_torque":
                    started_at = self.overload_started_at.setdefault(
                        arm_name, time.monotonic()
                    )
                    if time.monotonic() - started_at < self.break_load_duration:
                        continue
                self._release(
                    attachment, reason, require_reopen=True,
                )
        for attachment in self.attachments.values():
            self._update_marker(self.arms[attachment.arm_name], attachment.marker_body_id)
        grasp_counts = {}
        for attachment in self.attachments.values():
            grasp_counts[attachment.object_name] = (
                grasp_counts.get(attachment.object_name, 0) + 1
            )
        for arm_name, arm in self.arms.items():
            if arm_name in self.attachments:
                self._remove_contact_marker(arm_name)
                continue
            if arm_name not in snapshots:
                continue
            if arm_name in self.regrasp_blocked_arms:
                self._remove_contact_marker(arm_name)
                continue
            snapshot = snapshots[arm_name]
            if snapshot.jaw_measured is None or snapshot.jaw_measured > self.close_threshold:
                self._remove_contact_marker(arm_name)
                continue
            jaw_links = self._jaw_links(arm)
            if jaw_links is None:
                self._remove_contact_marker(arm_name)
                continue
            contact_object = None
            for object_name, item in self.objects.items():
                if self._contact_qualified(arm, item.body_id, jaw_links):
                    contact_object = (object_name, item)
                    break
            if contact_object is None:
                self._remove_contact_marker(arm_name)
                continue
            object_name, item = contact_object
            if (
                self.max_grasps_per_object > 0
                and grasp_counts.get(object_name, 0) >= self.max_grasps_per_object
            ):
                self._ensure_contact_marker(arm_name, arm)
                continue
            self._remove_contact_marker(arm_name)
            self._attach(arm_name, arm, object_name, item.body_id, jaw_links)
            grasp_counts[object_name] = grasp_counts.get(object_name, 0) + 1

    def release_all(self) -> None:
        for attachment in tuple(self.attachments.values()):
            self._release(attachment)
        for arm_name in tuple(self.contact_marker_body_ids):
            self._remove_contact_marker(arm_name)

    def marker_poses(self) -> dict[str, tuple]:
        """Return the active visual-marker poses for the camera renderer."""
        if not self.show_markers:
            return {}
        markers = {
            attachment.arm_name: self._grasp_pose(self.arms[attachment.arm_name])
            for attachment in self.attachments.values()
        }
        markers.update(
            (arm_name, self._grasp_pose(self.arms[arm_name]))
            for arm_name in self.contact_marker_body_ids
        )
        return markers

    @staticmethod
    def _jaw_links(arm) -> tuple[int, int] | None:
        prefix = f"{arm.config.name}_"
        links = arm.robot.link_indices
        names = (f"{prefix}jaw_1_link", f"{prefix}jaw_2_link")
        if any(name not in links for name in names):
            return None
        return links[names[0]], links[names[1]]

    def _contact_qualified(self, arm, object_id: int, jaw_links: tuple[int, int]) -> bool:
        contacts = [self.pybullet.getContactPoints(
            bodyA=arm.robot.body_id, bodyB=object_id, linkIndexA=link_index,
            physicsClientId=self.connection,
        ) for link_index in jaw_links]
        if not all(contacts):
            return False
        # Position on B (the object) is contact tuple element 6.  The fallback
        # retains two-jaw behavior for lightweight mock clients without full
        # PyBullet contact records.
        points = [contact[6] for jaw_contacts in contacts for contact in jaw_contacts
                  if len(contact) > 6 and len(contact[6]) == 3]
        if not points:
            return True
        tool = self.pybullet.getLinkState(
            arm.robot.body_id, arm.tool_link_index, computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        center, _ = self.pybullet.multiplyTransforms(
            tool[4], tool[5], self.contact_region_offset, (0.0, 0.0, 0.0, 1.0)
        )
        midpoint = tuple(sum(point[index] for point in points) / len(points)
                         for index in range(3))
        return math.dist(midpoint, center) <= self.contact_region_radius

    def _attach(self, arm_name, arm, object_name, object_id, jaw_links) -> None:
        tool_state = self.pybullet.getLinkState(
            arm.robot.body_id,
            arm.tool_link_index,
            computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        object_position, object_orientation = self.pybullet.getBasePositionAndOrientation(
            object_id, physicsClientId=self.connection
        )
        inverse = self.pybullet.invertTransform(tool_state[4], tool_state[5])
        relative = self.pybullet.multiplyTransforms(
            inverse[0], inverse[1], object_position, object_orientation
        )
        constraint_id = self.pybullet.createConstraint(
            parentBodyUniqueId=arm.robot.body_id,
            parentLinkIndex=arm.tool_link_index,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=self.pybullet.JOINT_FIXED,
            jointAxis=(0.0, 0.0, 0.0),
            parentFramePosition=relative[0],
            parentFrameOrientation=relative[1],
            childFramePosition=(0.0, 0.0, 0.0),
            childFrameOrientation=(0.0, 0.0, 0.0, 1.0),
            physicsClientId=self.connection,
        )
        self.pybullet.changeConstraint(
            constraint_id, maxForce=self.max_force, erp=self.constraint_erp,
            physicsClientId=self.connection
        )
        for link_index in jaw_links:
            self.pybullet.setCollisionFilterPair(
                arm.robot.body_id,
                object_id,
                link_index,
                -1,
                0,
                physicsClientId=self.connection,
            )
        marker_body_id = self._create_marker(arm)
        self.attachments[arm_name] = GraspAttachment(
            arm_name, object_name, constraint_id, jaw_links, marker_body_id,
            relative[0], relative[1]
        )
        print(f"PyBullet grasp attached: {arm_name} -> {object_name}")

    def _grasp_pose(self, arm):
        """Return a visible marker pose slightly proximal to the grasp point."""
        tool = self.pybullet.getLinkState(
            arm.robot.body_id, arm.tool_link_index, computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        grasp_position, grasp_orientation = self.pybullet.multiplyTransforms(
            tool[4], tool[5], self.contact_region_offset, (0.0, 0.0, 0.0, 1.0)
        )
        return self.pybullet.multiplyTransforms(
            grasp_position, grasp_orientation, (0.0, 0.0, -0.006),
            (0.0, 0.0, 0.0, 1.0),
        )

    def _create_marker(self, arm) -> int | None:
        """Create a non-colliding red 4 mm cube at the active grasp point."""
        if not self.show_markers:
            return None
        required = ("createVisualShape", "createMultiBody", "resetBasePositionAndOrientation")
        if not all(hasattr(self.pybullet, name) for name in required):
            return None
        position, orientation = self._grasp_pose(arm)
        shape = self.pybullet.createVisualShape(
            self.pybullet.GEOM_BOX,
            halfExtents=(0.002, 0.002, 0.002),
            rgbaColor=(1.0, 0.0, 0.0, 1.0),
            physicsClientId=self.connection,
        )
        return self.pybullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=shape,
            basePosition=position,
            baseOrientation=orientation,
            physicsClientId=self.connection,
        )

    def _update_marker(self, arm, marker_body_id: int | None) -> None:
        if marker_body_id is None:
            return
        position, orientation = self._grasp_pose(arm)
        self.pybullet.resetBasePositionAndOrientation(
            marker_body_id, position, orientation, physicsClientId=self.connection
        )

    def _ensure_contact_marker(self, arm_name, arm) -> None:
        marker = self.contact_marker_body_ids.get(arm_name)
        if marker is None:
            marker = self._create_marker(arm)
            if marker is not None:
                self.contact_marker_body_ids[arm_name] = marker
                print(f"PyBullet visual grasp contact: {arm_name}")
        self._update_marker(arm, marker)

    def _remove_contact_marker(self, arm_name) -> None:
        marker = self.contact_marker_body_ids.pop(arm_name, None)
        if marker is not None:
            self.pybullet.removeBody(marker, physicsClientId=self.connection)

    def _attachment_exceeds_error(self, arm, attachment: GraspAttachment) -> bool:
        tool = self.pybullet.getLinkState(
            arm.robot.body_id, arm.tool_link_index, computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        expected_position, expected_orientation = self.pybullet.multiplyTransforms(
            tool[4], tool[5], attachment.parent_frame_position,
            attachment.parent_frame_orientation,
        )
        actual_position, actual_orientation = self.pybullet.getBasePositionAndOrientation(
            self.objects[attachment.object_name].body_id, physicsClientId=self.connection
        )
        if math.dist(expected_position, actual_position) > self.break_distance:
            return True
        dot = abs(sum(
            expected * actual
            for expected, actual in zip(expected_orientation, actual_orientation)
        ))
        dot = min(1.0, max(-1.0, dot))
        return 2.0 * math.acos(dot) > self.break_orientation_radians

    def _break_reason(self, arm, attachment: GraspAttachment) -> str | None:
        if self._policy(attachment.arm_name) == "pose_error":
            return "constraint error" if self._attachment_exceeds_error(arm, attachment) else None
        if not hasattr(self.pybullet, "getConstraintState"):
            return None
        state = self.pybullet.getConstraintState(
            attachment.constraint_id, physicsClientId=self.connection
        )
        if len(state) < 6:
            return None
        tool = self.pybullet.getLinkState(
            arm.robot.body_id, arm.tool_link_index, computeForwardKinematics=True,
            physicsClientId=self.connection,
        )
        anchor, _ = self.pybullet.multiplyTransforms(
            tool[4], tool[5], attachment.parent_frame_position,
            attachment.parent_frame_orientation,
        )
        axis = tuple(anchor[index] - tool[4][index] for index in range(3))
        axis_norm = math.sqrt(sum(value * value for value in axis))
        force = tuple(float(value) for value in state[:3])
        torque = tuple(float(value) for value in state[3:6])
        tension = 0.0 if axis_norm < 1e-6 else sum(
            force[index] * axis[index] / axis_norm for index in range(3)
        )
        force_squared = sum(value * value for value in force)
        shear = math.sqrt(max(0.0, force_squared - tension * tension))
        torque_norm = math.sqrt(sum(value * value for value in torque))
        if tension > self.break_tension_force:
            return f"tension {tension:.1f} N"
        if shear > self.break_shear_force:
            return f"shear {shear:.1f} N"
        if torque_norm > self.break_torque:
            return f"torque {torque_norm:.3f} N m"
        return None

    def _policy(self, arm_name: str) -> str:
        return self.arm_policies.get(arm_name, self.default_policy)

    def _release(
        self, attachment: GraspAttachment, reason: str = "jaw opened",
        require_reopen: bool = False,
    ) -> None:
        arm = self.arms[attachment.arm_name]
        item = self.objects[attachment.object_name]
        self.pybullet.removeConstraint(attachment.constraint_id, physicsClientId=self.connection)
        for link_index in attachment.jaw_link_indices:
            self.pybullet.setCollisionFilterPair(
                arm.robot.body_id,
                item.body_id,
                link_index,
                -1,
                1,
                physicsClientId=self.connection,
            )
        if attachment.marker_body_id is not None:
            self.pybullet.removeBody(
                attachment.marker_body_id, physicsClientId=self.connection
            )
        del self.attachments[attachment.arm_name]
        self.overload_started_at.pop(attachment.arm_name, None)
        if require_reopen:
            self.regrasp_blocked_arms.add(attachment.arm_name)
        print(
            "PyBullet grasp released: "
            f"{attachment.arm_name} -> {attachment.object_name} ({reason})"
        )
