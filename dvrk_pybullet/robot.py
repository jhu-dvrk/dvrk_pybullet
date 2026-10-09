"""Load a materialized dVRK model and build name-based PyBullet mappings."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import numpy as np

from dvrk_simulator_base.urdf_chain import MimicJoint, extract_mimic_joints
from .errors import PyBulletBackendError


def _decode(value: bytes | str) -> str:
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


@dataclass(frozen=True)
class LoadedRobot:
    body_id: int
    joint_indices: dict[str, int]
    link_indices: dict[str, int]
    controlled_joint_names: tuple[str, ...]
    controlled_joint_indices: tuple[int, ...]
    mimic_joints: tuple[MimicJoint, ...]


def load_robot(
    pybullet: Any,
    urdf_path: str | Path,
    expected_joint_names: Iterable[str],
    *,
    base_position: Iterable[float] = (0.0, 0.0, 0.0),
    base_orientation_xyzw: Iterable[float] = (0.0, 0.0, 0.0, 1.0),
) -> LoadedRobot:
    """Load a fixed-base robot and validate its controlled joints by name."""
    path = Path(urdf_path).resolve()
    if not path.is_file():
        raise PyBulletBackendError(f"materialized URDF does not exist: {path}")
    body_id = pybullet.loadURDF(
        str(path),
        basePosition=tuple(float(value) for value in base_position),
        baseOrientation=tuple(float(value) for value in base_orientation_xyzw),
        useFixedBase=True,
    )
    if body_id < 0:
        raise PyBulletBackendError(f"PyBullet failed to load URDF: {path}")

    joint_indices: dict[str, int] = {}
    link_indices: dict[str, int] = {}
    body_info = pybullet.getBodyInfo(body_id)
    if body_info:
        link_indices[_decode(body_info[0])] = -1
    for index in range(pybullet.getNumJoints(body_id)):
        info = pybullet.getJointInfo(body_id, index)
        joint_name = _decode(info[1])
        link_name = _decode(info[12])
        if joint_name in joint_indices:
            raise PyBulletBackendError(f"duplicate PyBullet joint name {joint_name!r}")
        if link_name in link_indices:
            raise PyBulletBackendError(f"duplicate PyBullet link name {link_name!r}")
        joint_indices[joint_name] = index
        link_indices[link_name] = index

    expected = tuple(str(name) for name in expected_joint_names)
    missing = tuple(name for name in expected if name not in joint_indices)
    if missing:
        raise PyBulletBackendError(
            f"materialized robot is missing configured joints: {', '.join(missing)}"
        )
    try:
        parsed_mimics = extract_mimic_joints(path, error_cls=PyBulletBackendError)
    except PyBulletBackendError:
        raise
    except Exception as error:
        raise PyBulletBackendError(f"materialized URDF is invalid: {error}") from error

    mimic_joints = []
    for mimic in parsed_mimics:
        if mimic.joint_name not in joint_indices or mimic.source_joint_name not in joint_indices:
            raise PyBulletBackendError(
                f"mimic relationship references an unknown joint: {mimic.joint_name} -> {mimic.source_joint_name}"
            )
        mimic_joints.append(mimic)

    return LoadedRobot(
        body_id=body_id,
        joint_indices=joint_indices,
        link_indices=link_indices,
        controlled_joint_names=expected,
        controlled_joint_indices=tuple(joint_indices[name] for name in expected),
        mimic_joints=tuple(mimic_joints),
    )


def reset_joint_positions(
    pybullet: Any,
    robot: LoadedRobot,
    positions: Iterable[float],
    velocities: Iterable[float] | None = None,
) -> None:
    values = tuple(float(value) for value in positions)
    if len(values) != len(robot.controlled_joint_indices):
        raise ValueError("home position does not match the controlled joint count")
    velocity_values = (
        tuple(0.0 for _ in values)
        if velocities is None else tuple(float(value) for value in velocities)
    )
    if len(velocity_values) != len(values):
        raise ValueError("joint velocity does not match the controlled joint count")
    for name, position, velocity in zip(
        robot.controlled_joint_names, values, velocity_values
    ):
        reset_joint_with_mimics(pybullet, robot, name, position, velocity)


def reset_joint_with_mimics(
    pybullet: Any,
    robot: LoadedRobot,
    joint_name: str,
    position: float,
    velocity: float = 0.0,
) -> None:
    """Apply one logical joint and every parsed one-level mimic relation."""
    if joint_name not in robot.joint_indices:
        raise PyBulletBackendError(f"robot has no joint named {joint_name!r}")
    pybullet.resetJointState(
        robot.body_id,
        robot.joint_indices[joint_name],
        float(position),
        targetVelocity=float(velocity),
    )
    for mimic in robot.mimic_joints:
        if mimic.source_joint_name != joint_name:
            continue
        pybullet.resetJointState(
            robot.body_id,
            robot.joint_indices[mimic.joint_name],
            mimic.multiplier * float(position) + mimic.offset,
            targetVelocity=mimic.multiplier * float(velocity),
        )
