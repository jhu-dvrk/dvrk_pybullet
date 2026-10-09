"""Load backend-neutral scene objects into a PyBullet world."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

from dvrk_simulator_base.scene import SceneObject, resolve_asset_uri

from .errors import PyBulletBackendError


@dataclass(frozen=True)
class LoadedSceneObject:
    spec: SceneObject
    body_id: int


def load_scene_objects(
    pybullet: Any, objects: Iterable[SceneObject], *, connection: int
) -> dict[str, LoadedSceneObject]:
    """Load configured fixed or dynamic URDF objects into one Bullet world."""
    loaded = {}
    for spec in objects:
        body_id = pybullet.loadURDF(
            str(resolve_asset_uri(spec.asset, error_cls=PyBulletBackendError)),
            basePosition=spec.position,
            baseOrientation=spec.orientation_xyzw,
            useFixedBase=spec.fixed,
            physicsClientId=connection,
        )
        if body_id < 0:
            raise PyBulletBackendError(f"PyBullet could not load scene object {spec.name!r}")
        loaded[spec.name] = LoadedSceneObject(spec, body_id)
    return loaded
