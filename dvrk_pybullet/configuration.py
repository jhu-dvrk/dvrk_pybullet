"""Resolve installed common robot configuration for the PyBullet backend."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import yaml

from ament_index_python.packages import get_package_share_directory

from dvrk_arm_description import RobotConfig, load_robot_config
from dvrk_simulator_base.scene import SceneConfig, SceneResolver, load_scene_config


@dataclass(frozen=True)
class SimulatorConfig:
    renderer: str = "egl"
    gui: bool = True
    simulation_rate_hz: float = 120.0
    state_publish_rate_hz: float = 100.0
    generated_root: Path | None = None
    command_queue_capacity: int = 32
    grasp: "GraspConfig" = None
    scene: str | None = None


@dataclass(frozen=True)
class GraspConfig:
    show_grasps: bool = True
    max_grasps_per_object: int = 1
    policy: str = "pose_error"
    arm_policies: dict[str, str] = None
    close_threshold_rad: float = 0.04
    release_threshold_rad: float = 0.08
    break_distance_m: float = 0.005
    break_orientation_rad: float = 0.2617993877991494
    break_tension_force_n: float = 10.0
    break_shear_force_n: float = 10.0
    break_torque_nm: float = 0.25
    break_load_duration_s: float = 0.05
    max_force_n: float = 100.0
    constraint_erp: float = 0.8
    contact_region_offset_m: tuple[float, float, float] = (0.0, 0.0, -0.003)
    contact_region_radius_m: float = 0.008


def _boolean(value, *, source: Path, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{source}: {field} must be true or false")
    return value


def load_simulator_config(path: str | Path) -> SimulatorConfig:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: expected a YAML mapping")
    renderer = str(document.get("renderer", "egl")).lower()
    if renderer not in {"egl", "tiny"}:
        raise ValueError(f"{source}: renderer must be egl or tiny")
    simulation_rate = float(document.get("simulation_rate_hz", 120.0))
    state_rate = float(document.get("state_publish_rate_hz", 100.0))
    capacity = int(document.get("command_queue_capacity", 32))
    if simulation_rate <= 0.0 or state_rate <= 0.0:
        raise ValueError(f"{source}: simulator rates must be positive")
    if capacity <= 0:
        raise ValueError(f"{source}: command_queue_capacity must be positive")
    grasp_document = document.get("grasp", {})
    if not isinstance(grasp_document, dict):
        raise ValueError(f"{source}: grasp must be a mapping")
    policy = str(grasp_document.get("policy", "pose_error"))
    if policy not in {"pose_error", "force_torque"}:
        raise ValueError(f"{source}: invalid grasp.policy {policy!r}")
    arms_document = grasp_document.get("arms", {})
    if not isinstance(arms_document, dict):
        raise ValueError(f"{source}: grasp.arms must be a mapping")
    arm_policies = {}
    for arm_name, arm_document in arms_document.items():
        if not isinstance(arm_document, dict):
            raise ValueError(f"{source}: grasp.arms.{arm_name} must be a mapping")
        arm_policy = str(arm_document.get("policy", policy))
        if arm_policy not in {"pose_error", "force_torque"}:
            raise ValueError(
                f"{source}: invalid grasp.arms.{arm_name}.policy {arm_policy!r}"
            )
        arm_policies[str(arm_name)] = arm_policy
    offset = tuple(float(value) for value in grasp_document.get(
        "contact_region_offset_m", (0.0, 0.0, -0.003)
    ))
    if len(offset) != 3:
        raise ValueError(f"{source}: grasp.contact_region_offset_m must contain three values")
    grasp = GraspConfig(
        show_grasps=_boolean(
            grasp_document.get("show_grasps", True), source=source,
            field="grasp.show_grasps"
        ),
        max_grasps_per_object=int(grasp_document.get("max_grasps_per_object", 1)),
        policy=policy,
        arm_policies=arm_policies,
        close_threshold_rad=float(grasp_document.get("close_threshold_rad", 0.04)),
        release_threshold_rad=float(grasp_document.get("release_threshold_rad", 0.08)),
        break_distance_m=float(grasp_document.get("break_distance_m", 0.005)),
        break_orientation_rad=float(
            grasp_document.get("break_orientation_rad", 0.2617993877991494)
        ),
        break_tension_force_n=float(grasp_document.get("break_tension_force_n", 10.0)),
        break_shear_force_n=float(grasp_document.get("break_shear_force_n", 10.0)),
        break_torque_nm=float(grasp_document.get("break_torque_nm", 0.25)),
        break_load_duration_s=float(grasp_document.get("break_load_duration_s", 0.05)),
        max_force_n=float(grasp_document.get("max_force_n", 100.0)),
        constraint_erp=float(grasp_document.get("constraint_erp", 0.8)),
        contact_region_offset_m=offset,
        contact_region_radius_m=float(grasp_document.get("contact_region_radius_m", 0.008)),
    )
    if (
        grasp.close_threshold_rad < 0.0
        or grasp.max_grasps_per_object < 0
        or grasp.release_threshold_rad <= grasp.close_threshold_rad
        or grasp.break_distance_m <= 0.0
        or not 0.0 < grasp.break_orientation_rad <= math.pi
        or grasp.break_tension_force_n <= 0.0
        or grasp.break_shear_force_n <= 0.0
        or grasp.break_torque_nm <= 0.0
        or grasp.break_load_duration_s < 0.0
        or grasp.max_force_n <= 0.0
        or not 0.0 < grasp.constraint_erp <= 1.0
        or grasp.contact_region_radius_m <= 0.0
    ):
        raise ValueError(f"{source}: invalid grasp tuning values")
    generated = document.get("generated_root")
    generated_root = None
    if generated not in (None, ""):
        generated_root = Path(str(generated)).expanduser()
        if not generated_root.is_absolute():
            generated_root = (source.parent / generated_root).resolve()
    scene = document.get("scene")
    return SimulatorConfig(
        renderer=renderer,
        gui=_boolean(document.get("gui", True), source=source, field="gui"),
        simulation_rate_hz=simulation_rate,
        state_publish_rate_hz=state_rate,
        generated_root=generated_root,
        command_queue_capacity=capacity,
        grasp=grasp,
        scene=None if scene in (None, "") else str(scene),
    )


def load_installed_robot_config(model: str, instrument: str) -> RobotConfig:
    share = Path(get_package_share_directory("dvrk_arm_description"))
    path = share / "arms" / f"{model}.yaml"
    if not path.is_file():
        raise RuntimeError(f"installed robot configuration does not exist: {path}")
    if str(model).upper() == "ECM":
        return load_robot_config(path, endoscope=instrument)
    return load_robot_config(path, instrument=instrument)


def scene_search_paths(config_path: str | Path) -> tuple[Path, ...]:
    """Return the ordered directories used for a bare scene selection."""
    config = Path(config_path).expanduser().resolve()
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    simulator_base_share = Path(get_package_share_directory("dvrk_simulator_base"))
    candidates = (
        config.parent / "scenes",
        package_share / "share" / "scenes",
        simulator_base_share / "share" / "exercises",
    )
    paths = []
    for path in candidates:
        path = path.resolve()
        if path not in paths:
            paths.append(path)
    return tuple(paths)


def resolve_scene_path(
    config_path: str | Path,
    selection: str | Path | Sequence[str | Path],
) -> Path | tuple[Path, ...]:
    """Resolve an absolute path or scene name(s) found in the search paths."""
    config = Path(config_path).expanduser().resolve()
    resolver = SceneResolver(scene_search_paths(config), relative_root=config.parent)
    if isinstance(selection, (list, tuple)):
        return resolver.resolve_all(selection)
    return resolver.resolve(selection)


def load_installed_scene_config(
    path: str | Path | Sequence[str | Path],
    *,
    search_paths: Sequence[Path] | None = None,
) -> SceneConfig:
    share = Path(get_package_share_directory("dvrk_simulator_base"))
    arm_description_share = Path(get_package_share_directory("dvrk_arm_description"))
    pybullet_share = Path(get_package_share_directory("dvrk_pybullet"))
    default_search = (
        pybullet_share / "share" / "scenes",
        share / "share" / "exercises",
    )
    resolver = SceneResolver(tuple(search_paths or default_search))
    return load_scene_config(
        path,
        robot_config_root=arm_description_share / "arms",
        resolver=resolver,
    )
