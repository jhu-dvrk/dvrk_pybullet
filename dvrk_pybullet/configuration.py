"""Runtime options for the pybullet backend."""

from dataclasses import dataclass
from dvrk_simulator_base.configuration import RuntimeConfig, load_runtime_config
from dvrk_simulator_base import configuration as _shared
from dvrk_simulator_base.configuration import ConstraintGraspConfig as GraspConfig


@dataclass(frozen=True)
class SimulatorConfig(RuntimeConfig):
    renderer: str = "egl"
    grasp: GraspConfig | None = None


def load_simulator_config(path):
    return load_runtime_config(path, SimulatorConfig, renderers={"egl", "tiny"}, grasp_type=GraspConfig)


def scene_search_paths(config_path):
    return _shared.scene_search_paths("dvrk_pybullet", config_path)


def resolve_scene_path(config_path, selection):
    return _shared.resolve_scene_path("dvrk_pybullet", config_path, selection)


def load_installed_scene_config(path, *, search_paths=None):
    return _shared.load_installed_scene_config("dvrk_pybullet", path, search_paths=search_paths)
