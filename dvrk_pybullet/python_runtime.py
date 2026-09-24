"""Runtime selection of the Python interpreter used for PyBullet."""

from __future__ import annotations

from pathlib import Path

from dvrk_simulator_base.python_runtime import (
    SimulatorPython,
    check_python_imports,
    resolve_simulator_python,
)
from .urdf_materializer import default_generated_root


class PyBulletPython(SimulatorPython):
    pass


def _imports_pybullet(python: Path) -> bool:
    return check_python_imports(python, "pybullet")


def resolve_pybullet_python(
    generated_root: str | Path | None = None,
) -> PyBulletPython:
    """Select a PyBullet interpreter and record a portable workspace cache.

    An explicit ``DVRK_PYBULLET_PYTHON`` override takes precedence.  A valid
    saved interpreter is reused for later shells.  If neither is available,
    the current ROS Python is used when it imports PyBullet.
    """
    return resolve_simulator_python(
        simulator_name="PyBullet",
        env_var="DVRK_PYBULLET_PYTHON",
        generated_root=generated_root,
        default_generated_root=default_generated_root(),
        check_import_fn=_imports_pybullet,
        workspace_venv_names=(".venv-pybullet", ".venv"),
        bootstrap_command="./src/dvrk/dvrk_pybullet/scripts/bootstrap_venv.sh",
        result_factory=PyBulletPython,
        source_file=__file__,
    )
