"""PyBullet dependency boundary and backend bootstrap helpers."""

from __future__ import annotations

from types import ModuleType

from .errors import PyBulletDependencyError


def load_pybullet() -> ModuleType:
    """Load PyBullet lazily so package inspection does not require pip setup."""
    try:
        import pybullet
    except ImportError as error:
        raise PyBulletDependencyError(
            "PyBullet could not be imported by this executable.\n"
            "From the colcon workspace root, bootstrap the venv, activate it, "
            "and rebuild:\n\n"
            "  ./src/dvrk/dvrk_pybullet/scripts/bootstrap_venv.sh\n"
            "  source .venv-pybullet/bin/activate\n"
            "  hash -r\n"
            "  colcon build --symlink-install --packages-select "
            "dvrk_simulator_base dvrk_pybullet\n"
            "  source install/setup.bash\n\n"
            "Then retry the ros2 run command."
        ) from error
    return pybullet
