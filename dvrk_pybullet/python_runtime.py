"""Runtime selection of the Python interpreter used for PyBullet."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys

from .urdf_materializer import default_generated_root


_CACHE_FILE = "python-runtime.json"


@dataclass(frozen=True)
class PyBulletPython:
    path: Path
    source: str


def _imports_pybullet(python: Path) -> bool:
    if not python.is_file() or not os.access(python, os.X_OK):
        return False
    result = subprocess.run(
        [str(python), "-c", "import pybullet"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _cache_path(generated_root: str | Path | None) -> Path:
    root = Path(generated_root or default_generated_root()).expanduser().resolve()
    return root / _CACHE_FILE


def _read_cached_python(cache_path: Path) -> Path | None:
    try:
        document = json.loads(cache_path.read_text(encoding="utf-8"))
        value = document.get("python")
        if not isinstance(value, str) or not value:
            return None
        return Path(value).expanduser().absolute()
    except (OSError, ValueError, TypeError):
        return None


def _save_python(cache_path: Path, python: Path) -> None:
    """Persist a workspace-local interpreter choice without touching source."""
    try:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = cache_path.with_suffix(f".tmp-{os.getpid()}")
        temporary.write_text(
            json.dumps({"python": str(python)}, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(cache_path)
    except OSError:
        # Selection still works in read-only deployments; only persistence is
        # unavailable there.
        return


def resolve_pybullet_python(
    generated_root: str | Path | None = None,
) -> PyBulletPython:
    """Select a PyBullet interpreter and record a portable workspace cache.

    An explicit ``DVRK_PYBULLET_PYTHON`` override takes precedence.  A valid
    saved interpreter is reused for later shells.  If neither is available,
    the current ROS Python is used when it imports PyBullet.
    """
    cache_path = _cache_path(generated_root)
    configured = os.environ.get("DVRK_PYBULLET_PYTHON", "").strip()
    if configured:
        candidate = Path(configured).expanduser().absolute()
        if not _imports_pybullet(candidate):
            raise RuntimeError(
                f"DVRK_PYBULLET_PYTHON is '{candidate}', but it cannot import pybullet"
            )
        _save_python(cache_path, candidate)
        return PyBulletPython(candidate, "DVRK_PYBULLET_PYTHON")

    cached = _read_cached_python(cache_path)
    if cached is not None and _imports_pybullet(cached):
        return PyBulletPython(cached, f"saved selection in {cache_path}")

    current = Path(sys.executable).absolute()
    if _imports_pybullet(current):
        _save_python(cache_path, current)
        return PyBulletPython(current, "current ROS Python")
    raise RuntimeError(
        "PyBullet is unavailable in the current ROS Python and no valid saved "
        f"interpreter was found in {cache_path}. Set DVRK_PYBULLET_PYTHON to a "
        "Python interpreter that can import pybullet."
    )
