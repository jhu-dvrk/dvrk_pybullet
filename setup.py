from pathlib import Path
import os
import re
import subprocess
import sys
import warnings
import yaml

from setuptools import find_packages, setup


package_name = "dvrk_pybullet"
local_pybullet_config = Path("share/pybullet.yaml")
example_pybullet_config = Path("share/pybullet.yaml.example")
script_files = [
    "scripts/simulator.py",
]


def _test_pybullet(python_bin: Path) -> bool:
    if not python_bin.is_file() or not os.access(python_bin, os.X_OK):
        return False
    try:
        res = subprocess.run(
            [str(python_bin), "-c", "import pybullet"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return res.returncode == 0
    except Exception:
        return False


def _saved_pybullet_python(path: Path) -> Path | None:
    if not path.is_file():
        return None
    try:
        content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        val = content.get("pybullet_python")
        if val in {None, "", "null", "None"}:
            return None
        return Path(val).expanduser().absolute()
    except Exception:
        return None


def _workspace_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "src").is_dir():
            return parent
    return None


def _check_installed_configs(saved: Path) -> None:
    workspace = _workspace_root()
    if workspace is None:
        return
    for tree_name in ("build", "install"):
        tree = workspace / tree_name
        if not tree.is_dir():
            continue
        for config in tree.rglob("pybullet.yaml"):
            installed = _saved_pybullet_python(config)
            if installed is not None and installed != saved:
                warnings.warn(
                    f"{config} contains PyBullet Python path {installed}, but the "
                    f"source configuration contains {saved}. Rebuild the "
                    "package to refresh that tree.",
                    RuntimeWarning,
                )


def _configure_pybullet() -> bool:
    """Validate PyBullet Python interpreter and save to share/pybullet.yaml."""
    env_candidate = os.environ.get("DVRK_PYBULLET_PYTHON", "").strip() or os.environ.get("PYBULLET_PYTHON", "").strip()
    selected_python: Path | None = None

    if env_candidate:
        candidate_path = Path(env_candidate).expanduser().absolute()
        if _test_pybullet(candidate_path):
            selected_python = candidate_path
        else:
            print(
                f"error: DVRK_PYBULLET_PYTHON was set to '{env_candidate}', but this Python interpreter cannot import 'pybullet'.\n"
                "Please make sure 'pybullet' is installed in this Python environment.",
                file=sys.stderr,
            )
            raise SystemExit(2)

    if selected_python is None:
        saved_python = _saved_pybullet_python(local_pybullet_config)
        if saved_python is not None and _test_pybullet(saved_python):
            selected_python = saved_python

    if selected_python is None:
        sys_py = Path(sys.executable).absolute()
        if _test_pybullet(sys_py):
            selected_python = sys_py

    if selected_python is None and "VIRTUAL_ENV" in os.environ:
        venv_py = (Path(os.environ["VIRTUAL_ENV"]) / "bin" / "python").absolute()
        if _test_pybullet(venv_py):
            selected_python = venv_py

    if selected_python is None:
        workspace = _workspace_root()
        if workspace is not None:
            for venv_dir in (workspace / "venv", workspace / ".venv"):
                candidate = (venv_dir / "bin" / "python").absolute()
                if _test_pybullet(candidate):
                    selected_python = candidate
                    break

    if selected_python is None:
        print(
            f"\nerror: Could not find a Python interpreter with 'pybullet' installed.\n"
            f"The default Python interpreter ({sys.executable}) cannot import 'pybullet'.\n\n"
            "Please set the environment variable DVRK_PYBULLET_PYTHON pointing to your\n"
            "Python interpreter with pybullet installed, e.g.:\n"
            "  export DVRK_PYBULLET_PYTHON=~/wss/dvrk/venv/bin/python\n\n"
            "Then run colcon build again:\n"
            "  colcon build --packages-select dvrk_pybullet\n",
            file=sys.stderr,
        )
        raise SystemExit(2)

    # Save to share/pybullet.yaml
    config_dict = {}
    if local_pybullet_config.is_file():
        try:
            config_dict = yaml.safe_load(local_pybullet_config.read_text(encoding="utf-8")) or {}
        except Exception:
            pass
    elif example_pybullet_config.is_file():
        try:
            config_dict = yaml.safe_load(example_pybullet_config.read_text(encoding="utf-8")) or {}
        except Exception:
            pass

    config_dict["pybullet_python"] = str(selected_python)
    local_pybullet_config.parent.mkdir(parents=True, exist_ok=True)
    with local_pybullet_config.open("w", encoding="utf-8") as f:
        yaml.safe_dump(config_dict, f, default_flow_style=False)

    _check_installed_configs(selected_python)
    return True


def _remove_installed_script_links() -> None:
    workspace = _workspace_root()
    if workspace is None:
        return
    directory = workspace / "install" / package_name / "share" / package_name / "scripts"
    for script in script_files:
        installed = directory / Path(script).name
        if installed.is_symlink():
            installed.unlink()


if "--dry-run" not in sys.argv and not any(
        argument.startswith("--help") for argument in sys.argv):
    _configure_pybullet()
    _remove_installed_script_links()

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/share", [str(local_pybullet_config)]),
    (f"share/{package_name}/share", [str(example_pybullet_config)]),
    (f"share/{package_name}/share/scenes", [
        str(path) for path in sorted(Path("share/scenes").glob("*.yaml"))
    ]),
    (f"share/{package_name}/share/open-xr", [
        str(path) for path in sorted(Path("share/open-xr").glob("*"))
        if path.is_file()
    ]),
    (f"share/{package_name}/share/schemas", [
        "share/schemas/openxr-video.schema.json",
    ]),
    (f"share/{package_name}/launch", [
        "launch/open_xr.launch.py",
        "launch/simulator.launch.py",
        "launch/test_scene.launch.py",
    ]),
    (f"share/{package_name}/scripts", script_files),
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Anton Deguet",
    maintainer_email="anton.deguet@jhu.edu",
    description="PyBullet backend for the common dVRK simulator runtime.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simulator_node = dvrk_pybullet.node:main",
            "dvrk_pybullet_preview = dvrk_pybullet.preview:main",
        ],
    },
)
