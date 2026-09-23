"""Run a bounded headless PyBullet scene smoke test using the configured Python interpreter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from dvrk_pybullet.configuration import load_simulator_config


def _start_test(context):
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    simulator_config = load_simulator_config(config_path)

    pybullet_python = simulator_config.pybullet_python
    if pybullet_python is None or not Path(pybullet_python).is_file():
        raise RuntimeError(
            f"PyBullet Python interpreter is not configured or not found: {pybullet_python}. "
            "Rebuild the workspace with DVRK_PYBULLET_PYTHON set to the Python interpreter with pybullet installed."
        )

    scene = LaunchConfiguration("scene").perform(context) or simulator_config.scene
    script = package_share / "scripts" / "simulator.py"

    cmd = [
        str(pybullet_python),
        str(script),
        "--config", str(config_path),
        "--scene", str(scene),
    ]
    return [
        ExecuteProcess(cmd=cmd, output="screen"),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("dvrk_pybullet"))
    default_config = share / "share" / "pybullet.yaml"
    if not default_config.is_file():
        default_config = share / "share" / "pybullet.yaml.example"

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(default_config)),
        DeclareLaunchArgument("scene", description="Scene YAML path or installed scene filename"),
        DeclareLaunchArgument("timeout", default_value="1.0", description="Maximum test run duration in seconds"),
        SetEnvironmentVariable("DVRK_SIMULATOR_TEST_TIMEOUT", LaunchConfiguration("timeout")),
        OpaqueFunction(function=_start_test),
    ])