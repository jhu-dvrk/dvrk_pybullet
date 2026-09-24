"""Start one configured dVRK PyBullet scene using the configured Python interpreter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dvrk_pybullet.configuration import (
    load_installed_scene_config,
    load_simulator_config,
    resolve_scene_path,
)
from dvrk_pybullet.python_runtime import resolve_pybullet_python
from dvrk_pybullet.urdf_materializer import default_generated_root
from dvrk_simulator_base.rqt_perspective import (
    existing_ament_prefix_path,
    write_monitor_perspective,
)


def _start_sim(context):
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    simulator_config = load_simulator_config(config_path)

    selection = resolve_pybullet_python(simulator_config.generated_root)

    scene = LaunchConfiguration("scene").perform(context) or simulator_config.scene
    script = package_share / "scripts" / "simulator.py"

    cmd = [
        str(selection.path),
        str(script),
        "--config", str(config_path),
        "--scene", str(scene),
    ]
    actions = [
        LogInfo(
            msg=(
                f"Starting PyBullet simulator with Python {selection.path} "
                f"(selected via {selection.source})"
            )
        ),
        ExecuteProcess(cmd=cmd, output="screen"),
    ]
    if LaunchConfiguration("rqt").perform(context).lower() == "true":
        scene_config = resolve_scene_path(config_path, scene)
        scene_description = load_installed_scene_config(scene_config)
        arms = [robot.name for robot in scene_description.robots]
        perspective = write_monitor_perspective(
            (simulator_config.generated_root or default_generated_root()) / "rqt" / "monitor.perspective",
            arms,
            include_console=LaunchConfiguration("rqt_console").perform(context).lower() == "true",
        )
        rqt_environment = {"DVRK_RQT_ARMS": ",".join(arms)}
        if prefix_path := existing_ament_prefix_path():
            rqt_environment["AMENT_PREFIX_PATH"] = prefix_path
        actions.append(ExecuteProcess(
            cmd=["rqt", "--perspective-file", str(perspective)], output="screen",
            additional_env=rqt_environment,
        ))
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    default_config = package_share / "share" / "pybullet.yaml"
    if not default_config.is_file():
        default_config = package_share / "share" / "pybullet.yaml.example"

    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=str(default_config),
            description="Backend runtime configuration YAML",
        ),
        DeclareLaunchArgument(
            "scene", description="Scene YAML path or installed scene filename",
        ),
        DeclareLaunchArgument(
            "rqt", default_value="false",
            description="start a dockable dVRK rqt monitor",
        ),
        DeclareLaunchArgument(
            "rqt_console", default_value="false",
            description="include the dVRK Console widget in the rqt monitor",
        ),
        OpaqueFunction(function=_start_sim),
    ])
