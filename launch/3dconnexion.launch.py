"""Start the PyBullet patient cart with two 3Dconnexion MTMs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    config_directory = package_share / "share" / "3dconnexion"
    simulator_config = config_directory / "pybullet.yaml"
    system_config = (
        config_directory
        / "system-MTML-MTMR-3Dconnexion-patient-cart-ROS.json"
    )

    from dvrk_pybullet.configuration import load_simulator_config, resolve_scene_path
    from dvrk_pybullet.python_runtime import resolve_pybullet_python
    from dvrk_pybullet.urdf_materializer import default_generated_root
    from dvrk_simulator_base.rqt_perspective import (
        existing_ament_prefix_path,
        write_monitor_perspective,
    )

    pybullet_config = load_simulator_config(simulator_config)
    selection = resolve_pybullet_python(pybullet_config.generated_root)
    patient_cart = resolve_scene_path(
        simulator_config, "ECM_PSM1_PSM2_PSM3.yaml"
    )
    perspective = write_monitor_perspective(
        (pybullet_config.generated_root or default_generated_root())
        / "rqt"
        / "3dconnexion.perspective",
        ("ECM", "PSM1", "PSM2", "PSM3"),
        include_console=True,
    )

    simulator = ExecuteProcess(
        cmd=[
            str(selection.path),
            str(package_share / "scripts" / "simulator.py"),
            "--config",
            str(simulator_config),
            "--scene",
            str(patient_cart),
            "--scene",
            LaunchConfiguration("scene"),
            "--headless",
            LaunchConfiguration("headless"),
        ],
        output="screen",
    )
    dvrk_system = Node(
        package="dvrk_robot",
        executable="dvrk_system",
        name="dvrk_system",
        output="screen",
        cwd=str(config_directory),
        arguments=["--json-config", str(system_config)],
    )
    start_system = Node(
        package="dvrk_simulator_base",
        executable="start_dvrk_system",
        output="screen",
        arguments=["--console", LaunchConfiguration("console")],
    )
    rqt_environment = {
        "DVRK_RQT_ARMS": "ECM,PSM1,PSM2,PSM3",
        "DVRK_RQT_CONSOLE": LaunchConfiguration("console"),
    }
    if prefix_path := existing_ament_prefix_path():
        rqt_environment["AMENT_PREFIX_PATH"] = prefix_path
    rqt_monitor = ExecuteProcess(
        cmd=["rqt", "--perspective-file", str(perspective)],
        additional_env=rqt_environment,
        condition=IfCondition(LaunchConfiguration("rqt")),
        output="screen",
    )
    camera_preview = ExecuteProcess(
        cmd=[
            "gst-launch-1.0",
            "unixfdsrc",
            "socket-path=dvrk:simulator:stereo_source",
            "socket-type=abstract",
            "do-timestamp=true",
            "!",
            "queue",
            "leaky=downstream",
            "max-size-buffers=1",
            "!",
            "videoconvert",
            "!",
            "autovideosink",
            "sync=false",
        ],
        condition=IfCondition(LaunchConfiguration("preview")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene",
                default_value="tray_cubes.yaml",
                description="Exercise scene YAML path or installed exercise filename",
            ),
            DeclareLaunchArgument(
                "console",
                default_value="console",
                description="dVRK console ROS namespace",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="run without desktop GUI window",
            ),
            DeclareLaunchArgument(
                "rqt",
                default_value="false",
                description="start a dockable dVRK and CRTK rqt monitor",
            ),
            DeclareLaunchArgument(
                "preview",
                default_value="true",
                description="open the virtual stereo camera in GStreamer",
            ),
            LogInfo(
                msg=(
                    f"Starting PyBullet simulator with Python {selection.path} "
                    f"(selected via {selection.source})"
                )
            ),
            simulator,
            dvrk_system,
            start_system,
            rqt_monitor,
            TimerAction(period=2.0, actions=[camera_preview]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulator,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="PyBullet simulator exited"))
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=dvrk_system,
                    on_exit=[EmitEvent(event=Shutdown(reason="dvrk_system exited"))],
                )
            ),
        ]
    )
