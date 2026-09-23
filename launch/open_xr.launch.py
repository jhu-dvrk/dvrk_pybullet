"""Start the PyBullet patient cart and the optional Quest/OpenXR console."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_pybullet"))
    open_xr_directory = package_share / "share" / "open-xr"
    simulator_config = open_xr_directory / "pybullet.yaml"
    main_config = package_share / "share" / "pybullet.yaml"
    scene = package_share / "share" / "scenes" / "ECM_PSM1_PSM2_PSM3.yaml"
    system_config = (
        open_xr_directory / "system-MTML-MTMR-OpenXR-patient-cart-ROS.json"
    )
    overlay_config = open_xr_directory / "dvrk-console-overlay.json"

    from dvrk_pybullet.configuration import load_simulator_config
    cfg = load_simulator_config(simulator_config)
    pybullet_python = cfg.pybullet_python or load_simulator_config(main_config).pybullet_python
    if pybullet_python is None or not Path(pybullet_python).is_file():
        raise RuntimeError(
            f"PyBullet Python interpreter is not configured or not found: {pybullet_python}. "
            "Rebuild the workspace with DVRK_PYBULLET_PYTHON set to the Python interpreter with pybullet installed."
        )

    simulator = ExecuteProcess(
        cmd=[
            str(pybullet_python),
            str(package_share / "scripts" / "simulator.py"),
            "--config", str(simulator_config),
            "--scene", str(scene),
            "--scene", LaunchConfiguration("scene"),
            "--gui", LaunchConfiguration("gui"),
        ],
        output="screen",
    )
    dvrk_system = Node(
        package="dvrk_robot",
        executable="dvrk_system",
        name="dvrk_system",
        output="screen",
        cwd=str(open_xr_directory),
        arguments=["--json-config", str(system_config)],
    )
    console_overlay = Node(
        package="dvrk_console",
        executable="stereo_display",
        name="pybullet_console_overlay",
        output="screen",
        arguments=["-c", str(overlay_config)],
    )
    start_system = Node(
        package="dvrk_simulator_base",
        executable="start_dvrk_system",
        output="screen",
        arguments=["--console", LaunchConfiguration("console")],
    )

    stop_with_simulator = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason="PyBullet simulator exited"))],
        )
    )
    stop_with_console = RegisterEventHandler(
        OnProcessExit(
            target_action=dvrk_system,
            on_exit=[EmitEvent(event=Shutdown(reason="dvrk_system exited"))],
        )
    )
    stop_with_overlay = RegisterEventHandler(
        OnProcessExit(
            target_action=console_overlay,
            on_exit=[EmitEvent(event=Shutdown(reason="console video overlay exited"))],
        )
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
                "gui",
                default_value="false",
                description="show the local PyBullet debug GUI",
            ),
            simulator,
            console_overlay,
            dvrk_system,
            start_system,
            stop_with_simulator,
            stop_with_console,
            stop_with_overlay,
        ]
    )
