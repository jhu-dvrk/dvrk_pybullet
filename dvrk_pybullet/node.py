"""ROS 2 interfaces for a shared multi-arm PyBullet world."""

from __future__ import annotations

import argparse
from dataclasses import replace
import os
from pathlib import Path
import sys
import threading
import time
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from dvrk_simulator_base.snapshots import ArmSnapshot
from dvrk_simulator_base.ros_interface import ArmRosInterface, LatestSnapshot

from .camera import CameraOptions
from .configuration import (
    load_installed_robot_config,
    load_installed_scene_config,
    load_simulator_config,
    resolve_scene_path,
)
from .errors import PyBulletDependencyError
from .runtime import RuntimeOptions
from .urdf_materializer import SUPPORTED_ROBOTS
from .world_runtime import PyBulletWorldRuntime


class DvrkPyBulletNode(Node):
    def __init__(
        self,
        *,
        scene_path: Path | Sequence[Path] | None = None,
        model: str = "PSM1",
        instrument: str = "420006",
        endoscope: str = "Si_straight",
        gui: bool = True,
        simulation_rate_hz: float = 120.0,
        state_publish_rate_hz: float = 100.0,
        generated_root: Path | None = None,
        command_queue_capacity: int = 32,
        renderer: str = "egl",
    ) -> None:
        super().__init__("dvrk_pybullet")
        if scene_path is not None:
            scene = load_installed_scene_config(scene_path)
            configs = scene.robots
            scene_objects = scene.objects
            self.camera_options = replace(
                CameraOptions.from_scene(scene.camera),
                renderer=renderer,
            )
        else:
            model = model.upper()
            if model not in SUPPORTED_ROBOTS:
                raise ValueError(f"model must be one of {SUPPORTED_ROBOTS}")
            asset = endoscope if model == "ECM" else instrument
            configs = (load_installed_robot_config(model, asset),)
            scene_objects = ()
            self.camera_options = CameraOptions(enabled=False)

        self.configs = tuple(configs)
        self.scene_objects = tuple(scene_objects)
        self.gui = bool(gui)
        self.simulation_rate_hz = float(simulation_rate_hz)
        state_rate = float(state_publish_rate_hz)
        if self.simulation_rate_hz <= 0.0 or state_rate <= 0.0:
            raise ValueError("simulation and state publish rates must be positive")
        self.generated_root = generated_root
        capacity = int(command_queue_capacity)
        if capacity <= 0:
            raise ValueError("command queue capacity must be positive")

        ecm_config = next((item for item in self.configs if item.type == "ECM"), None)
        self.arm_interfaces: dict[str, ArmRosInterface] = {}
        if ecm_config is not None:
            ecm = ArmRosInterface(self, ecm_config, capacity)
            self.arm_interfaces[ecm_config.name] = ecm
        else:
            ecm = None
        for config in self.configs:
            if config.type != "ECM":
                self.arm_interfaces[config.name] = ArmRosInterface(
                    self, config, capacity, ecm_interface=ecm
                )
        self.create_timer(1.0 / state_rate, self._publish_latest)
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diagnostic_started_at = time.monotonic()
        self._diagnostic_snapshot_count = 0
        self._state_publish_rate_hz = state_rate
        self.create_timer(1.0, self._publish_diagnostics)
        self._install_single_arm_compatibility(self.arm_interfaces[self.configs[0].name])

    def _install_single_arm_compatibility(self, interface: ArmRosInterface) -> None:
        self.config = interface.config
        self.commands = interface.commands
        self.snapshots = interface.snapshots
        self.frame_id = interface.frame_id
        for name in (
            "measured_js", "setpoint_js", "measured_cp", "setpoint_cp", "measured_cv",
            "jaw_measured_js", "jaw_setpoint_js", "operating_state", "state", "tool_type",
            "info", "warning", "error", "servo_jp", "move_jp", "servo_cp", "move_cp",
            "jaw_servo_jp", "jaw_move_jp", "state_command",
        ):
            setattr(self, name, getattr(interface, name))
        self._primary_interface = interface

    def _servo_jp_callback(self, message) -> None:
        self._primary_interface._servo_jp_callback(message)

    def _move_jp_callback(self, message) -> None:
        self._primary_interface._move_jp_callback(message)

    def _servo_cp_callback(self, message) -> None:
        self._primary_interface._servo_cp_callback(message)

    def _move_cp_callback(self, message) -> None:
        self._primary_interface._move_cp_callback(message)

    def _jaw_servo_jp_callback(self, message) -> None:
        self._primary_interface._jaw_servo_jp_callback(message)

    def _jaw_move_jp_callback(self, message) -> None:
        self._primary_interface._jaw_move_jp_callback(message)

    def _state_command_callback(self, message) -> None:
        self._primary_interface._state_command_callback(message)

    def install_initial_snapshot(self, snapshot: ArmSnapshot) -> None:
        self._primary_interface.install_initial_snapshot(snapshot)

    def install_initial_snapshots(self, snapshots: dict[str, ArmSnapshot]) -> None:
        for name, snapshot in snapshots.items():
            self.arm_interfaces[name].install_initial_snapshot(snapshot)

    def accept_snapshots(self, snapshots: dict[str, ArmSnapshot]) -> None:
        for name, snapshot in snapshots.items():
            self.arm_interfaces[name].snapshots.set(snapshot)
        self._diagnostic_snapshot_count += 1

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._diagnostic_started_at, 1e-6)
        simulation_hz = self._diagnostic_snapshot_count / elapsed
        self._diagnostic_started_at = now
        self._diagnostic_snapshot_count = 0
        status = DiagnosticStatus()
        status.name = "dvrk_pybullet/runtime"
        status.hardware_id = "dvrk_pybullet"
        status.level = (
            DiagnosticStatus.OK if simulation_hz > 0.0 else DiagnosticStatus.WARN
        )
        status.message = "running" if simulation_hz > 0.0 else "waiting for simulation"
        status.values = [
            KeyValue(key="simulation_hz", value=f"{simulation_hz:.1f}"),
            KeyValue(key="state_publish_hz", value=f"{self._state_publish_rate_hz:.1f}"),
            KeyValue(key="arms", value=str(len(self.arm_interfaces))),
            KeyValue(key="camera_enabled", value=str(self.camera_options.enabled).lower()),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics.publish(message)

    def _publish_latest(self) -> None:
        for interface in self.arm_interfaces.values():
            interface.publish_latest()


def _spin_executor(executor: SingleThreadedExecutor) -> None:
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def _parse_command_line(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, help="simulator settings YAML (default: installed pybullet.yaml)"
    )
    parser.add_argument(
        "--scene", type=Path, required=True, action="append", metavar="FILE",
        help="scene YAML path or installed scene filename; may be repeated internally",
    )
    return parser.parse_args(remove_ros_args(args))


def main(args=None) -> int:
    raw_args = list(sys.argv[1:] if args is None else args)
    options = _parse_command_line(raw_args)
    config_path = options.config
    if config_path is None:
        config_path = (
            Path(get_package_share_directory("dvrk_pybullet")) / "share" / "pybullet.yaml"
        )
    try:
        config = load_simulator_config(config_path)
        scene_path = resolve_scene_path(
            config_path,
            options.scene[0] if len(options.scene) == 1 else options.scene,
        )
    except (FileNotFoundError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    rclpy.init(args=raw_args)
    node = None
    runtime = None
    executor = None
    executor_thread = None
    try:
        node = DvrkPyBulletNode(
            scene_path=scene_path,
            gui=False if "DVRK_SIMULATOR_TEST_TIMEOUT" in os.environ else config.gui,
            simulation_rate_hz=config.simulation_rate_hz,
            state_publish_rate_hz=config.state_publish_rate_hz,
            generated_root=(
                None if config.generated_root is None else str(config.generated_root)
            ),
            command_queue_capacity=config.command_queue_capacity,
            renderer=config.renderer,
        )
        runtime = PyBulletWorldRuntime(
            node.configs,
            RuntimeOptions(
                gui=node.gui,
                simulation_rate_hz=node.simulation_rate_hz,
                generated_root=node.generated_root,
            ),
            {name: interface.commands for name, interface in node.arm_interfaces.items()},
            camera_options=node.camera_options,
            scene_objects=node.scene_objects,
        )
        node.install_initial_snapshots(runtime.initialize())
        node.get_logger().info(
            "loaded shared PyBullet world: " + ", ".join(node.arm_interfaces)
        )
        if runtime.camera_worker is not None:
            camera = node.camera_options
            node.get_logger().info(
                f"ECM camera: {camera.mode} RGBA "
                f"{camera.transport_width}x{camera.height} at "
                f"{camera.rate_hz:g} Hz on {camera.socket_path}"
            )
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(
            target=_spin_executor, args=(executor,), daemon=True
        )
        executor_thread.start()
        timeout = os.environ.get("DVRK_SIMULATOR_TEST_TIMEOUT")
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        runtime.run(
            node.accept_snapshots,
            lambda: rclpy.ok() and (deadline is None or time.monotonic() < deadline),
        )
    except KeyboardInterrupt:
        pass
    except PyBulletDependencyError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    except (FileNotFoundError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    finally:
        if runtime is not None:
            runtime.shutdown()
        if executor is not None:
            executor.shutdown()
        if executor_thread is not None:
            executor_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
