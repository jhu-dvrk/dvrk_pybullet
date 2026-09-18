"""ROS 2 interfaces for a shared multi-arm PyBullet world."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import replace
from pathlib import Path
import sys
import threading
import time
import os
from typing import Any, Sequence

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from crtk_msgs.msg import OperatingState, StringStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState as JointStateMessage
from std_msgs.msg import String

from dvrk_simulator_base.cartesian_frames import (
    compose_pose,
    relative_pose,
    relative_twist,
    view_pose_from_optical,
)
from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.command_validation import (
    jaw_position_from_message,
    joint_positions_from_message,
    pose_from_message,
)
from dvrk_simulator_base.config import RobotConfig
from dvrk_simulator_base.ros_messages import (
    joint_state_message,
    operating_state_message,
    pose_stamped_message,
    string_stamped_message,
    twist_stamped_message,
)
from dvrk_simulator_base.ros_qos import (
    transient_local_event_qos,
    transient_local_latched_qos,
)
from dvrk_simulator_base.rotations import quaternion_matrix_xyzw
from dvrk_simulator_base.snapshots import ArmSnapshot, OperatingStateSnapshot
from dvrk_simulator_base.types import JointState, Pose

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


class LatestSnapshot:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: ArmSnapshot | None = None
        self._last_operating_state: OperatingStateSnapshot | None = None
        self._operating_state_events: deque[OperatingStateSnapshot] = deque(maxlen=32)

    def set_initial(self, snapshot: ArmSnapshot) -> None:
        with self._lock:
            self._value = snapshot
            self._last_operating_state = snapshot.operating_state
            self._operating_state_events.clear()

    def set(self, snapshot: ArmSnapshot) -> None:
        with self._lock:
            if (
                snapshot.operating_state_event
                or snapshot.operating_state != self._last_operating_state
            ):
                self._operating_state_events.append(snapshot.operating_state)
                self._last_operating_state = snapshot.operating_state
            self._value = snapshot

    def peek(self) -> ArmSnapshot | None:
        with self._lock:
            return self._value

    def get_with_events(
        self,
    ) -> tuple[ArmSnapshot | None, tuple[OperatingStateSnapshot, ...]]:
        with self._lock:
            events = tuple(self._operating_state_events)
            self._operating_state_events.clear()
            return self._value, events


class ArmRosInterface:
    def __init__(
        self,
        node: Node,
        config: RobotConfig,
        command_queue_capacity: int,
        ecm_interface: "ArmRosInterface | None" = None,
    ) -> None:
        self.node = node
        self.config = config
        self.ecm_interface = ecm_interface
        self.commands = CommandMailboxes(command_queue_capacity)
        self.snapshots = LatestSnapshot()
        self._last_event_stamp_ns = -1
        self.base_pose = Pose(
            config.base_position,
            quaternion_matrix_xyzw(config.base_orientation_xyzw),
        )
        self.frame_id = (
            "ECM_view"
            if config.type == "PSM" and ecm_interface is not None
            else config.parent_frame
        )
        prefix = f"/{config.name}"
        event_qos = transient_local_event_qos()
        self.measured_js = node.create_publisher(
            JointStateMessage, f"{prefix}/measured_js", 10
        )
        self.setpoint_js = node.create_publisher(
            JointStateMessage, f"{prefix}/setpoint_js", 10
        )
        self.measured_cp = node.create_publisher(PoseStamped, f"{prefix}/measured_cp", 10)
        self.setpoint_cp = node.create_publisher(PoseStamped, f"{prefix}/setpoint_cp", 10)
        self.measured_cv = node.create_publisher(TwistStamped, f"{prefix}/measured_cv", 10)
        self.operating_state = node.create_publisher(
            OperatingState, f"{prefix}/operating_state", event_qos
        )
        self.state = node.create_publisher(StringStamped, f"{prefix}/state", event_qos)
        self.info = node.create_publisher(StringStamped, f"{prefix}/info", 10)
        self.warning = node.create_publisher(StringStamped, f"{prefix}/warning", 10)
        self.error = node.create_publisher(StringStamped, f"{prefix}/error", 10)

        self.servo_jp = node.create_subscription(
            JointStateMessage, f"{prefix}/servo_jp", self._servo_jp_callback, 1
        )
        self.move_jp = node.create_subscription(
            JointStateMessage, f"{prefix}/move_jp", self._move_jp_callback, 10
        )
        self.servo_cp = node.create_subscription(
            PoseStamped, f"{prefix}/servo_cp", self._servo_cp_callback, 1
        )
        self.move_cp = node.create_subscription(
            PoseStamped, f"{prefix}/move_cp", self._move_cp_callback, 10
        )
        self.state_command = node.create_subscription(
            StringStamped, f"{prefix}/state_command", self._state_command_callback, 10
        )

        self.jaw_measured_js = None
        self.jaw_setpoint_js = None
        self.jaw_servo_jp = None
        self.jaw_move_jp = None
        self.tool_type = None
        self.local_measured_cp = None
        self.local_setpoint_cp = None
        if config.type == "PSM":
            self.jaw_measured_js = node.create_publisher(
                JointStateMessage, f"{prefix}/jaw/measured_js", 10
            )
            self.jaw_setpoint_js = node.create_publisher(
                JointStateMessage, f"{prefix}/jaw/setpoint_js", 10
            )
            self.jaw_servo_jp = node.create_subscription(
                JointStateMessage,
                f"{prefix}/jaw/servo_jp",
                self._jaw_servo_jp_callback,
                1,
            )
            self.jaw_move_jp = node.create_subscription(
                JointStateMessage,
                f"{prefix}/jaw/move_jp",
                self._jaw_move_jp_callback,
                10,
            )
            self.tool_type = node.create_publisher(
                String, f"{prefix}/tool_type", transient_local_latched_qos()
            )
            self.local_measured_cp = node.create_publisher(
                PoseStamped, f"{prefix}/local/measured_cp", 10
            )
            self.local_setpoint_cp = node.create_publisher(
                PoseStamped, f"{prefix}/local/setpoint_cp", 10
            )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.config.joints)

    def _publish_warning(self, value: str) -> None:
        stamp = self.node.get_clock().now().to_msg()
        self.warning.publish(string_stamped_message(value, stamp, self.frame_id))
        self.node.get_logger().warning(f"{self.config.name}: {value}")

    def _event_stamp(self):
        """Return a strictly increasing timestamp for CRTK state events."""
        stamp = self.node.get_clock().now().to_msg()
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        value = max(value, self._last_event_stamp_ns + 1)
        self._last_event_stamp_ns = value
        stamp.sec, stamp.nanosec = divmod(value, 1_000_000_000)
        return stamp

    def _joint_command(self, message: JointStateMessage, channel: str) -> None:
        try:
            target = joint_positions_from_message(message, self.joint_names)
        except (TypeError, ValueError) as error:
            self._publish_warning(f"rejected {channel}: {error}")
            return
        if channel == "servo_jp":
            self.commands.submit_servo(channel, target.copy())
        elif self.commands.submit_discrete(channel, target.copy()) is None:
            self._publish_warning(f"rejected {channel}: command queue is full")

    def _jaw_command(self, message: JointStateMessage, channel: str) -> None:
        try:
            target = jaw_position_from_message(message)
        except (TypeError, ValueError) as error:
            self._publish_warning(f"rejected {channel}: {error}")
            return
        if channel == "jaw/servo_jp":
            self.commands.submit_servo(channel, target)
        elif self.commands.submit_discrete(channel, target) is None:
            self._publish_warning(f"rejected {channel}: command queue is full")

    def _world_view_pose(self) -> Pose | None:
        if self.ecm_interface is None:
            return None
        snapshot = self.ecm_interface.snapshots.peek()
        if snapshot is None:
            return None
        return view_pose_from_optical(snapshot.measured_cp_world)

    def _cartesian_command(self, message: PoseStamped, channel: str) -> None:
        try:
            target = pose_from_message(message)
        except (TypeError, ValueError, AttributeError) as error:
            self._publish_warning(f"rejected {channel}: {error}")
            return
        frame_id = message.header.frame_id
        if frame_id == self.config.base_frame:
            target = compose_pose(self.base_pose, target)
        elif self.ecm_interface is not None and frame_id != self.config.parent_frame:
            if frame_id and frame_id != "ECM_view":
                self._publish_warning(
                    f"rejected {channel}: unsupported frame {frame_id!r}"
                )
                return
            world_view = self._world_view_pose()
            if world_view is None:
                self._publish_warning(f"rejected {channel}: ECM state is unavailable")
                return
            target = compose_pose(world_view, target)
        elif frame_id and frame_id != self.config.parent_frame:
            self._publish_warning(f"rejected {channel}: unsupported frame {frame_id!r}")
            return
        if channel == "servo_cp":
            self.commands.submit_servo(channel, target)
        elif self.commands.submit_discrete(channel, target) is None:
            self._publish_warning(f"rejected {channel}: command queue is full")

    def _servo_jp_callback(self, message: JointStateMessage) -> None:
        self._joint_command(message, "servo_jp")

    def _move_jp_callback(self, message: JointStateMessage) -> None:
        self._joint_command(message, "move_jp")

    def _servo_cp_callback(self, message: PoseStamped) -> None:
        self._cartesian_command(message, "servo_cp")

    def _move_cp_callback(self, message: PoseStamped) -> None:
        self._cartesian_command(message, "move_cp")

    def _jaw_servo_jp_callback(self, message: JointStateMessage) -> None:
        self._jaw_command(message, "jaw/servo_jp")

    def _jaw_move_jp_callback(self, message: JointStateMessage) -> None:
        self._jaw_command(message, "jaw/move_jp")

    def _state_command_callback(self, message: StringStamped) -> None:
        if self.commands.submit_discrete("state_command", message.string) is None:
            self._publish_warning("rejected state_command: command queue is full")

    def install_initial_snapshot(self, snapshot: ArmSnapshot) -> None:
        self.snapshots.set_initial(snapshot)
        stamp = self._event_stamp()
        self.operating_state.publish(
            operating_state_message(snapshot.operating_state, stamp, self.frame_id)
        )
        self.state.publish(
            string_stamped_message(snapshot.operating_state.state, stamp, self.frame_id)
        )
        if self.tool_type is not None:
            tool = String()
            tool.data = self.config.instrument or ""
            self.tool_type.publish(tool)

    def publish_latest(self) -> None:
        snapshot, operating_state_events = self.snapshots.get_with_events()
        if snapshot is None:
            return
        stamp = self.node.get_clock().now().to_msg()
        measured_pose = snapshot.measured_cp_world
        setpoint_pose = snapshot.setpoint_cp_world
        measured_twist = snapshot.measured_cv_world
        world_view = self._world_view_pose()
        if world_view is not None and self.ecm_interface is not None:
            ecm_snapshot = self.ecm_interface.snapshots.peek()
            measured_pose = relative_pose(measured_pose, world_view)
            setpoint_pose = relative_pose(setpoint_pose, world_view)
            if ecm_snapshot is not None:
                measured_twist = relative_twist(
                    snapshot.measured_cp_world,
                    snapshot.measured_cv_world,
                    world_view,
                    ecm_snapshot.measured_cv_world,
                )

        self.measured_js.publish(
            joint_state_message(snapshot.measured_js, stamp, self.frame_id)
        )
        self.setpoint_js.publish(
            joint_state_message(snapshot.setpoint_js, stamp, self.frame_id)
        )
        self.measured_cp.publish(pose_stamped_message(measured_pose, stamp, self.frame_id))
        self.setpoint_cp.publish(pose_stamped_message(setpoint_pose, stamp, self.frame_id))
        self.measured_cv.publish(
            twist_stamped_message(measured_twist, stamp, self.frame_id)
        )
        for state in operating_state_events:
            event_stamp = self._event_stamp()
            self.operating_state.publish(
                operating_state_message(state, event_stamp, self.frame_id)
            )
            self.state.publish(
                string_stamped_message(state.state, event_stamp, self.frame_id)
            )

        if self.config.type == "PSM":
            local_measured = relative_pose(snapshot.measured_cp_world, self.base_pose)
            local_setpoint = relative_pose(snapshot.setpoint_cp_world, self.base_pose)
            self.local_measured_cp.publish(
                pose_stamped_message(local_measured, stamp, self.config.base_frame)
            )
            self.local_setpoint_cp.publish(
                pose_stamped_message(local_setpoint, stamp, self.config.base_frame)
            )
        if snapshot.jaw_measured is not None and self.jaw_measured_js is not None:
            jaw_measured = JointState(("jaw",), [snapshot.jaw_measured], [0.0])
            jaw_value = (
                snapshot.jaw_setpoint
                if snapshot.jaw_setpoint is not None
                else snapshot.jaw_measured
            )
            jaw_setpoint = JointState(("jaw",), [jaw_value], [0.0])
            self.jaw_measured_js.publish(
                joint_state_message(jaw_measured, stamp, self.frame_id)
            )
            self.jaw_setpoint_js.publish(
                joint_state_message(jaw_setpoint, stamp, self.frame_id)
            )


from dvrk_simulator_base.ros_interface import ArmRosInterface, LatestSnapshot


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
        grasp_config=None,
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
        self.grasp_config = grasp_config
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
        self._publishing_enabled = True
        self._state_publish_timer = self.create_timer(
            1.0 / state_rate, self._publish_latest
        )
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diagnostic_started_at = time.monotonic()
        self._diagnostic_snapshot_count = 0
        self._state_publish_rate_hz = state_rate
        self._diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)
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
        if not self._publishing_enabled or not rclpy.ok():
            return
        try:
            self._publish_diagnostics_impl()
        except Exception:
            if not rclpy.ok():
                return
            raise

    def _publish_diagnostics_impl(self) -> None:
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
        if not self._publishing_enabled or not rclpy.ok():
            return
        try:
            for interface in self.arm_interfaces.values():
                interface.publish_latest()
        except Exception:
            # ros2 launch can shut down the global context before this timer's
            # executor thread exits.  Publishing is no longer legal then.
            if not rclpy.ok():
                return
            raise

    def stop_publishing(self) -> None:
        """Prevent periodic callbacks from publishing during ROS shutdown."""
        self._publishing_enabled = False
        self._state_publish_timer.cancel()
        self._diagnostics_timer.cancel()


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
    parser.add_argument(
        "--gui", choices=("true", "false"),
        help="override the PyBullet GUI setting from the simulator YAML",
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
            gui=(False if "DVRK_SIMULATOR_TEST_TIMEOUT" in os.environ else
                 config.gui if options.gui is None else options.gui == "true"),
            simulation_rate_hz=config.simulation_rate_hz,
            state_publish_rate_hz=config.state_publish_rate_hz,
            generated_root=(
                None if config.generated_root is None else str(config.generated_root)
            ),
            command_queue_capacity=config.command_queue_capacity,
            renderer=config.renderer,
        )
        grasp = config.grasp
        node.get_logger().info(
            "grasp tuning: "
            f"markers={grasp.show_grasps}, "
            f"max_grasps={grasp.max_grasps_per_object}, "
            f"policy={grasp.policy}, arm_policies={grasp.arm_policies}, "
            f"close={grasp.close_threshold_rad:.3f} rad, "
            f"release={grasp.release_threshold_rad:.3f} rad, "
            f"break={grasp.break_distance_m:.3f} m/{grasp.break_orientation_rad:.3f} rad, "
            f"load={grasp.break_tension_force_n:.1f} N tension/"
            f"{grasp.break_shear_force_n:.1f} N shear/"
            f"{grasp.break_torque_nm:.2f} N m torque/"
            f"{grasp.break_load_duration_s:.3f} s, "
            f"force={grasp.max_force_n:.1f} N, erp={grasp.constraint_erp:.2f}, "
            f"capture offset={grasp.contact_region_offset_m} m, "
            f"radius={grasp.contact_region_radius_m:.3f} m"
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
            grasp_config=config.grasp,
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
        if node is not None:
            node.stop_publishing()
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
