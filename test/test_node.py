import numpy as np
import pytest
import rclpy
from pathlib import Path
from types import SimpleNamespace
from crtk_msgs.msg import StringStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from dvrk_arm_description import JointConfig, RobotConfig
from dvrk_simulator_base.cartesian_command import CartesianCommand
from dvrk_simulator_base.scene import SceneCamera
import dvrk_simulator_base.ros_interface as ros_interface_module

import dvrk_pybullet.node as node_module


def _config():
    joints = tuple(
        JointConfig(name, "prismatic" if name == "insertion" else "revolute", -1.0, 1.0, 1.0)
        for name in ("yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw")
    )
    return RobotConfig(
        name="PSM1",
        type="PSM",
        instrument="420006",
        endoscope=None,
        parent_frame="world",
        base_frame="PSM1_base",
        tool_frame="PSM1_tool",
        adaptor_frame="PSM1_adaptor_link",
        base_position=np.zeros(3),
        base_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        joints=joints,
        home_position=np.zeros(6),
        raw={"robot": {"cartesian": {"reference_frame": "world"}}},
    )


def _ecm_config():
    joints = tuple(
        JointConfig(name, "prismatic" if name == "insertion" else "revolute", -1.0, 1.0, 1.0)
        for name in ("yaw", "pitch", "insertion", "roll")
    )
    return RobotConfig(
        name="ECM",
        type="ECM",
        instrument=None,
        endoscope="Si_straight",
        parent_frame="world",
        base_frame="ECM_base",
        tool_frame="ECM_optical",
        adaptor_frame="ECM_adaptor_link",
        base_position=np.zeros(3),
        base_orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        joints=joints,
        home_position=np.zeros(4),
        raw={"robot": {"cartesian": {"reference_frame": "world"}}},
    )


def test_scene_command_line_argument_is_parsed_without_ros_arguments():
    args = node_module._parse_command_line([
        "--scene", "ECM_PSM1_PSM2.yaml",
        "--ros-args", "-r", "__ns:=/simulation",
    ])

    assert args.scene == [Path("ECM_PSM1_PSM2.yaml")]


def test_unknown_scene_is_reported_without_a_traceback(capsys):
    assert node_module.main(["--scene", "does-not-exist"]) == 2
    error = capsys.readouterr().err
    assert "Scene configuration not found: does-not-exist" in error
    assert "Searched scene paths:" in error
    assert "Available scenes:" in error
    assert "ECM_PSM1_PSM2.yaml" in error
    assert "Traceback" not in error


def test_simulator_config_keeps_runtime_settings_out_of_cli(tmp_path):
    path = tmp_path / "pybullet.yaml"
    path.write_text(
        "renderer: tiny\nheadless: false\nsimulation_rate_hz: 240\n"
        "state_publish_rate_hz: 80\ncommand_queue_capacity: 12\n"
        "scene: ECM_PSM1_PSM2.yaml\n",
        encoding="utf-8",
    )
    config = node_module.load_simulator_config(path)
    assert config.renderer == "tiny"
    assert config.headless is False
    assert config.simulation_rate_hz == 240.0
    assert config.scene == "ECM_PSM1_PSM2.yaml"


def test_simulator_config_defaults_to_headless_false(tmp_path):
    path = tmp_path / "pybullet.yaml"
    path.write_text("", encoding="utf-8")
    assert node_module.load_simulator_config(path).headless is False


def test_simulator_config_requires_boolean_headless_value(tmp_path):
    path = tmp_path / "pybullet.yaml"
    path.write_text("headless: true\n", encoding="utf-8")
    config = node_module.load_simulator_config(path)
    assert config.headless is True

    path.write_text("headless: 'false'\n", encoding="utf-8")
    with pytest.raises(ValueError, match="headless must be true or false"):
        node_module.load_simulator_config(path)


def test_scene_argument_optional_when_in_config():
    args = node_module._parse_command_line([])
    assert args.scene is None


def test_scene_required_when_missing_from_cli_and_config(capsys, tmp_path):
    cfg_path = tmp_path / "pybullet.yaml"
    cfg_path.write_text("scene: null\n", encoding="utf-8")
    assert node_module.main(["--config", str(cfg_path)]) == 2
    error = capsys.readouterr().err
    assert "a scene is required: specify --scene or scene in the runtime YAML" in error


def test_headless_command_line_argument():
    args = node_module._parse_command_line(["--headless", "true"])
    assert args.headless == "true"
    args = node_module._parse_command_line(["--headless", "false"])
    assert args.headless == "false"


def test_node_exposes_state_and_command_topics(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    received = []

    def load_config(path):
        received.append(path)
        return SimpleNamespace(robots=(_config(),))

    monkeypatch.setattr(
        node_module, "load_installed_scene_config", load_config
    )
    rclpy.init()
    node = None
    try:
        node = node_module.DvrkPyBulletNode(scene_path=Path("scene.yaml"))
        assert received == [Path("scene.yaml")]
        arm = node.arm_interfaces["PSM1"]
        topics = {
            arm.measured_js.topic_name,
            arm.setpoint_js.topic_name,
            arm.measured_cp.topic_name,
            arm.setpoint_cp.topic_name,
            arm.measured_cv.topic_name,
            arm.jaw_measured_js.topic_name,
            arm.jaw_setpoint_js.topic_name,
            arm.operating_state.topic_name,
            arm.state.topic_name,
            arm.tool_type.topic_name,
            arm.info.topic_name,
            arm.warning.topic_name,
            arm.error.topic_name,
        }
        assert topics == {
            "/PSM1/measured_js",
            "/PSM1/setpoint_js",
            "/PSM1/measured_cp",
            "/PSM1/setpoint_cp",
            "/PSM1/measured_cv",
            "/PSM1/jaw/measured_js",
            "/PSM1/jaw/setpoint_js",
            "/PSM1/operating_state",
            "/PSM1/state",
            "/PSM1/tool_type",
            "/PSM1/info",
            "/PSM1/warning",
            "/PSM1/error",
        }
        subscriptions = {
            arm.servo_jp.topic_name,
            arm.move_jp.topic_name,
            arm.servo_cp.topic_name,
            arm.move_cp.topic_name,
            arm.jaw_servo_jp.topic_name,
            arm.jaw_move_jp.topic_name,
            arm.state_command.topic_name,
        }
        assert subscriptions == {
            "/PSM1/servo_jp",
            "/PSM1/move_jp",
            "/PSM1/servo_cp",
            "/PSM1/move_cp",
            "/PSM1/jaw/servo_jp",
            "/PSM1/jaw/move_jp",
            "/PSM1/state_command",
        }
        first = arm._event_stamp()
        second = arm._event_stamp()
        first_ns = first.sec * 1_000_000_000 + first.nanosec
        second_ns = second.sec * 1_000_000_000 + second.nanosec
        assert second_ns > first_ns
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_callbacks_validate_and_enqueue_without_touching_backend(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(node_module, "load_installed_scene_config", lambda *_: SimpleNamespace(robots=(_config(),)))
    rclpy.init()
    node = None
    try:
        node = node_module.DvrkPyBulletNode(scene_path=Path("scene.yaml"))
        arm = node.arm_interfaces["PSM1"]
        joint = JointState()
        joint.name = ["pitch", "yaw", "insertion", "roll", "wrist_pitch", "wrist_yaw"]
        joint.position = [0.2, 0.1, 0.12, 0.3, 0.4, 0.5]
        arm._servo_jp_callback(joint)

        jaw = JointState()
        jaw.position = [0.25]
        arm._jaw_move_jp_callback(jaw)
        commands = arm.commands.drain()

        pose = PoseStamped()
        pose.pose.position.x = 0.01
        pose.pose.position.y = 0.02
        pose.pose.position.z = 0.03
        pose.pose.orientation.w = 1.0
        original_pose_from_message = ros_interface_module.pose_from_message
        converted = []

        def record_conversion(message):
            converted.append(message)
            return original_pose_from_message(message)

        monkeypatch.setattr(ros_interface_module, "pose_from_message", record_conversion)
        invalid = PoseStamped()
        invalid.pose.orientation.w = 0.0
        arm._servo_cp_callback(invalid)  # Invalid input is rejected before IPC.
        arm._servo_cp_callback(pose)
        assert converted == [invalid, pose]

        state = StringStamped()
        state.string = "pause"
        arm._state_command_callback(state)

        commands += arm.commands.drain()
        assert [command.channel for command in commands] == [
            "servo_jp",
            "jaw/move_jp",
            "servo_cp",
            "state_command",
        ]
        np.testing.assert_allclose(
            commands[0].payload, [0.1, 0.2, 0.12, 0.3, 0.4, 0.5]
        )
        assert commands[1].payload == 0.25
        assert isinstance(commands[2].payload, CartesianCommand)
        target = commands[2].payload.pose
        assert converted == [invalid, pose]
        np.testing.assert_allclose(
            target.position, [0.01, 0.02, 0.03]
        )
        assert commands[3].payload == "pause"
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_scene_creates_independent_psm_and_ecm_interfaces(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        node_module,
        "load_installed_scene_config",
        lambda *_: SimpleNamespace(
            robots=(_config(), _ecm_config()), camera=SceneCamera(), objects=()
        ),
    )
    monkeypatch.setattr(node_module, "resolve_scene_path", lambda *_: "/tmp/test.yaml")
    rclpy.init()
    node = None
    try:
        node = node_module.DvrkPyBulletNode(scene_path=Path("/tmp/test.yaml"))
        assert set(node.arm_interfaces) == {"PSM1", "ECM"}
        psm = node.arm_interfaces["PSM1"]
        ecm = node.arm_interfaces["ECM"]
        assert psm.frame_id == "ECM_view"
        assert ecm.frame_id == "world"
        assert psm.commands is not ecm.commands
        assert psm.local_measured_cp.topic_name == "/PSM1/local/measured_cp"
        assert ecm.local_measured_cp is None
        assert ecm.jaw_measured_js is None
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_node_uses_the_scene_path_supplied_by_the_command_layer(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    received = []

    def load_scene(path):
        received.append(path)
        return SimpleNamespace(
            robots=(_config(), _ecm_config()), camera=SceneCamera(), objects=()
        )

    monkeypatch.setattr(node_module, "load_installed_scene_config", load_scene)
    rclpy.init()
    node = None
    try:
        node = node_module.DvrkPyBulletNode(
            scene_path=Path("ECM_PSM1_PSM2.yaml"),
        )
        assert received[0].name == "ECM_PSM1_PSM2.yaml"
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
