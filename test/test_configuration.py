from pathlib import Path

from dvrk_pybullet.configuration import (
    load_installed_scene_config,
    load_simulator_config,
    resolve_scene_path,
    scene_search_paths,
)


def test_grasp_markers_default_to_enabled_and_can_be_disabled(tmp_path):
    default_path = tmp_path / "default.yaml"
    default_path.write_text("{}\n", encoding="utf-8")
    defaults = load_simulator_config(default_path).grasp
    assert defaults.show_grasps is True
    assert defaults.max_grasps_per_object == 1
    assert defaults.break_distance_m == 0.005
    assert defaults.break_orientation_rad == 0.2617993877991494

    disabled_path = tmp_path / "disabled.yaml"
    disabled_path.write_text("grasp:\n  show_grasps: false\n", encoding="utf-8")
    assert load_simulator_config(disabled_path).grasp.show_grasps is False

    policies_path = tmp_path / "policies.yaml"
    policies_path.write_text(
        "grasp:\n  policy: force_torque\n  arms:\n    PSM1:\n      policy: pose_error\n",
        encoding="utf-8",
    )
    policies = load_simulator_config(policies_path).grasp
    assert policies.policy == "force_torque"
    assert policies.arm_policies == {"PSM1": "pose_error"}


def test_scene_search_paths_includes_exercises():
    paths = scene_search_paths(Path("/tmp/test_config.yaml"))
    path_strings = [str(p) for p in paths]
    assert any("exercises" in s for s in path_strings)
    assert any("scenes" in s for s in path_strings)


def test_resolve_scene_path_single_and_sequence():
    config_path = Path("/tmp/test_config.yaml")
    # Resolve single installed scene
    resolved_single = resolve_scene_path(config_path, "ECM_PSM1_PSM2.yaml")
    assert isinstance(resolved_single, Path)
    assert resolved_single.name == "ECM_PSM1_PSM2.yaml"
    assert resolved_single.is_file()

    # Resolve sequence including exercise
    resolved_seq = resolve_scene_path(
        config_path, ["ECM_PSM1_PSM2.yaml", "peg_board_ring.yaml"]
    )
    assert isinstance(resolved_seq, tuple)
    assert len(resolved_seq) == 2
    assert resolved_seq[0].name == "ECM_PSM1_PSM2.yaml"
    assert resolved_seq[1].name == "peg_board_ring.yaml"
    assert resolved_seq[1].is_file()


def test_load_installed_scene_config_with_exercise():
    config_path = Path("/tmp/test_config.yaml")
    resolved = resolve_scene_path(
        config_path, ["ECM_PSM1_PSM2.yaml", "peg_board_ring.yaml"]
    )
    scene = load_installed_scene_config(resolved)
    assert "ECM_PSM1_PSM2" in scene.name
    assert "peg_board_ring" in scene.name
    robot_names = [robot.name for robot in scene.robots]
    assert "PSM1" in robot_names
    assert "ECM" in robot_names
    object_names = [obj.name for obj in scene.objects]
    assert "table" in object_names
    assert "peg_board" in object_names
    assert "ring" in object_names


def test_load_installed_scene_config_with_cuhk_exercise():
    config_path = Path("/tmp/test_config.yaml")
    resolved = resolve_scene_path(
        config_path, ["ECM_PSM1_PSM2.yaml", "peg_board_CUHK.yaml"]
    )
    scene = load_installed_scene_config(resolved)
    assert "ECM_PSM1_PSM2" in scene.name
    assert "peg_board_CUHK" in scene.name
    object_names = [obj.name for obj in scene.objects]
    assert "table" in object_names
    assert "peg_board" in object_names
    assert "triangular_block" in object_names
    objects = {obj.name: obj for obj in scene.objects}
    quarter_turn = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
    assert objects["peg_board"].orientation_xyzw == quarter_turn
    assert objects["triangular_block"].position == (-0.013, -0.03, 0.065)
    assert objects["triangular_block"].orientation_xyzw == quarter_turn


def test_load_installed_scene_config_with_tray_cubes():
    config_path = Path("/tmp/test_config.yaml")
    resolved = resolve_scene_path(
        config_path, ["ECM_PSM1_PSM2.yaml", "tray_cubes.yaml"]
    )
    scene = load_installed_scene_config(resolved)
    assert "tray_cubes" in scene.name
    object_names = [obj.name for obj in scene.objects]
    assert "table" in object_names
    assert "tray" in object_names
    assert "cube" in object_names
    assert "grasp_cube" in object_names
