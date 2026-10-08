"""Scene-based ROS frontend for a separate PyBullet simulation process."""

import argparse
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.utilities import remove_ros_args

from dvrk_simulator_base.ros_node import SimulatorRosNode, run_frontend
from .configuration import load_installed_scene_config, load_simulator_config, resolve_scene_path
from .python_runtime import resolve_pybullet_python


class DvrkPyBulletNode(SimulatorRosNode):
    def __init__(self, *, scene_path, state_publish_rate_hz=100.0, command_queue_capacity=32):
        scene = load_installed_scene_config(scene_path)
        super().__init__("dvrk_pybullet", scene.robots, state_publish_rate_hz=state_publish_rate_hz,
                         command_queue_capacity=command_queue_capacity)


def _parse_command_line(args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="PyBullet runtime YAML")
    parser.add_argument("--scene", type=Path, required=True, action="append", help="scene YAML; may be repeated")
    parser.add_argument("--gui", choices=("true", "false"), help="override desktop viewer setting")
    return parser.parse_args(remove_ros_args(args))


def main(args=None):
    raw_args = list(sys.argv[1:] if args is None else args)
    options = _parse_command_line(raw_args)
    path = options.config or Path(get_package_share_directory("dvrk_pybullet")) / "share/pybullet.yaml"
    try:
        config = load_simulator_config(path)
        scenes = resolve_scene_path(path, options.scene)
        python = resolve_pybullet_python(config.generated_root).path
    except (RuntimeError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    gui = config.gui if options.gui is None else options.gui == "true"
    start = dict(config=str(path.expanduser().resolve()), scene=[str(item) for item in scenes],
                 gui=gui and "DVRK_SIMULATOR_TEST_TIMEOUT" not in os.environ)
    return run_frontend(
        lambda: DvrkPyBulletNode(scene_path=scenes, state_publish_rate_hz=config.state_publish_rate_hz,
                                command_queue_capacity=config.command_queue_capacity),
        python, "dvrk_pybullet.simulation_worker", start, raw_args,
    )


if __name__ == "__main__":
    raise SystemExit(main())
