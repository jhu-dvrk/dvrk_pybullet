"""Start the PyBullet patient cart with two 3Dconnexion MTMs."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from dvrk_simulator_base.launch import SystemProfile, simulator_launch


def generate_launch_description():
    directory = Path(get_package_share_directory("dvrk_pybullet")) / "share/3dconnexion"
    return simulator_launch("dvrk_pybullet", profile=SystemProfile(
        config=directory / "pybullet.yaml",
        cart_scene="ECM_PSM1_PSM2_PSM3.yaml",
        system_config=directory / "system-MTML-MTMR-3Dconnexion-patient-cart-ROS.json",
        cwd=directory, auto_start=True, preview=True,
    ))
