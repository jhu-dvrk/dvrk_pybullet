"""Start the pybullet simulator profile."""

from dvrk_simulator_base.launch import simulator_launch


def generate_launch_description():
    return simulator_launch("dvrk_pybullet")
