"""Initialize a PyBullet scene inside the shared IPC simulation worker."""


from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.process_worker import worker_main


def create_runtime(start):
    from .camera import CameraOptions
    from .configuration import load_installed_scene_config, load_simulator_config
    from .runtime import RuntimeOptions
    from .world_runtime import PyBulletWorldRuntime

    config = load_simulator_config(start["config"])
    scene = load_installed_scene_config(start["scene"])
    camera = CameraOptions.from_scene(scene.camera, renderer=config.renderer)
    commands = {item.name: CommandMailboxes(config.command_queue_capacity) for item in scene.robots}
    runtime = PyBulletWorldRuntime(scene.robots, RuntimeOptions(
        headless=start["headless"], simulation_rate_hz=config.simulation_rate_hz, generated_root=config.generated_root,
    ), commands, camera_options=camera, scene_objects=tuple(scene.objects), grasp_config=config.grasp)
    return runtime, commands


if __name__ == "__main__":
    raise SystemExit(worker_main(create_runtime))
